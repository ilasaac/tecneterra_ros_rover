#!/usr/bin/env python3
"""
tools/mission_data_collector.py — Comprehensive mission data collector (ROS2).

Runs INSIDE the rover Docker container alongside the navigator.  Subscribes to
all relevant topics and writes:

  /tmp/mission_data_YYYYMMDD_HHMMSS/
    flight_log.csv   — 25 Hz log with GPS, commands, actual speed, front antenna
    mission.json     — captured waypoints + obstacles + rerouted path

The flight_log.csv is a SUPERSET of navigator_diag.csv columns so it can be
loaded directly into mission_planner.py's Analyze panel.  Extra columns
(speed_actual, front_lat, front_lon, turn_rate, mode, armed) are ignored by
the analyzer but available for deeper comparison with sim_navigator.

Usage:
  # Start before arming — captures mission upload + full autonomous run
  docker exec -d agri_rover_rv1 python3 /workspaces/tools/mission_data_collector.py \\
      --ns rv1

  # Stop with Ctrl+C or kill.  Copy out:
  docker cp agri_rover_rv1:/tmp/mission_data_<timestamp>/ .

  # Then in mission_planner.py:
  #   1. Load the mission.json (or the same mission file used for upload)
  #   2. Click Simulate to run SIL
  #   3. Open Analyze panel, load flight_log.csv
  #   4. Compare real track (colored by CTE) vs sim path
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32, Int32, String

from agri_rover_interfaces.msg import MissionWaypoint, RCInput


EARTH_R = 6_371_000.0


def _haversine(lat1, lon1, lat2, lon2):
    """Distance in metres between two GPS positions."""
    r1, r2 = math.radians(lat1), math.radians(lat2)
    dlat = r2 - r1
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(r1) * math.cos(r2) * math.sin(dlon / 2) ** 2
    return EARTH_R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


class MissionDataCollector(Node):

    def __init__(self, ns: str, output_dir: str, max_speed: float):
        super().__init__('mission_data_collector')

        self._ns = ns
        self._max_speed = max_speed
        self._output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

        # ── CSV log ──────────────────────────────────────────────────────────
        csv_path = os.path.join(output_dir, 'flight_log.csv')
        self._csv_file = open(csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            # navigator_diag.csv compatible columns
            't', 'lat', 'lon', 'heading',
            'target_brg', 'hdg_err', 'cte',
            'steer_frac', 'steer_ppm', 'throttle_ppm',
            'speed_tgt', 'dist_to_wp', 'wp_idx', 'algo', 'fix_quality', 'hacc_mm',
            # extra columns for sim comparison
            'speed_actual', 'front_lat', 'front_lon', 'turn_rate',
            'mode', 'armed',
        ])
        self._csv_count = 0

        # ── Mission capture ──────────────────────────────────────────────────
        self._mission_wps: list[dict] = []
        self._obstacles: list = []
        self._rerouted_path: list = []
        self._mission_file = os.path.join(output_dir, 'mission.json')

        # ── Latest topic state ───────────────────────────────────────────────
        self._lat = 0.0
        self._lon = 0.0
        self._fix_quality = -1
        self._front_lat = 0.0
        self._front_lon = 0.0
        self._heading = 0.0
        self._hacc_mm = -1.0
        self._mode = 'MANUAL'
        self._armed = False
        self._wp_idx = 0
        self._xte = 0.0

        # cmd_override state (from navigator output)
        self._cmd_throttle = 1500
        self._cmd_steer = 1500
        self._cmd_time = 0.0

        # Speed computation from GPS deltas
        self._prev_lat = 0.0
        self._prev_lon = 0.0
        self._prev_fix_time = 0.0
        self._speed_actual = 0.0

        # Turn rate from heading deltas
        self._prev_heading = 0.0
        self._prev_hdg_time = 0.0
        self._turn_rate = 0.0

        # Track whether we're in autonomous mode (only log when active)
        self._logging_active = False
        self._first_fix = False

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(NavSatFix, f'/{ns}/fix',
                                 self._cb_fix, 10)
        self.create_subscription(NavSatFix, f'/{ns}/fix_front',
                                 self._cb_fix_front, 10)
        self.create_subscription(Float32, f'/{ns}/heading',
                                 self._cb_heading, 10)
        self.create_subscription(RCInput, f'/{ns}/cmd_override',
                                 self._cb_cmd, 10)
        self.create_subscription(String, f'/{ns}/mode',
                                 self._cb_mode, 10)
        self.create_subscription(Bool, f'/{ns}/armed',
                                 self._cb_armed, 10)
        self.create_subscription(Int32, f'/{ns}/wp_active',
                                 self._cb_wp_active, 10)
        self.create_subscription(Float32, f'/{ns}/xte',
                                 self._cb_xte, 10)
        self.create_subscription(Float32, f'/{ns}/hacc',
                                 self._cb_hacc, 10)
        self.create_subscription(MissionWaypoint, f'/{ns}/mission',
                                 self._cb_mission, 200)
        self.create_subscription(String, f'/{ns}/mission_fence',
                                 self._cb_fence, 10)
        self.create_subscription(String, f'/{ns}/rerouted_path',
                                 self._cb_rerouted, 10)
        self.create_subscription(Bool, f'/{ns}/mission_clear',
                                 self._cb_mission_clear, 10)

        # 25 Hz logging timer
        self.create_timer(0.04, self._tick)

        self.get_logger().info(f'Mission data collector started → {output_dir}')
        self.get_logger().info(f'Subscribed to /{ns}/* topics')
        self.get_logger().info('Logging starts when AUTONOMOUS + armed (or use --always)')

    # ── GPS callbacks ────────────────────────────────────────────────────────

    def _cb_fix(self, msg: NavSatFix):
        now = time.time()
        lat, lon = msg.latitude, msg.longitude
        if lat == 0.0 and lon == 0.0:
            return

        # Compute actual speed from GPS deltas
        if self._first_fix and self._prev_fix_time > 0:
            dt = now - self._prev_fix_time
            if 0.01 < dt < 2.0:
                dist = _haversine(self._prev_lat, self._prev_lon, lat, lon)
                self._speed_actual = dist / dt

        self._prev_lat = self._lat
        self._prev_lon = self._lon
        self._prev_fix_time = now
        self._lat = lat
        self._lon = lon
        self._fix_quality = msg.status.status
        self._first_fix = True

    def _cb_fix_front(self, msg: NavSatFix):
        if msg.latitude != 0.0 or msg.longitude != 0.0:
            self._front_lat = msg.latitude
            self._front_lon = msg.longitude

    def _cb_heading(self, msg: Float32):
        now = time.time()
        hdg = msg.data
        if self._prev_hdg_time > 0:
            dt = now - self._prev_hdg_time
            if 0.01 < dt < 2.0:
                dh = hdg - self._prev_heading
                # Normalize to [-180, 180]
                if dh > 180:
                    dh -= 360
                elif dh < -180:
                    dh += 360
                self._turn_rate = dh / dt
        self._prev_heading = self._heading
        self._prev_hdg_time = now
        self._heading = hdg

    def _cb_hacc(self, msg: Float32):
        self._hacc_mm = msg.data

    # ── Command callbacks ────────────────────────────────────────────────────

    def _cb_cmd(self, msg: RCInput):
        ch = list(msg.channels) + [1500] * (16 - len(msg.channels))
        self._cmd_throttle = ch[0]
        self._cmd_steer = ch[1]
        self._cmd_time = time.time()

    def _cb_mode(self, msg: String):
        prev = self._mode
        self._mode = msg.data
        if prev != msg.data:
            self.get_logger().info(f'Mode: {prev} → {msg.data}')

    def _cb_armed(self, msg: Bool):
        prev = self._armed
        self._armed = msg.data
        if prev != msg.data:
            self.get_logger().info(f'Armed: {prev} → {msg.data}')

    def _cb_wp_active(self, msg: Int32):
        self._wp_idx = msg.data

    def _cb_xte(self, msg: Float32):
        self._xte = msg.data

    # ── Mission capture callbacks ────────────────────────────────────────────

    def _cb_mission(self, msg: MissionWaypoint):
        wp = {
            'seq': msg.seq,
            'lat': round(msg.latitude, 8),
            'lon': round(msg.longitude, 8),
            'speed': round(msg.speed, 3),
            'acceptance_radius': round(msg.acceptance_radius, 3),
            'hold_secs': round(msg.hold_secs, 1) if hasattr(msg, 'hold_secs') else 0.0,
        }
        # Replace if same seq, else append
        for i, existing in enumerate(self._mission_wps):
            if existing['seq'] == msg.seq:
                self._mission_wps[i] = wp
                break
        else:
            self._mission_wps.append(wp)
        self._mission_wps.sort(key=lambda w: w['seq'])
        self._save_mission()
        if msg.seq == 0:
            self.get_logger().info('Mission upload started (seq=0)')

    def _cb_fence(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._obstacles = data.get('polygons', [])
            self._save_mission()
            n = len(self._obstacles)
            self.get_logger().info(f'Obstacles received: {n} polygon(s)')
        except json.JSONDecodeError:
            pass

    def _cb_rerouted(self, msg: String):
        try:
            self._rerouted_path = json.loads(msg.data)
            self._save_mission()
            n = len(self._rerouted_path)
            self.get_logger().info(f'Rerouted path received: {n} waypoints')
        except json.JSONDecodeError:
            pass

    def _cb_mission_clear(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Mission cleared')

    def _save_mission(self):
        """Save current mission state to mission.json."""
        data = {
            'waypoints': [{'lat': w['lat'], 'lon': w['lon'],
                           'speed': w.get('speed', 0),
                           'hold_secs': w.get('hold_secs', 0)}
                          for w in self._mission_wps],
            'obstacles': self._obstacles,
            'rerouted_path': self._rerouted_path,
            'mission_raw': self._mission_wps,
        }
        with open(self._mission_file, 'w') as f:
            json.dump(data, f, indent=2)

    # ── Logging tick ─────────────────────────────────────────────────────────

    def _tick(self):
        # Only log when we have GPS and rover is armed+autonomous
        if not self._first_fix:
            return
        if not self._armed or self._mode != 'AUTONOMOUS':
            if self._logging_active:
                self._logging_active = False
                self._csv_file.flush()
                self.get_logger().info(
                    f'Logging paused ({self._csv_count} rows so far)')
            return

        if not self._logging_active:
            self._logging_active = True
            self.get_logger().info('Logging started (AUTONOMOUS + armed)')

        # Derive speed_tgt from throttle PPM (approximate)
        speed_tgt = max(0.0, (self._cmd_throttle - 1500) / 500.0 * self._max_speed)

        # Derive steer_frac from steer PPM
        steer_frac = (1500 - self._cmd_steer) / 500.0

        self._csv_writer.writerow([
            # navigator_diag.csv compatible columns
            round(time.time(), 4),
            round(self._lat, 8),
            round(self._lon, 8),
            round(self._heading, 2),
            '',                          # target_brg (not available from topics)
            '',                          # hdg_err (not available from topics)
            round(self._xte, 4),         # cte (unsigned XTE — analyze takes abs())
            round(steer_frac, 4),
            self._cmd_steer,
            self._cmd_throttle,
            round(speed_tgt, 3),
            '',                          # dist_to_wp (not available from topics)
            self._wp_idx,
            'stanley',                   # algo (not available — placeholder)
            self._fix_quality,
            round(self._hacc_mm, 1),
            # extra columns
            round(self._speed_actual, 4),
            round(self._front_lat, 8),
            round(self._front_lon, 8),
            round(self._turn_rate, 2),
            self._mode,
            1 if self._armed else 0,
        ])
        self._csv_count += 1

        if self._csv_count % 250 == 0:
            self._csv_file.flush()
            self.get_logger().info(f'Logged {self._csv_count} rows')

    # ── Cleanup ──────────────────────────────────────────────────────────────

    def close(self):
        self._csv_file.flush()
        self._csv_file.close()
        self._save_mission()
        self.get_logger().info(
            f'Saved {self._csv_count} rows to {self._output_dir}/flight_log.csv')
        self.get_logger().info(
            f'Mission: {len(self._mission_wps)} wps, '
            f'{len(self._obstacles)} obstacles → {self._mission_file}')


def main():
    parser = argparse.ArgumentParser(
        description='Collect all mission data for sim comparison')
    parser.add_argument('--ns', default='rv1',
                        help='Rover namespace (rv1 or rv2)')
    parser.add_argument('--output', '-o', default=None,
                        help='Output directory (default: /tmp/mission_data_<timestamp>)')
    parser.add_argument('--max-speed', type=float, default=1.5,
                        help='Max speed m/s for PPM→speed conversion (default 1.5)')
    args = parser.parse_args()

    if args.output is None:
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        args.output = f'/tmp/mission_data_{ts}'

    rclpy.init()
    node = MissionDataCollector(args.ns, args.output, args.max_speed)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
