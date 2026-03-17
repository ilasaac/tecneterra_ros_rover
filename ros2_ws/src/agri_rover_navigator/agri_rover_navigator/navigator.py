"""
agri_rover_navigator — navigator node

Pure-pursuit mission follower for ground rover navigation.

Subscribes:
  ~/fix          (sensor_msgs/NavSatFix)   — current position
  ~/heading      (std_msgs/Float32)        — degrees from north
  ~/mode         (std_msgs/String)         — only active in AUTONOMOUS mode
  ~/mission      (MissionWaypoint)         — waypoints arrive sequentially

Publishes:
  ~/cmd_override (RCInput)                 — PPM override to rp2040_bridge

Services:
  ~/pause_mission  (std_srvs/Trigger)
  ~/resume_mission (std_srvs/Trigger)
"""

from __future__ import annotations

import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger

from agri_rover_interfaces.msg import RCInput, MissionWaypoint

PPM_CENTER = 1500
PPM_MIN    = 1000
PPM_MAX    = 2000
EARTH_R    = 6_371_000.0   # metres


def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Distance in metres between two GPS coordinates."""
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2
         + math.cos(math.radians(lat1))
         * math.cos(math.radians(lat2))
         * math.sin(dlon / 2) ** 2)
    return 2 * EARTH_R * math.asin(math.sqrt(a))


def bearing_to(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Bearing in degrees (0–360) from point 1 to point 2."""
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(math.radians(lat2))
    y = (math.cos(math.radians(lat1)) * math.sin(math.radians(lat2))
         - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(dlon))
    return (math.degrees(math.atan2(x, y)) + 360) % 360


class NavigatorNode(Node):

    def __init__(self):
        super().__init__('navigator')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('lookahead_distance',      3.0)
        self.declare_parameter('default_acceptance_radius', 1.5)
        self.declare_parameter('max_speed',               1.5)
        self.declare_parameter('min_speed',               0.3)
        self.declare_parameter('max_steering',            0.8)
        self.declare_parameter('control_rate',            10.0)
        self.declare_parameter('gps_timeout',             2.0)
        self.declare_parameter('heading_deadband',        3.0)

        self._lookahead   = self.get_parameter('lookahead_distance').value
        self._accept_r    = self.get_parameter('default_acceptance_radius').value
        self._max_speed   = self.get_parameter('max_speed').value
        self._min_speed   = self.get_parameter('min_speed').value
        self._max_steer   = self.get_parameter('max_steering').value
        self._gps_timeout = self.get_parameter('gps_timeout').value
        self._hdb         = self.get_parameter('heading_deadband').value

        # ── State ────────────────────────────────────────────────────────────
        self._fix:          NavSatFix | None = None
        self._fix_time:     float            = 0.0
        self._heading:      float            = 0.0
        self._mode:         str              = 'MANUAL'
        self._waypoints:    deque[MissionWaypoint] = deque()
        self._paused:       bool             = False
        self._active_wp:    MissionWaypoint | None = None

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(NavSatFix,       'fix',     self._cb_fix,     10)
        self.create_subscription(Float32,         'heading', self._cb_heading,  10)
        self.create_subscription(String,          'mode',    self._cb_mode,    10)
        self.create_subscription(MissionWaypoint, 'mission', self._cb_mission, 10)

        # ── Publisher ────────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(RCInput, 'cmd_override', 10)

        # ── Services ─────────────────────────────────────────────────────────
        self.create_service(Trigger, 'pause_mission',  self._svc_pause)
        self.create_service(Trigger, 'resume_mission', self._svc_resume)

        # ── Control loop ─────────────────────────────────────────────────────
        rate = self.get_parameter('control_rate').value
        self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info('Navigator ready')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_fix(self, msg: NavSatFix):
        self._fix      = msg
        self._fix_time = time.time()

    def _cb_heading(self, msg: Float32):
        self._heading = msg.data

    def _cb_mode(self, msg: String):
        self._mode = msg.data

    def _cb_mission(self, msg: MissionWaypoint):
        self._waypoints.append(msg)
        if self._active_wp is None:
            self._advance_waypoint()
        self.get_logger().info(
            f'Waypoint {msg.seq} queued ({len(self._waypoints)} pending)')

    # ── Services ──────────────────────────────────────────────────────────────

    def _svc_pause(self, _, response):
        self._paused       = True
        response.success   = True
        response.message   = 'Mission paused'
        return response

    def _svc_resume(self, _, response):
        self._paused       = False
        response.success   = True
        response.message   = 'Mission resumed'
        return response

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self):
        if self._mode != 'AUTONOMOUS' or self._paused or self._active_wp is None:
            return

        if self._fix is None or (time.time() - self._fix_time) > self._gps_timeout:
            self.get_logger().warn('GPS stale — halting')
            self._publish_halt()
            return

        wp     = self._active_wp
        dist   = haversine(self._fix.latitude, self._fix.longitude,
                           wp.latitude, wp.longitude)
        accept = wp.acceptance_radius if wp.acceptance_radius > 0 else self._accept_r

        if dist < accept:
            self.get_logger().info(f'Waypoint {wp.seq} reached')
            self._advance_waypoint()
            return

        # Pure pursuit — compute required heading
        target_bearing = bearing_to(
            self._fix.latitude, self._fix.longitude,
            wp.latitude, wp.longitude)

        heading_err = ((target_bearing - self._heading + 180) % 360) - 180

        # Steering: proportional to heading error, capped at max_steering
        steer_frac = max(-self._max_steer,
                         min(self._max_steer, heading_err / 45.0))
        steer_ppm  = int(PPM_CENTER + steer_frac * 500)

        # Speed: reduce when turning hard
        speed_frac  = 1.0 - 0.5 * abs(steer_frac)
        target_spd  = wp.speed if wp.speed > 0 else self._max_speed
        speed_frac *= target_spd / self._max_speed
        speed_frac  = max(self._min_speed / self._max_speed, speed_frac)
        throttle_ppm = int(PPM_CENTER + speed_frac * 500)

        self._publish_cmd(throttle_ppm, steer_ppm)

    def _advance_waypoint(self):
        if self._waypoints:
            self._active_wp = self._waypoints.popleft()
            self.get_logger().info(
                f'Navigating to waypoint {self._active_wp.seq}')
        else:
            self._active_wp = None
            self.get_logger().info('Mission complete')
            self._publish_halt()

    def _publish_cmd(self, throttle: int, steering: int):
        msg          = RCInput()
        channels     = [0] * 9          # 0 = "don't update" in rp2040_bridge
        channels[0]  = throttle         # PPM CH1 — only control throttle and steering
        channels[1]  = steering         # PPM CH2 — servo channels 4-7 managed separately
        msg.channels = channels
        msg.mode     = 'AUTONOMOUS'
        msg.stamp    = self.get_clock().now().to_msg()
        self.cmd_pub.publish(msg)

    def _publish_halt(self):
        self._publish_cmd(PPM_CENTER, PPM_CENTER)


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
