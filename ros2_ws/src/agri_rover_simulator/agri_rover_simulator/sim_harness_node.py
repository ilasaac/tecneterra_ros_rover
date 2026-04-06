"""
sim_harness_node — ROS2 simulation harness for on-Jetson SIL testing.

Replaces gps_driver + rp2040_bridge with DiffDriveState physics.
The real navigator runs unchanged, producing the exact same diag output
as a field run.

Lifecycle:
  1. Node starts, publishes static GPS at (0, 0) — waiting for mission.
  2. First MissionWaypoint (seq=0) arrives → captures start position = wp[0].
     Begins publishing GPS at that position.
  3. After arm_delay_s with no new mission items → publishes armed=True,
     mode=AUTONOMOUS.  Physics loop activates.
  4. Navigator drives cmd_override → physics → GPS feedback at 25 Hz.
  5. wp_active == -1 → mission complete.  Publishes MANUAL + disarmed,
     waits shutdown_delay_s, then exits.

Launch:
  ros2 launch agri_rover_bringup sim_harness.launch.py
"""

import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float32, String, Int32
from agri_rover_interfaces.msg import RCInput, MissionWaypoint

from agri_rover_simulator.diff_drive import DiffDriveState


class SimHarnessNode(Node):
    def __init__(self):
        super().__init__('sim_harness')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('physics_rate_hz', 25.0)
        self.declare_parameter('antenna_baseline_m', 1.0)
        self.declare_parameter('arm_delay_s', 1.0)
        self.declare_parameter('shutdown_delay_s', 2.0)
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('max_steering', 0.8)
        self.declare_parameter('wheelbase_m', 0.6)
        # DiffDriveState physics
        self.declare_parameter('track_width_m', 0.9)
        self.declare_parameter('max_wheel_mms', 1650.0)
        self.declare_parameter('steering_scale', 0.37)
        self.declare_parameter('accel_cap_mms_per_tick', 7.0)
        self.declare_parameter('decel_divisor', 1.0)
        self.declare_parameter('angular_diff_limit_mms', 275.0)
        self.declare_parameter('smooth_tick_hz', 100.0)
        self.declare_parameter('rotation_center_offset_m', 0.35)
        self.declare_parameter('steer_lag_s', 0.25)

        self._rate = self.get_parameter('physics_rate_hz').value
        self._baseline = self.get_parameter('antenna_baseline_m').value
        self._arm_delay = self.get_parameter('arm_delay_s').value
        self._shutdown_delay = self.get_parameter('shutdown_delay_s').value
        self._max_speed = self.get_parameter('max_speed').value
        self._max_steering = self.get_parameter('max_steering').value
        self._wheelbase = self.get_parameter('wheelbase_m').value

        ttr_phys = {
            'track_width_m': self.get_parameter('track_width_m').value,
            'max_wheel_mms': self.get_parameter('max_wheel_mms').value,
            'steering_scale': self.get_parameter('steering_scale').value,
            'accel_cap_mms_per_tick': self.get_parameter('accel_cap_mms_per_tick').value,
            'decel_divisor': self.get_parameter('decel_divisor').value,
            'angular_diff_limit_mms': self.get_parameter('angular_diff_limit_mms').value,
            'smooth_tick_hz': self.get_parameter('smooth_tick_hz').value,
            'rotation_center_offset_m': self.get_parameter('rotation_center_offset_m').value,
            'steer_lag_s': self.get_parameter('steer_lag_s').value,
        }

        # ── State ───────────────────────────────────────────────────────────
        self._rover = DiffDriveState(0.0, 0.0, 0.0,
                                     ttr_phys=ttr_phys,
                                     max_steer=self._max_steering)
        self._throttle = 1500
        self._steering = 1500
        self._started = False          # physics running
        self._mission_wps: dict[int, MissionWaypoint] = {}
        self._last_mission_time = 0.0  # monotonic time of last mission item
        self._armed = False
        self._done = False
        self._shutdown_at = 0.0
        self._start_heading: float | None = None

        # ── Publishers ──────────────────────────────────────────────────────
        self.pub_fix = self.create_publisher(NavSatFix, 'fix', 10)
        self.pub_fix_front = self.create_publisher(NavSatFix, 'fix_front', 10)
        self.pub_heading = self.create_publisher(Float32, 'heading', 10)
        self.pub_mode = self.create_publisher(String, 'mode', 10)
        self.pub_armed = self.create_publisher(Bool, 'armed', 10)
        self.pub_servo = self.create_publisher(RCInput, 'servo_state', 10)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(RCInput, 'cmd_override', self._cb_cmd, 10)
        self.create_subscription(Int32, 'wp_active', self._cb_wp_active, 10)
        self.create_subscription(MissionWaypoint, 'mission', self._cb_mission, 10)
        self.create_subscription(String, 'corridor_mission', self._cb_corridor, 10)

        # ── Timer ───────────────────────────────────────────────────────────
        self._dt = 1.0 / self._rate
        self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f'Sim harness ready — waiting for mission upload '
            f'(rate={self._rate} Hz, baseline={self._baseline} m)')

    # ── Callbacks ───────────────────────────────────────────────────────────

    def _cb_cmd(self, msg: RCInput):
        """Navigator sends throttle + steer via cmd_override."""
        if len(msg.channels) >= 2:
            self._throttle = msg.channels[0]
            self._steering = msg.channels[1]

    def _cb_wp_active(self, msg: Int32):
        """Detect mission complete (wp_active == -1)."""
        if msg.data == -1 and not self._done:
            self.get_logger().info('Mission complete (wp_active=-1) — shutting down')
            self._done = True
            self._started = False
            # Publish disarmed + manual
            self._armed = False
            m = String(); m.data = 'MANUAL'
            self.pub_mode.publish(m)
            b = Bool(); b.data = False
            self.pub_armed.publish(b)
            self._shutdown_at = time.monotonic() + self._shutdown_delay

    def _cb_mission(self, msg: MissionWaypoint):
        """Capture waypoints as they arrive for start position."""
        self._mission_wps[msg.seq] = msg
        self._last_mission_time = time.monotonic()
        if msg.seq == 0:
            self.get_logger().info(
                f'Mission wp[0] received: ({msg.latitude:.6f}, {msg.longitude:.6f})')

    def _cb_corridor(self, msg: String):
        """Corridor mission published — use first vertex for start position."""
        import json
        try:
            data = json.loads(msg.data)
            corridors = data.get('corridors', [])
            if corridors:
                cl = corridors[0].get('centerline', [])
                if cl:
                    lat, lon = cl[0][0], cl[0][1]
                    # Compute heading from first two vertices
                    hdg = 0.0
                    if len(cl) >= 2:
                        dlat = cl[1][0] - cl[0][0]
                        dlon = cl[1][1] - cl[0][1]
                        cos_lat = math.cos(math.radians(lat)) or 1e-9
                        hdg = math.degrees(math.atan2(dlon * cos_lat, dlat)) % 360
                    self._init_position(lat, lon, hdg)
                    self._last_mission_time = time.monotonic()
        except Exception as e:
            self.get_logger().warn(f'Corridor parse for start pos: {e}')

    # ── Position init ───────────────────────────────────────────────────────

    def _init_position(self, lat: float, lon: float, heading_deg: float):
        """Set rover at the given position, ready for sim start."""
        # Place rear antenna so centre lands on (lat, lon)
        half_bm = self._baseline / 2.0
        hdg_rad = math.radians(heading_deg)
        cos_lat = math.cos(math.radians(lat)) or 1e-9
        rear_lat = lat - (half_bm * math.cos(hdg_rad)) / 111320.0
        rear_lon = lon - (half_bm * math.sin(hdg_rad)) / (111320.0 * cos_lat)
        self._rover.lat = rear_lat
        self._rover.lon = rear_lon
        self._rover.heading_rad = hdg_rad
        self._start_heading = heading_deg
        self.get_logger().info(
            f'Position init: ({lat:.6f}, {lon:.6f}) hdg={heading_deg:.1f}°')

    # ── Main loop ───────────────────────────────────────────────────────────

    def _tick(self):
        now = time.monotonic()

        # Shutdown after delay
        if self._done:
            if self._shutdown_at and now >= self._shutdown_at:
                self.get_logger().info('Shutdown delay complete — exiting')
                raise SystemExit(0)
            return

        # Wait for mission, then init position
        if not self._started and not self._armed:
            # Waypoint mission: init from wp[0]
            if self._mission_wps and 0 in self._mission_wps and self._start_heading is None:
                wp0 = self._mission_wps[0]
                hdg = 0.0
                if 1 in self._mission_wps:
                    wp1 = self._mission_wps[1]
                    dlat = wp1.latitude - wp0.latitude
                    dlon = wp1.longitude - wp0.longitude
                    cos_lat = math.cos(math.radians(wp0.latitude)) or 1e-9
                    hdg = math.degrees(math.atan2(dlon * cos_lat, dlat)) % 360
                self._init_position(wp0.latitude, wp0.longitude, hdg)

            # Arm after delay since last mission item (works for both waypoint and corridor)
            if (self._last_mission_time > 0
                    and (now - self._last_mission_time) >= self._arm_delay
                    and self._start_heading is not None):
                self._armed = True
                self._started = True
                m = String(); m.data = 'AUTONOMOUS'
                self.pub_mode.publish(m)
                b = Bool(); b.data = True
                self.pub_armed.publish(b)
                self.get_logger().info(
                    f'Armed + AUTONOMOUS — sim started')

        # Publish GPS even before armed (navigator needs fix for path_origin)
        if self._start_heading is not None:
            if self._started:
                self._rover.update(
                    self._throttle, self._steering,
                    self._max_speed, self._wheelbase, self._dt)
                # Continuously publish mode+armed to override rp2040_bridge
                m = String(); m.data = 'AUTONOMOUS'
                self.pub_mode.publish(m)
                b = Bool(); b.data = True
                self.pub_armed.publish(b)

            self._publish_gps()

    def _publish_gps(self):
        """Publish fix, fix_front, heading from current rover state."""
        stamp = self.get_clock().now().to_msg()

        # Rear antenna (fix)
        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = 'gps'
        fix.status.status = NavSatStatus.STATUS_GBAS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = self._rover.lat
        fix.longitude = self._rover.lon
        fix.altitude = 45.0
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.pub_fix.publish(fix)

        # Front antenna (fix_front)
        f_lat, f_lon = self._rover.secondary_pos(self._baseline)
        fix_f = NavSatFix()
        fix_f.header.stamp = stamp
        fix_f.header.frame_id = 'gps'
        fix_f.status.status = NavSatStatus.STATUS_GBAS_FIX
        fix_f.status.service = NavSatStatus.SERVICE_GPS
        fix_f.latitude = f_lat
        fix_f.longitude = f_lon
        fix_f.altitude = 45.0
        fix_f.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.pub_fix_front.publish(fix_f)

        # Heading
        h = Float32()
        h.data = self._rover.heading_deg
        self.pub_heading.publish(h)


def main(args=None):
    rclpy.init(args=args)
    node = SimHarnessNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
