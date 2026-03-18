"""
agri_rover_navigator — navigator node

Pure-pursuit path follower for ground rover navigation.

Strategy
--------
Rather than seeking each waypoint position directly, the rover follows the
*path segment* between waypoints using a lookahead point:

  1. A lookahead point is projected `lookahead_distance` metres ahead of the
     rover's position along the current path segment.  The rover steers toward
     that point, not the waypoint itself — giving smooth, continuous curves.

  2. A waypoint is considered *reached* either by acceptance radius (GPS
     precision) OR by a projection test: when the rover has physically passed
     the waypoint along the path direction (overshoot detection).  This
     prevents the rover from turning 180° to go back to a missed point.

  3. Stop-and-spin (spin in place) is reserved for large initial heading errors
     only (> align_threshold, default 30°).  Normal course corrections use
     proportional steering while moving.

Subscribes:
  ~/fix          (sensor_msgs/NavSatFix)   — current position
  ~/heading      (std_msgs/Float32)        — degrees from north (dual-antenna)
  ~/mode         (std_msgs/String)         — only active in AUTONOMOUS mode
  ~/mission      (MissionWaypoint)         — waypoints arrive sequentially
  ~/servo_state  (RCInput)                 — channels 4-7 from mavlink_bridge DO_SET_SERVO

Publishes:
  ~/cmd_override (RCInput)                 — PPM override to rp2040_bridge (all 8 channels)

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
from std_msgs.msg import Bool, Float32, Int32, String
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
        self.declare_parameter('lookahead_distance',        3.0)
        self.declare_parameter('default_acceptance_radius', 0.3)
        self.declare_parameter('max_speed',                 1.5)
        self.declare_parameter('min_speed',                 0.3)
        self.declare_parameter('max_steering',              0.8)
        self.declare_parameter('control_rate',              25.0)
        self.declare_parameter('gps_timeout',               2.0)
        self.declare_parameter('heading_deadband',          3.0)
        # Stop-and-spin only when heading error exceeds this — keeps the rover
        # moving through normal curves instead of stopping at every bend.
        self.declare_parameter('align_threshold',           30.0)

        self._lookahead      = self.get_parameter('lookahead_distance').value
        self._accept_r       = self.get_parameter('default_acceptance_radius').value
        self._max_speed      = self.get_parameter('max_speed').value
        self._min_speed      = self.get_parameter('min_speed').value
        self._max_steer      = self.get_parameter('max_steering').value
        self._gps_timeout    = self.get_parameter('gps_timeout').value
        self._hdb            = self.get_parameter('heading_deadband').value
        self._align_thresh   = self.get_parameter('align_threshold').value

        # ── State ────────────────────────────────────────────────────────────
        self._fix:          NavSatFix | None = None
        self._fix_time:     float            = 0.0
        self._heading:      float            = 0.0
        self._mode:         str              = 'MANUAL'
        self._armed:        bool             = False
        self._waypoints:    deque[MissionWaypoint] = deque()
        self._paused:       bool             = False
        self._active_wp:    MissionWaypoint | None = None
        # Previous waypoint (or rover position at mission start) — used to
        # define the current path segment for lookahead and overshoot detection.
        self._prev_lat:     float | None     = None
        self._prev_lon:     float | None     = None
        # Hold state — rover waits at a waypoint for hold_secs before advancing.
        self._holding:      bool             = False
        self._hold_end:     float            = 0.0   # monotonic time when hold expires
        self._dt:           float            = 1.0 / self.get_parameter('control_rate').value
        # Servo state (PPM CH5-CH8) updated from servo_state topic; re-published
        # in every cmd_override so the RP2040 always has the current servo values.
        self._servo_ch:     list[int]        = [PPM_CENTER] * 4

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(NavSatFix,       'fix',         self._cb_fix,         10)
        self.create_subscription(Float32,         'heading',     self._cb_heading,      10)
        self.create_subscription(String,          'mode',        self._cb_mode,         10)
        self.create_subscription(Bool,            'armed',       self._cb_armed,        10)
        self.create_subscription(MissionWaypoint, 'mission',     self._cb_mission,      10)
        self.create_subscription(RCInput,         'servo_state', self._cb_servo_state,  10)

        # ── Publishers ───────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(RCInput, 'cmd_override', 10)
        self.wp_pub  = self.create_publisher(Int32,   'wp_active',    10)

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

    def _cb_armed(self, msg: Bool):
        self._armed = msg.data

    def _cb_servo_state(self, msg: RCInput):
        """Update servo state from mavlink_bridge DO_SET_SERVO commands."""
        for i in range(4):
            val = msg.channels[i + 4] if i + 4 < len(msg.channels) else 0
            if val != 0:
                self._servo_ch[i] = val

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

    # ── Path geometry ──────────────────────────────────────────────────────────

    def _lookahead_target(self, rover_lat: float, rover_lon: float,
                          wp: MissionWaypoint) -> tuple[float, float]:
        """
        Return the lookahead point: a position `lookahead_distance` metres
        ahead of the rover's projection onto the current path segment
        (prev_wp → active_wp).

        Falls back to the waypoint itself when:
          - no previous waypoint is known (first segment)
          - the segment is very short
          - the lookahead extends past the end of the segment
        """
        if self._prev_lat is None:
            return wp.latitude, wp.longitude

        # Flat-earth Cartesian (valid for segments < ~500 m)
        mid_lat  = math.radians((rover_lat + self._prev_lat) / 2.0)
        cos_lat  = math.cos(mid_lat) or 1e-9
        m_lat    = 111_320.0
        m_lon    = 111_320.0 * cos_lat

        # Segment vector (metres)
        seg_dlat = (wp.latitude  - self._prev_lat) * m_lat
        seg_dlon = (wp.longitude - self._prev_lon) * m_lon
        seg_len  = math.hypot(seg_dlat, seg_dlon)
        if seg_len < 0.1:
            return wp.latitude, wp.longitude

        # Rover offset from segment start (metres)
        rv_dlat  = (rover_lat  - self._prev_lat) * m_lat
        rv_dlon  = (rover_lon  - self._prev_lon) * m_lon

        # Scalar projection of rover onto segment unit vector
        t_proj = (rv_dlat * seg_dlat + rv_dlon * seg_dlon) / seg_len

        # Lookahead point: clamp so it never falls before the segment start
        t_la = max(t_proj, 0.0) + self._lookahead

        if t_la >= seg_len:
            # Lookahead reaches beyond this waypoint — aim at the waypoint
            return wp.latitude, wp.longitude

        frac   = t_la / seg_len
        la_lat = self._prev_lat + frac * (wp.latitude  - self._prev_lat)
        la_lon = self._prev_lon + frac * (wp.longitude - self._prev_lon)
        return la_lat, la_lon

    def _rover_past_waypoint(self, rover_lat: float, rover_lon: float,
                              wp: MissionWaypoint) -> bool:
        """
        Return True when the rover has passed the waypoint along the path
        direction (overshoot detection).

        Uses a bearing projection: bearing from wp→rover vs bearing of the
        incoming segment.  If they agree within ±90°, the rover is in the
        'beyond' half-plane.
        """
        if self._prev_lat is None:
            return False
        bearing_path     = bearing_to(self._prev_lat, self._prev_lon,
                                      wp.latitude, wp.longitude)
        bearing_wp_rover = bearing_to(wp.latitude, wp.longitude,
                                      rover_lat, rover_lon)
        angle = ((bearing_wp_rover - bearing_path + 180) % 360) - 180
        return abs(angle) < 90.0

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self):
        if self._mode != 'AUTONOMOUS' or not self._armed or self._paused or self._active_wp is None:
            return

        if self._fix is None or (time.time() - self._fix_time) > self._gps_timeout:
            self.get_logger().warn('GPS stale — halting')
            self._publish_halt()
            return

        # ── Hold at waypoint ─────────────────────────────────────────────────
        if self._holding:
            self._publish_halt()
            if time.monotonic() >= self._hold_end:
                self._holding = False
                self.get_logger().info('Hold complete — advancing')
                self._advance_waypoint()
            return

        wp     = self._active_wp
        rlat   = self._fix.latitude
        rlon   = self._fix.longitude
        dist   = haversine(rlat, rlon, wp.latitude, wp.longitude)
        accept = wp.acceptance_radius if wp.acceptance_radius > 0 else self._accept_r

        # ── Waypoint advance ─────────────────────────────────────────────────
        if dist < accept:
            if wp.hold_secs > 0.0:
                self.get_logger().info(
                    f'Waypoint {wp.seq} reached — holding {wp.hold_secs:.1f} s')
                self._holding  = True
                self._hold_end = time.monotonic() + wp.hold_secs
                self._publish_halt()
            else:
                self.get_logger().info(f'Waypoint {wp.seq} reached ({dist:.2f} m)')
                self._advance_waypoint()
            return

        # Overshoot: rover has passed the waypoint along the path direction.
        # Advance immediately instead of turning 180° to go back.
        if dist < self._lookahead * 2.0 and self._rover_past_waypoint(rlat, rlon, wp):
            self.get_logger().info(
                f'Waypoint {wp.seq} passed ({dist:.2f} m overshoot) — advancing')
            self._advance_waypoint()
            return

        # ── Steering toward lookahead target ─────────────────────────────────
        tgt_lat, tgt_lon = self._lookahead_target(rlat, rlon, wp)
        target_bearing   = bearing_to(rlat, rlon, tgt_lat, tgt_lon)
        heading_err      = ((target_bearing - self._heading + 180) % 360) - 180

        if abs(heading_err) > self._align_thresh:
            # Large error (e.g. mission start facing wrong way) — spin in place
            spin_dir     = math.copysign(1.0, heading_err)
            steer_ppm    = int(PPM_CENTER - spin_dir * self._max_steer * 500)
            throttle_ppm = PPM_CENTER
        else:
            # Normal path following — proportional steering while moving
            steer_frac   = max(-self._max_steer,
                               min(self._max_steer, heading_err / 45.0))
            steer_ppm    = int(PPM_CENTER - steer_frac * 500)
            speed_frac   = 1.0 - 0.5 * abs(steer_frac)
            target_spd   = wp.speed if wp.speed > 0 else self._max_speed
            speed_frac  *= target_spd / self._max_speed
            speed_frac   = max(self._min_speed / self._max_speed, speed_frac)
            throttle_ppm = int(PPM_CENTER + speed_frac * 500)

        self._publish_cmd(throttle_ppm, steer_ppm)

    def _advance_waypoint(self):
        # Record current waypoint as the new segment start before advancing
        if self._active_wp is not None:
            self._prev_lat = self._active_wp.latitude
            self._prev_lon = self._active_wp.longitude
        elif self._fix is not None:
            # First waypoint: use current rover position as segment start
            self._prev_lat = self._fix.latitude
            self._prev_lon = self._fix.longitude

        if self._waypoints:
            self._active_wp = self._waypoints.popleft()
            self.get_logger().info(
                f'Navigating to waypoint {self._active_wp.seq}')
            m = Int32(); m.data = self._active_wp.seq
            self.wp_pub.publish(m)
        else:
            self._active_wp = None
            self._prev_lat  = None
            self._prev_lon  = None
            self._holding   = False
            self.get_logger().info('Mission complete')
            m = Int32(); m.data = -1
            self.wp_pub.publish(m)
            self._publish_halt()

    def _publish_cmd(self, throttle: int, steering: int):
        msg          = RCInput()
        channels     = [PPM_CENTER] * 9
        channels[0]  = throttle
        channels[1]  = steering
        channels[4]  = self._servo_ch[0]   # PPM CH5 — servo 5
        channels[5]  = self._servo_ch[1]   # PPM CH6 — servo 6
        channels[6]  = self._servo_ch[2]   # PPM CH7 — servo 7
        channels[7]  = self._servo_ch[3]   # PPM CH8 — servo 8
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
