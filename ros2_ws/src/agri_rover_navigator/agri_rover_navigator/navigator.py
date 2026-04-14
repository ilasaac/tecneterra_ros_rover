"""
agri_rover_navigator — navigator node (full-path Stanley + obstacle avoidance)

Strategy
--------
The rover follows the COMPLETE uploaded path using a full-path Stanley lateral
controller:

  1. At every control tick the rover's centre position is projected onto the
     entire remaining path (not just the current segment) to find:
       - s_nearest  : arc-length progress along the path
       - best_seg   : which path segment is closest

  2. The lookahead point is taken at arc-length (s_nearest + lookahead_distance)
     on the path.  When the lookahead extends past a segment boundary it
     naturally follows the curve into the next segment, producing smooth steering
     through curves without the "snap-to-waypoint" artefact of single-segment
     tracking.

  3. Stanley steering: δ = θ_e + arctan(k·e_cte / (v + ε))
     e_cte is computed from the FRONT antenna to best_seg (front-axle reference
     as per the original Stanley paper).  This eliminates the rear-swing
     oscillation caused by measuring CTE at the rear antenna.

  4. Rover centre = midpoint(fix, fix_front) is used for arc-length progress,
     acceptance radius, and lookahead projection.

  5. A waypoint is considered reached when the rover centre is within
     acceptance_radius of it (direct distance), OR when arc-length progress
     has passed that waypoint's arc-length by more than acceptance_radius
     (overshoot / corner-cut detection).

  6. Stop-and-spin is reserved for large initial heading errors (> align_threshold).

  7. Cross-track error (XTE) is published on the `xte` topic at every tick.

Obstacle avoidance
------------------
When a `mission_fence` JSON message arrives (published by mavlink_bridge when
the full mission has been received), the navigator:
  1. Parses the fence polygons.
  2. Expands each polygon by a uniform edge offset of rover_width_m/2 + obstacle_clearance_m.
  3. Rebuilds `_path` from `_path_original` inserting bypass waypoints around
     any expanded polygon that a path segment intersects.
  4. Bypass direction (CW vs CCW around polygon boundary) is chosen to minimise
     total detour distance.
  5. Rerouting is pre-mission only (static obstacles).  Mid-mission rerouting
     is not supported.

Subscribes:
  ~/fix          (sensor_msgs/NavSatFix)   — rear antenna position
  ~/fix_front    (sensor_msgs/NavSatFix)   — front antenna position
  ~/heading      (std_msgs/Float32)        — degrees from north (dual-antenna baseline)
  ~/mode         (std_msgs/String)         — only active in AUTONOMOUS mode
  ~/mission      (MissionWaypoint)         — waypoints arrive sequentially
  ~/servo_state  (RCInput)                 — channels 4-7 from mavlink_bridge DO_SET_SERVO
  ~/mission_clear (std_msgs/Bool)          — clear path on empty-mission upload
  ~/mission_fence (std_msgs/String)        — JSON fence polygons from mavlink_bridge

Publishes:
  ~/cmd_override (RCInput)                 — PPM override to rp2040_bridge
  ~/wp_active    (std_msgs/Int32)          — seq of last reached waypoint (-1=complete)
  ~/xte          (std_msgs/Float32)        — absolute cross-track error in metres

Services:
  ~/pause_mission  (std_srvs/Trigger)
  ~/resume_mission (std_srvs/Trigger)
"""

from __future__ import annotations

import csv
import json
import math
import os
import time

try:
    from scipy.optimize import minimize as _scipy_minimize
    _SCIPY_OK = True
except ImportError:
    _scipy_minimize = None
    _SCIPY_OK = False

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Bool, Float32, Int32, String
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger

from agri_rover_interfaces.msg import RCInput, MissionWaypoint, SensorData, RoverStatus

PPM_CENTER = 1500
PPM_MIN    = 1000
PPM_MAX    = 2000
EARTH_R    = 6_371_000.0   # metres

# Tunable parameters exposed for live GQC editing via MAVLink PARAM_SET.
# Format: ros2_param_name → (instance_attr, min_value, max_value)
_PARAM_META: dict = {
    'max_speed':                 ('_max_speed',            0.1,  3.0),
    'min_speed':                 ('_min_speed',            0.1,  2.0),
    'lookahead_distance':        ('_lookahead',            0.5, 10.0),
    'stanley_k':                 ('_stanley_k',            0.0,  5.0),
    'stanley_cte_scale_m':       ('_stanley_cte_scale',   0.1,  5.0),
    'stanley_cte_alarm_m':       ('_stanley_cte_alarm',   0.5, 10.0),
    'align_threshold':           ('_align_thresh',        10.0, 90.0),
    'default_acceptance_radius': ('_accept_r',            0.05,  2.0),
    'min_throttle_ppm':          ('_min_throttle_ppm',   1500, 1900),
    'min_steer_ppm_delta':   ('_min_steer_delta',    0,  400),
    'steer_coast_angle':     ('_steer_coast',        0.0, 60.0),
    'gps_accuracy_alarm_mm': ('_gps_acc_alarm',     -1.0, 2000.0),
}


def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Distance in metres between two GPS coordinates."""
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2
         + math.cos(math.radians(lat1))
         * math.cos(math.radians(lat2))
         * math.sin(dlon / 2) ** 2)
    return 2 * EARTH_R * math.asin(math.sqrt(max(0.0, a)))


def bearing_to(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Bearing in degrees (0–360) from point 1 to point 2."""
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(math.radians(lat2))
    y = (math.cos(math.radians(lat1)) * math.sin(math.radians(lat2))
         - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(dlon))
    return (math.degrees(math.atan2(x, y)) + 360) % 360


# ── Proximity-safety geometry helpers ─────────────────────────────────────────

def _prox_corners(rear_lat: float, rear_lon: float, heading_deg: float,
                  baseline_m: float, front_ov: float, rear_ov: float, half_w: float,
                  ref_lat: float, ref_lon: float) -> list[tuple[float, float]]:
    """
    Return the 4 bounding-box corners of a rover in flat-earth XY (X=East, Y=North).

    rear_lat/lon  — rear (primary) GPS antenna position
    heading_deg   — rover heading (0=North, 90=East)
    baseline_m    — distance between rear and front antennas (≈ 1.0 m)
    front_ov      — how far the rover body extends FORWARD of the front antenna
    rear_ov       — how far the rover body extends REARWARD of the rear antenna
    half_w        — half the rover physical width
    ref_lat/lon   — flat-earth origin (shared between the two rovers)

    Returns [FL, FR, RL, RR] in counter-clockwise winding order.
    """
    cos_ref = math.cos(math.radians(ref_lat)) or 1e-9
    rx = (rear_lon - ref_lon) * 111_320.0 * cos_ref   # East metres from ref
    ry = (rear_lat - ref_lat) * 111_320.0              # North metres from ref
    h  = math.radians(heading_deg)
    fw = (math.sin(h),  math.cos(h))   # forward unit vector (E, N)
    rt = (math.cos(h), -math.sin(h))   # right   unit vector (E, N)
    # Forward end (baseline + front overhang ahead of rear antenna)
    fwd_off = baseline_m + front_ov
    return [
        (rx + fwd_off * fw[0] +  half_w * rt[0], ry + fwd_off * fw[1] +  half_w * rt[1]),  # FL
        (rx + fwd_off * fw[0] -  half_w * rt[0], ry + fwd_off * fw[1] -  half_w * rt[1]),  # FR
        (rx - rear_ov * fw[0] -  half_w * rt[0], ry - rear_ov * fw[1] -  half_w * rt[1]),  # RR
        (rx - rear_ov * fw[0] +  half_w * rt[0], ry - rear_ov * fw[1] +  half_w * rt[1]),  # RL
    ]


def _seg_dist_sq(px: float, py: float,
                 ax: float, ay: float, bx: float, by: float) -> float:
    """Squared distance from point P to line segment A–B."""
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx*dx + dy*dy + 1e-12)))
    return (px - ax - t*dx)**2 + (py - ay - t*dy)**2


def _rects_clearance(ca: list[tuple[float, float]],
                     cb: list[tuple[float, float]]) -> float:
    """
    Minimum clearance (metres) between two convex quadrilaterals.
    Returns 0.0 if they overlap.  Corners must be in consistent winding order.
    """
    def _inside(px: float, py: float, corners: list) -> bool:
        """True if point is inside a convex polygon (CCW winding)."""
        n, sign = len(corners), None
        for i in range(n):
            ax, ay = corners[i]
            bx, by = corners[(i + 1) % n]
            cross = (bx - ax) * (py - ay) - (by - ay) * (px - ax)
            if abs(cross) < 1e-9:
                continue
            s = cross > 0
            if sign is None:
                sign = s
            elif sign != s:
                return False
        return True

    # Overlap → clearance is 0
    for x, y in ca:
        if _inside(x, y, cb):
            return 0.0
    for x, y in cb:
        if _inside(x, y, ca):
            return 0.0

    # Minimum corner-to-edge distance
    min_dsq = float('inf')
    n = len(cb)
    for px, py in ca:
        for i in range(n):
            min_dsq = min(min_dsq, _seg_dist_sq(px, py, *cb[i], *cb[(i + 1) % n]))
    n = len(ca)
    for px, py in cb:
        for i in range(n):
            min_dsq = min(min_dsq, _seg_dist_sq(px, py, *ca[i], *ca[(i + 1) % n]))
    return math.sqrt(min_dsq)


class NavigatorNode(Node):

    def __init__(self):
        super().__init__('navigator')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('lookahead_distance',        3.0)
        self.declare_parameter('default_acceptance_radius', 0.3)
        self.declare_parameter('max_speed',                 1.5)
        self.declare_parameter('min_speed',                 0.3)
        self.declare_parameter('max_steering',              1.0)
        self.declare_parameter('control_rate',              25.0)
        self.declare_parameter('gps_timeout',               2.0)
        self.declare_parameter('heading_deadband',          3.0)
        self.declare_parameter('align_threshold',           30.0)
        self.declare_parameter('stanley_k',                 1.0)
        self.declare_parameter('stanley_softening',         0.3)
        self.declare_parameter('stanley_cte_scale_m',       1.0)   # CTE (m) at which speed halves
        self.declare_parameter('stanley_cte_alarm_m',       1.0)   # CTE (m) → disarm + halt
        # Corridor arc turn — post-turn exit speed (m/s)
        self.declare_parameter('post_turn_speed',           0.5)
        # Rover geometry for the proximity check
        self.declare_parameter('rover_width_m',              1.4)
        # Minimum corridor arc turn radius (metres). Missions with tighter arcs
        # are rejected on load.
        self.declare_parameter('min_turn_radius_m',          2.0)
        # Motor stiction floors (apply to all Stanley outputs):
        #   min_throttle_ppm    — hard PPM floor for any forward motion output.
        #                         Prevents motor stall when CTE scaling reduces throttle
        #                         below the stiction threshold.
        #   min_steer_ppm_delta — minimum |steer_ppm − 1500| during active turns, pivot
        #                         spins, align-spins, and normal curve driving. Ensures
        #                         motors overcome stiction.
        #   steer_coast_angle   — |heading_error| (degrees) below which the steer floor
        #                         is NOT applied. Allows proportional control to decay
        #                         to zero near the target. Set close to heading_deadband
        #                         (e.g. 5°) so the floor applies until the deadband exit.
        self.declare_parameter('min_throttle_ppm',        1550)
        self.declare_parameter('min_steer_ppm_delta',       50)
        self.declare_parameter('steer_coast_angle',        5.0)

        # ── GPS accuracy alarm ─────────────────────────────────────────────────
        # gps_accuracy_alarm_mm: hAcc from UBX NAV-PVT above which the rover halts
        #   and an alert is issued.  Only active when hacc > 0 (UBX data received).
        #   -1 disables the alarm.  Typical: 200 mm (0.20 m) for RTK-class accuracy.
        self.declare_parameter('gps_accuracy_alarm_mm', 200.0)

        # ── Inter-rover proximity safety ──────────────────────────────────────
        # peer_rover_ns: absolute ROS2 namespace of the other rover (e.g. '/rv2').
        #   Leave empty '' to disable proximity safety (single-rover mode).
        # rover_front_corner_dist_m: straight-line distance from the FRONT antenna
        #   to each front corner of the rover body (metres).  With a 1m-wide rover
        #   and 0.6m corner distance, the front overhang = sqrt(0.6²−0.5²) ≈ 0.33m.
        # rover_rear_corner_dist_m: same for the REAR antenna to rear corners.
        #   With 1.0m corner distance, rear overhang = sqrt(1.0²−0.5²) ≈ 0.87m.
        # rover_half_width_m: half the physical width of the rover body (metres).
        # proximity_slow_m:  inter-rover body clearance (m) below which SLAVE halves speed.
        # proximity_halt_m:  clearance below which SLAVE halts.
        # proximity_estop_m: clearance below which BOTH rovers halt and disarm.
        self.declare_parameter('peer_rover_ns',             '')
        self.declare_parameter('rover_front_corner_dist_m', 0.6)
        self.declare_parameter('rover_rear_corner_dist_m',  1.0)
        self.declare_parameter('rover_half_width_m',        0.5)
        self.declare_parameter('proximity_slow_m',          1.5)
        self.declare_parameter('proximity_halt_m',          1.0)
        self.declare_parameter('proximity_estop_m',         0.5)

        self._lookahead           = self.get_parameter('lookahead_distance').value
        self._accept_r            = self.get_parameter('default_acceptance_radius').value
        self._max_speed           = self.get_parameter('max_speed').value
        self._min_speed           = self.get_parameter('min_speed').value
        self._max_steer           = self.get_parameter('max_steering').value
        self._gps_timeout         = self.get_parameter('gps_timeout').value
        self._hdb                 = self.get_parameter('heading_deadband').value
        self._align_thresh        = self.get_parameter('align_threshold').value
        self._stanley_k           = self.get_parameter('stanley_k').value
        self._stanley_softening   = self.get_parameter('stanley_softening').value
        self._stanley_cte_scale   = self.get_parameter('stanley_cte_scale_m').value
        self._stanley_cte_alarm   = self.get_parameter('stanley_cte_alarm_m').value
        self._post_turn_speed     = self.get_parameter('post_turn_speed').value
        self._min_turn_radius     = self.get_parameter('min_turn_radius_m').value
        self._min_throttle_ppm    = self.get_parameter('min_throttle_ppm').value
        self._min_steer_delta     = self.get_parameter('min_steer_ppm_delta').value
        self._steer_coast         = self.get_parameter('steer_coast_angle').value
        self._gps_acc_alarm       = self.get_parameter('gps_accuracy_alarm_mm').value

        # Proximity safety — derived geometry
        _half_w  = self.get_parameter('rover_half_width_m').value
        _fcd     = self.get_parameter('rover_front_corner_dist_m').value
        _rcd     = self.get_parameter('rover_rear_corner_dist_m').value
        self._prox_half_w   = _half_w
        self._prox_front_ov = math.sqrt(max(0.0, _fcd**2 - _half_w**2))
        self._prox_rear_ov  = math.sqrt(max(0.0, _rcd**2 - _half_w**2))
        self._prox_slow_m   = self.get_parameter('proximity_slow_m').value
        self._prox_halt_m   = self.get_parameter('proximity_halt_m').value
        self._prox_estop_m  = self.get_parameter('proximity_estop_m').value

        self.get_logger().info(
            f'Rover bounding box: front_ov={self._prox_front_ov:.3f} m, '
            f'rear_ov={self._prox_rear_ov:.3f} m, half_w={self._prox_half_w:.3f} m')

        # ── State ────────────────────────────────────────────────────────────
        self._fix:       NavSatFix | None = None
        self._fix_front: NavSatFix | None = None   # front (secondary) antenna
        self._fix_time:  float            = 0.0
        self._heading:   float            = 0.0
        self._mode:      str              = 'MANUAL'
        self._armed:     bool             = False
        self._paused:    bool             = False
        self._hacc_mm:   float            = -1.0   # UBX NAV-PVT hAcc in mm; -1 = not received
        self._hacc_time: float           = 0.0    # time.time() of last hAcc message

        # Inter-rover proximity state
        self._peer_fix:       NavSatFix | None = None
        self._peer_heading:   float            = 0.0
        self._peer_fix_time:  float            = 0.0
        self._prox_level:     str              = 'ok'   # 'ok'|'slow'|'halt'|'estop'

        # Full-path state — replaces single-segment prev/active_wp/deque.
        # _path[i]  : i-th waypoint in mission order (may include bypass waypoints)
        # _path_s[i]: cumulative arc-length from path origin to _path[i]
        #             (origin = rover centre at mission start, or _path[0] if GPS unavailable)
        # _path_idx : index of the next waypoint not yet reached
        self._path:            list[MissionWaypoint] = []
        self._path_s:          list[float]           = []
        self._path_origin_lat: float | None          = None
        self._path_origin_lon: float | None          = None
        self._path_idx:        int                   = 0
        self._log_tick:        int                   = 0   # for throttling periodic logs

        # ── Diagnostic CSV logger ─────────────────────────────────────────────
        self.declare_parameter('enable_diag_log', False)
        self.declare_parameter('diag_log_path',   '/tmp/navigator_diag.csv')
        self._diag_enabled = self.get_parameter('enable_diag_log').value
        self._diag_file   = None
        self._diag_writer = None
        self._run_dir     = None   # per-run directory under /tmp/rover_runs/
        self._raw_corridor_json = None  # raw corridor JSON for saving
        if self._diag_enabled:
            self._open_diag_log(self.get_parameter('diag_log_path').value)

        # Clean copy of the received corridor-derived path (unchanged after load).
        self._path_original:     list[MissionWaypoint]             = []

        # Corridor mode state (the only mode). _corridor_mode flag kept so
        # callers can still check it — always True once a mission loads.
        self._corridor_mode:    bool        = False
        self._corridor_widths:  list[float] = []    # half-width per path point
        self._corridor_total_s: float       = 0.0
        self._corridor_entered: bool        = False

        # Spin-bearing freeze for large heading errors at mission start.
        self._spin_target_brg:  float | None = None

        self._dt = 1.0 / self.get_parameter('control_rate').value

        # Servo state (PPM CH5-CH8) re-published in every cmd_override.
        self._servo_ch: list[int] = [PPM_CENTER] * 4
        self._servo_ch_logged: list[int] = [PPM_CENTER] * 4  # last logged state

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(NavSatFix,       'fix',           self._cb_fix,          10)
        self.create_subscription(NavSatFix,       'fix_front',     self._cb_fix_front,    10)
        self.create_subscription(Float32,         'heading',       self._cb_heading,      10)
        self.create_subscription(String,          'mode',          self._cb_mode,         10)
        self.create_subscription(Bool,            'armed',         self._cb_armed,        10)
        self.create_subscription(RCInput,         'servo_state',   self._cb_servo_state,  10)
        self.create_subscription(Bool,            'mission_clear', self._cb_mission_clear, 10)
        self.create_subscription(String,          'corridor_mission', self._cb_corridor_mission, 10)
        self.create_subscription(Float32,         'hacc',          self._cb_hacc,          10)
        self.create_subscription(SensorData,      'sensors',       self._cb_sensors,       10)
        self.create_subscription(RoverStatus,     'status',        self._cb_status,        10)

        # ── Inter-rover proximity subscriptions (cross-namespace, absolute paths) ──
        peer_ns = self.get_parameter('peer_rover_ns').value
        self._peer_ns = peer_ns
        if peer_ns:
            self.create_subscription(NavSatFix, f'{peer_ns}/fix',     self._cb_peer_fix,     10)
            self.create_subscription(Float32,   f'{peer_ns}/heading', self._cb_peer_heading, 10)
            self._peer_estop_mode_pub = self.create_publisher(String, f'{peer_ns}/mode', 10)
            self.get_logger().info(f'Proximity safety enabled: peer={peer_ns}')
        else:
            self._peer_estop_mode_pub = None
            self._peer_ns = ''

        # ── Publishers ───────────────────────────────────────────────────────
        self.cmd_pub          = self.create_publisher(RCInput,  'cmd_override',    10)
        self.wp_pub           = self.create_publisher(Int32,    'wp_active',       10)
        self.xte_pub          = self.create_publisher(Float32,  'xte',             10)
        self.rerouted_pub       = self.create_publisher(String,   'rerouted_path',   10)
        self.path_version_pub   = self.create_publisher(Int32,    'path_version',    10)
        self._path_version      = 0
        self.nav_status_pub     = self.create_publisher(String,   'nav_status',      10)
        self.center_pos_pub     = self.create_publisher(NavSatFix, 'center_pos',     10)
        self.armed_pub          = self.create_publisher(Bool,     'armed',           10)
        self._last_nav_status   = ''
        self.create_timer(1.0, self._publish_nav_status)
        self.nav_params_pub     = self.create_publisher(String,   'nav_params',      10)
        self.create_subscription(String, 'nav_param_set', self._cb_nav_param_set, 10)
        self.add_on_set_parameters_callback(self._on_set_parameters_callback)
        # Publish current params periodically so mavlink_bridge always has fresh values.
        # Also fires immediately so the first subscriber (after any node restart) gets them.
        self.create_timer(5.0, self._publish_nav_params)

        # ── Services ─────────────────────────────────────────────────────────
        self.create_service(Trigger, 'pause_mission',  self._svc_pause)
        self.create_service(Trigger, 'resume_mission', self._svc_resume)

        # ── Control loop ─────────────────────────────────────────────────────
        rate = self.get_parameter('control_rate').value
        self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info(
            'Navigator ready — Stanley + full-path tracking + obstacle avoidance')

    # ── Live parameter tuning ─────────────────────────────────────────────────

    def _publish_nav_status(self):
        """Publish navigator status: NA (no mission), MSL (mission loaded), ARM (armed).
        Also re-publishes rerouted_path every 5s so late rosbridge subscribers get it."""
        if self._armed:
            status = 'ARM'
        elif self._path:
            status = 'MSL'
        else:
            status = 'NA'
        msg = String()
        msg.data = status
        self.nav_status_pub.publish(msg)
        if status != self._last_nav_status:
            self.get_logger().info(f'nav_status: {self._last_nav_status} → {status}')
            self._last_nav_status = status
        # Re-publish path every 5s for late-joining rosbridge clients
        self._status_tick = getattr(self, '_status_tick', 0) + 1
        if self._path and self._status_tick % 5 == 0:
            self._publish_full_path()

    def _publish_nav_params(self):
        """Publish all exposed parameters as a JSON dict so mavlink_bridge can respond
        to PARAM_REQUEST_LIST.  Called every 5 s and on every param change."""
        data = {k: float(getattr(self, meta[0])) for k, meta in _PARAM_META.items()}
        msg = String()
        msg.data = json.dumps(data, separators=(',', ':'))
        self.nav_params_pub.publish(msg)

    def _cb_nav_param_set(self, msg: String):
        """Apply a live parameter change forwarded by mavlink_bridge from GQC PARAM_SET."""
        try:
            req   = json.loads(msg.data)
            name  = req['name']
            value = float(req['value'])
        except Exception as e:
            self.get_logger().warn(f'nav_param_set parse error: {e}')
            return
        meta = _PARAM_META.get(name)
        if meta is None:
            self.get_logger().warn(f'nav_param_set: unknown param "{name}"')
            return
        attr, lo, hi = meta
        clamped = max(lo, min(hi, value))
        if abs(clamped - value) > 1e-6:
            self.get_logger().warn(
                f'nav_param_set: {name}={value} out of range [{lo}, {hi}] — clamped to {clamped}')
        setattr(self, attr, type(getattr(self, attr))(clamped))
        self.get_logger().info(f'nav_param_set: {name} = {getattr(self, attr)}')
        self._publish_nav_params()

    def _on_set_parameters_callback(self, params) -> SetParametersResult:
        """Handle ROS2 CLI parameter changes (ros2 param set /rv1/navigator <name> <val>)."""
        changed = False
        for p in params:
            meta = _PARAM_META.get(p.name)
            if meta is None:
                continue
            attr, lo, hi = meta
            clamped = max(lo, min(hi, float(p.value)))
            setattr(self, attr, type(getattr(self, attr))(clamped))
            self.get_logger().info(f'ROS2 param set: {p.name} = {getattr(self, attr)}')
            changed = True
        if changed:
            self._publish_nav_params()
        return SetParametersResult(successful=True)

    # ── Sensor callbacks ──────────────────────────────────────────────────────

    def _cb_fix(self, msg: NavSatFix):
        self._fix      = msg
        self._fix_time = time.time()
        self._publish_center_pos()

    def _cb_fix_front(self, msg: NavSatFix):
        self._fix_front = msg

    def _publish_center_pos(self):
        """Publish center position (midpoint of rear+front antenna) for GQC map."""
        if self._fix is None:
            return
        clat, clon = self._center_pos()
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps'
        msg.status = self._fix.status
        msg.latitude = clat
        msg.longitude = clon
        msg.altitude = self._fix.altitude
        msg.position_covariance_type = self._fix.position_covariance_type
        self.center_pos_pub.publish(msg)

    def _center_pos(self) -> tuple[float, float]:
        """Midpoint between rear and front antenna — best estimate of rover centre."""
        f  = self._fix
        ff = self._fix_front
        if ff is None or ff.latitude == 0.0:
            return f.latitude, f.longitude
        return (f.latitude + ff.latitude) / 2.0, (f.longitude + ff.longitude) / 2.0

    def _front_pos(self) -> tuple[float, float]:
        """Front antenna position — reference point for Stanley CTE."""
        ff = self._fix_front
        if ff is None or ff.latitude == 0.0:
            return self._fix.latitude, self._fix.longitude
        return ff.latitude, ff.longitude

    def _cb_heading(self, msg: Float32):
        self._heading = msg.data

    def _cb_hacc(self, msg: Float32):
        self._hacc_mm = msg.data
        self._hacc_time = time.time()

    def _cb_sensors(self, msg: SensorData):
        # Sensor telemetry not consumed by navigator (corridor-only mode).
        pass

    def _cb_status(self, msg: RoverStatus):
        # Rover status telemetry not consumed by navigator (corridor-only mode).
        pass

    def _cb_mode(self, msg: String):
        self._mode = msg.data

    def _cb_armed(self, msg: Bool):
        self._armed = msg.data

    def _cb_servo_state(self, msg: RCInput):
        changed = False
        for i in range(4):
            val = msg.channels[i + 4] if i + 4 < len(msg.channels) else 0
            if val != 0 and val != self._servo_ch[i]:
                self._servo_ch[i] = val
                changed = True
        if changed:
            self.get_logger().info(f'[SERVO] servo_state applied → _servo_ch={self._servo_ch}')

    # ── Inter-rover proximity callbacks ───────────────────────────────────────

    def _cb_peer_fix(self, msg: NavSatFix):
        self._peer_fix      = msg
        self._peer_fix_time = time.time()

    def _cb_peer_heading(self, msg: Float32):
        self._peer_heading = msg.data

    def _compute_proximity(self):
        """Compute inter-rover bounding-box clearance and update _prox_level."""
        if self._peer_fix is None or self._fix is None:
            self._prox_level = 'ok'
            return
        if time.time() - self._peer_fix_time > self._gps_timeout:
            # Peer GPS stale — treat as safe (avoid false e-stops)
            self._prox_level = 'ok'
            return

        ref_lat, ref_lon = self._fix.latitude, self._fix.longitude

        # Rover antenna baseline — compute from actual GPS if available
        if self._fix_front and self._fix_front.latitude != 0.0:
            my_baseline = haversine(self._fix.latitude, self._fix.longitude,
                                    self._fix_front.latitude, self._fix_front.longitude)
        else:
            my_baseline = 1.0   # default 1 m antenna separation

        my_corners   = _prox_corners(
            self._fix.latitude, self._fix.longitude, self._heading,
            my_baseline, self._prox_front_ov, self._prox_rear_ov, self._prox_half_w,
            ref_lat, ref_lon)
        peer_corners = _prox_corners(
            self._peer_fix.latitude, self._peer_fix.longitude, self._peer_heading,
            1.0,   # assume same hardware; peer publishes its own heading
            self._prox_front_ov, self._prox_rear_ov, self._prox_half_w,
            ref_lat, ref_lon)

        clearance  = _rects_clearance(my_corners, peer_corners)
        prev_level = self._prox_level

        if clearance < self._prox_estop_m:
            new_level = 'estop'
        elif clearance < self._prox_halt_m:
            new_level = 'halt'
        elif clearance < self._prox_slow_m:
            new_level = 'slow'
        else:
            new_level = 'ok'

        if new_level != prev_level:
            self.get_logger().warn(
                f'Proximity: clearance={clearance:.2f} m  level={new_level} '
                f'(slow≤{self._prox_slow_m} halt≤{self._prox_halt_m} '
                f'estop≤{self._prox_estop_m})')

        self._prox_level = new_level

        # E-stop: signal peer to stop (fires only on transition to estop)
        if new_level == 'estop' and prev_level != 'estop':
            self.get_logger().error(
                f'PROXIMITY E-STOP: clearance {clearance:.2f} m < {self._prox_estop_m} m '
                f'— halting and signalling peer {self._peer_ns}')
            if self._peer_estop_mode_pub is not None:
                m = String()
                m.data = 'MANUAL'
                self._peer_estop_mode_pub.publish(m)

    # ── Run directory & diagnostic log management ──────────────────────────────

    def _open_diag_log(self, path: str):
        """Open (or reopen) the diagnostic CSV at the given path."""
        if self._diag_file is not None:
            self._diag_file.flush()
            self._diag_file.close()
        self._diag_file   = open(path, 'w', newline='')
        self._diag_writer = csv.writer(self._diag_file)
        self._diag_writer.writerow([
            't', 'lat', 'lon', 'heading',
            'target_brg', 'hdg_err', 'cte',
            'steer_frac', 'steer_ppm', 'throttle_ppm',
            'speed_tgt', 'dist_to_wp', 'wp_idx', 'algo', 'fix_quality', 'hacc_mm',
        ])
        self.get_logger().info(f'Diagnostic log: {path}')

    def _start_new_run(self):
        """Create a timestamped run directory and rotate the diag CSV into it."""
        ts = time.strftime('%Y%m%d_%H%M%S')
        self._run_dir = f'/tmp/rover_runs/run_{ts}'
        os.makedirs(self._run_dir, exist_ok=True)
        if self._diag_enabled:
            self._open_diag_log(os.path.join(self._run_dir, 'navigator_diag.csv'))
        self.get_logger().info(f'New run directory: {self._run_dir}')

    def _save_run_mission(self):
        """Save mission definition to the current run directory."""
        if not self._run_dir or not self._path_original:
            return
        try:
            path_data = [
                [round(wp.latitude, 7), round(wp.longitude, 7),
                 0,  # bypass flag (always 0 — rerouting removed)
                 round(wp.speed, 2), round(wp.hold_secs, 1)]
                for wp in self._path
            ]
            mission_out = {
                'waypoints': [{'lat': round(wp.latitude, 7),
                               'lon': round(wp.longitude, 7),
                               'speed': round(wp.speed, 2),
                               'hold_secs': round(wp.hold_secs, 1)}
                              for wp in self._path_original],
                'rerouted_path': path_data,
                'corridor_mode': True,
                'algorithm': 'corridor',
            }
            path = os.path.join(self._run_dir, 'mission.json')
            with open(path, 'w') as f:
                json.dump(mission_out, f, separators=(',', ':'))

            # Save raw corridor data (original vertices with turn markers)
            if hasattr(self, '_raw_corridor_json') and self._raw_corridor_json:
                raw_path = os.path.join(self._run_dir, 'original_corridors.json')
                with open(raw_path, 'w') as f:
                    f.write(self._raw_corridor_json)

            # Save optimized path (what the rover actually follows)
            turn_set = getattr(self, '_corridor_turn_indices', set())
            servo_list = getattr(self, '_corridor_servo', [])
            opt_data = [
                {'lat': round(wp.latitude, 7), 'lon': round(wp.longitude, 7),
                 'speed': round(wp.speed, 2),
                 'turn': i in turn_set,
                 'servo': servo_list[i] if i < len(servo_list) else None}
                for i, wp in enumerate(self._path)
            ]
            opt_path = os.path.join(self._run_dir, 'optimized_path.json')
            with open(opt_path, 'w') as f:
                json.dump(opt_data, f, separators=(',', ':'))
        except Exception:
            pass

    # ── Mission callbacks ─────────────────────────────────────────────────────

    def _cb_mission_clear(self, msg: Bool):
        if not msg.data:
            return
        self._path.clear()
        self._path_s.clear()
        self._path_original.clear()
        self._path_idx               = 0
        self._path_origin_lat        = None
        self._path_origin_lon        = None
        self._spin_target_brg        = None
        self._servo_ch               = [1061, 1061, PPM_CENTER, PPM_CENTER]  # CH5,CH6=off CH7,CH8=neutral
        clr_msg = String(); clr_msg.data = '[]'
        self.rerouted_pub.publish(clr_msg)
        self._publish_halt()
        self.get_logger().info('Mission cleared — path + servos reset')

    @staticmethod
    def _svc_pause(self, _, response):
        self._paused     = True
        response.success = True
        response.message = 'Mission paused'
        return response

    def _svc_resume(self, _, response):
        self._paused     = False
        response.success = True
        response.message = 'Mission resumed'
        return response

    # ── Obstacle geometry ─────────────────────────────────────────────────────

    def _cb_corridor_mission(self, msg: String):
        """Parse corridor mission JSON → build polyline path for Stanley CTE following."""
        try:
            from agri_rover_navigator.corridor import (
                corridor_mission_from_json, corridors_to_path,
                auto_split_corridors, optimize_corridor_speeds,
            )

            mission = corridor_mission_from_json(msg.data)

            # Save raw corridor data before any optimization
            self._raw_corridor_json = msg.data

            # Auto-split: if mission has only 1 corridor with many points,
            # it's a raw recording — split at sharp turns into multiple
            # corridors with headland crossings.
            if (len(mission.corridors) == 1
                    and len(mission.corridors[0].centerline) > 5):
                c0 = mission.corridors[0]
                mission = auto_split_corridors(
                    c0.centerline, turn_threshold_deg=70.0,
                    width=c0.width, speeds=c0.speeds,
                    ch5s=c0.ch5, ch6s=c0.ch6, ch7s=c0.ch7, ch8s=c0.ch8)
                self.get_logger().info(
                    f'Auto-split: {len(c0.centerline)} points -> '
                    f'{len(mission.corridors)} corridors')

            path_pts = corridors_to_path(mission, default_speed=self._max_speed,
                                        post_turn_speed=self._post_turn_speed)

            if not path_pts:
                self.get_logger().warn('Corridor mission: empty path')
                return

            # Mission validator: reject arcs tighter than min_turn_radius_m.
            # At each interior point, R = chord/(2·sin(turn_angle/2)).
            worst_r   = float('inf')
            worst_idx = -1
            for i in range(1, len(path_pts) - 1):
                a_lat, a_lon = path_pts[i - 1][0], path_pts[i - 1][1]
                b_lat, b_lon = path_pts[i    ][0], path_pts[i    ][1]
                c_lat, c_lon = path_pts[i + 1][0], path_pts[i + 1][1]
                brg_in  = bearing_to(a_lat, a_lon, b_lat, b_lon)
                brg_out = bearing_to(b_lat, b_lon, c_lat, c_lon)
                turn_deg = abs(((brg_out - brg_in + 180) % 360) - 180)
                if turn_deg < 1e-3:
                    continue
                chord_m = haversine(a_lat, a_lon, c_lat, c_lon)
                r_m = chord_m / (2.0 * math.sin(math.radians(turn_deg / 2.0)))
                if r_m < worst_r:
                    worst_r, worst_idx = r_m, i
            if worst_idx >= 0 and worst_r < self._min_turn_radius:
                self.get_logger().error(
                    f'Mission rejected: arc at path[{worst_idx}] has radius '
                    f'{worst_r:.2f} m < min_turn_radius_m={self._min_turn_radius:.2f} m')
                a = Bool(); a.data = False
                self.armed_pub.publish(a)
                return

            # Turn indices and servo state from corridors_to_path
            # Always include index 0 — rover aligns heading before driving
            self._corridor_turn_indices = {
                i for i, pt in enumerate(path_pts) if pt[4]
            }
            self._corridor_servo = [pt[5] for pt in path_pts]

            # New run — create timestamped directory for diag + mission
            self._start_new_run()
            # Clear any existing mission
            self._path.clear()
            self._path_s.clear()
            self._path_original.clear()
            self._path_idx          = 0
            self._spin_target_brg   = None
            self._corridor_mode     = True
            self._corridor_entered  = False
            self._corridor_widths.clear()

            # Build path from corridor polyline
            for i, pt in enumerate(path_pts):
                lat, lon, speed, width, _is_turn, _srv = pt[:6]
                wp = MissionWaypoint()
                wp.seq               = i
                wp.latitude          = lat
                wp.longitude         = lon
                wp.speed             = speed
                wp.acceptance_radius = 0.0
                wp.hold_secs         = 0.0
                self._path.append(wp)
                self._path_original.append(wp)
                self._corridor_widths.append(width)

            # Set origin to first point
            self._path_origin_lat = self._path[0].latitude
            self._path_origin_lon = self._path[0].longitude
            self._path_s = self._rebuild_path_s(self._path,
                                                 self._path_origin_lat,
                                                 self._path_origin_lon)
            self._corridor_total_s = self._path_s[-1] if self._path_s else 0.0

            # Apply servo state for first path point
            self._apply_corridor_servo(0)

            self.get_logger().info(
                f'Corridor mission loaded: {len(mission.corridors)} corridors, '
                f'{len(self._path)} path pts, {self._corridor_total_s:.1f} m total')

            self._publish_full_path()

        except Exception as e:
            self.get_logger().error(f'Corridor mission parse error: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def _apply_corridor_servo(self, idx: int):
        """Apply per-point servo values from corridor path to _servo_ch."""
        servo_list = getattr(self, '_corridor_servo', [])
        if idx < 0 or idx >= len(servo_list):
            return
        srv = servo_list[idx]
        if srv is None:
            return
        changed = False
        for ch in (5, 6, 7, 8):
            val = srv.get(ch, 0) or srv.get(str(ch), 0)
            if val and val != self._servo_ch[ch - 5]:
                self._servo_ch[ch - 5] = val
                changed = True
        if changed:
            self.get_logger().info(
                f'[SERVO] corridor pt {idx} → _servo_ch={self._servo_ch}')

    def _control_loop_corridor(self):
        """Corridor-following control: pure CTE minimization on the polyline.

        No waypoint advance logic.  Progress is tracked by arc-length.
        The rover reaches the end when s_nearest >= total_s - accept.
        Corridor width enforces a hard CTE boundary.
        """
        if self._fix is None or (time.time() - self._fix_time) > self._gps_timeout:
            self.get_logger().warn('GPS stale — halting')
            self._publish_halt()
            return

        # ── GPS accuracy alarm ─────────────────────────────────────────────────
        if self._gps_acc_alarm > 0:
            if (self._hacc_mm > 0
                    and self._hacc_mm > self._gps_acc_alarm):
                self.get_logger().warn(
                    f'GPS accuracy degraded: hAcc={self._hacc_mm:.0f} mm '
                    f'> alarm={self._gps_acc_alarm:.0f} mm — halting')
                self._publish_halt()
                return
            if (self._hacc_time > 0
                    and (time.time() - self._hacc_time) > self._gps_timeout):
                self.get_logger().warn(
                    f'hAcc stale ({time.time() - self._hacc_time:.1f} s) — halting')
                self._publish_halt()
                return

        # ── Inter-rover proximity safety ──────────────────────────────────────
        if self._peer_ns:
            self._compute_proximity()
            if self._prox_level in ('halt', 'estop'):
                self._publish_halt()
                return

        rlat, rlon = self._center_pos()
        flat, flon = self._front_pos()

        # Project onto path
        s_nearest, best_seg = self._nearest_on_path(rlat, rlon)

        # Mission complete: near the end of the full path
        if s_nearest >= self._corridor_total_s - self._accept_r:
            self.get_logger().info('Corridor mission complete')
            m = Int32(); m.data = -1
            self.wp_pub.publish(m)
            self._publish_halt()
            # Auto-disarm (navigator owns lifecycle — no mavlink_bridge dependency)
            # Mode is NOT published here — it comes from rp2040_bridge (CH9 switch)
            self._armed = False
            a = Bool(); a.data = False
            self.armed_pub.publish(a)
            self._path.clear()
            self._path_s.clear()
            return

        # CTE to current segment — skip zero-length segments (spin-in-place
        # waypoints) which return CTE=0 regardless of rover position.
        seg_idx = min(best_seg, len(self._path) - 1)
        cte = self._cte_to_seg(flat, flon, seg_idx)
        if cte == 0.0 and seg_idx + 1 < len(self._path):
            # Zero-length segment: try the next non-degenerate segment for CTE
            for try_seg in range(seg_idx + 1, min(seg_idx + 10, len(self._path))):
                try_cte = self._cte_to_seg(flat, flon, try_seg)
                if try_cte != 0.0:
                    cte = try_cte
                    seg_idx = try_seg
                    break

        # CTE hard limit — disarm if rover drifts too far from path.
        # Only enforce once the rover has entered the corridor (CTE was within
        # limit at least once).  On mission start the rover may be outside and
        # needs to drive to the path first.
        cte_limit = self._stanley_cte_alarm
        if abs(cte) < cte_limit:
            self._corridor_entered = True
        if self._corridor_entered and abs(cte) >= cte_limit:
            self.get_logger().error(
                f'CTE ALARM: |CTE|={abs(cte):.2f}m >= limit {cte_limit:.1f}m — disarming')
            self._publish_halt()
            m = Int32(); m.data = -3
            self.wp_pub.publish(m)
            self._corridor_entered = False
            return

        # XTE telemetry
        xte_msg = Float32(); xte_msg.data = abs(cte)
        self.xte_pub.publish(xte_msg)

        # Advance _path_idx forward with the rover, clamped at next turn point.
        if seg_idx > self._path_idx:
            limit = len(self._path)
            for ti in sorted(self._corridor_turn_indices):
                if ti > self._path_idx:
                    limit = ti
                    break
            new_idx = min(seg_idx, limit)
            if new_idx > self._path_idx:
                self._apply_corridor_servo(new_idx)
                self._path_idx = new_idx
                wp_msg = Int32(); wp_msg.data = self._path_idx
                self.wp_pub.publish(wp_msg)

        # Find next turn point ahead (corridor boundary)
        turn_s = None
        turn_idx = None
        for ti in sorted(self._corridor_turn_indices):
            if ti < len(self._path_s) and self._path_s[ti] >= s_nearest - self._accept_r:
                turn_s = self._path_s[ti]
                turn_idx = ti
                break

        # Clamp s_nearest to not overshoot past the turn point.
        # _nearest_on_path searches 10 segments ahead and may project onto
        # the next corridor, making s_nearest > turn_s.
        if turn_s is not None and s_nearest > turn_s:
            s_nearest = turn_s

        # Lookahead — clamp to turn point so rover never sees next corridor
        s_limit = turn_s if turn_s is not None else self._corridor_total_s
        s_look = min(s_nearest + self._lookahead, s_limit)
        la_lat, la_lon = self._point_at_s(s_look)

        target_bearing = bearing_to(rlat, rlon, la_lat, la_lon)

        # Advance past a turn point when the rover's projected arc-length passes it.
        # Arc-only corridors: no on-axis spin — Stanley steers through the arc and
        # this just advances _path_idx so the next corridor becomes the active target.
        if turn_idx is not None and s_nearest > turn_s - self._accept_r:
            self._corridor_turn_indices.discard(turn_idx)
            self._path_idx = turn_idx + 1
            self._apply_corridor_servo(self._path_idx)
            self._corridor_entered = False
            self.get_logger().info(
                f'TURN POINT PASSED: turn_idx={turn_idx} new_path_idx={self._path_idx} '
                f'remaining_turns={self._corridor_turn_indices}')

        # Freeze target bearing during align-spin
        if self._spin_target_brg is not None:
            target_bearing = self._spin_target_brg

        heading_err = ((target_bearing - self._heading + 180) % 360) - 180

        # Speed: per-point speed with CTE reduction
        wp = self._path[seg_idx]
        target_spd = wp.speed if wp.speed > 0 else self._max_speed
        cte_factor = max(0.0, 1.0 - abs(cte) / max(self._stanley_cte_scale, 0.01))
        v_mps = max(self._min_speed, target_spd * cte_factor)

        # Decelerate approaching turn point (linear ramp over last 2 × lookahead)
        if turn_s is not None:
            dist_to_turn = turn_s - s_nearest
            approach = max(1.0, self._lookahead * 2.0)
            if dist_to_turn < approach:
                v_mps = max(self._min_speed, v_mps * (dist_to_turn / approach))

        # Large heading error → spin in place
        if abs(heading_err) > self._align_thresh:
            if self._spin_target_brg is None:
                self._spin_target_brg = target_bearing
            steer_frac = max(-self._max_steer, min(self._max_steer, heading_err / 45.0))
            steer_ppm = int(PPM_CENTER - steer_frac * 500)
            if steer_ppm != PPM_CENTER:
                sign = 1 if steer_ppm > PPM_CENTER else -1
                steer_ppm = PPM_CENTER + sign * max(abs(steer_ppm - PPM_CENTER),
                                                     self._min_steer_delta)
            self._publish_cmd(PPM_CENTER, steer_ppm)
            return

        # Release frozen bearing within deadband
        if self._spin_target_brg is not None and abs(heading_err) < self._hdb:
            self._spin_target_brg = None

        # Stanley steering
        # Debug: log post-pivot state for 5 seconds after a pivot
        if hasattr(self, '_pivot_debug_until') and time.time() < self._pivot_debug_until:
            self.get_logger().info(
                f'POST-PIVOT seg={seg_idx} s={s_nearest:.1f} cte={cte:.3f} '
                f'hdg={self._heading:.1f} tgt_brg={target_bearing:.1f} '
                f'herr={heading_err:.1f} path_idx={self._path_idx}')
        throttle_ppm = int(PPM_CENTER + (v_mps / self._max_speed) * 500)
        stanley_ang = heading_err + math.degrees(
            math.atan2(self._stanley_k * cte, max(v_mps, self._stanley_softening)))
        stanley_ang = max(-90.0, min(90.0, stanley_ang))
        steer_frac = max(-self._max_steer, min(self._max_steer, stanley_ang / 45.0))
        steer_ppm = int(PPM_CENTER - steer_frac * 500)

        if steer_ppm != PPM_CENTER and abs(heading_err) > self._steer_coast:
            sign = 1 if steer_ppm > PPM_CENTER else -1
            steer_ppm = PPM_CENTER + sign * max(abs(steer_ppm - PPM_CENTER),
                                                 self._min_steer_delta)

        # Proximity speed reduction
        if self._prox_level == 'slow' and throttle_ppm > PPM_CENTER:
            throttle_ppm = PPM_CENTER + (throttle_ppm - PPM_CENTER) // 2

        # Diag log
        if self._diag_writer is not None:
            sf = (PPM_CENTER - steer_ppm) / 500.0
            self._diag_writer.writerow([
                round(time.time(), 4),
                round(rlat, 8), round(rlon, 8),
                round(self._heading, 2),
                round(target_bearing, 2),
                round(heading_err, 2),
                round(cte, 4),
                round(sf, 4),
                steer_ppm, throttle_ppm,
                round(target_spd, 3),
                round(self._corridor_total_s - s_nearest, 3),  # remaining distance
                seg_idx,
                'corridor',
                self._fix.status.status if self._fix else -1,
                round(self._hacc_mm, 1),
            ])
            self._diag_file.flush()

        self._publish_cmd(throttle_ppm, steer_ppm)

    def _publish_full_path(self):
        """Publish current _path as JSON for GQC mission sync (rerouted_path topic).

        Format: [[lat, lon, bypass, speed, hold_secs], ...]
        Used for the GQC mission overlay and run-folder save.

        Burst guard: skips a publish if the content is identical to the last
        one AND the previous publish was less than 1 s ago. This collapses
        the well-known double-publish during a mission upload (one from
        _cb_corridor_mission, one from _cb_mission_fence -> _check_path_obstacles)
        into a single WebSocket message — important on the SIYI MK32 whose
        WiFi stack can drop the association under bursty rosbridge load.
        Late-joining clients are still served because the 5 s nav_status
        timer republish bypasses the cooldown after 1 s elapses.
        """
        path_data = [
            [round(wp.latitude, 7), round(wp.longitude, 7),
             0, round(wp.speed, 2), round(wp.hold_secs, 1)]
            for wp in self._path
        ]
        json_data = json.dumps(path_data, separators=(',', ':'))
        now = time.time()
        last_hash = getattr(self, '_last_full_path_hash', None)
        last_time = getattr(self, '_last_full_path_time', 0.0)
        new_hash = hash(json_data)
        if new_hash == last_hash and (now - last_time) < 1.0:
            return  # identical content within cooldown — drop the burst dup
        self._last_full_path_hash = new_hash
        self._last_full_path_time = now
        rp_msg = String()
        rp_msg.data = json_data
        self.rerouted_pub.publish(rp_msg)
        self._save_run_mission()

    @staticmethod
    def _clone_wp(self, wp: MissionWaypoint) -> MissionWaypoint:
        c = MissionWaypoint()
        c.seq = wp.seq
        c.latitude = wp.latitude
        c.longitude = wp.longitude
        c.speed = wp.speed
        c.hold_secs = wp.hold_secs
        c.acceptance_radius = wp.acceptance_radius
        return c

    # _reroute_path removed — obstacles now disarm the rover instead of rerouting.

    def _rebuild_path_s(self, wps, origin_lat, origin_lon) -> list:
        """Rebuild cumulative arc-lengths for a waypoint list."""
        s = []
        for k, wp in enumerate(wps):
            if k == 0:
                d = (haversine(origin_lat, origin_lon, wp.latitude, wp.longitude)
                     if origin_lat is not None else 0.0)
            else:
                prev = wps[k - 1]
                d    = s[-1] + haversine(prev.latitude, prev.longitude,
                                         wp.latitude, wp.longitude)
            s.append(d)
        return s

    # ── Full-path geometry ────────────────────────────────────────────────────

    def _turn_angle_at(self, idx: int) -> float:
        """Absolute heading change at _path[idx] (degrees, 0–180). 0 for the last point."""
        if idx >= len(self._path) - 1:
            return 0.0
        hdg_out = bearing_to(self._path[idx].latitude,     self._path[idx].longitude,
                             self._path[idx + 1].latitude, self._path[idx + 1].longitude)
        if idx == 0:
            if self._path_origin_lat is None:
                return 0.0
            hdg_in = bearing_to(self._path_origin_lat,      self._path_origin_lon,
                                self._path[0].latitude,     self._path[0].longitude)
        else:
            hdg_in = bearing_to(self._path[idx - 1].latitude, self._path[idx - 1].longitude,
                                self._path[idx].latitude,     self._path[idx].longitude)
        return abs(((hdg_out - hdg_in + 180) % 360) - 180)

    def _nearest_on_path(self, lat: float, lon: float) -> tuple[float, int]:
        """
        Find the nearest point on the remaining path (segments from _path_idx onward).

        Returns (s_nearest, seg_idx):
          s_nearest — arc-length of the nearest projected point on the path
          seg_idx   — segment index (0 = virtual origin→wp[0], k = wp[k-1]→wp[k])

        The search never looks back past _path_idx, preventing the controller
        from snapping to already-completed segments.
        """
        if not self._path:
            return 0.0, 0

        best_s    = 0.0
        best_seg  = self._path_idx
        best_dist = float('inf')

        # Search ahead limit: 3 segments in corridor mode, 1 in waypoint mode.
        # Clamped to the next turn point so the rover never snaps to the return pass.
        search_ahead = 3 if self._corridor_mode else 1
        search_limit = self._path_idx + search_ahead
        if self._corridor_mode:
            for ti in sorted(self._corridor_turn_indices):
                if ti > self._path_idx:
                    search_limit = min(search_limit, ti)
                    break
        for seg_k in range(self._path_idx, min(search_limit, len(self._path))):
            # Segment endpoints and arc-length bounds
            if seg_k == 0:
                if self._path_origin_lat is None:
                    continue
                a_lat = self._path_origin_lat
                a_lon = self._path_origin_lon
                s_a   = 0.0
            else:
                a_lat = self._path[seg_k - 1].latitude
                a_lon = self._path[seg_k - 1].longitude
                s_a   = self._path_s[seg_k - 1]

            b_lat = self._path[seg_k].latitude
            b_lon = self._path[seg_k].longitude
            s_b   = self._path_s[seg_k]

            # Flat-earth Cartesian (valid for segments < ~500 m)
            mid_lat = math.radians((a_lat + b_lat) / 2)
            cos_lat = math.cos(mid_lat) or 1e-9
            m_lat   = 111_320.0
            m_lon   = 111_320.0 * cos_lat

            seg_dy  = (b_lat - a_lat) * m_lat
            seg_dx  = (b_lon - a_lon) * m_lon
            seg_len = math.hypot(seg_dx, seg_dy)

            if seg_len < 0.01:
                # Degenerate segment — use endpoint distance
                d = haversine(lat, lon, b_lat, b_lon)
                if d < best_dist:
                    best_dist = d
                    best_s    = s_b
                    best_seg  = seg_k
                continue

            rv_dy = (lat - a_lat) * m_lat
            rv_dx = (lon - a_lon) * m_lon

            # Scalar projection of rover onto segment, clamped to [0, 1]
            t = (rv_dx * seg_dx + rv_dy * seg_dy) / (seg_len ** 2)
            t = max(0.0, min(1.0, t))

            # Distance from rover to nearest point on segment
            d = math.hypot(t * seg_dx - rv_dx, t * seg_dy - rv_dy)

            if d < best_dist:
                best_dist = d
                best_s    = s_a + t * (s_b - s_a)
                best_seg  = seg_k

        return best_s, best_seg

    def _cte_to_seg(self, lat: float, lon: float, seg_idx: int) -> float:
        """
        Signed cross-track error of (lat, lon) to segment seg_idx.
        Positive = point is to the LEFT of the segment direction.
        """
        if seg_idx == 0:
            if self._path_origin_lat is None or not self._path:
                return 0.0
            a_lat = self._path_origin_lat
            a_lon = self._path_origin_lon
        else:
            if seg_idx >= len(self._path):
                return 0.0
            a_lat = self._path[seg_idx - 1].latitude
            a_lon = self._path[seg_idx - 1].longitude

        b_lat = self._path[seg_idx].latitude
        b_lon = self._path[seg_idx].longitude

        mid_lat = math.radians((a_lat + b_lat) / 2)
        cos_lat = math.cos(mid_lat) or 1e-9
        m_lat   = 111_320.0
        m_lon   = 111_320.0 * cos_lat

        seg_dy  = (b_lat - a_lat) * m_lat
        seg_dx  = (b_lon - a_lon) * m_lon
        seg_len = math.hypot(seg_dx, seg_dy)
        if seg_len < 0.01:
            # Zero-length segment (spin-in-place waypoint): return distance to the
            # point as unsigned CTE.  Returning 0 hides the rover drifting away.
            return math.hypot((lat - b_lat) * m_lat, (lon - b_lon) * m_lon)

        rv_dy = (lat - a_lat) * m_lat
        rv_dx = (lon - a_lon) * m_lon
        # 2-D cross product gives signed perpendicular distance
        return (seg_dx * rv_dy - seg_dy * rv_dx) / seg_len

    def _point_at_s(self, s_target: float) -> tuple[float, float]:
        """
        Return (lat, lon) at arc-length s_target on the path.
        Clamps to the last waypoint if s_target exceeds total path length.
        """
        if not self._path:
            return 0.0, 0.0

        if s_target >= self._path_s[-1]:
            return self._path[-1].latitude, self._path[-1].longitude

        # Check virtual segment (arc-length 0 → _path_s[0])
        if (self._path_origin_lat is not None
                and self._path_s
                and s_target <= self._path_s[0]):
            s0   = self._path_s[0]
            frac = s_target / s0 if s0 > 0.01 else 0.0
            return (self._path_origin_lat + frac * (self._path[0].latitude  - self._path_origin_lat),
                    self._path_origin_lon + frac * (self._path[0].longitude - self._path_origin_lon))

        # Walk numbered segments (1 … N-1)
        for k in range(1, len(self._path)):
            s_a = self._path_s[k - 1]
            s_b = self._path_s[k]
            if s_target <= s_b:
                seg_len = s_b - s_a
                if seg_len < 0.01:
                    return self._path[k].latitude, self._path[k].longitude
                frac = (s_target - s_a) / seg_len
                return (self._path[k - 1].latitude  + frac * (self._path[k].latitude  - self._path[k - 1].latitude),
                        self._path[k - 1].longitude + frac * (self._path[k].longitude - self._path[k - 1].longitude))

        return self._path[-1].latitude, self._path[-1].longitude

    # -- Control loop (corridor-only) ----------------------------------------

    def _control_loop(self):
        if self._mode != 'AUTONOMOUS' or not self._armed or self._paused:
            return
        if not self._path:
            return
        if self._fix is None or (time.time() - self._fix_time) > self._gps_timeout:
            self.get_logger().warn('GPS stale -- halting')
            self._publish_halt()
            return

        # GPS accuracy alarm (hAcc + staleness)
        if self._gps_acc_alarm > 0:
            if self._hacc_mm > 0 and self._hacc_mm > self._gps_acc_alarm:
                self.get_logger().warn(
                    f'GPS accuracy degraded: hAcc={self._hacc_mm:.0f} mm '
                    f'> alarm={self._gps_acc_alarm:.0f} mm -- halting')
                self._publish_halt()
                return
            if self._hacc_time > 0 and (time.time() - self._hacc_time) > self._gps_timeout:
                self.get_logger().warn('hAcc stale -- halting')
                self._publish_halt()
                return

        # Inter-rover proximity safety
        if self._peer_ns:
            self._compute_proximity()
            if self._prox_level in ('halt', 'estop'):
                self._publish_halt()
                return

        # All missions are corridors
        self._control_loop_corridor()

    # ── Output helpers ────────────────────────────────────────────────────────

    def _publish_cmd(self, throttle: int, steering: int):
        # Apply proximity speed reduction before sending
        if self._prox_level == 'slow' and throttle > PPM_CENTER:
            throttle = PPM_CENTER + (throttle - PPM_CENTER) // 2

        msg          = RCInput()
        channels     = [PPM_CENTER] * 9
        channels[0]  = throttle
        channels[1]  = steering
        channels[2]  = 1939   # PPM CH3 = SBUS CH5 (no inversion)
        channels[3]  = 1061   # PPM CH4 = PPM_INV(SBUS CH6) = 3000-1939
        channels[4]  = self._servo_ch[0]  # PPM CH5 = servo 5
        channels[5]  = self._servo_ch[1]  # PPM CH6 = servo 6
        channels[6]  = self._servo_ch[2]  # PPM CH7 = servo 7 (firmware inverts)
        channels[7]  = self._servo_ch[3]  # PPM CH8 = servo 8 (firmware inverts)
        msg.channels = channels
        msg.mode     = 'AUTONOMOUS'
        msg.stamp    = self.get_clock().now().to_msg()
        self.cmd_pub.publish(msg)
        if self._servo_ch != self._servo_ch_logged:
            self.get_logger().info(
                f'[SERVO] cmd_override publishing servo CH5-8={self._servo_ch}')
            self._servo_ch_logged = list(self._servo_ch)

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
        if node._diag_file is not None:
            node._diag_file.close()
        node.destroy_node()
        rclpy.shutdown()
