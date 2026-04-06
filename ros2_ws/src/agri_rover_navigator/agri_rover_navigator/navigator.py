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
    'pivot_threshold':           ('_pivot_threshold',     10.0,180.0),
    'pivot_approach_dist':       ('_pivot_approach_dist',  0.5, 10.0),
    'align_threshold':           ('_align_thresh',        10.0, 90.0),
    'default_acceptance_radius': ('_accept_r',            0.05,  2.0),
    'afs_min_throttle_ppm':      ('_min_throttle_ppm',   1500, 1900),
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


class _PID:
    """Minimal PID controller — matches TTR PIDController.hpp compute() signature."""
    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0):
        self.kp = kp; self.ki = ki; self.kd = kd
        self._integral = 0.0; self._prev_error = 0.0

    def compute(self, setpoint: float, feedback: float) -> float:
        err = setpoint - feedback
        self._integral += err
        deriv = err - self._prev_error
        self._prev_error = err
        return self.kp * err + self.ki * self._integral + self.kd * deriv

    def clear(self):
        self._integral = 0.0; self._prev_error = 0.0


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
        self.declare_parameter('stanley_cte_alarm_m',       1.5)   # CTE (m) → disarm + halt
        # Pivot-turn parameters:
        #   pivot_threshold    — turn angle (degrees) above which an in-place pivot
        #                        is performed instead of curving through the waypoint.
        #   pivot_approach_dist — distance (metres) at which the rover switches from
        #                        path-lookahead to direct-to-waypoint aiming and begins
        #                        slowing to min_speed, ensuring it arrives precisely.
        self.declare_parameter('pivot_threshold',           30.0)
        self.declare_parameter('pivot_approach_dist',        2.0)
        self.declare_parameter('post_turn_speed',           0.5)
        # Obstacle avoidance:
        #   rover_width_m       — physical rover width; half of this is the minimum
        #                         clearance needed to keep the rover body off obstacles.
        #   obstacle_clearance_m — additional safety buffer on top of rover half-width.
        #   effective clearance  = rover_width_m/2 + obstacle_clearance_m
        self.declare_parameter('rover_width_m',              1.4)
        self.declare_parameter('obstacle_clearance_m',       0.5)
        # Bypass arc smoothing: replace sharp bypass corners with arc interpolation.
        # bypass_arc_radius_m  — minimum arc radius at corners (larger = smoother, wider turns)
        # bypass_arc_speed_k   — speed scaling factor: speed = max_speed * (1 - k * angle/180)
        #                        k=0 → no slowdown, k=1 → full slowdown on 180° turns
        self.declare_parameter('bypass_arc_radius_m',        0.0)
        self.declare_parameter('bypass_arc_speed_k',         0.7)
        # Minimum segment length (metres) for pivot-turn detection.  Bearing
        # between two points closer than this is considered unreliable (GPS noise).
        # RTK precision is < 0.02 m so any segment > 0.3 m has a reliable bearing.
        # Set lower than the densest recorded waypoint spacing in your missions.
        self.declare_parameter('min_pivot_segment_m',        0.3)
        # Control algorithm — 'stanley', 'mpc', 'ttr', or 'afs'
        self.declare_parameter('control_algorithm',          'stanley')
        # AFS (Always Forward Strategy) parameters (only used when control_algorithm == 'afs')
        #   afs_cte_scale_m    — CTE (m) at which throttle scales down to min_speed.
        #                        throttle = max(min_speed, target_spd × (1 − |CTE|/scale))
        #   afs_cte_alarm_m    — CTE (m) above which the rover halts and logs an alarm.
        #                        Must be > afs_cte_scale_m or the rover halts before slowing.
        #   afs_approach_dist_m — distance (m) from a turn waypoint at which AFS switches
        #                        from segment-following to direct-to-wp approach steering.
        #                        Approach continues until the rover passes closest approach,
        #                        then the rover stops and spins to the outgoing bearing.
        self.declare_parameter('afs_cte_scale_m',            2.0)
        self.declare_parameter('afs_cte_alarm_m',            3.0)
        self.declare_parameter('afs_approach_dist_m',        5.0)
        #   afs_min_throttle_ppm    — hard PPM floor for any forward motion output.
        #                             Prevents motor stall when CTE scaling or approach
        #                             mode reduces throttle below the stiction threshold.
        #   min_steer_ppm_delta — minimum |steer_ppm − 1500| during ALL active turns
        #                             and curves (pivot spins, align-spins, and normal curve
        #                             driving).  Ensures motors overcome stiction regardless
        #                             of control algorithm (Stanley / MPC / TTR / AFS).
        self.declare_parameter('afs_min_throttle_ppm',    1550)
        self.declare_parameter('min_steer_ppm_delta',   50)
        #   steer_coast_angle   — |heading_error| (degrees) below which the steer floor
        #                             is NOT applied.  Allows proportional control to decay
        #                             to zero near the target.  Set close to heading_deadband
        #                             (e.g. 5°) so the floor applies until the deadband exits
        #                             the spin — prevents motor stall near the target heading.
        self.declare_parameter('steer_coast_angle',    5.0)
        # TTR parameters (only used when control_algorithm == 'ttr')
        # Cascaded dual-PID: HightPid (CTE) feeds into AnglePid (heading+CTE)
        self.declare_parameter('ttr_angle_kp',               3.0)
        self.declare_parameter('ttr_angle_ki',               0.0)
        self.declare_parameter('ttr_angle_kd',               0.1)
        self.declare_parameter('ttr_hight_kp',               1.5)
        self.declare_parameter('ttr_hight_ki',               0.0)
        self.declare_parameter('ttr_hight_kd',               0.0)
        # CTE at which lineSpeedFactor reaches minimum (0.4×); typical 2–5 m
        self.declare_parameter('ttr_max_yaw_distance',       3.0)
        # Distance from waypoint at which deceleration begins
        self.declare_parameter('ttr_target_dece_dis',        4.0)
        # MPC parameters (only used when control_algorithm == 'mpc')
        self.declare_parameter('mpc_horizon',                10)
        self.declare_parameter('mpc_dt',                     0.2)
        self.declare_parameter('mpc_w_cte',                  2.0)
        self.declare_parameter('mpc_w_heading',              1.0)
        self.declare_parameter('mpc_w_steer',                0.1)
        self.declare_parameter('mpc_w_dsteer',               0.05)
        self.declare_parameter('wheelbase_m',                0.6)

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

        # ── Resource management parameters ───────────────────────────────────
        self.declare_parameter('recharge_lat',      0.0)
        self.declare_parameter('recharge_lon',      0.0)
        self.declare_parameter('water_lat',         0.0)
        self.declare_parameter('water_lon',         0.0)
        self.declare_parameter('battery_low_pct',  15.0)
        self.declare_parameter('tank_low_pct',      5.0)
        self.declare_parameter('base_speed',        0.9)
        self.declare_parameter('base_acceptance_m', 1.5)
        self.declare_parameter('test_tank_pct',    -1.0)   # -1 = use real sensor
        self.declare_parameter('test_batt_pct',    -1.0)   # -1 = use real sensor

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
        self._pivot_threshold     = self.get_parameter('pivot_threshold').value
        self._pivot_approach_dist = self.get_parameter('pivot_approach_dist').value
        self._post_turn_speed    = self.get_parameter('post_turn_speed').value
        self._min_pivot_seg       = self.get_parameter('min_pivot_segment_m').value
        rover_width               = self.get_parameter('rover_width_m').value
        self._clearance           = (rover_width / 2.0
                                     + self.get_parameter('obstacle_clearance_m').value)
        self._bypass_arc_radius   = self.get_parameter('bypass_arc_radius_m').value
        self._bypass_arc_speed_k  = self.get_parameter('bypass_arc_speed_k').value
        self._algo             = self.get_parameter('control_algorithm').value
        self._afs_cte_scale       = self.get_parameter('afs_cte_scale_m').value
        self._afs_cte_alarm       = self.get_parameter('afs_cte_alarm_m').value
        self._afs_approach_dist   = self.get_parameter('afs_approach_dist_m').value
        self._min_throttle_ppm    = self.get_parameter('afs_min_throttle_ppm').value
        self._min_steer_delta = self.get_parameter('min_steer_ppm_delta').value
        self._steer_coast     = self.get_parameter('steer_coast_angle').value
        self._gps_acc_alarm   = self.get_parameter('gps_accuracy_alarm_mm').value
        self._ttr_hpid    = _PID(self.get_parameter('ttr_hight_kp').value,
                                  self.get_parameter('ttr_hight_ki').value,
                                  self.get_parameter('ttr_hight_kd').value)
        self._ttr_apid    = _PID(self.get_parameter('ttr_angle_kp').value,
                                  self.get_parameter('ttr_angle_ki').value,
                                  self.get_parameter('ttr_angle_kd').value)
        self._ttr_max_yaw = self.get_parameter('ttr_max_yaw_distance').value
        self._ttr_dece    = self.get_parameter('ttr_target_dece_dis').value
        self._mpc_N       = int(self.get_parameter('mpc_horizon').value)
        self._mpc_dt      = self.get_parameter('mpc_dt').value
        self._mpc_w_cte   = self.get_parameter('mpc_w_cte').value
        self._mpc_w_hdg   = self.get_parameter('mpc_w_heading').value
        self._mpc_w_str   = self.get_parameter('mpc_w_steer').value
        self._mpc_w_dstr  = self.get_parameter('mpc_w_dsteer').value
        self._mpc_wb      = max(self.get_parameter('wheelbase_m').value, 0.1)

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

        # Resource management values
        self._recharge_lat    = self.get_parameter('recharge_lat').value
        self._recharge_lon    = self.get_parameter('recharge_lon').value
        self._water_lat       = self.get_parameter('water_lat').value
        self._water_lon       = self.get_parameter('water_lon').value
        self._battery_low_pct = self.get_parameter('battery_low_pct').value
        self._tank_low_pct    = self.get_parameter('tank_low_pct').value
        self._base_speed      = self.get_parameter('base_speed').value
        self._base_accept     = self.get_parameter('base_acceptance_m').value
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

        # Obstacle avoidance state
        # _path_original: clean copy of received mission waypoints (without bypass points).
        #                 _reroute_path() always rebuilds _path from this — idempotent.
        # _obstacle_polygons: raw fence polygons [(lat,lon), ...] received from mission_fence.
        # _expanded_polygons: _obstacle_polygons each expanded by _clearance metres.
        # _bypass_indices: set of indices in _path that are synthetic bypass waypoints.
        #                  _advance_path() skips publishing wp_active for these.
        self._path_original:     list[MissionWaypoint]             = []
        self._obstacle_polygons: list[list[tuple[float, float]]]   = []
        self._expanded_polygons: list[list[tuple[float, float]]]   = []
        self._bypass_indices:    set[int]                          = set()

        # ── Corridor mode state ──────────────────────────────────────────────
        # When a corridor mission is loaded, the navigator converts corridors
        # + turn arcs into a single polyline stored in _path (same as waypoints).
        # _corridor_mode=True activates the simpler corridor control loop:
        # pure CTE following with no waypoint-advance logic.
        self._corridor_mode:    bool        = False
        self._corridor_widths:  list[float] = []    # half-width per path point
        self._corridor_total_s: float       = 0.0   # total path arc-length
        self._corridor_entered: bool        = False  # True once rover is inside corridor width

        # Hold state — rover waits at a waypoint for hold_secs before advancing.
        self._holding:  bool  = False
        self._hold_end: float = 0.0

        # Pivot-turn state — rover spins in place to the outgoing heading before
        # advancing past a sharp-turn waypoint.
        self._pivoting:           bool  = False
        self._pivot_target_hdg:   float = 0.0
        # Closest distance observed to the current pivot waypoint during approach.
        # Reset on each waypoint advance.  Used for overshoot detection: if the rover
        # arcs past the waypoint without entering accept_r, fires when it starts
        # moving away (dist > closest + accept).
        self._pivot_closest_dist: float = float('inf')

        # AFS (Always Forward Strategy) state.
        #   _afs_phase:        current phase — 'straight' | 'approach' | 'spin'
        #   _afs_closest_dist: minimum dist-to-wp observed during approach phase
        #   _afs_spin_hdg:     target heading for AFS post-approach spin
        self._afs_phase:        str   = 'straight'
        self._afs_closest_dist: float = float('inf')
        self._afs_spin_hdg:     float = 0.0

        # Spin-bearing freeze for large heading errors (applies to all algorithms).
        # Initialized here so _control_loop never hits AttributeError on first tick.
        self._spin_target_brg:  float | None = None

        # Resource management state machine: 'normal' → 'going_to_base' → 'at_base' → 'normal'
        self._resource_state:  str        = 'normal'
        self._resource_reason: str | None = None       # 'battery' | 'tank'
        self._saved_path_original: list   = []         # snapshot of _path_original at trigger
        self._saved_wp_idx:    int        = 0          # _path_idx at trigger (next unvisited)
        self._saved_bypass:    set        = set()      # bypass indices
        self._saved_fence:     list       = []         # expanded polygons
        self._battery_pct:     float | None = None     # latest battery % from real sensors
        self._tank_pct:        float | None = None     # latest tank % from real sensors
        self._test_tank:       float | None = None     # test-injected tank % (station_update)
        self._test_batt:       float | None = None     # test-injected batt % (station_update)

        self._dt = 1.0 / self.get_parameter('control_rate').value

        # MPC warm-start: previous horizon steer sequence (N steer_frac values)
        self._mpc_prev_steers: list[float] = []

        # Servo state (PPM CH5-CH8) re-published in every cmd_override.
        self._servo_ch: list[int] = [PPM_CENTER] * 4
        self._servo_ch_logged: list[int] = [PPM_CENTER] * 4  # last logged state

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(NavSatFix,       'fix',           self._cb_fix,          10)
        self.create_subscription(NavSatFix,       'fix_front',     self._cb_fix_front,    10)
        self.create_subscription(Float32,         'heading',       self._cb_heading,      10)
        self.create_subscription(String,          'mode',          self._cb_mode,         10)
        self.create_subscription(Bool,            'armed',         self._cb_armed,        10)
        self.create_subscription(MissionWaypoint, 'mission',       self._cb_mission,      200)
        self.create_subscription(RCInput,         'servo_state',   self._cb_servo_state,  10)
        self.create_subscription(Bool,            'mission_clear', self._cb_mission_clear, 10)
        self.create_subscription(String,          'mission_fence', self._cb_mission_fence, 10)
        self.create_subscription(String,          'corridor_mission', self._cb_corridor_mission, 10)
        self.create_subscription(Float32,         'hacc',          self._cb_hacc,          10)
        self.create_subscription(SensorData,      'sensors',       self._cb_sensors,       10)
        self.create_subscription(RoverStatus,     'status',        self._cb_status,        10)
        self.create_subscription(String,          'station_update', self._cb_station_update, 10)

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
        self.reroute_pending_pub = self.create_publisher(Bool,    'reroute_pending', 10)
        self.create_subscription(Bool, 'reroute_response', self._cb_reroute_response, 10)
        self._reroute_pending    = False
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
        self.create_timer(1.0, self._check_resources)

        # ── Services ─────────────────────────────────────────────────────────
        self.create_service(Trigger, 'pause_mission',  self._svc_pause)
        self.create_service(Trigger, 'resume_mission', self._svc_resume)

        # ── Control loop ─────────────────────────────────────────────────────
        rate = self.get_parameter('control_rate').value
        self.create_timer(1.0 / rate, self._control_loop)

        algo_info = self._algo
        if self._algo == 'mpc' and not _SCIPY_OK:
            algo_info = 'mpc→stanley(scipy missing)'
        if self._algo == 'afs':
            algo_info = (f'afs(cte_scale={self._afs_cte_scale:.1f}m '
                         f'alarm={self._afs_cte_alarm:.1f}m '
                         f'approach={self._afs_approach_dist:.1f}m)')
        self.get_logger().info(
            f'Navigator ready — algorithm={algo_info}, '
            f'full-path tracking + obstacle avoidance')

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

    def _cb_sensors(self, msg: SensorData):
        raw = msg.tank_level
        self._tank_pct = raw if raw > 0 else None

    def _cb_status(self, msg: RoverStatus):
        raw = msg.battery_remaining
        self._battery_pct = (raw * 100) if raw > 0 else None

    def _cb_station_update(self, msg: String):
        """Update station coordinates or test sensor values from mavlink_bridge."""
        try:
            data = json.loads(msg.data)
            stype = data.get('type', '')
            if stype == 'battery':
                lat = float(data.get('lat', 0.0))
                lon = float(data.get('lon', 0.0))
                self._recharge_lat = lat
                self._recharge_lon = lon
                self.get_logger().info(f'[RESOURCE] Battery station: ({lat:.5f},{lon:.5f})')
            elif stype == 'water':
                lat = float(data.get('lat', 0.0))
                lon = float(data.get('lon', 0.0))
                self._water_lat = lat
                self._water_lon = lon
                self.get_logger().info(f'[RESOURCE] Water station: ({lat:.5f},{lon:.5f})')
            if 'test_tank' in data:
                self._test_tank = float(data['test_tank'])
                self.get_logger().info(f'[TEST] tank={self._test_tank:.0f}%')
            if 'test_batt' in data:
                self._test_batt = float(data['test_batt'])
                self.get_logger().info(f'[TEST] battery={self._test_batt:.0f}%')
        except Exception as e:
            self.get_logger().warn(f'station_update parse error: {e}')

    def _cb_mode(self, msg: String):
        self._mode = msg.data

    def _cb_reroute_response(self, msg: Bool):
        if not self._reroute_pending:
            return
        self._reroute_pending = False
        rp = Bool(); rp.data = False
        self.reroute_pending_pub.publish(rp)
        if msg.data:
            self.get_logger().info('Reroute confirmed — resuming navigation')
        else:
            self.get_logger().info('Reroute rejected — reverting to original path')
            self._path = list(self._path_original)
            self._bypass_indices = set()
            # Rebuild arc-lengths
            self._path_s = []
            for k, wp in enumerate(self._path):
                if k == 0:
                    if self._path_origin_lat is not None:
                        d = haversine(self._path_origin_lat, self._path_origin_lon,
                                      wp.latitude, wp.longitude)
                    else:
                        d = 0.0
                    self._path_s.append(d)
                else:
                    prev = self._path[k - 1]
                    d = haversine(prev.latitude, prev.longitude,
                                  wp.latitude, wp.longitude)
                    self._path_s.append(self._path_s[-1] + d)
            self._path_version += 1
            pv = Int32(); pv.data = self._path_version
            self.path_version_pub.publish(pv)

    def _cb_armed(self, msg: Bool):
        was_armed = self._armed
        self._armed = msg.data
        # Approach path: if arming far from WP[0], plan route and disarm
        if (msg.data and not was_armed and self._path
                and self._path_idx == 0 and self._fix is not None
                and not getattr(self, '_approach_planned', False)):
            rlat, rlon = self._center_pos()
            wp0 = self._path[0]
            dist = haversine(rlat, rlon, wp0.latitude, wp0.longitude)
            if dist > 1.5:
                self.get_logger().info(
                    f'Rover {dist:.1f}m from WP[0] — planning approach path')
                self._plan_approach_path(rlat, rlon)
                return
        # Resume from waiting point on re-arm
        if msg.data and not was_armed and self._path_idx < len(self._path):
            wp = self._path[self._path_idx]
            if wp.hold_secs < 0.0:
                self.get_logger().info(
                    f'Re-armed at waiting point WP{wp.seq} — advancing')
                self._advance_path()
        # Resume from base on re-arm
        if msg.data and not was_armed and self._resource_state == 'at_base':
            self._resource_state = 'normal'
            self.get_logger().info('[RESOURCE] Re-armed at base — resuming mission')

    def _plan_approach_path(self, rover_lat: float, rover_lon: float):
        """Plan A-star path from rover to WP[0], prepend to mission, disarm for review."""
        from agri_rover_navigator.grid_planner import plan_around_obstacles
        wp0 = self._path[0]
        planned = plan_around_obstacles(
            start=(rover_lat, rover_lon),
            goal=(wp0.latitude, wp0.longitude),
            obstacles=self._obstacle_polygons,
            clearance_m=self._clearance,
            resolution_m=0.10,
            padding_m=self._clearance + 2.0,
        )
        if not planned or len(planned) < 2:
            self.get_logger().warn('Approach planning failed — proceeding without')
            self._approach_planned = True
            return
        # Build approach waypoints — speed proportional to remaining distance
        total_dist = sum(
            haversine(planned[i][0], planned[i][1], planned[i+1][0], planned[i+1][1])
            for i in range(len(planned) - 1))
        approach_wps = []
        remaining = total_dist
        for i, (lat, lon) in enumerate(planned[:-1]):
            frac = remaining / max(total_dist, 0.1)
            spd = self._min_speed + (self._max_speed - self._min_speed) * frac
            if i + 1 < len(planned):
                remaining -= haversine(lat, lon, planned[i+1][0], planned[i+1][1])
            wp = MissionWaypoint()
            wp.seq = -99
            wp.latitude = lat
            wp.longitude = lon
            wp.speed = max(self._min_speed, spd)
            wp.hold_secs = 0.0
            wp.acceptance_radius = self._accept_r
            approach_wps.append(wp)
        # Prepend to mission
        self._path = approach_wps + list(self._path)
        self._path_origin_lat = rover_lat
        self._path_origin_lon = rover_lon
        self._path_s = self._rebuild_path_s(self._path, rover_lat, rover_lon)
        self._bypass_indices = set(range(len(approach_wps)))
        # Shift corridor turn indices by number of prepended approach points
        shift = len(approach_wps)
        self._corridor_turn_indices = {ti + shift for ti in self._corridor_turn_indices}
        # Also update corridor total arc-length if in corridor mode
        if self._corridor_mode and self._path_s:
            self._corridor_total_s = self._path_s[-1]
        self._approach_planned = True
        # Show on map
        self._publish_full_path()
        # Disarm — user reviews approach then arms again
        self._armed = False
        a = Bool(); a.data = False
        self.armed_pub.publish(a)
        self.get_logger().info(
            f'Approach: {len(approach_wps)} pts to WP[0] — arm again to start')

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
                 1 if k in self._bypass_indices else 0,
                 round(wp.speed, 2), round(wp.hold_secs, 1)]
                for k, wp in enumerate(self._path)
            ]
            mission_out = {
                'waypoints': [{'lat': round(wp.latitude, 7),
                               'lon': round(wp.longitude, 7),
                               'speed': round(wp.speed, 2),
                               'hold_secs': round(wp.hold_secs, 1)}
                              for wp in self._path_original],
                'obstacles': [[(round(lat, 7), round(lon, 7)) for lat, lon in poly]
                              for poly in self._obstacle_polygons],
                'rerouted_path': path_data,
                'corridor_mode': self._corridor_mode,
                'algorithm': self._algo if not self._corridor_mode else 'corridor',
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
        self._bypass_indices.clear()
        self._path_idx               = 0
        self._path_origin_lat        = None
        self._path_origin_lon        = None
        self._holding                = False
        self._pivoting               = False
        self._pivot_closest_dist     = float('inf')
        self._spin_target_brg        = None
        self._afs_phase              = 'straight'
        self._afs_closest_dist       = float('inf')
        self._mpc_prev_steers        = []
        self._expanded_polygons      = []
        self._obstacle_polygons      = []
        self._reroute_pending        = False
        self._servo_ch               = [1061, 1061, PPM_CENTER, PPM_CENTER]  # CH5,CH6=off CH7,CH8=neutral
        clr_msg = String(); clr_msg.data = '[]'
        self.rerouted_pub.publish(clr_msg)
        self._publish_halt()
        self.get_logger().info('Mission cleared — path + servos reset')

    def _cb_mission(self, msg: MissionWaypoint):
        """Append waypoint to path and update cumulative arc-lengths."""
        if msg.seq == 0:
            # New run — create timestamped directory for diag + mission
            self._start_new_run()
            # New mission — discard any previous path
            self._path.clear()
            self._path_s.clear()
            self._path_original.clear()
            self._bypass_indices.clear()
            self._path_idx        = 0
            self._holding         = False
            self._pivoting               = False
            self._pivot_closest_dist     = float('inf')
            self._spin_target_brg        = None
            self._afs_phase              = 'straight'
            self._afs_closest_dist       = float('inf')
            self._mpc_prev_steers        = []
            self._expanded_polygons      = []
            self._obstacle_polygons      = []
            self._reroute_pending        = False
            # Record rover centre as path origin (start of virtual segment → wp[0]).
            if self._fix is not None:
                self._path_origin_lat, self._path_origin_lon = self._center_pos()
            else:
                self._path_origin_lat = None
                self._path_origin_lon = None

        self._path.append(msg)
        self._path_original.append(msg)

        if len(self._path) == 1:
            # Arc-length from origin to first waypoint
            if self._path_origin_lat is not None:
                d = haversine(self._path_origin_lat, self._path_origin_lon,
                              msg.latitude, msg.longitude)
            else:
                d = 0.0
            self._path_s.append(d)
        else:
            prev = self._path[-2]
            d    = haversine(prev.latitude, prev.longitude,
                             msg.latitude, msg.longitude)
            self._path_s.append(self._path_s[-1] + d)

        self.get_logger().info(
            f'WP seq={msg.seq} loaded ({msg.latitude:.7f},{msg.longitude:.7f}) '
            f'spd={msg.speed:.2f} hold={msg.hold_secs:.1f} '
            f'— {len(self._path)} total, path {self._path_s[-1]:.1f} m')

        # If expanded obstacle polygons are already available (fence arrived before
        # the last waypoint), reroute incrementally so the final path is correct.
        if self._expanded_polygons:
            self._reroute_path()

    def _collapse_spin_clusters(self):
        """Collapse turn-marked waypoint clusters into single waypoints.

        GQC marks waypoints recorded during zero-throttle turns with
        speed = -1.  Consecutive turn-marked waypoints are replaced by a
        single centroid waypoint.  The outgoing speed is taken from the
        first normal waypoint after the cluster.
        """
        if len(self._path_original) < 3:
            return
        result: list[MissionWaypoint] = []
        cluster: list[MissionWaypoint] = []

        for wp in self._path_original:
            if wp.speed < 0:
                # Turn-marked waypoint — accumulate into cluster
                cluster.append(wp)
            else:
                if cluster:
                    # Flush turn cluster → single centroid waypoint
                    result.append(self._merge_cluster(cluster))
                    cluster = []
                result.append(wp)
        if cluster:
            result.append(self._merge_cluster(cluster))

        removed = len(self._path_original) - len(result)
        if removed > 0:
            self.get_logger().info(
                f'Spin optimizer: {len(self._path_original)} → {len(result)} wps '
                f'({removed} spin-cluster points collapsed)')
            # Rebuild path and arc-lengths from optimized list
            self._path_original = result
            self._path = list(result)
            self._bypass_indices.clear()
            self._path_s = self._rebuild_path_s(
                self._path, self._path_origin_lat, self._path_origin_lon)
            # Re-number sequences
            for i, wp in enumerate(self._path):
                wp.seq = i

    @staticmethod
    def _merge_cluster(cluster: list[MissionWaypoint]) -> MissionWaypoint:
        """Merge a cluster of turn-marked waypoints into one.

        Uses the centroid position.  Speed = last positive speed in the cluster,
        or 0 (navigator default) if all are turn-marked.  Hold = max hold.
        """
        if len(cluster) == 1:
            wp = cluster[0]
            if wp.speed < 0:
                wp.speed = 0.0  # clear turn marker
            return wp
        lat_avg = sum(wp.latitude for wp in cluster) / len(cluster)
        lon_avg = sum(wp.longitude for wp in cluster) / len(cluster)
        # Use last non-zero speed (outgoing direction speed)
        speed = 0.0
        for wp in reversed(cluster):
            if wp.speed > 0:
                speed = wp.speed
                break
        hold = max(wp.hold_secs for wp in cluster)
        merged = MissionWaypoint()
        merged.seq = cluster[0].seq
        merged.latitude = lat_avg
        merged.longitude = lon_avg
        merged.speed = speed
        merged.acceptance_radius = cluster[0].acceptance_radius
        merged.hold_secs = hold
        return merged

    def _cb_mission_fence(self, msg: String):
        """Parse JSON fence polygons and reroute the current mission path."""
        # Collapse turn-marked waypoints (speed=-1 from GQC recording) into
        # single centroid waypoints before rerouting.
        if self._path_original and not self._corridor_mode:
            self._collapse_spin_clusters()

        try:
            data  = json.loads(msg.data)
            polys = data.get('polygons', [])
        except (json.JSONDecodeError, AttributeError, TypeError):
            self.get_logger().warn('mission_fence: invalid JSON — obstacle avoidance disabled')
            return

        self._obstacle_polygons = [
            [(float(v[0]), float(v[1])) for v in poly]
            for poly in polys
            if len(poly) >= 3
        ]
        self._expanded_polygons = [
            self._expand_polygon(poly, self._clearance)
            for poly in self._obstacle_polygons
        ]

        if self._obstacle_polygons:
            self.get_logger().info(
                f'Obstacle fence: {len(self._obstacle_polygons)} polygon(s), '
                f'clearance={self._clearance:.1f} m, '
                f'path_original={len(self._path_original)} wps')
            for pi, poly in enumerate(self._obstacle_polygons):
                verts_str = '  '.join(f'{lat:.7f},{lon:.7f}' for lat, lon in poly)
                self.get_logger().info(f'  poly[{pi}] raw ({len(poly)} verts): {verts_str}')
                exp = self._expanded_polygons[pi]
                exp_str = '  '.join(f'{lat:.7f},{lon:.7f}' for lat, lon in exp)
                self.get_logger().info(f'  poly[{pi}] expanded: {exp_str}')

        # Only reroute if there are actual obstacles — empty fence should not
        # trigger _reroute_path() because waypoints may still be arriving and
        # a premature split would scramble the waypoint order.
        if self._obstacle_polygons and self._path_original:
            self._reroute_path()
        elif self._obstacle_polygons:
            self.get_logger().warn(
                'Obstacle fence arrived before waypoints — reroute deferred '
                '(will trigger on next waypoint via _cb_mission)')
        else:
            self.get_logger().info('Obstacle fence: empty (no polygons)')
            # No obstacles — publish plain path so mavlink_bridge has the full
            # path for MSN_ID hash and mission download.
            if self._path:
                self._publish_full_path()

    # ── Services ──────────────────────────────────────────────────────────────

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

    def _expand_polygon(self, polygon: list[tuple[float, float]],
                        clearance_m: float) -> list[tuple[float, float]]:
        """
        Offset each polygon edge outward by clearance_m (uniform Minkowski buffer).

        Each edge is moved along its outward normal, then adjacent offset edges are
        re-intersected to find the new vertices.  This guarantees exactly clearance_m
        perpendicular distance from every edge, unlike centroid-based expansion which
        gives unequal offsets on non-circular polygons.
        """
        n = len(polygon)
        if n < 3:
            return list(polygon)

        # Flat-earth conversion centred at polygon centroid
        c_lat = sum(p[0] for p in polygon) / n
        c_lon = sum(p[1] for p in polygon) / n
        cos_lat = math.cos(math.radians(c_lat)) or 1e-9
        m_lat   = 111_320.0
        m_lon   = 111_320.0 * cos_lat

        def to_m(lat: float, lon: float) -> tuple[float, float]:
            return (lon - c_lon) * m_lon, (lat - c_lat) * m_lat

        def to_ll(x: float, y: float) -> tuple[float, float]:
            return c_lat + y / m_lat, c_lon + x / m_lon

        pts = [to_m(lat, lon) for lat, lon in polygon]

        # Signed area — positive = CCW, negative = CW
        area2 = sum(pts[i][0] * pts[(i + 1) % n][1] -
                    pts[(i + 1) % n][0] * pts[i][1]
                    for i in range(n))
        # Outward normal direction: left of edge for CCW (+1), right for CW (-1)
        w = 1.0 if area2 > 0 else -1.0

        # Offset each edge outward by clearance_m
        off: list[tuple[float, float, float, float]] = []
        for i in range(n):
            x1, y1 = pts[i]
            x2, y2 = pts[(i + 1) % n]
            dx, dy = x2 - x1, y2 - y1
            length  = math.hypot(dx, dy) or 1e-9
            nx, ny  =  w * dy / length, -w * dx / length     # outward unit normal
            ox, oy  = nx * clearance_m, ny * clearance_m
            off.append((x1 + ox, y1 + oy, x2 + ox, y2 + oy))

        # Intersect consecutive offset edges to produce new vertices
        result: list[tuple[float, float]] = []
        for i in range(n):
            x1, y1, x2, y2 = off[i]
            x3, y3, x4, y4 = off[(i + 1) % n]
            dx1, dy1 = x2 - x1, y2 - y1
            dx2, dy2 = x4 - x3, y4 - y3
            denom = dx1 * dy2 - dy1 * dx2
            if abs(denom) < 1e-9:
                result.append(to_ll(x2, y2))   # parallel — use edge endpoint
            else:
                t = ((x3 - x1) * dy2 - (y3 - y1) * dx2) / denom
                result.append(to_ll(x1 + t * dx1, y1 + t * dy1))
        return result

    def _seg_intersect_polygon(
            self,
            a_lat: float, a_lon: float,
            b_lat: float, b_lon: float,
            polygon: list[tuple[float, float]]) -> list[tuple[float, int]]:
        """
        Return sorted list of (t, edge_idx) where t ∈ (0, 1) is the position
        along segment A→B at which it crosses polygon edge edge_idx.

        Uses flat-earth Cartesian coordinates centred at the midpoint of A and B.
        Valid for segments shorter than ~500 m.
        """
        if len(polygon) < 3:
            return []
        c_lat = (a_lat + b_lat) / 2.0
        c_lon = (a_lon + b_lon) / 2.0
        cos_lat = math.cos(math.radians(c_lat)) or 1e-9
        m_lat = 111_320.0
        m_lon = 111_320.0 * cos_lat

        def xy(lat: float, lon: float) -> tuple[float, float]:
            return (lon - c_lon) * m_lon, (lat - c_lat) * m_lat

        ax, ay = xy(a_lat, a_lon)
        bx, by = xy(b_lat, b_lon)
        dx, dy = bx - ax, by - ay   # direction of A→B

        hits: list[tuple[float, int]] = []
        n = len(polygon)
        for i in range(n):
            cx, cy = xy(*polygon[i])
            nx, ny = xy(*polygon[(i + 1) % n])
            ex, ey = nx - cx, ny - cy   # direction of polygon edge i→(i+1)
            rx, ry = cx - ax, cy - ay   # vector from A to edge start

            # 2-D cross products: det = e×d, t = e×r / det, u = d×r / det
            det = ex * dy - ey * dx
            if abs(det) < 1e-9:
                continue   # parallel
            t = (ex * ry - ey * rx) / det
            u = (dx * ry - dy * rx) / det
            if 1e-6 < t < 1.0 - 1e-6 and 0.0 <= u <= 1.0:
                hits.append((t, i))

        hits.sort(key=lambda h: h[0])
        return hits

    def _bypass_arc(
            self,
            entry_pt: tuple[float, float],
            exit_pt: tuple[float, float],
            entry_edge: int,
            exit_edge: int,
            polygon: list[tuple[float, float]]) -> list[tuple[float, float]]:
        """Return the shorter arc path: entry_pt → polygon vertices → exit_pt."""
        n = len(polygon)

        ccw_verts: list[tuple[float, float]] = []
        idx = (entry_edge + 1) % n
        for _ in range(n):
            ccw_verts.append(polygon[idx])
            if idx == exit_edge:
                break
            idx = (idx + 1) % n

        cw_verts: list[tuple[float, float]] = []
        target = (exit_edge + 1) % n
        idx = entry_edge
        for _ in range(n):
            cw_verts.append(polygon[idx])
            if idx == target:
                break
            idx = (idx - 1 + n) % n

        def path_len(pts: list[tuple[float, float]]) -> float:
            total = 0.0
            for k in range(len(pts) - 1):
                total += haversine(pts[k][0], pts[k][1], pts[k + 1][0], pts[k + 1][1])
            return total

        ccw_path = [entry_pt] + ccw_verts + [exit_pt]
        cw_path  = [entry_pt] + cw_verts  + [exit_pt]
        return ccw_path if path_len(ccw_path) <= path_len(cw_path) else cw_path

    def _bypass_verts(
            self,
            a_lat: float, a_lon: float,
            b_lat: float, b_lon: float,
            hits: list[tuple[float, int]],
            polygon: list[tuple[float, float]]) -> list[tuple[float, float]]:
        """
        Given segment A→B and its sorted intersections with polygon, return a list
        of (lat, lon) points that detour around the polygon boundary.
        Returns an empty list if fewer than 2 intersections.
        """
        if len(hits) < 2:
            return []
        t_entry, entry_edge = hits[0]
        t_exit,  exit_edge  = hits[-1]

        def interp(t: float) -> tuple[float, float]:
            return (a_lat + t * (b_lat - a_lat),
                    a_lon + t * (b_lon - a_lon))

        return self._bypass_arc(interp(t_entry), interp(t_exit),
                                entry_edge, exit_edge, polygon)

    # ── Corridor mission handling ────────────────────────────────────────────

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
            # it's likely a raw recording. Split at sharp turns into multiple
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

            # Turn indices and servo state from corridors_to_path
            # Always include index 0 — rover aligns heading before driving
            self._corridor_turn_indices = {
                i for i, pt in enumerate(path_pts) if pt[4]
            }
            self._corridor_servo = [pt[5] for pt in path_pts]

            # New run — create timestamped directory for diag + mission
            self._start_new_run()
            # Clear any existing waypoint mission
            self._path.clear()
            self._path_s.clear()
            self._path_original.clear()
            self._bypass_indices.clear()
            self._path_idx          = 0
            self._holding           = False
            self._pivoting          = False
            self._spin_target_brg   = None
            self._corridor_mode     = True
            self._corridor_entered  = False
            self._pivot_min_dist    = None
            self._corridor_widths.clear()

            # Build path from corridor polyline
            for i, (lat, lon, speed, width, _is_turn, _srv) in enumerate(path_pts):
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
            self._sim_validate_path()

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

        # Corridor width boundary — only enforce once the rover has entered the
        # corridor (CTE was within width at least once).  On mission start the
        # rover may be outside the corridor and needs to drive to the path first.
        width = self._corridor_widths[seg_idx] if seg_idx < len(self._corridor_widths) else 1.5
        if abs(cte) < width:
            self._corridor_entered = True
        if self._corridor_entered and abs(cte) >= width:
            self.get_logger().error(
                f'CORRIDOR BOUNDARY: CTE {cte:.2f}m >= width {width:.1f}m — disarming')
            self._publish_halt()
            # Use -3 (at_base signal) to disarm but keep mission loaded (STATUS=MSL).
            # User can reposition and re-arm to continue.
            m = Int32(); m.data = -3
            self.wp_pub.publish(m)
            self._corridor_entered = False  # reset so entry grace applies on re-arm
            return

        # CTE safety alarm removed — user confirms mission by arming twice
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

        # Pivot at turn point: drive past it, then stop and spin when distance increases
        if turn_idx is not None:
            dist_to_turn = turn_s - s_nearest
            tp = self._path[turn_idx]
            direct_dist = haversine(rlat, rlon, tp.latitude, tp.longitude)
            # Track closest approach to turn point
            if not hasattr(self, '_pivot_min_dist') or self._pivot_min_dist is None:
                self._pivot_min_dist = direct_dist
            if direct_dist < self._pivot_min_dist:
                self._pivot_min_dist = direct_dist
            # Trigger: within acceptance radius AND distance is increasing (passed it)
            passed_turn = (self._pivot_min_dist < self._accept_r
                           and direct_dist > self._pivot_min_dist + 0.05)
            if passed_turn:
                # Find a point well into the next corridor for a stable bearing.
                # Use at least 2m ahead (or the farthest available point).
                tp = self._path[turn_idx]
                nxt = turn_idx + 1
                best_nxt = min(nxt, len(self._path) - 1)
                while nxt < len(self._path):
                    d = haversine(tp.latitude, tp.longitude,
                                  self._path[nxt].latitude, self._path[nxt].longitude)
                    if d > 0.1:
                        best_nxt = nxt  # at least a distinct point
                    if d >= 2.0:
                        best_nxt = nxt
                        break
                    nxt += 1
                next_brg = bearing_to(
                    tp.latitude, tp.longitude,
                    self._path[best_nxt].latitude, self._path[best_nxt].longitude)
                pivot_err = ((next_brg - self._heading + 180) % 360) - 180
                if abs(pivot_err) > self._hdb:
                    # Still turning — spin in place
                    steer_frac = max(-self._max_steer, min(self._max_steer, pivot_err / 45.0))
                    steer_ppm = int(PPM_CENTER - steer_frac * 500)
                    if steer_ppm != PPM_CENTER:
                        sign = 1 if steer_ppm > PPM_CENTER else -1
                        steer_ppm = PPM_CENTER + sign * max(abs(steer_ppm - PPM_CENTER),
                                                             self._min_steer_delta)
                    self._publish_cmd(PPM_CENTER, steer_ppm)
                    # Log pivot spin to diag
                    if self._diag_writer is not None:
                        sf = (PPM_CENTER - steer_ppm) / 500.0
                        self._diag_writer.writerow([
                            round(time.time(), 4),
                            round(rlat, 8), round(rlon, 8),
                            round(self._heading, 2),
                            round(next_brg, 2),
                            round(pivot_err, 2),
                            round(cte, 4),
                            round(sf, 4),
                            steer_ppm, PPM_CENTER,
                            0, round(direct_dist, 3),
                            seg_idx, 'pivot',
                            self._fix.status.status if self._fix else -1,
                            round(self._hacc_mm, 1),
                        ])
                    return
                # Spin complete — advance past turn into new corridor
                self._corridor_turn_indices.discard(turn_idx)
                self._spin_target_brg = None
                self._pivot_min_dist = None  # reset for next turn
                self._path_idx = turn_idx + 1  # post-turn slow point
                self._apply_corridor_servo(self._path_idx)
                self._corridor_entered = False  # re-enter corridor grace
                self.get_logger().info(
                    f'PIVOT DONE: turn_idx={turn_idx} nxt={nxt} '
                    f'new_path_idx={self._path_idx} hdg={self._heading:.1f} '
                    f'next_brg={next_brg:.1f} remaining_turns={self._corridor_turn_indices}')
                return  # recompute everything on next tick with new _path_idx

        # Freeze target bearing during align-spin
        if self._spin_target_brg is not None:
            target_bearing = self._spin_target_brg

        heading_err = ((target_bearing - self._heading + 180) % 360) - 180

        # Speed: per-point speed with CTE reduction
        wp = self._path[seg_idx]
        target_spd = wp.speed if wp.speed > 0 else self._max_speed
        cte_factor = max(0.0, 1.0 - abs(cte) / max(self._stanley_cte_scale, 0.01))
        v_mps = max(self._min_speed, target_spd * cte_factor)

        # Decelerate approaching turn point
        if turn_s is not None:
            dist_to_turn = turn_s - s_nearest
            approach = self._pivot_approach_dist
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
        """Publish current _path as JSON for mavlink_bridge → GQC mission sync.

        Format: [[lat, lon, bypass, speed, hold_secs], ...]
        mavlink_bridge uses this for MSN_ID hash and MISSION_REQUEST_LIST downloads.
        """
        path_data = [
            [round(wp.latitude, 7), round(wp.longitude, 7),
             1 if k in self._bypass_indices else 0,
             round(wp.speed, 2), round(wp.hold_secs, 1)]
            for k, wp in enumerate(self._path)
        ]
        rp_msg = String()
        rp_msg.data = json.dumps(path_data, separators=(',', ':'))
        self.rerouted_pub.publish(rp_msg)
        self._save_run_mission()

    def _smooth_bypass_corners(self):
        """Replace bypass zones with A-star planned smooth paths.

        For each contiguous group of bypass indices, plans a smooth path
        from the entry point to the exit point using A-star on a grid
        with inflated obstacle polygons. The planned path naturally
        curves around all obstacles simultaneously.

        bypass_arc_radius_m > 0 enables smoothing (0 = disabled).
        """
        if not self._bypass_indices or not self._path or not self._path_s:
            return
        if self._bypass_arc_radius <= 0:
            return

        from agri_rover_navigator.grid_planner import plan_around_obstacles

        # Find contiguous bypass groups
        bp_sorted = sorted(self._bypass_indices)
        groups = []
        g_start = bp_sorted[0]
        g_end = bp_sorted[0]
        for bp in bp_sorted[1:]:
            if bp <= g_end + 2:  # within 2 indices = same group
                g_end = bp
            else:
                groups.append((g_start, g_end))
                g_start = g_end = bp
        groups.append((g_start, g_end))

        self.get_logger().info(
            f'A-star smoother: {len(groups)} bypass group(s) to plan')

        # Process groups in reverse order so index shifts don't affect earlier groups
        new_bypass = set()
        for g_start, g_end in reversed(groups):
            # Entry = point before bypass, exit = point after bypass
            entry_idx = max(0, g_start - 1)
            exit_idx = min(len(self._path) - 1, g_end + 1)
            entry_wp = self._path[entry_idx]
            exit_wp = self._path[exit_idx]

            start_gps = (entry_wp.latitude, entry_wp.longitude)
            goal_gps = (exit_wp.latitude, exit_wp.longitude)

            planned = plan_around_obstacles(
                start=start_gps,
                goal=goal_gps,
                obstacles=self._obstacle_polygons,
                clearance_m=self._clearance,
                resolution_m=0.10,
                padding_m=self._clearance + 2.0,
            )

            if not planned or len(planned) < 2:
                self.get_logger().warn(
                    f'A-star: no path for group [{g_start}..{g_end}] — keeping bypass')
                continue

            # Map speeds from nearest original waypoint
            smooth_wps = []
            orig_zone = self._path[entry_idx:exit_idx + 1]
            for lat, lon in planned[1:-1]:  # skip first/last (they're entry/exit)
                base_spd = self._max_speed
                best_d = float('inf')
                for wp in orig_zone:
                    d = haversine(lat, lon, wp.latitude, wp.longitude)
                    if d < best_d:
                        best_d = d
                        base_spd = wp.speed if wp.speed > 0 else self._max_speed
                bwp = MissionWaypoint()
                bwp.seq = -99  # bypass marker
                bwp.latitude = lat
                bwp.longitude = lon
                bwp.speed = base_spd
                bwp.hold_secs = 0.0
                bwp.acceptance_radius = self._accept_r
                smooth_wps.append(bwp)

            # Splice: keep entry + smooth path + exit, remove old bypass
            before = list(self._path[:entry_idx + 1])
            after = list(self._path[exit_idx:])
            self._path = before + smooth_wps + after

            # Track new bypass indices
            for k in range(len(before), len(before) + len(smooth_wps)):
                new_bypass.add(k)

            self.get_logger().info(
                f'A-star: group [{g_start}..{g_end}] → {len(smooth_wps)} smooth pts '
                f'(was {g_end - g_start + 1} bypass)')

        self._bypass_indices = new_bypass
        origin_lat = self._path_origin_lat or self._path[0].latitude
        origin_lon = self._path_origin_lon or self._path[0].longitude
        self._path_s = self._rebuild_path_s(self._path, origin_lat, origin_lon)
        self._corridor_turn_indices = set()

        self.get_logger().info(
            f'A-star smoothed: {len(self._path)} total pts')

        # Run Stanley sim through the A-star+Chaikin path — iterate up to 3x if CTE > 0.30m
        orig_speeds = [(wp.latitude, wp.longitude,
                        wp.speed if wp.speed > 0 else self._max_speed)
                       for wp in self._path]
        for sim_pass in range(3):
            result = self._sim_validate_path()
            max_cte = result.get('max_cte', 99)
            sim_trace = result.get('sim_path', [])
            if not sim_trace or len(sim_trace) < 2:
                break
            # Replace path with sim trace
            new_path = []
            for lat, lon in sim_trace:
                base_spd = self._max_speed
                best_d = float('inf')
                for olat, olon, ospd in orig_speeds:
                    d = haversine(lat, lon, olat, olon)
                    if d < best_d:
                        best_d = d
                        base_spd = ospd
                wp = MissionWaypoint()
                wp.seq = len(new_path)
                wp.latitude = lat
                wp.longitude = lon
                wp.speed = base_spd
                wp.hold_secs = 0.0
                wp.acceptance_radius = self._accept_r
                new_path.append(wp)
            # Drop sim points inside obstacles
            clean_path = []
            for wp in new_path:
                inside = False
                for poly in self._expanded_polygons:
                    if self._point_in_polygon(wp.latitude, wp.longitude, poly):
                        inside = True
                        break
                if not inside:
                    clean_path.append(wp)
            if len(clean_path) >= 2:
                self._path = clean_path
                self._bypass_indices = set()
                self._corridor_turn_indices = set()
                self._path_s = self._rebuild_path_s(
                    clean_path,
                    self._path_origin_lat or clean_path[0].latitude,
                    self._path_origin_lon or clean_path[0].longitude)
            self.get_logger().info(
                f'Sim pass {sim_pass}: {len(clean_path)} pts '
                f'(dropped {len(new_path) - len(clean_path)}), '
                f'max_cte={max_cte:.3f}m')
            if max_cte <= 0.30:
                break

    @staticmethod
    def _point_in_polygon(lat: float, lon: float, poly: list) -> bool:
        """Ray-casting point-in-polygon test."""
        inside = False
        n = len(poly)
        j = n - 1
        for i in range(n):
            yi, xi = poly[i]
            yj, xj = poly[j]
            if ((yi > lat) != (yj > lat)) and (lon < (xj - xi) * (lat - yi) / (yj - yi) + xi):
                inside = not inside
            j = i
        return inside

    def _clone_wp(self, wp: MissionWaypoint) -> MissionWaypoint:
        c = MissionWaypoint()
        c.seq = wp.seq
        c.latitude = wp.latitude
        c.longitude = wp.longitude
        c.speed = wp.speed
        c.hold_secs = wp.hold_secs
        c.acceptance_radius = wp.acceptance_radius
        return c

    def _reroute_path(self):
        """
        Rebuild _path and _path_s from _path_original, inserting bypass waypoints
        around any expanded obstacle polygon that a path segment intersects.

        Handles polygons that span across a waypoint (entry and exit on different
        segments) via cross-segment scan-ahead.

        If the rover is actively navigating (armed + AUTONOMOUS + path_idx > 0),
        the reroute is applied but the rover pauses for user confirmation via GQC.
        """
        was_navigating = (self._mode == 'AUTONOMOUS' and self._armed
                          and self._path_idx > 0)

        if not self._path_original:
            return

        new_wps: list[MissionWaypoint] = []
        new_bypass_indices: set[int]   = set()

        def make_bypass_wp(lat: float, lon: float,
                           ref_wp: MissionWaypoint) -> MissionWaypoint:
            bp = MissionWaypoint()
            bp.seq               = -99        # bypass marker — filtered in _advance_path
            bp.latitude          = lat
            bp.longitude         = lon
            bp.speed             = ref_wp.speed
            bp.hold_secs         = 0.0
            bp.acceptance_radius = self._accept_r
            return bp

        orig = self._path_original

        # Prepend a synthetic origin waypoint so the rover-start → orig[0]
        # segment is also checked for obstacle intersections.
        # It is stripped from new_wps at the end (identified by seq == -9999).
        _ORIGIN_SEQ = -9999
        if self._path_origin_lat is not None and orig:
            origin_wp = MissionWaypoint()
            origin_wp.seq               = _ORIGIN_SEQ
            origin_wp.latitude          = self._path_origin_lat
            origin_wp.longitude         = self._path_origin_lon
            origin_wp.speed             = orig[0].speed
            origin_wp.hold_secs         = 0.0
            origin_wp.acceptance_radius = self._accept_r
            all_wps = [origin_wp] + list(orig)
        else:
            all_wps = list(orig)

        i = 0
        while i < len(all_wps):
            if not new_wps:
                new_wps.append(all_wps[i])
                i += 1
                continue

            prev     = new_wps[-1]
            wp       = all_wps[i]
            a_lat, a_lon = prev.latitude, prev.longitude
            b_lat, b_lon = wp.latitude, wp.longitude

            # Compute intersections with all polygons for this segment
            seg_hits: list[tuple[int, list]] = []
            for poly_idx, poly in enumerate(self._expanded_polygons):
                hits = self._seg_intersect_polygon(a_lat, a_lon, b_lat, b_lon, poly)
                if hits:
                    seg_hits.append((poly_idx, hits))

            # Case 1: complete in-segment crossing (>=2 hits)
            complete = [(pi, h) for pi, h in seg_hits if len(h) >= 2]
            if complete:
                complete.sort(key=lambda x: x[1][0][0])
                best_pi, best_hits = complete[0]
                poly   = self._expanded_polygons[best_pi]
                bypass = self._bypass_verts(a_lat, a_lon, b_lat, b_lon, best_hits, poly)
                for bp_lat, bp_lon in bypass:
                    idx = len(new_wps)
                    new_bypass_indices.add(idx)
                    new_wps.append(make_bypass_wp(bp_lat, bp_lon, wp))
                new_wps.append(wp)
                i += 1
                continue

            # Case 2: 1-hit entry — polygon spans across a waypoint, scan ahead
            entries = [(pi, h[0]) for pi, h in seg_hits if len(h) == 1]
            if entries:
                entries.sort(key=lambda x: x[1][0])
                entry_pi, (entry_t, entry_edge) = entries[0]
                poly = self._expanded_polygons[entry_pi]
                entry_pt = (a_lat + entry_t * (b_lat - a_lat),
                            a_lon + entry_t * (b_lon - a_lon))

                # Scan ahead through all_wps to find the exit crossing
                j = i + 1
                found_exit = False
                while j < len(all_wps):
                    c_lat = all_wps[j - 1].latitude
                    c_lon = all_wps[j - 1].longitude
                    d_lat = all_wps[j].latitude
                    d_lon = all_wps[j].longitude
                    exit_hits = self._seg_intersect_polygon(c_lat, c_lon, d_lat, d_lon, poly)
                    if exit_hits:
                        exit_t, exit_edge = exit_hits[0]
                        exit_pt = (c_lat + exit_t * (d_lat - c_lat),
                                   c_lon + exit_t * (d_lon - c_lon))
                        bypass = self._bypass_arc(entry_pt, exit_pt,
                                                  entry_edge, exit_edge, poly)
                        self.get_logger().info(
                            f'Cross-segment bypass: {len(bypass)} pts, '
                            f'poly[{entry_pi}] spans wps {i}..{j-1}')
                        for bp_lat, bp_lon in bypass:
                            idx = len(new_wps)
                            new_bypass_indices.add(idx)
                            new_wps.append(make_bypass_wp(bp_lat, bp_lon, all_wps[j]))
                        new_wps.append(all_wps[j])
                        i = j + 1
                        found_exit = True
                        break
                    j += 1

                if not found_exit:
                    new_wps.append(wp)
                    i += 1
                continue

            # No intersection — keep original waypoint
            new_wps.append(wp)
            i += 1

        # Strip synthetic origin waypoint (it is not a real mission waypoint)
        if new_wps and new_wps[0].seq == _ORIGIN_SEQ:
            new_wps.pop(0)
            # Shift bypass indices — the origin occupied index 0
            new_bypass_indices = {idx - 1 for idx in new_bypass_indices if idx > 0}

        # Rebuild path arc-lengths from new_wps
        new_s: list[float] = []
        for k, wp in enumerate(new_wps):
            if k == 0:
                d = (haversine(self._path_origin_lat, self._path_origin_lon,
                               wp.latitude, wp.longitude)
                     if self._path_origin_lat is not None else 0.0)
            else:
                prev = new_wps[k - 1]
                d    = new_s[-1] + haversine(prev.latitude, prev.longitude,
                                             wp.latitude, wp.longitude)
            new_s.append(d)

        self._path          = new_wps
        self._path_s        = new_s
        self._bypass_indices = new_bypass_indices
        self._path_idx      = 0
        self._holding       = False
        self._pivoting      = False

        # Save entry/exit GPS anchors for final validation
        _entry_ll = None
        _exit_ll = None
        if new_bypass_indices:
            bp_sorted = sorted(new_bypass_indices)
            entry_idx = max(0, bp_sorted[0] - 1)
            exit_idx = min(len(self._path) - 1, bp_sorted[-1] + 1)
            _entry_ll = (self._path[entry_idx].latitude, self._path[entry_idx].longitude)
            _exit_ll = (self._path[exit_idx].latitude, self._path[exit_idx].longitude)

        # Reduce redundant bypass points — 3 passes, front to back.
        if new_bypass_indices:
            total_removed = 0
            for reduce_pass in range(3):
                bp_set = set(self._bypass_indices)
                if not bp_set:
                    break
                bp_sorted = sorted(bp_set)
                first_bp = bp_sorted[0]
                last_bp = bp_sorted[-1]
                # Target: first non-bypass point after the bypass zone
                exit_idx = last_bp + 1
                exit_wp = self._path[exit_idx] if exit_idx < len(self._path) else self._path[-1]

                reduced = []
                i = 0
                removed = 0
                while i < len(self._path):
                    if i not in bp_set:
                        reduced.append(self._path[i])
                        i += 1
                        continue
                    # Bypass point — check if skipping ahead gets closer to exit
                    reduced.append(self._path[i])
                    best_next = i + 1
                    if i + 1 < len(self._path):
                        d_next = haversine(self._path[i + 1].latitude, self._path[i + 1].longitude,
                                           exit_wp.latitude, exit_wp.longitude)
                        for k in range(2, 6):
                            if i + k >= len(self._path):
                                break
                            if (i + k) not in bp_set:
                                break  # don't skip over non-bypass points
                            d_k = haversine(self._path[i + k].latitude, self._path[i + k].longitude,
                                            exit_wp.latitude, exit_wp.longitude)
                            if d_k < d_next:
                                best_next = i + k
                                d_next = d_k
                    if best_next > i + 1:
                        removed += best_next - (i + 1)
                    i = best_next

                if removed == 0:
                    break
                total_removed += removed
                self._path = reduced
                # Rebuild bypass indices
                new_bp = set()
                for k, wp in enumerate(reduced):
                    if wp.seq == -99:
                        new_bp.add(k)
                self._bypass_indices = new_bp
                new_bypass_indices = new_bp
                origin_lat = self._path_origin_lat or reduced[0].latitude
                origin_lon = self._path_origin_lon or reduced[0].longitude
                self._path_s = self._rebuild_path_s(reduced, origin_lat, origin_lon)

            if total_removed > 0:
                self.get_logger().info(
                    f'Bypass reducer: removed {total_removed} pts in {reduce_pass + 1} passes')

        # Smooth full mission — the sim naturally curves around obstacles
        if new_bypass_indices:
            self._smooth_bypass_corners()

        # Final validation: entry/exit GPS anchors must still exist in path
        if _entry_ll and _exit_ll:
            entry_ok = any(
                haversine(wp.latitude, wp.longitude, _entry_ll[0], _entry_ll[1]) < 0.1
                for wp in self._path)
            exit_ok = any(
                haversine(wp.latitude, wp.longitude, _exit_ll[0], _exit_ll[1]) < 0.1
                for wp in self._path)
            if entry_ok and exit_ok:
                self.get_logger().info('Bypass anchors validated: entry/exit GPS intact')
            else:
                self.get_logger().warn(
                    f'Bypass anchor MISMATCH: entry={entry_ok} exit={exit_ok}')

        self._publish_full_path()
        self._path_version += 1
        pv = Int32(); pv.data = self._path_version
        self.path_version_pub.publish(pv)

        n_bypass = len(self._bypass_indices)
        if n_bypass > 0:
            self.get_logger().info(
                f'Path rerouted: {len(self._path_original)} → {len(self._path)} wps '
                f'({n_bypass} bypass inserted)')
            for idx in sorted(new_bypass_indices):
                bp = new_wps[idx]
                self.get_logger().info(
                    f'  bypass[{idx}]: {bp.latitude:.7f},{bp.longitude:.7f}')
        else:
            self.get_logger().info(
                f'Reroute complete: {len(self._path_original)} wps, '
                f'no bypass needed (path does not cross any obstacle)')
        # Log surrounding waypoints for context
        for k, wp in enumerate(new_wps):
            if k in new_bypass_indices or (k > 0 and k - 1 in new_bypass_indices) or (k < len(new_wps) - 1 and k + 1 in new_bypass_indices):
                tag = 'BYPASS' if k in new_bypass_indices else 'orig'
                self.get_logger().info(f'  path[{k}] {tag}: seq={wp.seq} {wp.latitude:.7f},{wp.longitude:.7f}')

        self._sim_validate_path()

        # If rover was actively navigating and bypass was inserted, pause for
        # user confirmation via GQC before proceeding with the new route.
        if was_navigating and n_bypass > 0:
            self._reroute_pending = True
            rp = Bool(); rp.data = True
            self.reroute_pending_pub.publish(rp)
            self._publish_halt()
            self.get_logger().info(
                'Reroute pending — waiting for user confirmation via GQC')

    # ── Fast internal simulation (preflight CTE check) ─────────────────────

    def _sim_validate_path(self) -> dict:
        """Run fast offline simulation of the current path using DiffDriveState.

        Calls the navigator's own geometry methods (_cte_to_seg, _point_at_s)
        on self._path / self._path_s — no ROS2 topics, no code duplication.
        Runs in <1s for typical missions (tight Python loop, ~1000x real-time).

        Outputs sim_diag.csv in the same format as navigator_diag.csv so the
        two can be compared directly in mission_planner.

        Returns dict with max_cte, avg_cte, complete, sim_path.
        """
        from agri_rover_simulator.diff_drive import DiffDriveState

        if not self._path or len(self._path) < 2 or not self._path_s:
            return {'ok': False, 'reason': 'no path'}

        # Init position at first path point, heading toward second
        wp0 = self._path[0]
        wp1 = self._path[1]
        hdg = bearing_to(wp0.latitude, wp0.longitude,
                         wp1.latitude, wp1.longitude)

        baseline = 1.0  # antenna baseline metres
        half_bm = baseline / 2.0
        hdg_rad = math.radians(hdg)
        cos_lat0 = math.cos(math.radians(wp0.latitude)) or 1e-9
        rear_lat = wp0.latitude - (half_bm * math.cos(hdg_rad)) / 111320.0
        rear_lon = wp0.longitude - (half_bm * math.sin(hdg_rad)) / (111320.0 * cos_lat0)

        rover = DiffDriveState(rear_lat, rear_lon, hdg,
                               max_steer=self._max_steer)

        dt = self._dt
        rate = 1.0 / dt
        total_s = self._path_s[-1] if self._path_s else 0.0
        max_steps = int(600.0 * rate)  # 10 min hard cap

        # Sim-local state (does NOT touch self._path_idx etc.)
        sim_path_idx = 0
        sim_spin_brg = None
        sim_pivot_min = None
        sim_pivoting = False       # True while spinning at a turn point
        sim_pivot_brg = 0.0        # target bearing during pivot
        turn_indices = set(getattr(self, '_corridor_turn_indices', set()))
        is_corridor = self._corridor_mode

        max_cte = 0.0
        cte_sum = 0.0
        steps = 0
        sim_trace = []
        sim_time = 0.0

        # Open diag CSV (same columns as real navigator_diag.csv)
        sim_diag_file = None
        sim_diag_writer = None
        if self._run_dir:
            try:
                sim_diag_path = os.path.join(self._run_dir, 'sim_diag.csv')
                sim_diag_file = open(sim_diag_path, 'w', newline='')
                sim_diag_writer = csv.writer(sim_diag_file)
                sim_diag_writer.writerow([
                    't', 'lat', 'lon', 'heading',
                    'target_brg', 'hdg_err', 'cte',
                    'steer_frac', 'steer_ppm', 'throttle_ppm',
                    'speed_tgt', 'dist_to_wp', 'wp_idx', 'algo', 'fix_quality', 'hacc_mm',
                ])
            except Exception:
                sim_diag_writer = None

        def _log(lat, lon, heading, tgt_brg, herr, cte_v, sf, sppm, tppm, spd, dist, idx, algo):
            if sim_diag_writer is not None:
                sim_diag_writer.writerow([
                    round(sim_time, 4),
                    round(lat, 8), round(lon, 8),
                    round(heading, 2), round(tgt_brg, 2), round(herr, 2),
                    round(cte_v, 4), round(sf, 4),
                    sppm, tppm, round(spd, 3), round(dist, 3),
                    idx, algo, 4, 0,  # fix_quality=RTK, hacc=0
                ])

        for _ in range(max_steps):
            # Centre and front positions from rear antenna
            cos_lat = math.cos(math.radians(rover.lat)) or 1e-9
            rlat = rover.lat + (half_bm * math.cos(rover.heading_rad)) / 111320.0
            rlon = rover.lon + (half_bm * math.sin(rover.heading_rad)) / (111320.0 * cos_lat)
            flat = rover.lat + (baseline * math.cos(rover.heading_rad)) / 111320.0
            flon = rover.lon + (baseline * math.sin(rover.heading_rad)) / (111320.0 * cos_lat)
            heading = rover.heading_deg

            # Nearest on path (local search from sim_path_idx, 3 segments)
            best_s, best_seg, best_dist = 0.0, sim_path_idx, float('inf')
            search_limit = min(sim_path_idx + (3 if is_corridor else 1), len(self._path))
            if is_corridor:
                for ti in sorted(turn_indices):
                    if ti > sim_path_idx:
                        search_limit = min(search_limit, ti)
                        break
            for seg_k in range(sim_path_idx, search_limit):
                if seg_k == 0:
                    a_lat = self._path_origin_lat or wp0.latitude
                    a_lon = self._path_origin_lon or wp0.longitude
                    s_a = 0.0
                else:
                    a_lat = self._path[seg_k - 1].latitude
                    a_lon = self._path[seg_k - 1].longitude
                    s_a = self._path_s[seg_k - 1]
                b_lat = self._path[seg_k].latitude
                b_lon = self._path[seg_k].longitude
                s_b = self._path_s[seg_k]
                mid = math.radians((a_lat + b_lat) / 2)
                cl = math.cos(mid) or 1e-9
                ml, mo = 111320.0, 111320.0 * cl
                sy = (b_lat - a_lat) * ml
                sx = (b_lon - a_lon) * mo
                sl = math.hypot(sx, sy)
                if sl < 0.01:
                    d = haversine(rlat, rlon, b_lat, b_lon)
                    if d < best_dist:
                        best_dist, best_s, best_seg = d, s_b, seg_k
                    continue
                ry = (rlat - a_lat) * ml
                rx = (rlon - a_lon) * mo
                t = max(0.0, min(1.0, (rx * sx + ry * sy) / sl ** 2))
                d = math.hypot(t * sx - rx, t * sy - ry)
                if d < best_dist:
                    best_dist = d
                    best_s = s_a + t * (s_b - s_a)
                    best_seg = seg_k
            s_nearest = best_s
            seg_idx = best_seg

            # Mission complete? Arc-length OR proximity to last point
            last_wp = self._path[-1]
            d_last = haversine(rlat, rlon, last_wp.latitude, last_wp.longitude)
            if s_nearest >= total_s - self._accept_r or d_last < self._accept_r:
                break

            # CTE (front antenna, same as real navigator)
            cte = self._cte_to_seg(flat, flon, seg_idx)
            abs_cte = abs(cte)
            if abs_cte > max_cte:
                max_cte = abs_cte
            cte_sum += abs_cte
            steps += 1

            # Advance sim_path_idx — arc-length OR proximity
            if seg_idx > sim_path_idx:
                limit = len(self._path)
                if is_corridor:
                    for ti in sorted(turn_indices):
                        if ti > sim_path_idx:
                            limit = ti
                            break
                sim_path_idx = min(seg_idx, limit)
            # Proximity advance: disabled — let arc-length handle it.
            # The proximity advance was pushing sim_path_idx forward prematurely,
            # causing CTE to be computed against the wrong segment.
            if sim_path_idx >= len(self._path):
                break

            # Turn handling for corridors
            turn_s = None
            turn_idx = None
            if is_corridor:
                for ti in sorted(turn_indices):
                    if ti < len(self._path_s) and self._path_s[ti] >= s_nearest - self._accept_r:
                        turn_s = self._path_s[ti]
                        turn_idx = ti
                        break
                if turn_s is not None and s_nearest > turn_s:
                    s_nearest = turn_s

            # Lookahead
            s_limit = turn_s if turn_s is not None else total_s
            s_look = min(s_nearest + self._lookahead, s_limit)
            la_lat, la_lon = self._point_at_s(s_look)
            target_bearing = bearing_to(rlat, rlon, la_lat, la_lon)

            # Pivot state machine — stays in pivot mode until heading aligned
            if sim_pivoting:
                pivot_err = ((sim_pivot_brg - heading + 180) % 360) - 180
                if abs(pivot_err) > self._hdb:
                    steer_frac = max(-self._max_steer, min(self._max_steer, pivot_err / 45.0))
                    steer_ppm = int(1500 - steer_frac * 500)
                    _log(rlat, rlon, heading, sim_pivot_brg, pivot_err, cte,
                         steer_frac, steer_ppm, 1500, 0, 0, seg_idx, 'sim-pivot')
                    rover.update(1500, steer_ppm, self._max_speed, 0.6, dt)
                    sim_time += dt
                    continue
                # Pivot done — heading aligned
                sim_pivoting = False
                _log(rlat, rlon, heading, sim_pivot_brg, pivot_err, cte,
                     0, 1500, 1500, 0, 0, seg_idx, 'sim-pivot-done')
                sim_time += dt
                continue

            # Pivot trigger at corridor turn
            if turn_idx is not None:
                tp = self._path[turn_idx]
                direct_dist = haversine(rlat, rlon, tp.latitude, tp.longitude)
                if sim_pivot_min is None:
                    sim_pivot_min = direct_dist
                if direct_dist < sim_pivot_min:
                    sim_pivot_min = direct_dist
                passed = (sim_pivot_min < self._accept_r
                          and direct_dist > sim_pivot_min + 0.05)
                if passed:
                    # Enter pivot mode
                    turn_indices.discard(turn_idx)
                    sim_pivot_min = None
                    sim_path_idx = turn_idx + 1
                    # Compute spin target: bearing to point 2m into next corridor
                    nxt = turn_idx + 1
                    best_nxt = min(nxt, len(self._path) - 1)
                    while nxt < len(self._path):
                        d = haversine(tp.latitude, tp.longitude,
                                      self._path[nxt].latitude, self._path[nxt].longitude)
                        if d > 0.1:
                            best_nxt = nxt
                        if d >= 2.0:
                            best_nxt = nxt
                            break
                        nxt += 1
                    sim_pivot_brg = bearing_to(tp.latitude, tp.longitude,
                                               self._path[best_nxt].latitude,
                                               self._path[best_nxt].longitude)
                    sim_pivoting = True
                    rover.update(1500, 1500, self._max_speed, 0.6, dt)  # stop first
                    sim_time += dt
                    continue
            heading_err = ((target_bearing - heading + 180) % 360) - 180

            # Speed
            wp = self._path[min(seg_idx, len(self._path) - 1)]
            target_spd = wp.speed if wp.speed > 0 else self._max_speed
            cte_factor = max(0.0, 1.0 - abs_cte / max(self._stanley_cte_scale, 0.01))
            v_mps = max(self._min_speed, target_spd * cte_factor)

            # Decel approaching turn
            if turn_s is not None:
                dist_to_turn = turn_s - s_nearest
                approach = self._pivot_approach_dist
                if dist_to_turn < approach:
                    v_mps = max(self._min_speed, v_mps * (dist_to_turn / approach))

            # No general spin in sim — pivot code handles turn rotation,
            # Stanley handles mid-corridor heading corrections naturally.
            sim_spin_brg = None

            # Stanley steering
            stanley_ang = heading_err + math.degrees(
                math.atan2(self._stanley_k * cte, max(v_mps, self._stanley_softening)))
            stanley_ang = max(-90.0, min(90.0, stanley_ang))
            steer_frac = max(-self._max_steer, min(self._max_steer, stanley_ang / 45.0))

            throttle_ppm = int(1500 + (v_mps / self._max_speed) * 500)
            steer_ppm = int(1500 - steer_frac * 500)

            _log(rlat, rlon, heading, target_bearing, heading_err, cte,
                 steer_frac, steer_ppm, throttle_ppm, target_spd,
                 total_s - s_nearest, seg_idx, 'sim-corridor')

            rover.update(throttle_ppm, steer_ppm, self._max_speed, 0.6, dt)
            sim_time += dt

            if steps % 25 == 0:  # ~1 per sim-second
                sim_trace.append((rlat, rlon))

        if sim_diag_file is not None:
            sim_diag_file.close()

        avg_cte = cte_sum / max(steps, 1)
        result = {
            'ok': True,
            'max_cte': round(max_cte, 4),
            'avg_cte': round(avg_cte, 4),
            'steps': steps,
            'complete': s_nearest >= total_s - self._accept_r if steps > 0 else False,
            'sim_path': sim_trace,
        }
        self.get_logger().info(
            f'Preflight sim: {steps} steps, max_cte={max_cte:.3f}m, '
            f'avg_cte={avg_cte:.3f}m, complete={result["complete"]}')

        # Save sim trace to run directory
        if self._run_dir:
            try:
                import json as _json
                sim_path_file = os.path.join(self._run_dir, 'sim_preflight.json')
                with open(sim_path_file, 'w') as f:
                    _json.dump(result, f, separators=(',', ':'))
            except Exception:
                pass

        return result

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
        """
        Absolute heading change at waypoint _path[idx] (degrees, 0–180).
        Returns 0 for the last waypoint (no outgoing segment).
        Returns 0 when either adjacent segment is shorter than min_pivot_segment_m
        — bearing between nearly-coincident points is dominated by GPS noise and
        must not trigger pivot detection (common with closely-recorded waypoints).
        """
        if idx >= len(self._path) - 1:
            return 0.0
        # Outgoing segment must be long enough for reliable bearing
        out_len = haversine(self._path[idx].latitude,     self._path[idx].longitude,
                            self._path[idx + 1].latitude, self._path[idx + 1].longitude)
        if out_len < self._min_pivot_seg:
            return 0.0
        # Outgoing bearing: path[idx] → path[idx+1]
        hdg_out = bearing_to(self._path[idx].latitude,     self._path[idx].longitude,
                             self._path[idx + 1].latitude, self._path[idx + 1].longitude)
        # Incoming segment
        if idx == 0:
            if self._path_origin_lat is None:
                return 0.0
            in_len = haversine(self._path_origin_lat, self._path_origin_lon,
                               self._path[0].latitude, self._path[0].longitude)
            if in_len < self._min_pivot_seg:
                return 0.0
            hdg_in = bearing_to(self._path_origin_lat,      self._path_origin_lon,
                                self._path[0].latitude,     self._path[0].longitude)
        else:
            in_len = haversine(self._path[idx - 1].latitude, self._path[idx - 1].longitude,
                               self._path[idx].latitude,     self._path[idx].longitude)
            if in_len < self._min_pivot_seg:
                return 0.0
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

    def _past_waypoint(self, lat: float, lon: float) -> bool:
        """True if the rover has passed the current waypoint along the segment."""
        seg_k = self._path_idx
        if seg_k >= len(self._path):
            return False
        if seg_k == 0:
            if self._path_origin_lat is None:
                return False
            a_lat, a_lon = self._path_origin_lat, self._path_origin_lon
        else:
            a_lat = self._path[seg_k - 1].latitude
            a_lon = self._path[seg_k - 1].longitude
        b_lat = self._path[seg_k].latitude
        b_lon = self._path[seg_k].longitude

        mid_lat = math.radians((a_lat + b_lat) / 2)
        cos_lat = math.cos(mid_lat) or 1e-9
        m_lat, m_lon = 111_320.0, 111_320.0 * cos_lat
        seg_dy = (b_lat - a_lat) * m_lat
        seg_dx = (b_lon - a_lon) * m_lon
        seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy
        if seg_len_sq < 0.01:
            return False
        rv_dy = (lat - a_lat) * m_lat
        rv_dx = (lon - a_lon) * m_lon
        t = (rv_dx * seg_dx + rv_dy * seg_dy) / seg_len_sq
        return t >= 1.0

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

    # ── Waypoint advance ──────────────────────────────────────────────────────

    def _advance_path(self):
        """Publish waypoint-reached event and advance _path_idx."""
        if self._path_idx >= len(self._path):
            return
        wp = self._path[self._path_idx]
        # Only publish real waypoint arrivals — bypass points are transparent to GQC.
        if self._path_idx not in self._bypass_indices:
            m = Int32(); m.data = wp.seq
            self.wp_pub.publish(m)
        self._path_idx += 1
        self._pivot_closest_dist = float('inf')   # reset for next waypoint
        self._spin_target_brg    = None            # stale bearing from previous WP approach

        if self._path_idx >= len(self._path):
            if self._resource_state == 'going_to_base':
                # Base trip complete — build resume mission and wait for re-arm
                self._arrive_at_base()
            else:
                self.get_logger().info('Mission complete')
                m = Int32(); m.data = -1
                self.wp_pub.publish(m)
                self._publish_halt()
                self._armed = False
                a = Bool(); a.data = False
                self.armed_pub.publish(a)
        else:
            nxt = self._path[self._path_idx]
            if self._path_idx - 1 not in self._bypass_indices:
                self.get_logger().info(
                    f'Waypoint {wp.seq} reached — navigating to {nxt.seq}')

    # ── Resource management ──────────────────────────────────────────────────

    def _check_resources(self):
        """1 Hz: trigger base return if battery or tank falls below threshold."""
        if self._resource_state != 'normal':
            return
        if not self._armed or self._mode != 'AUTONOMOUS':
            return
        if not self._path or self._path_idx <= 0 or self._path_idx >= len(self._path):
            return

        # Priority: ROS2 param > station_update test injection > real sensor.
        # Separate _test_tank/_test_batt fields prevent _cb_sensors from
        # overwriting test values (was a race: sensor stub resets to None at 1 Hz).
        test_tank_param = self.get_parameter('test_tank_pct').value
        test_batt_param = self.get_parameter('test_batt_pct').value
        batt = (test_batt_param if test_batt_param >= 0
                else self._test_batt if self._test_batt is not None
                else self._battery_pct)
        tank = (test_tank_param if test_tank_param >= 0
                else self._test_tank if self._test_tank is not None
                else self._tank_pct)
        batt_low = self._battery_low_pct
        tank_low = self._tank_low_pct

        if batt is not None and batt < batt_low:
            self.get_logger().warn(f'[RESOURCE] Battery {batt:.0f}% < {batt_low:.0f}%')
            self._go_to_base('battery')
        elif tank is not None and tank < tank_low:
            self.get_logger().warn(f'[RESOURCE] Tank {tank:.0f}% < {tank_low:.0f}%')
            self._go_to_base('tank')

    def _go_to_base(self, reason: str):
        """Save mission state and navigate to recharge/water station."""
        self._resource_reason = reason

        if reason == 'battery':
            station_lat = self._recharge_lat
            station_lon = self._recharge_lon
            label = 'BATTERY LOW'
        else:
            station_lat = self._water_lat
            station_lon = self._water_lon
            label = 'TANK LOW'

        if station_lat == 0.0 and station_lon == 0.0:
            self.get_logger().error(
                f'[RESOURCE] {label} but station not configured')
            return

        # Save current mission for later resume
        self._saved_path_original = list(self._path_original)
        self._saved_wp_idx        = self._path_idx
        self._saved_bypass        = set(self._bypass_indices)
        self._saved_fence         = list(self._expanded_polygons)
        self._resource_state      = 'going_to_base'

        # Build 2-WP base trip: current position -> station
        rlat, rlon = self._center_pos()
        wp_here = MissionWaypoint()
        wp_here.seq               = 0
        wp_here.latitude          = rlat
        wp_here.longitude         = rlon
        wp_here.speed             = self._base_speed
        wp_here.acceptance_radius = self._base_accept
        wp_here.hold_secs         = 0.0

        wp_base = MissionWaypoint()
        wp_base.seq               = 1
        wp_base.latitude          = station_lat
        wp_base.longitude         = station_lon
        wp_base.speed             = self._base_speed
        wp_base.acceptance_radius = self._base_accept
        wp_base.hold_secs         = 0.0

        # Replace current path with base trip
        self._path.clear()
        self._path_s.clear()
        self._path_original.clear()
        self._bypass_indices.clear()

        self._path_origin_lat, self._path_origin_lon = rlat, rlon
        self._path.extend([wp_here, wp_base])
        self._path_original.extend([wp_here, wp_base])
        self._path_s = self._rebuild_path_s(self._path, rlat, rlon)
        self._path_idx = 0
        self._holding = False
        self._pivoting = False
        self._reroute_pending = False

        # Reroute base trip around obstacles (keep saved polygons active)
        self._expanded_polygons = list(self._saved_fence)
        if self._expanded_polygons:
            self._reroute_path()
            self.get_logger().info(
                f'[RESOURCE] Base trip rerouted around {len(self._expanded_polygons)} obstacle(s)')

        # Publish status signals
        m = Int32(); m.data = -2  # "going to base" signal
        self.wp_pub.publish(m)

        self.get_logger().warn(
            f'[RESOURCE] {label} — navigating to base '
            f'({station_lat:.6f},{station_lon:.6f}), '
            f'resume from path_idx={self._saved_wp_idx}')

        # Publish path version so GQC updates the map
        self._path_version += 1
        pv = Int32(); pv.data = self._path_version
        self.path_version_pub.publish(pv)
        self._publish_full_path()

    def _arrive_at_base(self):
        """Base trip complete. Build resume mission, disarm, wait for re-arm."""
        self._resource_state = 'at_base'
        reason_label = 'battery' if self._resource_reason == 'battery' else 'tank'

        # Build resume: base position -> remaining saved WPs
        rlat, rlon = self._center_pos()
        remaining = self._saved_path_original[self._saved_wp_idx:]
        if not remaining:
            self.get_logger().warn('[RESOURCE] No waypoints to resume — mission complete')
            m = Int32(); m.data = -1
            self.wp_pub.publish(m)
            self._publish_halt()
            self._resource_state = 'normal'
            return

        renumbered = []
        seq = 0
        # WP0 — base station (current position)
        wp_here = MissionWaypoint()
        wp_here.seq               = seq
        wp_here.latitude          = rlat
        wp_here.longitude         = rlon
        wp_here.speed             = remaining[0].speed
        wp_here.acceptance_radius = remaining[0].acceptance_radius
        wp_here.hold_secs         = 0.0
        renumbered.append(wp_here)
        seq += 1

        for orig in remaining:
            wp = MissionWaypoint()
            wp.seq               = seq
            wp.latitude          = orig.latitude
            wp.longitude         = orig.longitude
            wp.speed             = orig.speed
            wp.acceptance_radius = orig.acceptance_radius
            wp.hold_secs         = orig.hold_secs
            renumbered.append(wp)
            seq += 1

        # Install resume path (ready for when user re-arms)
        self._path.clear()
        self._path_s.clear()
        self._path_original.clear()
        self._bypass_indices.clear()

        self._path_origin_lat = rlat
        self._path_origin_lon = rlon
        self._path.extend(renumbered)
        self._path_original.extend(renumbered)
        self._path_s = self._rebuild_path_s(renumbered, rlat, rlon)
        self._path_idx = 0
        self._holding = False
        self._pivoting = False

        # Restore saved obstacles for resume rerouting
        if self._saved_fence:
            self._expanded_polygons = list(self._saved_fence)
            self._reroute_path()

        self.get_logger().info(
            f'[RESOURCE] At base ({reason_label}) — resume mission: '
            f'{len(renumbered)} WPs, re-arm to continue')

        # Signal mavlink_bridge: disarm + keep mission loaded
        m = Int32(); m.data = -3  # "at base" signal
        self.wp_pub.publish(m)
        self._publish_halt()

        # Publish path version so GQC updates map with resume route
        self._path_version += 1
        pv = Int32(); pv.data = self._path_version
        self.path_version_pub.publish(pv)
        self._publish_full_path()

    # ── MPC controller ────────────────────────────────────────────────────────

    def _mpc_steer(self, rlat: float, rlon: float,
                   heading_deg: float, s_nearest: float,
                   v_mps: float) -> float:
        """
        MPC lateral controller — receding horizon over self._mpc_N steps.

        Kinematic skid-steer model in flat-earth XY (X=east, Y=north):
          h  += -steer * 2*v/wb * dt   (positive steer_frac = right turn = -omega)
          x  += v * cos(h) * dt
          y  += v * sin(h) * dt

        Reference path sampled from _point_at_s at s_nearest + k*v*mpc_dt.
        Cost: CTE² × w_cte + heading_err² × w_hdg + steer² × w_str
              + Δsteer² × w_dstr  (over N steps).

        Returns steer_frac ∈ [−max_steer, +max_steer].
        Falls back to heading-error if scipy is unavailable.
        """
        if not _SCIPY_OK:
            la_lat, la_lon = self._point_at_s(s_nearest + self._lookahead)
            brg = bearing_to(rlat, rlon, la_lat, la_lon)
            err = ((brg - heading_deg + 180) % 360) - 180
            return max(-self._max_steer, min(self._max_steer, err / 45.0))

        N, dt, wb   = self._mpc_N, self._mpc_dt, self._mpc_wb
        max_s       = self._max_steer
        w_cte, w_h  = self._mpc_w_cte, self._mpc_w_hdg
        w_str, w_ds = self._mpc_w_str, self._mpc_w_dstr

        # Flat-earth conversion centred at rover position
        cos_lat = math.cos(math.radians(rlat)) or 1e-9
        m_lat   = 111_320.0
        m_lon   = 111_320.0 * cos_lat

        def to_xy(lat: float, lon: float) -> tuple[float, float]:
            return (lon - rlon) * m_lon, (lat - rlat) * m_lat

        # Clip horizon at the next pivot waypoint so MPC never sees past a sharp
        # turn — rover must reach and stop at the pivot before the post-turn
        # segment becomes visible.
        s_clip = float('inf')
        for i in range(self._path_idx, len(self._path)):
            if self._turn_angle_at(i) >= self._pivot_threshold:
                if i < len(self._path_s):
                    s_clip = self._path_s[i]
                break

        # When s_nearest approaches s_clip all reference points would collapse to
        # the same location, producing a degenerate ref_h.  Instead, once the
        # path arc hits s_clip, project reference points along the INCOMING
        # tangent at the pivot — rover tracks correctly to the turn and the
        # reference stays non-degenerate all the way up to arrival.
        if s_clip < float('inf'):
            eps = 0.5
            pt_before = self._point_at_s(max(0.0, s_clip - eps))
            pt_at     = self._point_at_s(s_clip)
            clip_x, clip_y = to_xy(pt_at[0], pt_at[1])
            bx, by = to_xy(pt_before[0], pt_before[1])
            dx = clip_x - bx; dy = clip_y - by
            norm = math.hypot(dx, dy) or 1.0
            tang_x = dx / norm; tang_y = dy / norm
        else:
            clip_x = clip_y = tang_x = tang_y = 0.0

        ref_x: list[float] = []
        ref_y: list[float] = []
        for ki in range(N + 1):
            s_ref = s_nearest + v_mps * ki * dt
            if s_clip < float('inf') and s_ref >= s_clip:
                beyond = s_ref - s_clip
                ref_x.append(clip_x + tang_x * beyond)
                ref_y.append(clip_y + tang_y * beyond)
            else:
                pt = self._point_at_s(s_ref)
                rx, ry = to_xy(pt[0], pt[1])
                ref_x.append(rx)
                ref_y.append(ry)

        # Reference headings in standard math angle (radians, east=0 CCW)
        h0 = math.radians(90.0 - heading_deg)   # rover heading → math angle
        ref_h: list[float] = []
        for ki in range(N):
            dx = ref_x[ki + 1] - ref_x[ki]
            dy = ref_y[ki + 1] - ref_y[ki]
            ref_h.append(math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-6 else h0)

        prev0 = self._mpc_prev_steers[0] if self._mpc_prev_steers else 0.0

        def cost_fn(steers) -> float:
            x, y, h = 0.0, 0.0, h0
            cost, prev = 0.0, prev0
            for ki in range(N):
                s  = float(steers[ki])
                h += -s * 2.0 * v_mps / wb * dt      # right turn = −omega
                x += v_mps * math.cos(h) * dt
                y += v_mps * math.sin(h) * dt
                rh  = ref_h[ki]
                cte = -(x - ref_x[ki]) * math.sin(rh) + (y - ref_y[ki]) * math.cos(rh)
                herr = (h - rh + math.pi) % (2 * math.pi) - math.pi
                cost += w_cte * cte * cte + w_h * herr * herr
                cost += w_str * s * s + w_ds * (s - prev) * (s - prev)
                prev  = s
            return cost

        # Warm start: shift left by 1, repeat last value
        if len(self._mpc_prev_steers) == N:
            x0 = self._mpc_prev_steers[1:] + [self._mpc_prev_steers[-1]]
        else:
            x0 = [0.0] * N

        try:
            result = _scipy_minimize(
                cost_fn, x0, method='L-BFGS-B',
                bounds=[(-max_s, max_s)] * N,
                options={'maxiter': 50, 'ftol': 1e-4})
            steers = list(result.x)
        except Exception:
            steers = x0

        self._mpc_prev_steers = steers
        return max(-max_s, min(max_s, float(steers[0])))

    # ── TTR controller ────────────────────────────────────────────────────────

    def _ttr_steer(self, flat: float, flon: float,
                   best_seg: int, dist_to_wp: float,
                   v_target: float,
                   heading_err: float = 0.0) -> tuple[float, float]:
        """
        TTR cascaded dual-PID straight-line controller.

        Port of Robot::NewGoStraightByPlanD():
          1. dis_output  = HightPid.compute(0, cte)
             cte > 0 = rover LEFT of route (same as _cte_to_seg convention)
          2. angle_output = AnglePid.compute(-dis_output, angle_diff)
             angle_diff = heading − target  (positive = heading right of route)
          3. angle_output > 0 → steer RIGHT (steer_frac > 0 in navigator PPM convention)

        Uses lookahead-based heading_err (from step()) instead of segment bearing.
        This gives TTR curve anticipation on short segments where heading ≈ seg_bearing.

        Returns (steer_frac, v_mps).
        """
        # Heading error: use lookahead-based heading_err from the control loop.
        # heading_err = target_bearing - heading (positive = need left turn).
        # TTR convention: angle_diff = heading - target (positive = heading right).
        # So angle_diff = -heading_err.
        angle_diff = -heading_err

        # CTE (positive = rover LEFT of route — matches _cte_to_seg)
        cte = self._cte_to_seg(flat, flon, best_seg)

        # Dual PID (TTR NewGoStraightByPlanD)
        dis_output   = self._ttr_hpid.compute(0.0, cte)
        angle_output = self._ttr_apid.compute(-dis_output, angle_diff)
        angle_output = max(-25.0, min(25.0, angle_output))

        # Map to steer_frac: angle_output > 0 → right steer → steer_frac > 0
        steer_frac = max(-self._max_steer,
                         min(self._max_steer, (angle_output / 25.0) * self._max_steer))

        # Speed: lineSpeedFactor reduces with CTE magnitude
        line_factor = max(0.4, min(0.8, 1.0 - abs(cte) / max(self._ttr_max_yaw, 0.1)))
        v = v_target * line_factor

        # Decelerate near waypoint
        if dist_to_wp < self._ttr_dece:
            v *= max(0.3, dist_to_wp / self._ttr_dece)

        return steer_frac, max(self._min_speed, v)

    # ── AFS controller ────────────────────────────────────────────────────────

    def _afs_step(self) -> tuple[int, int]:
        """
        AFS (Always Forward Strategy) control step.

        Three phases that form a state machine per waypoint:

          straight — Standard segment following with CTE-proportional throttle.
                     Throttle = max(min_speed, target_spd × (1 − |CTE| / afs_cte_scale_m)).
                     Steering uses Stanley (heading error + CTE feedforward).
                     If |CTE| ≥ afs_cte_alarm_m: halt and log an alarm.
                     Transitions to approach when rover enters afs_approach_dist_m
                     of a turn waypoint.

          approach — Rover drives toward the waypoint centre with direct-bearing
                     steering and min_speed throttle.  Tracks the minimum distance
                     seen so far (_afs_closest_dist).  When dist_to_wp exceeds
                     closest + 0.1 m (closest approach has passed): transitions
                     to spin.

          spin     — Proportional spin (neutral throttle) to align with the
                     outgoing segment bearing.  Calls _advance_path() and returns
                     to straight when heading error < heading_deadband.

        Turn detection uses pivot_threshold so that AFS approach triggers
        and pivot detection are consistent.

        Returns (throttle_ppm, steer_ppm).
        """
        wp               = self._path[self._path_idx]
        accept           = wp.acceptance_radius if wp.acceptance_radius > 0 else self._accept_r
        rlat, rlon       = self._center_pos()
        flat, flon       = self._front_pos()
        dist_to_wp       = haversine(rlat, rlon, wp.latitude, wp.longitude)
        is_bypass        = self._path_idx in self._bypass_indices

        # Turn detection — use _turn_angle_at() which works on the full path.
        turn_angle = self._turn_angle_at(self._path_idx)
        needs_turn = (turn_angle >= self._pivot_threshold
                      and self._path_idx < len(self._path) - 1)

        # ── Spin phase ─────────────────────────────────────────────────────
        if self._afs_phase == 'spin':
            pivot_err = ((self._afs_spin_hdg - self._heading + 180) % 360) - 180
            if abs(pivot_err) < self._hdb:
                self._afs_phase        = 'straight'
                self._afs_closest_dist = float('inf')
                self._ttr_apid.clear()
                self._ttr_hpid.clear()
                self.get_logger().info('AFS spin complete — advancing')
                self._advance_path()
                # Return halt — next tick _control_loop will call _afs_step()
                # fresh with updated path_idx.
                return PPM_CENTER, PPM_CENTER
            else:
                steer_frac = max(-self._max_steer, min(self._max_steer, pivot_err / 45.0))
                steer_ppm  = int(PPM_CENTER - steer_frac * 500)
                if steer_ppm != PPM_CENTER:
                    sign = 1 if steer_ppm > PPM_CENTER else -1
                    steer_ppm = PPM_CENTER + sign * max(abs(steer_ppm - PPM_CENTER), self._min_steer_delta)
                return PPM_CENTER, steer_ppm

        # ── Approach phase ─────────────────────────────────────────────────
        if self._afs_phase == 'approach':
            if dist_to_wp < self._afs_closest_dist:
                self._afs_closest_dist = dist_to_wp

            # Overshoot: distance started increasing from closest approach
            if dist_to_wp > self._afs_closest_dist + 0.1:
                self._afs_phase = 'spin'
                # Spin target: outgoing segment bearing from rover's current position
                nxt = self._path[self._path_idx + 1]
                nxt_lat, nxt_lon = nxt.latitude, nxt.longitude
                self._afs_spin_hdg = bearing_to(rlat, rlon, nxt_lat, nxt_lon)
                self.get_logger().info(
                    f'AFS wp{wp.seq} overshoot: closest={self._afs_closest_dist:.2f}m '
                    f'now={dist_to_wp:.2f}m — spinning to {self._afs_spin_hdg:.0f}°')
                return PPM_CENTER, PPM_CENTER  # halt one tick before spin begins

            # Drive toward wp: direct bearing, min_speed throttle
            target_brg  = bearing_to(rlat, rlon, wp.latitude, wp.longitude)
            heading_err = ((target_brg - self._heading + 180) % 360) - 180
            steer_frac  = max(-self._max_steer, min(self._max_steer, heading_err / 45.0))
            thr = max(self._min_throttle_ppm, int(PPM_CENTER + (self._min_speed / self._max_speed) * 500))
            steer_ppm = int(PPM_CENTER - steer_frac * 500)
            if steer_ppm != PPM_CENTER and abs(heading_err) > self._steer_coast:
                sign = 1 if steer_ppm > PPM_CENTER else -1
                steer_ppm = PPM_CENTER + sign * max(abs(steer_ppm - PPM_CENTER), self._min_steer_delta)
            return thr, steer_ppm

        # ── Straight phase ─────────────────────────────────────────────────
        s_nearest, best_seg = self._nearest_on_path(rlat, rlon)

        if needs_turn:
            # Turn waypoints: enter approach when within approach distance.
            # Never use proximity advance — overshoot detection handles advance.
            if dist_to_wp < self._afs_approach_dist:
                self._afs_phase        = 'approach'
                self._afs_closest_dist = dist_to_wp
                self._spin_target_brg  = None  # clear any pending align-spin freeze
                self.get_logger().info(
                    f'AFS entering approach for wp{wp.seq}: '
                    f'dist={dist_to_wp:.2f}m  turn={turn_angle:.0f}°')
                # Begin approach this tick: direct bearing, min_speed
                target_brg  = bearing_to(rlat, rlon, wp.latitude, wp.longitude)
                heading_err = ((target_brg - self._heading + 180) % 360) - 180
                steer_frac  = max(-self._max_steer, min(self._max_steer, heading_err / 45.0))
                thr = max(self._min_throttle_ppm, int(PPM_CENTER + (self._min_speed / self._max_speed) * 500))
                steer_ppm = int(PPM_CENTER - steer_frac * 500)
                if steer_ppm != PPM_CENTER and abs(heading_err) > self._steer_coast:
                    sign = 1 if steer_ppm > PPM_CENTER else -1
                    steer_ppm = PPM_CENTER + sign * max(abs(steer_ppm - PPM_CENTER), self._min_steer_delta)
                return thr, steer_ppm
        else:
            # Non-turn waypoint: standard proximity advance
            past_wp = (not is_bypass
                       and dist_to_wp < accept * 4.0
                       and self._past_waypoint(rlat, rlon))
            reached = dist_to_wp < accept or past_wp
            if reached:
                if not is_bypass and wp.hold_secs < 0.0:
                    # Waiting point — signal disarm, keep mission
                    self.get_logger().info(
                        f'AFS wp{wp.seq} reached — WAITING POINT')
                    self._publish_halt()
                    m = Int32(); m.data = -(wp.seq + 1000)
                    self.wp_pub.publish(m)
                    return PPM_CENTER, PPM_CENTER
                if not is_bypass and wp.hold_secs > 0.0:
                    self._holding  = True
                    self._hold_end = time.monotonic() + wp.hold_secs
                    self.get_logger().info(
                        f'AFS wp{wp.seq} reached — holding {wp.hold_secs:.1f}s')
                    self._publish_halt()
                    return PPM_CENTER, PPM_CENTER
                if not is_bypass:
                    self.get_logger().info(f'AFS wp{wp.seq} reached ({dist_to_wp:.2f}m)')
                self._advance_path()
                return PPM_CENTER, PPM_CENTER

        # CTE (front antenna projected onto current segment)
        cte_seg = self._path_idx
        cte     = 0.0 if is_bypass else self._cte_to_seg(flat, flon, cte_seg)

        # XTE telemetry
        xte_msg      = Float32()
        xte_msg.data = abs(cte)
        self.xte_pub.publish(xte_msg)

        # CTE alarm — rover has drifted too far from the planned line
        if abs(cte) >= self._afs_cte_alarm:
            self.get_logger().error(
                f'AFS ALARM: CTE {cte:.3f}m >= {self._afs_cte_alarm:.1f}m — halting rover')
            self._publish_halt()
            return PPM_CENTER, PPM_CENTER

        # Throttle: inversely proportional to CTE, guaranteed ≥ min_speed and min_throttle_ppm
        target_spd   = wp.speed if wp.speed > 0 else self._max_speed
        cte_factor   = max(0.0, 1.0 - abs(cte) / max(self._afs_cte_scale, 0.01))
        v_mps        = max(self._min_speed, target_spd * cte_factor)
        throttle_ppm = max(self._min_throttle_ppm, int(PPM_CENTER + (v_mps / self._max_speed) * 500))

        # Lookahead
        if is_bypass:
            la_lat, la_lon = wp.latitude, wp.longitude
        else:
            la_lat, la_lon = self._point_at_s(s_nearest + self._lookahead)

        target_bearing = bearing_to(rlat, rlon, la_lat, la_lon)

        if self._spin_target_brg is not None:
            target_bearing = self._spin_target_brg

        heading_err = ((target_bearing - self._heading + 180) % 360) - 180

        # Large heading error → align spin (neutral throttle — shared with all algos)
        if abs(heading_err) > self._align_thresh:
            if self._spin_target_brg is None:
                self._spin_target_brg = target_bearing
            steer_frac = max(-self._max_steer, min(self._max_steer, heading_err / 45.0))
            self._ttr_apid.clear()
            self._ttr_hpid.clear()
            steer_ppm  = int(PPM_CENTER - steer_frac * 500)
            if steer_ppm != PPM_CENTER:
                sign = 1 if steer_ppm > PPM_CENTER else -1
                steer_ppm = PPM_CENTER + sign * max(abs(steer_ppm - PPM_CENTER), self._min_steer_delta)
            return PPM_CENTER, steer_ppm

        if self._spin_target_brg is not None and abs(heading_err) < self._hdb:
            self._spin_target_brg = None

        # Stanley steering (AFS uses Stanley for the path-following component)
        stanley_ang = heading_err + math.degrees(
            math.atan2(self._stanley_k * cte, max(v_mps, self._stanley_softening)))
        stanley_ang = max(-90.0, min(90.0, stanley_ang))
        steer_frac  = max(-self._max_steer, min(self._max_steer, stanley_ang / 45.0))

        if self._diag_writer is not None:
            self._diag_writer.writerow([
                round(time.time(), 4),
                round(rlat, 8), round(rlon, 8),
                round(self._heading, 2),
                round(target_bearing, 2),
                round(heading_err, 2),
                round(cte, 4),
                round(steer_frac, 4),
                int(PPM_CENTER - steer_frac * 500), throttle_ppm,
                round(target_spd, 3),
                round(dist_to_wp, 3),
                self._path_idx,
                'afs-straight',
                self._fix.status.status if self._fix else -1,
                round(self._hacc_mm, 1),
            ])
            self._diag_file.flush()

        return throttle_ppm, int(PPM_CENTER - steer_frac * 500)

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self):
        if self._mode != 'AUTONOMOUS' or not self._armed or self._paused:
            return
        if self._reroute_pending:
            self._publish_halt()
            return
        if self._resource_state == 'at_base':
            self._publish_halt()
            return
        if not self._path:
            return

        # Corridor mode: simpler control loop (no waypoint advance)
        if self._corridor_mode:
            self._control_loop_corridor()
            return

        if self._path_idx >= len(self._path):
            return
        if self._fix is None or (time.time() - self._fix_time) > self._gps_timeout:
            self.get_logger().warn('GPS stale — halting')
            self._publish_halt()
            return

        # ── GPS accuracy alarm ─────────────────────────────────────────────────
        if (self._gps_acc_alarm > 0 and self._hacc_mm > 0
                and self._hacc_mm > self._gps_acc_alarm):
            self.get_logger().warn(
                f'GPS accuracy degraded: hAcc={self._hacc_mm:.0f} mm '
                f'> alarm={self._gps_acc_alarm:.0f} mm — halting')
            self._publish_halt()
            return

        # ── Inter-rover proximity safety ──────────────────────────────────────
        if self._peer_ns:
            self._compute_proximity()
            if self._prox_level in ('halt', 'estop'):
                self._publish_halt()
                return

        # ── Lazy path_origin — synthesise if GPS was not ready at mission upload ─
        # If seq=0 arrived before the first GPS fix, _path_origin_lat is None.
        # This breaks _past_waypoint() and _nearest_on_path() for segment 0,
        # preventing wp0 from ever being accepted via arc-length advance.
        # Fix: place a synthetic origin 'lookahead' metres behind wp0 on the
        # reverse of the outgoing segment bearing.  This guarantees _past_waypoint
        # returns True when the rover is already on the far side of wp0 (e.g.
        # rover was placed at or past wp0 before the mission was uploaded).
        if self._path_idx == 0 and self._path_origin_lat is None and self._path:
            first_wp = self._path[0]
            if len(self._path) > 1:
                out_brg = bearing_to(first_wp.latitude, first_wp.longitude,
                                     self._path[1].latitude, self._path[1].longitude)
            else:
                out_brg = self._heading
            rev_brg = (out_brg + 180.0) % 360.0
            step_m  = self._lookahead
            cos_lat = math.cos(math.radians(first_wp.latitude)) or 1e-9
            self._path_origin_lat = (first_wp.latitude
                                     + step_m * math.cos(math.radians(rev_brg)) / 111_320.0)
            self._path_origin_lon = (first_wp.longitude
                                     + step_m * math.sin(math.radians(rev_brg)) / (111_320.0 * cos_lat))
            self._path_s = self._rebuild_path_s(
                self._path, self._path_origin_lat, self._path_origin_lon)
            self.get_logger().info(
                f'Path origin synthesised (GPS not ready at upload): '
                f'{self._path_origin_lat:.7f},{self._path_origin_lon:.7f} '
                f'— {step_m:.1f} m behind wp0 on bearing {rev_brg:.0f}°')
            if self._expanded_polygons:
                self._reroute_path()

        # ── wp0 degenerate-segment skip ──────────────────────────────────────
        # When path_origin ≈ wp0 (e.g. resume mission where both were captured
        # at the base station), the origin→wp0 segment has near-zero length and
        # steering degenerates even if the rover drifted a few metres via GPS.
        # Skip wp0 if (a) the segment itself is short OR (b) the rover is close.
        if self._path_idx == 0 and len(self._path) > 1:
            wp0 = self._path[0]
            ar = wp0.acceptance_radius if wp0.acceptance_radius > 0 else self._accept_r
            # Check segment length (origin→wp0)
            seg_len = self._path_s[0] if self._path_s else 0.0
            # Check rover distance to wp0
            rlat, rlon = self._center_pos()
            d = haversine(rlat, rlon, wp0.latitude, wp0.longitude)
            if seg_len < ar or d < ar:
                self.get_logger().info(
                    f'wp0 skip (seq={wp0.seq}, seg_len={seg_len:.2f}m, '
                    f'dist={d:.2f}m, accept={ar:.2f}m) — advancing')
                self._advance_path()
                return

        # ── Hold at waypoint ─────────────────────────────────────────────────
        if self._holding:
            self._publish_halt()
            if time.monotonic() >= self._hold_end:
                self._holding = False
                self.get_logger().info('Hold complete — advancing')
                self._advance_path()
            return

        # ── AFS algorithm ────────────────────────────────────────────────────
        if self._algo == 'afs':
            thr, steer = self._afs_step()
            self._publish_cmd(thr, steer)
            return

        # ── Pivot turn ───────────────────────────────────────────────────────
        # Rover has reached a sharp-turn waypoint and is spinning in place to
        # align with the outgoing segment before continuing.
        if self._pivoting:
            pivot_err = ((self._pivot_target_hdg - self._heading + 180) % 360) - 180
            if abs(pivot_err) < self._hdb:
                self._pivoting = False
                self._ttr_apid.clear(); self._ttr_hpid.clear()
                self.get_logger().info('Pivot complete — continuing')
                self._advance_path()
            else:
                # Proportional spin: full speed above 45°, linear ramp below.
                # Always apply floor during spin — no coast zone. Throttle is neutral
                # so there is no forward momentum; overshoot risk is negligible and the
                # floor must be active right up to the heading_deadband exit condition.
                steer_frac = max(-self._max_steer,
                                 min(self._max_steer, pivot_err / 45.0))
                steer_ppm  = int(PPM_CENTER - steer_frac * 500)
                if steer_ppm != PPM_CENTER:
                    sign = 1 if steer_ppm > PPM_CENTER else -1
                    steer_ppm = PPM_CENTER + sign * max(abs(steer_ppm - PPM_CENTER),
                                                        self._min_steer_delta)
                self._publish_cmd(PPM_CENTER, steer_ppm)
            return

        wp          = self._path[self._path_idx]
        accept      = wp.acceptance_radius if wp.acceptance_radius > 0 else self._accept_r
        rlat, rlon  = self._center_pos()
        flat, flon  = self._front_pos()

        is_bypass   = self._path_idx in self._bypass_indices

        # Pivot detection — _turn_angle_at() works on the full path.
        turn_angle  = self._turn_angle_at(self._path_idx)
        needs_pivot = (turn_angle >= self._pivot_threshold
                       and self._path_idx < len(self._path) - 1)

        # ── Full-path nearest-point projection ───────────────────────────────
        s_nearest, best_seg = self._nearest_on_path(rlat, rlon)
        wp_s                = self._path_s[self._path_idx]
        dist_to_wp          = haversine(rlat, rlon, wp.latitude, wp.longitude)

        # ── Waypoint advance ─────────────────────────────────────────────────
        # Track closest approach to pivot waypoint so we can detect the moment
        # the rover arcs past it without entering accept_r (Option C).
        if needs_pivot:
            if dist_to_wp < self._pivot_closest_dist:
                self._pivot_closest_dist = dist_to_wp

        # Closest-approach overshoot: rover entered the approach zone (was within
        # pivot_approach_dist), reached its nearest point, and has since moved at
        # least accept metres further away — the arc has carried it past the turn
        # point even though it never entered accept_r.
        pivot_overshot = (needs_pivot
                          and self._pivot_closest_dist < self._pivot_approach_dist
                          and dist_to_wp > self._pivot_closest_dist + accept)

        # For normal waypoints: proximity OR segment overshoot (rover passed
        # the waypoint along the segment direction) triggers advance.
        # Proximity guard: only fire when rover is already close — prevents
        # TTR/MPC lookahead corner-cutting from cascading waypoint advances on
        # dense missions (t>=1 while still far away from the waypoint).
        # During align-spin, skip geometric advance (arc-length/past-waypoint)
        # which gives false positives on zigzag missions where the next WP is
        # spatially close but behind.  Distance advance (dist < accept) is always
        # allowed — if the rover is physically at the WP, it should advance.
        in_spin = self._spin_target_brg is not None
        past_wp = (not needs_pivot and not is_bypass and not in_spin
                   and dist_to_wp < accept * 4.0
                   and self._past_waypoint(rlat, rlon))
        reached = dist_to_wp < accept or past_wp or pivot_overshot

        if reached:
            if not is_bypass and wp.hold_secs < 0.0:
                # Waiting point — signal disarm, keep mission
                self.get_logger().info(
                    f'Waypoint {wp.seq} reached — WAITING POINT')
                self._publish_halt()
                m = Int32(); m.data = -(wp.seq + 1000)
                self.wp_pub.publish(m)
                return
            if not is_bypass and wp.hold_secs > 0.0:
                self.get_logger().info(
                    f'Waypoint {wp.seq} reached — holding {wp.hold_secs:.1f} s')
                self._holding  = True
                self._hold_end = time.monotonic() + wp.hold_secs
                self._publish_halt()
            elif needs_pivot:
                # Bearing from rover's ACTUAL position to the next WP (not from the
                # waypoint itself — the rover may have triggered "reached" up to
                # accept_r metres short, so the bearing from wp would be wrong).
                nxt = self._path[self._path_idx + 1]
                nxt_lat, nxt_lon = nxt.latitude, nxt.longitude
                self._pivot_target_hdg = bearing_to(rlat, rlon, nxt_lat, nxt_lon)
                self._pivoting        = True
                self._mpc_prev_steers = []   # warm-start stale after stop-and-spin
                self._ttr_apid.clear(); self._ttr_hpid.clear()
                trigger = 'overshoot' if pivot_overshot else 'proximity'
                self.get_logger().info(
                    f'Waypoint {wp.seq} reached ({trigger}, '
                    f'closest={self._pivot_closest_dist:.2f}m) — '
                    f'pivot {turn_angle:.0f}° to {self._pivot_target_hdg:.0f}°')
                self._publish_halt()
            else:
                if not is_bypass:
                    self.get_logger().info(f'Waypoint {wp.seq} reached ({dist_to_wp:.2f} m)')
                self._advance_path()
            return

        # ── Lookahead and speed ───────────────────────────────────────────────
        # Precision approach: when within pivot_approach_dist of a sharp-turn
        # waypoint, aim directly at the waypoint and slow to min_speed so the
        # rover arrives precisely instead of cutting the corner.
        target_spd = wp.speed if wp.speed > 0 else self._max_speed

        if needs_pivot:
            # Always aim directly at the pivot waypoint — never use arc-length
            # projection past it.  _nearest_on_path can snap ahead to the
            # post-turn segment when the rover is close to the pivot, projecting
            # the lookahead into the outgoing direction and triggering a premature
            # ~180° spin before arrival.
            la_lat = wp.latitude
            la_lon = wp.longitude
            if dist_to_wp < self._pivot_approach_dist:
                # Scale speed linearly from target_spd down to min_speed
                approach_t = dist_to_wp / self._pivot_approach_dist
                target_spd = max(self._min_speed, target_spd * approach_t)
        elif is_bypass:
            # Bypass waypoints: steer directly to the waypoint.
            # _nearest_on_path can snap to later original-route segments (they are
            # geometrically closer while the rover is still on the straight original
            # line), giving a wrong lookahead that makes the rover ignore the detour.
            # Direct-to-waypoint steering forces the rover to physically follow each
            # bypass point.  CTE is zeroed — heading error alone is sufficient.
            la_lat, la_lon = wp.latitude, wp.longitude
        else:
            # Clamp lookahead to not project past current WP when the next
            # segment changes direction — prevents the projected point from
            # swinging into a different segment and triggering spin oscillation.
            # (pivot detection can miss this when the origin→WP0 segment is
            # too short for reliable bearing, i.e. in_len < min_pivot_seg.)
            s_look = min(s_nearest + self._lookahead, wp_s)
            la_lat, la_lon = self._point_at_s(s_look)

        # Periodic log (every 5 s) — helps verify rover is tracking rerouted path
        self._log_tick += 1
        if self._log_tick % 125 == 1:
            self.get_logger().info(
                f'NAV path_idx={self._path_idx}/{len(self._path)} '
                f'bypass_idx={sorted(self._bypass_indices)} '
                f'wp_target={"BYPASS" if is_bypass else "orig"} '
                f'lookahead=({la_lat:.7f},{la_lon:.7f}) '
                f'rover=({rlat:.7f},{rlon:.7f})')

        target_bearing = bearing_to(rlat, rlon, la_lat, la_lon)

        # Freeze target bearing during spin so that the GPS antenna sweeping an
        # arc as the rover rotates (antenna is not at the physical rotation centre)
        # does not cause target_bearing to oscillate and produce zig-zag steer.
        # _spin_target_brg is set on the first spin tick and cleared on exit.
        if self._spin_target_brg is not None:
            target_bearing = self._spin_target_brg

        heading_err    = ((target_bearing - self._heading + 180) % 360) - 180

        # ── CTE: front antenna projected onto current target segment ─────────
        # Use _path_idx (current target segment) — not best_seg.  On near-straight
        # paths _nearest_on_path can snap best_seg to a far-ahead collinear segment,
        # destabilising the PID (especially TTR's cascaded dual-PID).
        # Bypass waypoints: zero CTE — heading error to the bypass point is enough.
        cte_seg = self._path_idx
        cte = 0.0 if is_bypass else self._cte_to_seg(flat, flon, cte_seg)

        # XTE telemetry
        xte_msg      = Float32()
        xte_msg.data = abs(cte)
        self.xte_pub.publish(xte_msg)

        # CTE safety alarm removed — user confirms mission by arming twice

        if abs(heading_err) > self._align_thresh:
            # Large error — spin in place (same for all algorithms).
            # Freeze target bearing on first spin tick so GPS antenna arc
            # (rover not rotating around antenna position) doesn't cause
            # heading_err to oscillate and produce zig-zag steering.
            if self._spin_target_brg is None:
                self._spin_target_brg = target_bearing
            steer_frac   = max(-self._max_steer, min(self._max_steer, heading_err / 45.0))
            steer_ppm    = int(PPM_CENTER - steer_frac * 500)
            # Always apply floor during align-spin — no coast zone (zero throttle,
            # negligible overshoot risk; floor must be active up to heading_deadband).
            if steer_ppm != PPM_CENTER:
                sign = 1 if steer_ppm > PPM_CENTER else -1
                steer_ppm = PPM_CENTER + sign * max(abs(steer_ppm - PPM_CENTER),
                                                    self._min_steer_delta)
            throttle_ppm      = PPM_CENTER
            self._mpc_prev_steers = []   # heading jump → stale warm start
            self._ttr_apid.clear(); self._ttr_hpid.clear()
        else:
            # Release frozen bearing only once within heading deadband (3°).
            # Releasing at align_thresh (60°) means a drifted GPS centre on the next
            # tick can produce a fresh bearing that triggers another full spin —
            # the multi-spin oscillation seen at mission start.
            if self._spin_target_brg is not None and abs(heading_err) < self._hdb:
                self._spin_target_brg = None
            # CTE-proportional speed reduction: slow down as rover drifts from path
            cte_factor   = max(0.0, 1.0 - abs(cte) / max(self._stanley_cte_scale, 0.01))
            v_mps        = max(self._min_speed, target_spd * cte_factor)
            throttle_ppm = int(PPM_CENTER + (v_mps / self._max_speed) * 500)

            if self._algo == 'ttr':
                # TTR dual-PID: CTE correction feeds heading correction
                steer_frac, ttr_v = self._ttr_steer(
                    flat, flon, cte_seg, dist_to_wp, v_mps, heading_err)
                throttle_ppm = int(PPM_CENTER + (ttr_v / self._max_speed) * 500)
            elif self._algo == 'mpc' and not is_bypass:
                # MPC active for all non-bypass segments — including pivot approach.
                steer_frac = self._mpc_steer(
                    rlat, rlon, self._heading, s_nearest, v_mps)
            else:
                # Stanley controller: δ = θ_e + arctan(k · e_cte / (v + ε))
                stanley_ang = heading_err + math.degrees(
                    math.atan2(self._stanley_k * cte,
                               max(v_mps, self._stanley_softening)))
                stanley_ang = max(-90.0, min(90.0, stanley_ang))
                steer_frac  = max(-self._max_steer,
                                  min(self._max_steer, stanley_ang / 45.0))
            steer_ppm = int(PPM_CENTER - steer_frac * 500)
            # Apply steer floor so motors overcome stiction during active corrections.
            # Same floor as spin modes — coast zone exempts small errors near straight-ahead
            # so we don't force unnecessary weave during fine tracking.
            if steer_ppm != PPM_CENTER and abs(heading_err) > self._steer_coast:
                sign = 1 if steer_ppm > PPM_CENTER else -1
                steer_ppm = PPM_CENTER + sign * max(abs(steer_ppm - PPM_CENTER),
                                                    self._min_steer_delta)

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
                round(dist_to_wp, 3),
                self._path_idx,
                self._algo,
                self._fix.status.status if self._fix else -1,
                round(self._hacc_mm, 1),
            ])
            self._diag_file.flush()

        self._publish_cmd(throttle_ppm, steer_ppm)

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
