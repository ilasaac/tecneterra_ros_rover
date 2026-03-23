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

import json
import math
import time

try:
    from scipy.optimize import minimize as _scipy_minimize
    _SCIPY_OK = True
except ImportError:
    _scipy_minimize = None
    _SCIPY_OK = False

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
    return 2 * EARTH_R * math.asin(math.sqrt(max(0.0, a)))


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
        self.declare_parameter('align_threshold',           30.0)
        self.declare_parameter('stanley_k',                 1.0)
        self.declare_parameter('stanley_softening',         0.3)
        # Pivot-turn parameters:
        #   pivot_threshold    — turn angle (degrees) above which an in-place pivot
        #                        is performed instead of curving through the waypoint.
        #   pivot_approach_dist — distance (metres) at which the rover switches from
        #                        path-lookahead to direct-to-waypoint aiming and begins
        #                        slowing to min_speed, ensuring it arrives precisely.
        self.declare_parameter('pivot_threshold',           60.0)
        self.declare_parameter('pivot_approach_dist',        2.0)
        # Obstacle avoidance:
        #   rover_width_m       — physical rover width; half of this is the minimum
        #                         clearance needed to keep the rover body off obstacles.
        #   obstacle_clearance_m — additional safety buffer on top of rover half-width.
        #   effective clearance  = rover_width_m/2 + obstacle_clearance_m
        self.declare_parameter('rover_width_m',              1.4)
        self.declare_parameter('obstacle_clearance_m',       0.5)
        # Control algorithm — 'stanley' (default) or 'mpc'
        self.declare_parameter('control_algorithm',          'mpc')
        # MPC parameters (only used when control_algorithm == 'mpc')
        self.declare_parameter('mpc_horizon',                10)
        self.declare_parameter('mpc_dt',                     0.2)
        self.declare_parameter('mpc_w_cte',                  2.0)
        self.declare_parameter('mpc_w_heading',              1.0)
        self.declare_parameter('mpc_w_steer',                0.1)
        self.declare_parameter('mpc_w_dsteer',               0.05)
        self.declare_parameter('wheelbase_m',                0.6)

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
        self._pivot_threshold     = self.get_parameter('pivot_threshold').value
        self._pivot_approach_dist = self.get_parameter('pivot_approach_dist').value
        rover_width               = self.get_parameter('rover_width_m').value
        self._clearance           = (rover_width / 2.0
                                     + self.get_parameter('obstacle_clearance_m').value)
        self._algo        = self.get_parameter('control_algorithm').value
        self._mpc_N       = int(self.get_parameter('mpc_horizon').value)
        self._mpc_dt      = self.get_parameter('mpc_dt').value
        self._mpc_w_cte   = self.get_parameter('mpc_w_cte').value
        self._mpc_w_hdg   = self.get_parameter('mpc_w_heading').value
        self._mpc_w_str   = self.get_parameter('mpc_w_steer').value
        self._mpc_w_dstr  = self.get_parameter('mpc_w_dsteer').value
        self._mpc_wb      = max(self.get_parameter('wheelbase_m').value, 0.1)

        # ── State ────────────────────────────────────────────────────────────
        self._fix:       NavSatFix | None = None
        self._fix_front: NavSatFix | None = None   # front (secondary) antenna
        self._fix_time:  float            = 0.0
        self._heading:   float            = 0.0
        self._mode:      str              = 'MANUAL'
        self._armed:     bool             = False
        self._paused:    bool             = False

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

        # Hold state — rover waits at a waypoint for hold_secs before advancing.
        self._holding:  bool  = False
        self._hold_end: float = 0.0

        # Pivot-turn state — rover spins in place to the outgoing heading before
        # advancing past a sharp-turn waypoint.
        self._pivoting:         bool  = False
        self._pivot_target_hdg: float = 0.0

        # Chunk-based mission: _path holds only the current chunk (up to the next
        # pivot point inclusive).  Remaining chunks are queued here as
        # (wps, origin_lat, origin_lon, bypass_indices, pivot_target_wp_or_None).
        # pivot_target_wp: first waypoint of the next chunk — needed to compute
        # the pivot heading when the pivot is the last wp in the current chunk.
        self._pending_path_chunks: list = []
        self._chunk_end_pivot_target: 'MissionWaypoint | None' = None

        self._dt = 1.0 / self.get_parameter('control_rate').value

        # MPC warm-start: previous horizon steer sequence (N steer_frac values)
        self._mpc_prev_steers: list[float] = []

        # Servo state (PPM CH5-CH8) re-published in every cmd_override.
        self._servo_ch: list[int] = [PPM_CENTER] * 4

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(NavSatFix,       'fix',           self._cb_fix,          10)
        self.create_subscription(NavSatFix,       'fix_front',     self._cb_fix_front,    10)
        self.create_subscription(Float32,         'heading',       self._cb_heading,      10)
        self.create_subscription(String,          'mode',          self._cb_mode,         10)
        self.create_subscription(Bool,            'armed',         self._cb_armed,        10)
        self.create_subscription(MissionWaypoint, 'mission',       self._cb_mission,      10)
        self.create_subscription(RCInput,         'servo_state',   self._cb_servo_state,  10)
        self.create_subscription(Bool,            'mission_clear', self._cb_mission_clear, 10)
        self.create_subscription(String,          'mission_fence', self._cb_mission_fence, 10)

        # ── Publishers ───────────────────────────────────────────────────────
        self.cmd_pub          = self.create_publisher(RCInput,  'cmd_override',    10)
        self.wp_pub           = self.create_publisher(Int32,    'wp_active',       10)
        self.xte_pub          = self.create_publisher(Float32,  'xte',             10)
        self.rerouted_pub     = self.create_publisher(String,   'rerouted_path',   10)

        # ── Services ─────────────────────────────────────────────────────────
        self.create_service(Trigger, 'pause_mission',  self._svc_pause)
        self.create_service(Trigger, 'resume_mission', self._svc_resume)

        # ── Control loop ─────────────────────────────────────────────────────
        rate = self.get_parameter('control_rate').value
        self.create_timer(1.0 / rate, self._control_loop)

        algo_info = self._algo
        if self._algo == 'mpc' and not _SCIPY_OK:
            algo_info = 'mpc→stanley(scipy missing)'
        self.get_logger().info(
            f'Navigator ready — algorithm={algo_info}, '
            f'full-path tracking + obstacle avoidance')

    # ── Sensor callbacks ──────────────────────────────────────────────────────

    def _cb_fix(self, msg: NavSatFix):
        self._fix      = msg
        self._fix_time = time.time()

    def _cb_fix_front(self, msg: NavSatFix):
        self._fix_front = msg

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

    def _cb_mode(self, msg: String):
        self._mode = msg.data

    def _cb_armed(self, msg: Bool):
        self._armed = msg.data

    def _cb_servo_state(self, msg: RCInput):
        for i in range(4):
            val = msg.channels[i + 4] if i + 4 < len(msg.channels) else 0
            if val != 0:
                self._servo_ch[i] = val

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
        self._mpc_prev_steers        = []
        self._pending_path_chunks    = []
        self._chunk_end_pivot_target = None
        clr_msg = String(); clr_msg.data = '[]'
        self.rerouted_pub.publish(clr_msg)
        self._publish_halt()
        self.get_logger().info('Mission cleared — path reset')

    def _cb_mission(self, msg: MissionWaypoint):
        """Append waypoint to path and update cumulative arc-lengths."""
        if msg.seq == 0:
            # New mission — discard any previous path
            self._path.clear()
            self._path_s.clear()
            self._path_original.clear()
            self._bypass_indices.clear()
            self._path_idx        = 0
            self._holding         = False
            self._pivoting               = False
            self._mpc_prev_steers        = []
            self._pending_path_chunks    = []
            self._chunk_end_pivot_target = None
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
            f'Waypoint {msg.seq} loaded '
            f'({len(self._path)} total, path length {self._path_s[-1]:.1f} m)')

        # If expanded obstacle polygons are already available (fence arrived before
        # the last waypoint), reroute incrementally so the final path is correct.
        if self._expanded_polygons:
            self._reroute_path()

    def _cb_mission_fence(self, msg: String):
        """Parse JSON fence polygons and reroute the current mission path."""
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

        # Always reroute when we have a path — with no obstacles this just publishes
        # the plain mission path to GQC for the map overlay.
        if self._path_original:
            self._reroute_path()
        elif self._obstacle_polygons:
            self.get_logger().warn(
                'Obstacle fence arrived before waypoints — reroute deferred '
                '(will trigger on next waypoint via _cb_mission)')
        else:
            self.get_logger().info('Obstacle fence: empty (no polygons)')

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

    def _reroute_path(self):
        """
        Rebuild _path and _path_s from _path_original, inserting bypass waypoints
        around any expanded obstacle polygon that a path segment intersects.

        Handles polygons that span across a waypoint (entry and exit on different
        segments) via cross-segment scan-ahead.

        Skipped if the rover is currently navigating autonomously (path_idx > 0
        while armed and in AUTONOMOUS mode) to avoid disrupting an active mission.
        """
        if self._mode == 'AUTONOMOUS' and self._armed and self._path_idx > 0:
            self.get_logger().warn('Obstacle reroute skipped — rover is navigating')
            return

        if not self._path_original:
            return

        new_wps: list[MissionWaypoint] = []
        new_bypass_indices: set[int]   = set()

        def make_bypass_wp(lat: float, lon: float,
                           ref_wp: MissionWaypoint) -> MissionWaypoint:
            bp = MissionWaypoint()
            bp.seq               = 0          # sentinel — filtered in _advance_path
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

        # Publish rerouted path so GQC can display it as a separate overlay.
        # Format: JSON array of [lat, lon, is_bypass] per waypoint.
        path_data = [
            [round(wp.latitude, 7), round(wp.longitude, 7),
             1 if k in new_bypass_indices else 0]
            for k, wp in enumerate(new_wps)
        ]
        rp_msg = String(); rp_msg.data = json.dumps(path_data, separators=(',', ':'))
        self.rerouted_pub.publish(rp_msg)

        n_bypass = len(new_bypass_indices)
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

        # Split the complete rerouted path into sub-mission chunks at pivot turns.
        # _path is replaced with the first chunk; the rest queue in _pending_path_chunks.
        self._split_path_into_chunks()

    def _split_path_into_chunks(self):
        """Split self._path at pivot turns into sequential sub-missions.

        After the split, self._path contains only the first chunk.  Each
        subsequent chunk is stored in self._pending_path_chunks as
        (wps, origin_lat, origin_lon, bypass_indices_set, pivot_target_wp_or_None).

        The pivot waypoint is the LAST element of its chunk.  After the rover
        reaches it and completes the spin, _advance_path() increments _path_idx
        to len(_path), which triggers _load_next_path_chunk() to load the next
        chunk with the pivot point as the new origin.
        """
        full_path     = self._path
        full_bypass   = self._bypass_indices
        origin_lat    = self._path_origin_lat
        origin_lon    = self._path_origin_lon

        if not full_path:
            return

        # Build chunks: scan for pivot waypoints in the full path.
        chunks: list[tuple[list, set, float, float]] = []   # (wps, bypass, orig_lat, orig_lon)
        current_wps:    list = []
        current_bypass: set  = set()
        chunk_origin_lat = origin_lat
        chunk_origin_lon = origin_lon

        n = len(full_path)
        for i, wp in enumerate(full_path):
            local_idx = len(current_wps)
            current_wps.append(wp)
            if i in full_bypass:
                current_bypass.add(local_idx)

            # Compute turn angle at wp[i] if not the last waypoint
            if i < n - 1:
                if i == 0 and origin_lat is not None:
                    in_brg = bearing_to(origin_lat, origin_lon,
                                        full_path[0].latitude, full_path[0].longitude)
                elif i == 0:
                    in_brg = bearing_to(full_path[0].latitude, full_path[0].longitude,
                                        full_path[1].latitude, full_path[1].longitude)
                else:
                    in_brg = bearing_to(full_path[i - 1].latitude, full_path[i - 1].longitude,
                                        full_path[i].latitude,     full_path[i].longitude)
                out_brg = bearing_to(full_path[i].latitude,     full_path[i].longitude,
                                     full_path[i + 1].latitude, full_path[i + 1].longitude)
                ta = abs(((out_brg - in_brg + 180) % 360) - 180)
                if ta >= self._pivot_threshold:
                    # Close current chunk at this pivot wp; next chunk origin = this wp
                    chunks.append((list(current_wps), set(current_bypass),
                                   chunk_origin_lat, chunk_origin_lon))
                    chunk_origin_lat = wp.latitude
                    chunk_origin_lon = wp.longitude
                    current_wps    = []
                    current_bypass = set()

        if current_wps:
            chunks.append((current_wps, current_bypass, chunk_origin_lat, chunk_origin_lon))

        if not chunks:
            return

        # Build pending queue from chunks[1:]
        self._pending_path_chunks = []
        for i, (wps, byp, olat, olon) in enumerate(chunks):
            pivot_target = chunks[i + 1][0][0] if i + 1 < len(chunks) else None
            if i > 0:
                self._pending_path_chunks.append((wps, olat, olon, byp, pivot_target))

        # Activate first chunk
        first_wps, first_byp, first_olat, first_olon = chunks[0][:4]
        self._chunk_end_pivot_target = chunks[1][0][0] if len(chunks) > 1 else None
        self._path            = first_wps
        self._path_origin_lat = first_olat
        self._path_origin_lon = first_olon
        self._bypass_indices  = first_byp
        # Rebuild arc-lengths for the first chunk
        self._path_s = self._rebuild_path_s(first_wps, first_olat, first_olon)
        self._path_idx = 0

        self.get_logger().info(
            f'Mission split into {len(chunks)} sub-missions at pivot turns; '
            f'chunk[0] has {len(first_wps)} wps')

    def _rebuild_path_s(self, wps, origin_lat, origin_lon) -> list:
        """Rebuild cumulative arc-lengths for a chunk."""
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

    def _load_next_path_chunk(self) -> bool:
        """Load the next pending chunk into _path.  Returns True if available."""
        if not self._pending_path_chunks:
            return False
        wps, olat, olon, byp, pivot_target = self._pending_path_chunks.pop(0)
        self._path                  = wps
        self._path_origin_lat       = olat
        self._path_origin_lon       = olon
        self._bypass_indices        = byp
        self._chunk_end_pivot_target = pivot_target
        self._path_s                = self._rebuild_path_s(wps, olat, olon)
        self._path_idx              = 0
        self._mpc_prev_steers       = []
        self._pivoting              = False
        self._holding               = False
        self.get_logger().info(
            f'Sub-mission loaded: {len(wps)} wps, '
            f'{len(self._pending_path_chunks)} chunks remaining')
        return True

    # ── Full-path geometry ────────────────────────────────────────────────────

    def _turn_angle_at(self, idx: int) -> float:
        """
        Absolute heading change at waypoint _path[idx] (degrees, 0–180).
        Returns 0 for the last waypoint (no outgoing segment).
        Uses path origin as the incoming direction for the first waypoint.
        """
        if idx >= len(self._path) - 1:
            return 0.0
        # Outgoing segment: path[idx] → path[idx+1]
        hdg_out = bearing_to(self._path[idx].latitude,     self._path[idx].longitude,
                             self._path[idx + 1].latitude, self._path[idx + 1].longitude)
        # Incoming segment
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

        for seg_k in range(self._path_idx, len(self._path)):
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
            return 0.0

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

        if self._path_idx >= len(self._path):
            if self._pending_path_chunks:
                # More chunks queued — _control_loop will load the next one; don't signal done
                self.get_logger().info(
                    f'Chunk complete — {len(self._pending_path_chunks)} chunk(s) remaining')
            else:
                self.get_logger().info('Mission complete')
                m = Int32(); m.data = -1
                self.wp_pub.publish(m)
                self._publish_halt()
        else:
            nxt = self._path[self._path_idx]
            if self._path_idx - 1 not in self._bypass_indices:
                self.get_logger().info(
                    f'Waypoint {wp.seq} reached — navigating to {nxt.seq}')

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

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self):
        if self._mode != 'AUTONOMOUS' or not self._armed or self._paused:
            return
        if not self._path:
            return
        if self._path_idx >= len(self._path):
            if self._load_next_path_chunk():
                pass   # fall through to control this tick with new chunk
            else:
                return   # all chunks complete
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
                self._advance_path()
            return

        # ── Pivot turn ───────────────────────────────────────────────────────
        # Rover has reached a sharp-turn waypoint and is spinning in place to
        # align with the outgoing segment before continuing.
        if self._pivoting:
            pivot_err = ((self._pivot_target_hdg - self._heading + 180) % 360) - 180
            if abs(pivot_err) < self._hdb:
                self._pivoting = False
                self.get_logger().info('Pivot complete — continuing')
                self._advance_path()
            else:
                # Proportional spin: full speed above 45°, linear ramp below.
                # Prevents overshoot oscillation — at 25 Hz full steer gives
                # ~9°/step which exceeds the 3° deadband and causes hunting.
                steer_frac = max(-self._max_steer,
                                 min(self._max_steer, pivot_err / 45.0))
                steer_ppm  = int(PPM_CENTER - steer_frac * 500)
                self._publish_cmd(PPM_CENTER, steer_ppm)
            return

        wp          = self._path[self._path_idx]
        accept      = wp.acceptance_radius if wp.acceptance_radius > 0 else self._accept_r
        rlat, rlon  = self._center_pos()
        flat, flon  = self._front_pos()

        is_bypass        = self._path_idx in self._bypass_indices
        is_last_in_chunk = (self._path_idx == len(self._path) - 1)
        chunk_pivot_nxt  = self._chunk_end_pivot_target   # MissionWaypoint | None

        # Pivot detection — for the last wp in a chunk _turn_angle_at() returns 0
        # (no outgoing segment in _path).  Look at the next chunk's first wp instead.
        if is_last_in_chunk and chunk_pivot_nxt is not None:
            if len(self._path) > 1:
                in_brg = bearing_to(self._path[-2].latitude, self._path[-2].longitude,
                                    wp.latitude, wp.longitude)
            elif self._path_origin_lat is not None:
                in_brg = bearing_to(self._path_origin_lat, self._path_origin_lon,
                                    wp.latitude, wp.longitude)
            else:
                in_brg = 0.0
            out_brg    = bearing_to(wp.latitude, wp.longitude,
                                    chunk_pivot_nxt.latitude, chunk_pivot_nxt.longitude)
            turn_angle = abs(((out_brg - in_brg + 180) % 360) - 180)
            needs_pivot = turn_angle >= self._pivot_threshold
        else:
            turn_angle  = self._turn_angle_at(self._path_idx)
            needs_pivot = (turn_angle >= self._pivot_threshold
                           and self._path_idx < len(self._path) - 1)

        # ── Full-path nearest-point projection ───────────────────────────────
        s_nearest, best_seg = self._nearest_on_path(rlat, rlon)
        wp_s                = self._path_s[self._path_idx]
        dist_to_wp          = haversine(rlat, rlon, wp.latitude, wp.longitude)

        # ── Waypoint advance ─────────────────────────────────────────────────
        # For pivot waypoints: only proximity triggers advance (arc-length shortcut
        # disabled) so the rover must physically arrive at the turn point.
        # For normal waypoints: both proximity and arc-length overshoot trigger.
        reached = dist_to_wp < accept or (not needs_pivot and not is_bypass and s_nearest > wp_s + accept)

        if reached:
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
                # At a chunk boundary use the first wp of the next chunk.
                if is_last_in_chunk and chunk_pivot_nxt is not None:
                    nxt_lat, nxt_lon = chunk_pivot_nxt.latitude, chunk_pivot_nxt.longitude
                else:
                    nxt = self._path[self._path_idx + 1]
                    nxt_lat, nxt_lon = nxt.latitude, nxt.longitude
                self._pivot_target_hdg = bearing_to(rlat, rlon, nxt_lat, nxt_lon)
                self._pivoting        = True
                self._mpc_prev_steers = []   # warm-start stale after stop-and-spin
                self.get_logger().info(
                    f'Waypoint {wp.seq} reached — pivot {turn_angle:.0f}° '
                    f'to {self._pivot_target_hdg:.0f}°')
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
            la_lat, la_lon = self._point_at_s(s_nearest + self._lookahead)

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
        heading_err    = ((target_bearing - self._heading + 180) % 360) - 180

        # ── Stanley CTE: front antenna projected onto best_seg ───────────────
        # Bypass waypoints: zero CTE — heading error to the bypass point is enough.
        # Using best_seg (from _nearest_on_path) for CTE while is_bypass would pick
        # a wrong segment and add spurious correction opposing the detour.
        cte = 0.0 if is_bypass else self._cte_to_seg(flat, flon, best_seg)

        # XTE telemetry
        xte_msg      = Float32()
        xte_msg.data = abs(cte)
        self.xte_pub.publish(xte_msg)

        if abs(heading_err) > self._align_thresh:
            # Large error — spin in place (same for both algorithms)
            spin_dir          = math.copysign(1.0, heading_err)
            steer_ppm         = int(PPM_CENTER - spin_dir * self._max_steer * 500)
            throttle_ppm      = PPM_CENTER
            self._mpc_prev_steers = []   # heading jump → stale warm start
        else:
            v_mps        = max(target_spd, self._min_speed)
            throttle_ppm = int(PPM_CENTER + (v_mps / self._max_speed) * 500)

            # MPC active for all non-bypass segments — including pivot approach.
            # The horizon clips at the next pivot; beyond s_clip the reference is
            # projected along the incoming tangent so it never degenerates when
            # s_nearest ≈ s_clip.  Once the rover reaches the pivot it stops and
            # spins before the post-turn segment becomes visible.
            if self._algo == 'mpc' and not is_bypass:
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

        self._publish_cmd(throttle_ppm, steer_ppm)

    # ── Output helpers ────────────────────────────────────────────────────────

    def _publish_cmd(self, throttle: int, steering: int):
        msg          = RCInput()
        channels     = [PPM_CENTER] * 9
        channels[0]  = throttle
        channels[1]  = steering
        channels[2]  = 1939   # PPM CH3 = SBUS CH5  (no inversion)
        channels[3]  = 1061   # PPM CH4 = PPM_INV(SBUS CH6) = 3000-1939
        channels[4]  = 1061   # PPM CH5 = SBUS CH11 (no inversion)
        channels[5]  = 1061   # PPM CH6 = SBUS CH12 (no inversion)
        channels[6]  = 1500   # PPM CH7 = PPM_INV(SBUS CH7) = 3000-1500
        channels[7]  = 1061   # PPM CH8 = PPM_INV(SBUS CH8) = 3000-1939
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
