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
  2. Expands each polygon radially by `obstacle_clearance_m` from its centroid.
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
        #   obstacle_clearance_m — radial buffer (metres) added around obstacle polygons.
        self.declare_parameter('obstacle_clearance_m',       1.0)

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
        self._clearance           = self.get_parameter('obstacle_clearance_m').value

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

        self._dt = 1.0 / self.get_parameter('control_rate').value

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
        self.cmd_pub = self.create_publisher(RCInput,  'cmd_override', 10)
        self.wp_pub  = self.create_publisher(Int32,    'wp_active',    10)
        self.xte_pub = self.create_publisher(Float32,  'xte',          10)

        # ── Services ─────────────────────────────────────────────────────────
        self.create_service(Trigger, 'pause_mission',  self._svc_pause)
        self.create_service(Trigger, 'resume_mission', self._svc_resume)

        # ── Control loop ─────────────────────────────────────────────────────
        rate = self.get_parameter('control_rate').value
        self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info('Navigator ready (full-path Stanley + obstacle avoidance)')

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
        self._path_idx        = 0
        self._path_origin_lat = None
        self._path_origin_lon = None
        self._holding         = False
        self._pivoting        = False
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
            self._path_idx = 0
            self._holding  = False
            self._pivoting = False
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
            if self._path_original:
                self._reroute_path()
            else:
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
        """Radially expand polygon vertices away from the centroid by clearance_m."""
        if len(polygon) < 3:
            return list(polygon)
        c_lat = sum(p[0] for p in polygon) / len(polygon)
        c_lon = sum(p[1] for p in polygon) / len(polygon)
        result = []
        for (lat, lon) in polygon:
            d = haversine(c_lat, c_lon, lat, lon)
            if d < 0.01:
                # Vertex at centroid — push north by clearance_m
                result.append((lat + clearance_m / 111_320.0, lon))
                continue
            scale = (d + clearance_m) / d
            result.append((c_lat + (lat - c_lat) * scale,
                            c_lon + (lon - c_lon) * scale))
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
        i = 0
        while i < len(orig):
            if not new_wps:
                new_wps.append(orig[i])
                i += 1
                continue

            prev     = new_wps[-1]
            wp       = orig[i]
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

                # Scan ahead through original waypoints to find the exit crossing
                j = i + 1
                found_exit = False
                while j < len(orig):
                    c_lat = orig[j - 1].latitude
                    c_lon = orig[j - 1].longitude
                    d_lat = orig[j].latitude
                    d_lon = orig[j].longitude
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
                            new_wps.append(make_bypass_wp(bp_lat, bp_lon, orig[j]))
                        new_wps.append(orig[j])
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

        n_bypass = len(new_bypass_indices)
        if n_bypass > 0:
            self.get_logger().info(
                f'Path rerouted: {len(self._path_original)} → {len(self._path)} wps '
                f'({n_bypass} bypass inserted)')
        else:
            self.get_logger().info(
                f'Reroute complete: {len(self._path_original)} wps, '
                f'no bypass needed (path does not cross any obstacle)')

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
            self.get_logger().info('Mission complete')
            m = Int32(); m.data = -1
            self.wp_pub.publish(m)
            self._publish_halt()
        else:
            nxt = self._path[self._path_idx]
            if self._path_idx - 1 not in self._bypass_indices:
                self.get_logger().info(
                    f'Waypoint {wp.seq} reached — navigating to {nxt.seq}')

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self):
        if self._mode != 'AUTONOMOUS' or not self._armed or self._paused:
            return
        if not self._path or self._path_idx >= len(self._path):
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
                spin_dir  = math.copysign(1.0, pivot_err)
                steer_ppm = int(PPM_CENTER - spin_dir * self._max_steer * 500)
                self._publish_cmd(PPM_CENTER, steer_ppm)
            return

        wp          = self._path[self._path_idx]
        accept      = wp.acceptance_radius if wp.acceptance_radius > 0 else self._accept_r
        rlat, rlon  = self._center_pos()
        flat, flon  = self._front_pos()

        # Bypass waypoints never trigger pivot turns — they are smooth throughpoints.
        is_bypass   = self._path_idx in self._bypass_indices
        turn_angle  = self._turn_angle_at(self._path_idx) if not is_bypass else 0.0
        needs_pivot = (not is_bypass
                       and turn_angle >= self._pivot_threshold
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
                nxt = self._path[self._path_idx + 1]
                self._pivot_target_hdg = bearing_to(
                    wp.latitude, wp.longitude, nxt.latitude, nxt.longitude)
                self._pivoting = True
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

        if needs_pivot and dist_to_wp < self._pivot_approach_dist:
            la_lat     = wp.latitude
            la_lon     = wp.longitude
            # Scale speed linearly from target_spd down to min_speed
            approach_t = dist_to_wp / self._pivot_approach_dist   # 1.0 → 0.0
            target_spd = max(self._min_speed, target_spd * approach_t)
        else:
            la_lat, la_lon = self._point_at_s(s_nearest + self._lookahead)

        target_bearing = bearing_to(rlat, rlon, la_lat, la_lon)
        heading_err    = ((target_bearing - self._heading + 180) % 360) - 180

        # ── Stanley CTE: front antenna projected onto best_seg ───────────────
        cte = self._cte_to_seg(flat, flon, best_seg)

        # XTE telemetry
        xte_msg      = Float32()
        xte_msg.data = abs(cte)
        self.xte_pub.publish(xte_msg)

        if abs(heading_err) > self._align_thresh:
            # Large error — spin in place
            spin_dir     = math.copysign(1.0, heading_err)
            steer_ppm    = int(PPM_CENTER - spin_dir * self._max_steer * 500)
            throttle_ppm = PPM_CENTER
        else:
            # Stanley controller: δ = θ_e + arctan(k · e_cte / (v + ε))
            v_mps        = max(target_spd, self._min_speed)
            throttle_ppm = int(PPM_CENTER + (v_mps / self._max_speed) * 500)

            stanley_ang  = heading_err + math.degrees(
                math.atan2(self._stanley_k * cte,
                           max(v_mps, self._stanley_softening)))
            stanley_ang  = max(-90.0, min(90.0, stanley_ang))
            steer_frac   = max(-self._max_steer,
                               min(self._max_steer, stanley_ang / 45.0))
            steer_ppm    = int(PPM_CENTER - steer_frac * 500)

        self._publish_cmd(throttle_ppm, steer_ppm)

    # ── Output helpers ────────────────────────────────────────────────────────

    def _publish_cmd(self, throttle: int, steering: int):
        msg          = RCInput()
        channels     = [PPM_CENTER] * 9
        channels[0]  = throttle
        channels[1]  = steering
        channels[4]  = self._servo_ch[0]   # PPM CH5
        channels[5]  = self._servo_ch[1]   # PPM CH6
        channels[6]  = self._servo_ch[2]   # PPM CH7
        channels[7]  = self._servo_ch[3]   # PPM CH8
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
