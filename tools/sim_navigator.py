"""
tools/sim_navigator.py — Software-in-the-loop simulation of the full-path Stanley navigator.

Replicates navigator.py (full-path Stanley + pivot turns + obstacle avoidance) in pure
Python with no ROS2 dependency.  Uses the dead-reckoning physics from simulator.py's
RoverState.

Importable by monitor.py for automatic pre-mission simulation, or run standalone:

  python tools/sim_navigator.py --lat 20.727715 --lon -103.566782 --heading 90 \\
         --waypoints missions/field.csv

  # With obstacle polygons (JSON file: [[lat,lon],...] per polygon):
  python tools/sim_navigator.py ... --obstacles missions/obstacles.json

The navigator parameters default to the same values as navigator_params.yaml.
Physical parameters (wheelbase, turn_scale) can be overridden via DEFAULT_PHYS or
the --wheelbase / --turn-scale CLI args.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from dataclasses import dataclass, field

try:
    from scipy.optimize import minimize as _scipy_minimize
    _SCIPY_OK = True
except ImportError:
    _scipy_minimize = None
    _SCIPY_OK = False

# Import RoverState (dead-reckoning physics) from sibling simulator.py
sys.path.insert(0, os.path.dirname(__file__))

_DBG_LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'obstacle_debug.log')

def _dbg(msg: str):
    """Append a debug line to obstacle_debug.log (visible even after terminal clear)."""
    with open(_DBG_LOG, 'a') as _f:
        _f.write(msg + '\n')
from simulator import RoverState   # noqa: E402

# ── Constants ─────────────────────────────────────────────────────────────────

PPM_CENTER = 1500
EARTH_R    = 6_371_000.0

# Navigator parameter defaults — keep in sync with navigator_params.yaml
DEFAULT_NAV: dict = {
    'lookahead_distance':        3.0,
    'default_acceptance_radius': 1.5,
    'max_speed':                 1.5,
    'min_speed':                 0.3,
    'max_steering':              0.8,
    'align_threshold':           30.0,
    'heading_deadband':          3.0,
    'stanley_k':                 1.0,
    'stanley_softening':         0.3,
    'pivot_threshold':           25.0,
    'pivot_approach_dist':       4.0,
    'obstacle_clearance_m':      0.5,
    'rover_width_m':             1.0,
    'control_rate':              25.0,
    'max_timeout':               300.0,  # simulation hard stop (seconds)
    # Control algorithm — 'stanley' or 'mpc'
    'control_algorithm':         'mpc',
    'mpc_horizon':               5,
    'mpc_dt':                    0.1,
    'mpc_w_cte':                 5.0,
    'mpc_w_heading':             1.0,
    'mpc_w_steer':               0.1,
    'mpc_w_dsteer':              0.05,
    'wheelbase_m':               0.6,
}

# Physical rover parameters — tune to match the real rover
DEFAULT_PHYS: dict = {
    'wheelbase':  0.6,    # metres — distance between left and right wheels
    'turn_scale': 1.0,    # turn rate fraction (0.1 = slow, 1.0 = full differential) # matches MPC kinematic model
    'baseline_m': 1.0,    # metres — rear-to-front antenna separation
}

# ── Data classes ──────────────────────────────────────────────────────────────

@dataclass
class SimWaypoint:
    seq:               int
    lat:               float
    lon:               float
    speed:             float = 0.0
    hold_secs:         float = 0.0
    acceptance_radius: float = 0.0
    is_bypass:         bool  = False


@dataclass
class SimResult:
    path:               list   # [(lat, lon), ...] at each sim step
    xte_log:            list   # [float, ...] absolute CTE (metres) per step
    waypoints_reached:  list   # [seq, ...] in order reached
    rms_xte:            float
    max_xte:            float
    avg_xte:            float
    total_steps:        int
    complete:           bool   # True if all waypoints were reached within timeout
    obstacle_polygons:  list   # [[(lat,lon),...], ...] polygons used (expanded)
    rerouted_wps:       list   # [[lat,lon], ...] effective waypoints after rerouting
                               # (includes bypass points; same as original when no obstacles)
    step_log:           list   = field(default_factory=list)  # debug info per step (verbose mode)


# ── Math helpers ──────────────────────────────────────────────────────────────

def _haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2
         + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2))
         * math.sin(dlon / 2) ** 2)
    return 2 * EARTH_R * math.asin(math.sqrt(max(0.0, a)))


def _bearing_to(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(math.radians(lat2))
    y = (math.cos(math.radians(lat1)) * math.sin(math.radians(lat2))
         - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(dlon))
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def _center_pos(rover: RoverState, baseline_m: float) -> tuple[float, float]:
    """Midpoint between rear and front antenna (rover centre)."""
    half    = baseline_m / 2.0
    lat_r   = math.radians(rover.lat)
    cos_lat = math.cos(lat_r) or 1e-9
    return (rover.lat + (half * math.cos(rover.heading_rad)) / 111_320.0,
            rover.lon + (half * math.sin(rover.heading_rad)) / (111_320.0 * cos_lat))


def _front_pos(rover: RoverState, baseline_m: float) -> tuple[float, float]:
    """Front antenna position."""
    lat_r   = math.radians(rover.lat)
    cos_lat = math.cos(lat_r) or 1e-9
    return (rover.lat + (baseline_m * math.cos(rover.heading_rad)) / 111_320.0,
            rover.lon + (baseline_m * math.sin(rover.heading_rad)) / (111_320.0 * cos_lat))


# ── Obstacle geometry (mirrors navigator.py) ──────────────────────────────────

def _expand_polygon(polygon: list[tuple[float, float]],
                    clearance_m: float) -> list[tuple[float, float]]:
    """
    Offset each polygon edge outward by clearance_m (uniform Minkowski buffer).
    Gives exactly clearance_m perpendicular distance from every edge.
    """
    n = len(polygon)
    if n < 3:
        return list(polygon)
    c_lat = sum(p[0] for p in polygon) / n
    c_lon = sum(p[1] for p in polygon) / n
    cos_lat = math.cos(math.radians(c_lat)) or 1e-9
    m_lat, m_lon = 111_320.0, 111_320.0 * cos_lat

    def to_m(lat, lon):
        return (lon - c_lon) * m_lon, (lat - c_lat) * m_lat

    def to_ll(x, y):
        return c_lat + y / m_lat, c_lon + x / m_lon

    pts = [to_m(lat, lon) for lat, lon in polygon]
    area2 = sum(pts[i][0] * pts[(i+1)%n][1] - pts[(i+1)%n][0] * pts[i][1]
                for i in range(n))
    w = 1.0 if area2 > 0 else -1.0   # +1 = CCW, -1 = CW

    off = []
    for i in range(n):
        x1, y1 = pts[i]; x2, y2 = pts[(i+1)%n]
        dx, dy = x2-x1, y2-y1
        lg = math.hypot(dx, dy) or 1e-9
        nx, ny = w*dy/lg, -w*dx/lg
        off.append((x1+nx*clearance_m, y1+ny*clearance_m,
                    x2+nx*clearance_m, y2+ny*clearance_m))

    result = []
    for i in range(n):
        x1,y1,x2,y2 = off[i]; x3,y3,x4,y4 = off[(i+1)%n]
        dx1,dy1 = x2-x1,y2-y1; dx2,dy2 = x4-x3,y4-y3
        denom = dx1*dy2 - dy1*dx2
        if abs(denom) < 1e-9:
            result.append(to_ll(x2, y2))
        else:
            t = ((x3-x1)*dy2 - (y3-y1)*dx2) / denom
            result.append(to_ll(x1+t*dx1, y1+t*dy1))
    return result


def _seg_intersect_polygon(
        a_lat: float, a_lon: float,
        b_lat: float, b_lon: float,
        polygon: list[tuple[float, float]]) -> list[tuple[float, int]]:
    """Return sorted (t, edge_idx) intersections of segment A→B with polygon boundary."""
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
    dx, dy = bx - ax, by - ay

    hits: list[tuple[float, int]] = []
    n = len(polygon)
    for i in range(n):
        cx, cy = xy(*polygon[i])
        nx, ny = xy(*polygon[(i + 1) % n])
        ex, ey = nx - cx, ny - cy
        rx, ry = cx - ax, cy - ay
        det = ex * dy - ey * dx
        if abs(det) < 1e-9:
            continue
        t = (ex * ry - ey * rx) / det
        u = (dx * ry - dy * rx) / det
        if 1e-6 < t < 1.0 - 1e-6 and 0.0 <= u <= 1.0:
            hits.append((t, i))
    hits.sort(key=lambda h: h[0])
    return hits


def _bypass_arc(
        entry_pt: tuple[float, float],
        exit_pt: tuple[float, float],
        entry_edge: int,
        exit_edge: int,
        polygon: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Return the shorter arc path: entry_pt → polygon vertices → exit_pt."""
    n = len(polygon)

    ccw: list[tuple[float, float]] = []
    idx = (entry_edge + 1) % n
    for _ in range(n):
        ccw.append(polygon[idx])
        if idx == exit_edge:
            break
        idx = (idx + 1) % n

    cw: list[tuple[float, float]] = []
    target = (exit_edge + 1) % n
    idx = entry_edge
    for _ in range(n):
        cw.append(polygon[idx])
        if idx == target:
            break
        idx = (idx - 1 + n) % n

    def path_len(pts: list[tuple[float, float]]) -> float:
        return sum(_haversine(pts[k][0], pts[k][1], pts[k+1][0], pts[k+1][1])
                   for k in range(len(pts) - 1))

    ccw_path = [entry_pt] + ccw + [exit_pt]
    cw_path  = [entry_pt] + cw  + [exit_pt]
    return ccw_path if path_len(ccw_path) <= path_len(cw_path) else cw_path


def _bypass_verts(
        a_lat: float, a_lon: float,
        b_lat: float, b_lon: float,
        hits: list[tuple[float, int]],
        polygon: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Return bypass (lat, lon) points that detour around the polygon boundary."""
    if len(hits) < 2:
        return []
    t_entry, entry_edge = hits[0]
    t_exit,  exit_edge  = hits[-1]

    def interp(t: float) -> tuple[float, float]:
        return (a_lat + t * (b_lat - a_lat), a_lon + t * (b_lon - a_lon))

    return _bypass_arc(interp(t_entry), interp(t_exit), entry_edge, exit_edge, polygon)


def _reroute_waypoints(
        waypoints: list[SimWaypoint],
        expanded_polygons: list[list[tuple[float, float]]],
        origin_lat: float, origin_lon: float) -> list[SimWaypoint]:
    """
    Return a new waypoint list with bypass points inserted around obstacles.

    Handles the case where an obstacle polygon spans across a waypoint (entry and
    exit on different segments) by scanning ahead when a segment has a 1-hit entry.
    """
    if not waypoints or not expanded_polygons:
        return list(waypoints)

    new_wps: list[SimWaypoint] = []
    seq_counter = [max(w.seq for w in waypoints) + 1]  # synthetic seq for bypass wps

    def make_bypass(lat: float, lon: float, ref: SimWaypoint) -> SimWaypoint:
        s = seq_counter[0]; seq_counter[0] += 1
        return SimWaypoint(seq=s, lat=lat, lon=lon, speed=ref.speed, is_bypass=True)

    i = 0
    while i < len(waypoints):
        if not new_wps:
            new_wps.append(waypoints[i])
            i += 1
            continue

        prev = new_wps[-1]
        wp   = waypoints[i]
        a_lat, a_lon = prev.lat, prev.lon
        b_lat, b_lon = wp.lat, wp.lon

        # Compute intersections with all polygons for this segment
        seg_hits: list[tuple[int, list]] = []
        for pi, poly in enumerate(expanded_polygons):
            hits = _seg_intersect_polygon(a_lat, a_lon, b_lat, b_lon, poly)
            _dbg(f'seg ({a_lat:.6f},{a_lon:.6f})->({b_lat:.6f},{b_lon:.6f}) '
                 f'poly[{pi}] verts={len(poly)} hits={len(hits)}')
            if hits:
                seg_hits.append((pi, hits))

        # Case 1: complete in-segment crossing (>=2 hits) — use existing logic
        complete = [(pi, h) for pi, h in seg_hits if len(h) >= 2]
        if complete:
            complete.sort(key=lambda x: x[1][0][0])
            best_pi, best_hits = complete[0]
            bypass = _bypass_verts(a_lat, a_lon, b_lat, b_lon,
                                   best_hits, expanded_polygons[best_pi])
            _dbg(f'  -> bypass inserted: {len(bypass)} points around poly[{best_pi}]')
            for blat, blon in bypass:
                new_wps.append(make_bypass(blat, blon, wp))
            new_wps.append(wp)
            i += 1
            continue

        # Case 2: 1-hit entry — polygon spans across one or more waypoints
        entries = [(pi, h[0]) for pi, h in seg_hits if len(h) == 1]
        if entries:
            entries.sort(key=lambda x: x[1][0])
            entry_pi, (entry_t, entry_edge) = entries[0]
            poly = expanded_polygons[entry_pi]
            entry_pt = (a_lat + entry_t * (b_lat - a_lat),
                        a_lon + entry_t * (b_lon - a_lon))

            # Scan ahead through original waypoints to find the exit crossing
            j = i + 1
            found_exit = False
            while j < len(waypoints):
                c_lat = waypoints[j - 1].lat
                c_lon = waypoints[j - 1].lon
                d_lat = waypoints[j].lat
                d_lon = waypoints[j].lon
                exit_hits = _seg_intersect_polygon(c_lat, c_lon, d_lat, d_lon, poly)
                _dbg(f'  scan-ahead j={j}: '
                     f'seg ({c_lat:.6f},{c_lon:.6f})->({d_lat:.6f},{d_lon:.6f}) '
                     f'hits={len(exit_hits)}')
                if exit_hits:
                    exit_t, exit_edge = exit_hits[0]
                    exit_pt = (c_lat + exit_t * (d_lat - c_lat),
                               c_lon + exit_t * (d_lon - c_lon))
                    bypass = _bypass_arc(entry_pt, exit_pt,
                                         entry_edge, exit_edge, poly)
                    _dbg(f'  -> cross-segment bypass: {len(bypass)} pts, '
                         f'poly[{entry_pi}] spans wps {i}..{j-1}')
                    for blat, blon in bypass:
                        new_wps.append(make_bypass(blat, blon, waypoints[j]))
                    new_wps.append(waypoints[j])
                    i = j + 1
                    found_exit = True
                    break
                j += 1

            if not found_exit:
                _dbg(f'  scan-ahead: no exit found for poly[{entry_pi}] — appending wp')
                new_wps.append(wp)
                i += 1
            continue

        # No intersection — keep original waypoint
        new_wps.append(wp)
        i += 1

    return new_wps


# ── Path navigator (mirrors navigator.py) ────────────────────────────────────

class PathNavigator:
    """
    Stateful full-path Stanley navigator for simulation.
    Call step() once per control tick; it returns PPM outputs and whether the
    mission is complete.
    """

    def __init__(self, waypoints: list[SimWaypoint],
                 origin_lat: float, origin_lon: float,
                 nav: dict):
        self._wps        = waypoints
        self._origin_lat = origin_lat
        self._origin_lon = origin_lon
        self._nav        = nav
        self.path_idx    = 0
        self._pivoting   = False
        self._pivot_hdg  = 0.0
        self._holding    = False
        self._hold_end   = 0.0          # simulated monotonic time
        self._path_s          = self._compute_arclens()
        self._algo            = nav.get('control_algorithm', 'stanley')
        self._mpc_prev_steers: list[float] = []
        self._step_info: dict = {}

    # ── Arc-length ─────────────────────────────────────────────────────────

    def _compute_arclens(self) -> list[float]:
        s = []
        if not self._wps:
            return s
        s.append(_haversine(self._origin_lat, self._origin_lon,
                            self._wps[0].lat, self._wps[0].lon))
        for i in range(1, len(self._wps)):
            s.append(s[-1] + _haversine(self._wps[i - 1].lat, self._wps[i - 1].lon,
                                        self._wps[i].lat, self._wps[i].lon))
        return s

    # ── Geometry helpers (mirrors navigator.py) ────────────────────────────

    def _nearest_on_path(self, lat: float, lon: float) -> tuple[float, int]:
        if not self._wps:
            return 0.0, 0
        best_s, best_seg, best_dist = 0.0, self.path_idx, float('inf')

        for seg_k in range(self.path_idx, len(self._wps)):
            if seg_k == 0:
                a_lat, a_lon, s_a = self._origin_lat, self._origin_lon, 0.0
            else:
                a_lat = self._wps[seg_k - 1].lat
                a_lon = self._wps[seg_k - 1].lon
                s_a   = self._path_s[seg_k - 1]
            b_lat, b_lon, s_b = self._wps[seg_k].lat, self._wps[seg_k].lon, self._path_s[seg_k]

            mid_lat = math.radians((a_lat + b_lat) / 2)
            cos_lat = math.cos(mid_lat) or 1e-9
            m_lat, m_lon = 111_320.0, 111_320.0 * cos_lat

            seg_dy = (b_lat - a_lat) * m_lat
            seg_dx = (b_lon - a_lon) * m_lon
            seg_len = math.hypot(seg_dx, seg_dy)

            if seg_len < 0.01:
                d = _haversine(lat, lon, b_lat, b_lon)
                if d < best_dist:
                    best_dist, best_s, best_seg = d, s_b, seg_k
                continue

            rv_dy = (lat - a_lat) * m_lat
            rv_dx = (lon - a_lon) * m_lon
            t = max(0.0, min(1.0, (rv_dx * seg_dx + rv_dy * seg_dy) / seg_len ** 2))
            d = math.hypot(t * seg_dx - rv_dx, t * seg_dy - rv_dy)

            if d < best_dist:
                best_dist = d
                best_s    = s_a + t * (s_b - s_a)
                best_seg  = seg_k

        return best_s, best_seg

    def _cte_to_seg(self, lat: float, lon: float, seg_idx: int) -> float:
        if seg_idx == 0:
            if not self._wps:
                return 0.0
            a_lat, a_lon = self._origin_lat, self._origin_lon
        else:
            if seg_idx >= len(self._wps):
                return 0.0
            a_lat, a_lon = self._wps[seg_idx - 1].lat, self._wps[seg_idx - 1].lon
        b_lat, b_lon = self._wps[seg_idx].lat, self._wps[seg_idx].lon

        mid_lat = math.radians((a_lat + b_lat) / 2)
        cos_lat = math.cos(mid_lat) or 1e-9
        m_lat, m_lon = 111_320.0, 111_320.0 * cos_lat
        seg_dy  = (b_lat - a_lat) * m_lat
        seg_dx  = (b_lon - a_lon) * m_lon
        seg_len = math.hypot(seg_dx, seg_dy)
        if seg_len < 0.01:
            return 0.0
        rv_dy = (lat - a_lat) * m_lat
        rv_dx = (lon - a_lon) * m_lon
        return (seg_dx * rv_dy - seg_dy * rv_dx) / seg_len

    def _point_at_s(self, s_target: float) -> tuple[float, float]:
        if not self._wps or not self._path_s:
            return 0.0, 0.0
        if s_target >= self._path_s[-1]:
            return self._wps[-1].lat, self._wps[-1].lon
        if s_target <= self._path_s[0]:
            s0   = self._path_s[0]
            frac = s_target / s0 if s0 > 0.01 else 0.0
            return (self._origin_lat + frac * (self._wps[0].lat - self._origin_lat),
                    self._origin_lon + frac * (self._wps[0].lon - self._origin_lon))
        for k in range(1, len(self._wps)):
            s_a, s_b = self._path_s[k - 1], self._path_s[k]
            if s_target <= s_b:
                sl = s_b - s_a
                if sl < 0.01:
                    return self._wps[k].lat, self._wps[k].lon
                frac = (s_target - s_a) / sl
                return (self._wps[k - 1].lat + frac * (self._wps[k].lat - self._wps[k - 1].lat),
                        self._wps[k - 1].lon + frac * (self._wps[k].lon - self._wps[k - 1].lon))
        return self._wps[-1].lat, self._wps[-1].lon

    def _turn_angle_at(self, idx: int) -> float:
        if idx >= len(self._wps) - 1:
            return 0.0
        hdg_out = _bearing_to(self._wps[idx].lat, self._wps[idx].lon,
                              self._wps[idx + 1].lat, self._wps[idx + 1].lon)
        if idx == 0:
            hdg_in = _bearing_to(self._origin_lat, self._origin_lon,
                                 self._wps[0].lat, self._wps[0].lon)
        else:
            hdg_in = _bearing_to(self._wps[idx - 1].lat, self._wps[idx - 1].lon,
                                 self._wps[idx].lat, self._wps[idx].lon)
        return abs(((hdg_out - hdg_in + 180) % 360) - 180)

    # ── MPC controller ─────────────────────────────────────────────────────

    def _mpc_steer(self, rlat: float, rlon: float,
                   heading_deg: float, s_nearest: float,
                   v_mps: float) -> float:
        """MPC lateral controller — mirrors navigator.py._mpc_steer()."""
        nav      = self._nav
        N        = int(nav.get('mpc_horizon', 10))
        dt       = nav.get('mpc_dt', 0.2)
        wb       = max(nav.get('wheelbase_m', 0.6), 0.1)
        max_s    = nav['max_steering']
        w_cte    = nav.get('mpc_w_cte',     2.0)
        w_hdg    = nav.get('mpc_w_heading', 1.0)
        w_str    = nav.get('mpc_w_steer',   0.1)
        w_dstr   = nav.get('mpc_w_dsteer',  0.05)
        lookahead = nav['lookahead_distance']

        if not _SCIPY_OK:
            la_lat, la_lon = self._point_at_s(s_nearest + lookahead)
            brg = _bearing_to(rlat, rlon, la_lat, la_lon)
            err = ((brg - heading_deg + 180) % 360) - 180
            return max(-max_s, min(max_s, err / 45.0))

        cos_lat = math.cos(math.radians(rlat)) or 1e-9
        m_lat   = 111_320.0
        m_lon   = 111_320.0 * cos_lat

        def to_xy(lat: float, lon: float) -> tuple[float, float]:
            return (lon - rlon) * m_lon, (lat - rlat) * m_lat

        # Clip horizon at the next pivot waypoint so MPC never optimises
        # across a sharp turn (mirrors navigator.py._mpc_steer).
        pivot_thresh_nav = self._nav.get('pivot_threshold', 25.0)
        s_clip = float('inf')
        for i in range(self.path_idx, len(self._wps)):
            if not self._wps[i].is_bypass and self._turn_angle_at(i) >= pivot_thresh_nav:
                if i < len(self._path_s):
                    s_clip = self._path_s[i]
                break
        self._step_info['s_clip'] = s_clip

        ref_x: list[float] = []
        ref_y: list[float] = []
        for ki in range(N + 1):
            s_ref = min(s_nearest + v_mps * ki * dt, s_clip)
            pt = self._point_at_s(s_ref)
            rx, ry = to_xy(pt[0], pt[1])
            ref_x.append(rx); ref_y.append(ry)

        h0 = math.radians(90.0 - heading_deg)
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
                s = float(steers[ki])
                h += -s * 2.0 * v_mps / wb * dt
                x += v_mps * math.cos(h) * dt
                y += v_mps * math.sin(h) * dt
                rh   = ref_h[ki]
                cte  = -(x - ref_x[ki]) * math.sin(rh) + (y - ref_y[ki]) * math.cos(rh)
                herr = (h - rh + math.pi) % (2 * math.pi) - math.pi
                cost += w_cte * cte * cte + w_hdg * herr * herr
                cost += w_str * s * s + w_dstr * (s - prev) * (s - prev)
                prev = s
            return cost

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

    # ── Control step ───────────────────────────────────────────────────────

    def step(self, rlat: float, rlon: float,
             flat: float, flon: float,
             heading: float, t_mono: float) -> tuple[int, int, int, bool]:
        """
        One control tick.
        Returns (throttle_ppm, steer_ppm, best_seg, done).
        """
        nav = self._nav
        lookahead       = nav['lookahead_distance']
        accept_r        = nav['default_acceptance_radius']
        max_spd         = nav['max_speed']
        min_spd         = nav['min_speed']
        max_steer       = nav['max_steering']
        align_thresh    = nav['align_threshold']
        hdb             = nav['heading_deadband']
        k               = nav['stanley_k']
        softening       = nav['stanley_softening']
        pivot_thresh    = nav['pivot_threshold']
        pivot_app_dist  = nav['pivot_approach_dist']

        self._step_info = {'t': t_mono, 'pivoting': self._pivoting, 'holding': self._holding}

        if self.path_idx >= len(self._wps):
            return PPM_CENTER, PPM_CENTER, 0, True

        # Hold
        if self._holding:
            if t_mono >= self._hold_end:
                self._holding = False
                self._advance()
            return PPM_CENTER, PPM_CENTER, self.path_idx, False

        # Pivot
        if self._pivoting:
            pivot_err = ((self._pivot_hdg - heading + 180) % 360) - 180
            if abs(pivot_err) < hdb:
                self._pivoting = False
                self._advance()
            else:
                spin_dir  = math.copysign(1.0, pivot_err)
                steer_ppm = int(PPM_CENTER - spin_dir * max_steer * 500)
                return PPM_CENTER, steer_ppm, self.path_idx, False
            return PPM_CENTER, PPM_CENTER, self.path_idx, False

        wp          = self._wps[self.path_idx]
        accept      = wp.acceptance_radius if wp.acceptance_radius > 0 else accept_r
        is_bypass   = wp.is_bypass
        turn_angle  = self._turn_angle_at(self.path_idx) if not is_bypass else 0.0
        needs_pivot = (not is_bypass and turn_angle >= pivot_thresh and self.path_idx < len(self._wps) - 1)

        s_nearest, best_seg = self._nearest_on_path(rlat, rlon)
        wp_s                = self._path_s[self.path_idx]
        dist_to_wp          = _haversine(rlat, rlon, wp.lat, wp.lon)

        self._step_info.update({
            'wp_idx': self.path_idx,
            'dist_to_wp': dist_to_wp,
            'turn_angle': turn_angle,
            'needs_pivot': needs_pivot,
            's_nearest': s_nearest,
            's_clip': float('inf'),  # updated by _mpc_steer if called
        })

        # Waypoint advance — bypass waypoints require physical proximity (arc-length
        # advance is disabled so short bypass arcs are not immediately skipped).
        reached = dist_to_wp < accept or (not needs_pivot and not is_bypass and s_nearest > wp_s + accept)
        if reached:
            if wp.hold_secs > 0.0:
                self._holding  = True
                self._hold_end = t_mono + wp.hold_secs
                return PPM_CENTER, PPM_CENTER, best_seg, False
            elif needs_pivot:
                nxt = self._wps[self.path_idx + 1]
                self._pivot_hdg       = _bearing_to(wp.lat, wp.lon, nxt.lat, nxt.lon)
                self._pivoting        = True
                self._mpc_prev_steers = []
                return PPM_CENTER, PPM_CENTER, best_seg, False
            else:
                self._advance()
                return PPM_CENTER, PPM_CENTER, best_seg, self.path_idx >= len(self._wps)

        # Lookahead target
        target_spd = wp.speed if wp.speed > 0 else max_spd
        if needs_pivot:
            # Always aim directly at the pivot waypoint — never use arc-length
            # projection past it.  _nearest_on_path can snap ahead to the
            # post-turn segment when the rover is physically close to the pivot,
            # which would project the lookahead into the outgoing direction and
            # trigger a premature ~180° spin before arrival.
            la_lat, la_lon = wp.lat, wp.lon
            if dist_to_wp < pivot_app_dist:
                target_spd = max(min_spd, target_spd * dist_to_wp / pivot_app_dist)
        elif is_bypass:
            # Same reason as pivot: bypass arcs are short and _nearest_on_path
            # can snap to a later original segment.
            la_lat, la_lon = wp.lat, wp.lon
        else:
            la_lat, la_lon = self._point_at_s(s_nearest + lookahead)

        target_bearing = _bearing_to(rlat, rlon, la_lat, la_lon)
        heading_err    = ((target_bearing - heading + 180) % 360) - 180
        # Bypass waypoints: zero CTE — heading error to the bypass point is enough.
        cte            = 0.0 if is_bypass else self._cte_to_seg(flat, flon, best_seg)

        self._step_info.update({'heading_err': heading_err, 'cte': cte})

        if abs(heading_err) > align_thresh:
            spin_dir              = math.copysign(1.0, heading_err)
            steer_ppm             = int(PPM_CENTER - spin_dir * max_steer * 500)
            self._mpc_prev_steers = []
            return PPM_CENTER, steer_ppm, best_seg, False

        v_mps        = max(target_spd, min_spd)
        throttle_ppm = int(PPM_CENTER + (v_mps / max_spd) * 500)

        # Use MPC only on normal (non-pivot, non-bypass) segments.
        # For pivot WPs: Stanley with direct-to-waypoint aim is reliable;
        # MPC with a horizon clipped at the pivot produces a degenerate
        # reference (all points at the same location) when s_nearest is
        # close to s_clip, causing erratic steering.
        use_mpc = self._algo == 'mpc' and not is_bypass and not needs_pivot
        if use_mpc:
            steer_frac = self._mpc_steer(rlat, rlon, heading, s_nearest, v_mps)
        else:
            # Stanley controller: δ = θ_e + arctan(k · e_cte / (v + ε))
            stanley_ang = heading_err + math.degrees(
                math.atan2(k * cte, max(v_mps, softening)))
            stanley_ang = max(-90.0, min(90.0, stanley_ang))
            steer_frac  = max(-max_steer, min(max_steer, stanley_ang / 45.0))
        self._step_info.update({
            'steer_frac': steer_frac,
            'v_mps': v_mps,
            'mode': 'mpc' if use_mpc else 'stanley',
        })
        steer_ppm = int(PPM_CENTER - steer_frac * 500)

        return throttle_ppm, steer_ppm, best_seg, False

    def _advance(self):
        if self.path_idx < len(self._wps):
            self.path_idx += 1


# ── Public entry point ────────────────────────────────────────────────────────

def simulate(waypoints:       list[SimWaypoint],
             start_lat:       float,
             start_lon:       float,
             start_heading:   float,
             nav_params:      dict | None = None,
             phys_params:     dict | None = None,
             obstacles:       list[list[list[float]]] | None = None,
             verbose:         bool = False) -> SimResult:
    """
    Run a software-in-the-loop simulation of the rover following *waypoints*.

    Parameters
    ----------
    waypoints     : ordered list of SimWaypoint
    start_lat/lon : rover starting position (degrees)
    start_heading : rover starting heading (degrees from north)
    nav_params    : override any key in DEFAULT_NAV
    phys_params   : override any key in DEFAULT_PHYS
    obstacles     : list of polygons [[[lat,lon],...], ...] — raw (unexpanded).
                    If provided, each polygon is expanded by nav_params['obstacle_clearance_m']
                    and bypass waypoints are inserted into the path automatically.
    """
    nav   = {**DEFAULT_NAV,  **(nav_params  or {})}
    phys  = {**DEFAULT_PHYS, **(phys_params or {})}
    dt    = 1.0 / nav['control_rate']
    bm    = phys['baseline_m']

    # Build expanded polygons and rerouted waypoints if obstacles provided
    expanded_polygons: list[list[tuple[float, float]]] = []
    effective_wps = list(waypoints)
    import time as _time
    _dbg(f'\n=== simulate() called {_time.strftime("%H:%M:%S")} ==='
         f' obstacles={len(obstacles) if obstacles else 0} wps={len(waypoints)} ===')
    _dbg(f'start=({start_lat:.6f},{start_lon:.6f})')
    if waypoints:
        _dbg(f'WP[0]=({waypoints[0].lat:.6f},{waypoints[0].lon:.6f})'
             f'  WP[-1]=({waypoints[-1].lat:.6f},{waypoints[-1].lon:.6f})')
    if obstacles:
        clearance = nav['rover_width_m'] / 2.0 + nav['obstacle_clearance_m']
        for i, poly_raw in enumerate(obstacles):
            poly = [(float(v[0]), float(v[1])) for v in poly_raw]
            c_lat = sum(p[0] for p in poly) / len(poly)
            c_lon = sum(p[1] for p in poly) / len(poly)
            _dbg(f'obstacle[{i}]: {len(poly)} verts, centroid=({c_lat:.6f},{c_lon:.6f})')
            for j, v in enumerate(poly):
                _dbg(f'  v[{j}]=({v[0]:.6f},{v[1]:.6f})')
            if len(poly) >= 3:
                expanded_polygons.append(_expand_polygon(poly, clearance))
            else:
                _dbg(f'obstacle[{i}] SKIPPED — only {len(poly)} vertices (need >=3)')
        if expanded_polygons:
            _dbg(f'--- reroute start: {len(waypoints)} wps, '
                 f'{len(expanded_polygons)} expanded polys ---')
            effective_wps = _reroute_waypoints(
                waypoints, expanded_polygons, start_lat, start_lon)
            _dbg(f'--- reroute done: {len(effective_wps)} effective wps ---')
    _dbg(f'simulate: obstacles={len(obstacles) if obstacles else 0} '
         f'expanded={len(expanded_polygons)} '
         f'wps {len(waypoints)}->{len(effective_wps)}')

    rover  = RoverState(start_lat, start_lon, start_heading)
    path_n = PathNavigator(effective_wps, start_lat, start_lon, nav)

    if verbose:
        print(f'\n{"─"*60}')
        print(f'Mission analysis  ({len(effective_wps)} waypoints, algo={nav["control_algorithm"]})')
        print(f'  pivot_threshold={nav["pivot_threshold"]}°  '
              f'mpc_horizon={nav.get("mpc_horizon",10)}  '
              f'mpc_dt={nav.get("mpc_dt",0.2)}  '
              f'mpc_w_cte={nav.get("mpc_w_cte",2.0)}')
        print(f'  {"WP":>3}  {"lat":>11}  {"lon":>12}  {"turn°":>6}  {"type":>8}  {"arc_s":>6}')
        for i, wp in enumerate(effective_wps):
            ta = path_n._turn_angle_at(i)
            pv = '★PIVOT' if (not wp.is_bypass and ta >= nav['pivot_threshold']) else ('bypass' if wp.is_bypass else 'normal')
            arc = path_n._path_s[i] if i < len(path_n._path_s) else 0.0
            print(f'  {wp.seq:>3}  {wp.lat:>11.6f}  {wp.lon:>12.6f}  {ta:>6.1f}  {pv:>8}  {arc:>6.1f}m')
        print(f'{"─"*60}')
        print(f'  {"step":>5}  {"t":>5}  {"wp":>3}  {"dist":>6}  {"hdg_err":>8}  {"cte":>7}  {"steer":>6}  {"mode":>7}  {"s_clip":>8}')

    result_path: list[tuple[float, float]] = [_center_pos(rover, bm)]
    xte_log:     list[float]               = []
    wps_reached: list[int]                 = []
    t_mono = 0.0
    prev_idx = 0

    max_steps = int(nav['max_timeout'] / dt)
    step_log_data: list = []

    for step in range(max_steps):
        rlat, rlon = _center_pos(rover, bm)
        flat, flon = _front_pos(rover, bm)
        thr, steer, best_seg, done = path_n.step(rlat, rlon, flat, flon,
                                                  rover.heading_deg, t_mono)
        info = dict(path_n._step_info)
        info['step'] = step
        step_log_data.append(info)

        if verbose and step % max(1, int(nav['control_rate'])) == 0:
            si = info
            s_clip_str = f'{si.get("s_clip", float("inf")):.1f}m' if si.get('s_clip', float('inf')) < 1e6 else '    inf'
            print(f'  {step:>5}  {t_mono:>5.1f}  '
                  f'{si.get("wp_idx", 0):>3}  '
                  f'{si.get("dist_to_wp", 0):>6.2f}  '
                  f'{si.get("heading_err", 0):>+8.2f}  '
                  f'{si.get("cte", 0):>+7.3f}  '
                  f'{si.get("steer_frac", 0):>+6.3f}  '
                  f'{si.get("mode", "?"):>7}  '
                  f'{s_clip_str:>8}')

        # Track newly reached waypoints (only original, not synthetic bypass)
        while prev_idx < path_n.path_idx and prev_idx < len(effective_wps):
            wp = effective_wps[prev_idx]
            if wp.seq <= max(w.seq for w in waypoints):
                wps_reached.append(wp.seq)
            prev_idx += 1

        if done:
            break

        rover.update(thr, steer, nav['max_speed'], phys['wheelbase'],
                     dt, phys['turn_scale'])
        t_mono += dt

        rlat_new, rlon_new = _center_pos(rover, bm)
        flat_new, flon_new = _front_pos(rover, bm)
        result_path.append((rlat_new, rlon_new))
        cte = path_n._cte_to_seg(flat_new, flon_new, best_seg)
        xte_log.append(abs(cte))

    xte_arr  = [x for x in xte_log if x >= 0]
    rms_xte  = math.sqrt(sum(x ** 2 for x in xte_arr) / len(xte_arr)) if xte_arr else 0.0
    max_xte  = max(xte_arr, default=0.0)
    avg_xte  = sum(xte_arr) / len(xte_arr) if xte_arr else 0.0

    if verbose:
        print(f'\n{"─"*60}')
        print(f'Complete: {path_n.path_idx >= len(effective_wps)}  '
              f'{step+1} steps  {(step+1)/nav["control_rate"]:.1f}s simulated')
        print(f'Reached : {len(wps_reached)}/{len(waypoints)} waypoints')
        print(f'XTE rms : {rms_xte:.3f} m   max: {max_xte:.3f} m')

    return SimResult(
        path              = result_path,
        xte_log           = xte_log,
        waypoints_reached = wps_reached,
        rms_xte           = round(rms_xte, 4),
        max_xte           = round(max_xte, 4),
        avg_xte           = round(avg_xte, 4),
        total_steps       = step + 1,
        complete          = path_n.path_idx >= len(effective_wps),
        obstacle_polygons = [list(poly) for poly in expanded_polygons],
        rerouted_wps      = [[wp.lat, wp.lon] for wp in effective_wps],
        step_log          = step_log_data,
    )


# ── HTML result export ────────────────────────────────────────────────────────

def _write_html_result(result: SimResult,
                       waypoints: list[SimWaypoint],
                       start_lat: float, start_lon: float,
                       nav: dict,
                       outfile: str) -> None:
    """Write a self-contained Leaflet HTML file visualising the simulation result."""

    # Compute pivot waypoints
    if result.rerouted_wps:
        rw = [SimWaypoint(seq=i, lat=r[0], lon=r[1],
                          is_bypass=not any(abs(w.lat - r[0]) < 1e-8 and
                                            abs(w.lon - r[1]) < 1e-8
                                            for w in waypoints))
              for i, r in enumerate(result.rerouted_wps)]
    else:
        rw = list(waypoints)
    path_n_tmp = PathNavigator(rw, start_lat, start_lon, nav)
    pivot_wps = []
    for i in range(len(path_n_tmp._wps)):
        if not path_n_tmp._wps[i].is_bypass:
            ta = path_n_tmp._turn_angle_at(i)
            if ta >= nav.get('pivot_threshold', 25.0):
                pivot_wps.append({'lat': path_n_tmp._wps[i].lat,
                                  'lon': path_n_tmp._wps[i].lon,
                                  'turn_angle': round(ta, 1)})

    centre_lat = start_lat
    centre_lon = start_lon

    data_js = json.dumps({
        'path':        result.path,
        'xte_log':     result.xte_log,
        'waypoints':   [{'lat': w.lat, 'lon': w.lon, 'seq': w.seq} for w in waypoints],
        'pivot_wps':   pivot_wps,
        'rerouted':    result.rerouted_wps,
        'stats': {
            'complete':    result.complete,
            'rms_xte':     result.rms_xte,
            'max_xte':     result.max_xte,
            'total_steps': result.total_steps,
            'n_wps':       len(waypoints),
            'reached':     len(result.waypoints_reached),
        },
    })

    html = f"""<!DOCTYPE html>
<html><head><meta charset="utf-8">
<title>Sim Result</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<style>
body{{margin:0;font-family:Arial,sans-serif}}
#map{{height:100vh}}
#panel{{position:absolute;top:10px;right:10px;z-index:999;background:rgba(20,20,40,.9);
       color:#ddd;padding:12px 16px;border-radius:6px;font-size:12px;line-height:1.8;
       min-width:200px;border:1px solid #444}}
#panel h3{{color:#2ecc71;margin-bottom:6px;font-size:13px}}
.lbl{{color:#888}}
</style>
</head>
<body>
<div id="map"></div>
<div id="panel"><h3>Simulation Result</h3><div id="stats-text"></div></div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
const DATA = {data_js};

const map = L.map('map').setView([{centre_lat}, {centre_lon}], 18);
L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{{z}}/{{y}}/{{x}}',
  {{attribution:'ESRI',maxZoom:21,maxNativeZoom:19}}).addTo(map);

// Planned route
if (DATA.waypoints.length > 1) {{
  L.polyline(DATA.waypoints.map(w=>[w.lat,w.lon]),
    {{color:'#2ecc71',weight:2,dashArray:'8,5',opacity:.7}}).addTo(map)
    .bindTooltip('Planned route');
}}

// Waypoint markers
DATA.waypoints.forEach((w,i) => {{
  const icon = L.divIcon({{
    html:`<div style="background:#1a7a3a;color:#fff;border-radius:50%;width:20px;height:20px;
          display:flex;align-items:center;justify-content:center;font-size:10px;font-weight:bold;
          border:2px solid #fff">${{i}}</div>`,
    className:'',iconAnchor:[10,10]
  }});
  L.marker([w.lat,w.lon],{{icon}}).addTo(map).bindTooltip(`WP${{i}}`);
}});

// Pivot markers — added after path segments so they appear on top
DATA.pivot_wps.forEach(pw => {{
  const icon = L.divIcon({{
    html:`<div style="background:#e67e22;color:#fff;border-radius:50%;width:28px;height:28px;
          display:flex;align-items:center;justify-content:center;font-size:11px;font-weight:bold;
          border:3px solid #fff;box-shadow:0 1px 6px rgba(0,0,0,.8)">&#8635;</div>`,
    className:'',iconAnchor:[14,14]
  }});
  L.marker([pw.lat,pw.lon],{{icon,zIndexOffset:1000}})
    .addTo(map).bindTooltip(`Pivot ${{pw.turn_angle.toFixed(0)}}\u00b0`);
}});

// Simulated path - coloured by XTE
const xteMax = Math.max(...(DATA.xte_log||[0.001]),0.001);
for (let i=0;i+1<DATA.path.length && i<DATA.xte_log.length;i++) {{
  const t = DATA.xte_log[i]/xteMax;
  const r2 = Math.round(255*Math.min(t*2,1));
  const g2 = Math.round(255*Math.max(1-t*2+1,0));
  L.polyline([DATA.path[i],DATA.path[i+1]],
    {{color:`rgb(${{r2}},${{g2}},30)`,weight:3,opacity:.85}}).addTo(map);
}}

// Stats
const s = DATA.stats;
document.getElementById('stats-text').innerHTML = `
  <div><span class="lbl">Complete:</span> ${{s.complete?'\u2713 Yes':'\u2717 Timeout'}}</div>
  <div><span class="lbl">Duration:</span> ${{(s.total_steps/25).toFixed(1)}} s</div>
  <div><span class="lbl">WP reached:</span> ${{s.reached}}/${{s.n_wps}}</div>
  <div><span class="lbl">XTE rms:</span> ${{s.rms_xte.toFixed(3)}} m</div>
  <div><span class="lbl">XTE max:</span> ${{s.max_xte.toFixed(3)}} m</div>
  <div><span class="lbl">Pivots:</span> ${{DATA.pivot_wps.length}}</div>
  <div style="font-size:10px;color:#555;margin-top:6px">green dashed = planned<br>coloured = simulated (green=low XTE, red=high)<br>orange circles = pivot turns</div>
`;

// Fit map to path
if (DATA.path.length) {{
  map.fitBounds(L.latLngBounds(DATA.path.concat(DATA.waypoints.map(w=>[w.lat,w.lon]))).pad(.05));
}}
</script>
</body></html>"""

    with open(outfile, 'w') as f:
        f.write(html)
    print(f'Map written -> {outfile}')


# ── CLI standalone ────────────────────────────────────────────────────────────

def _load_csv(path: str) -> list[SimWaypoint]:
    wps = []
    with open(path, newline='') as f:
        for i, row in enumerate(csv.DictReader(f)):
            wps.append(SimWaypoint(
                seq=i,
                lat=float(row['lat']),
                lon=float(row['lon']),
                speed=float(row.get('speed', 0) or 0),
                hold_secs=float(row.get('hold_secs', 0) or 0),
            ))
    return wps


def _load_obstacles(path: str) -> list[list[list[float]]]:
    """Load obstacle polygons from a JSON file: [[[lat,lon],...], ...]"""
    with open(path) as f:
        data = json.load(f)
    if isinstance(data, list):
        return data
    return data.get('polygons', [])


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description='Simulate rover mission path')
    ap.add_argument('--lat',         type=float, required=True, help='Start latitude')
    ap.add_argument('--lon',         type=float, required=True, help='Start longitude')
    ap.add_argument('--heading',     type=float, default=0.0,   help='Start heading (deg N)')
    ap.add_argument('--wheelbase',   type=float, default=DEFAULT_PHYS['wheelbase'])
    ap.add_argument('--turn-scale',  type=float, default=DEFAULT_PHYS['turn_scale'])
    ap.add_argument('--obstacles',   type=str,   default=None,
                    help='JSON file with obstacle polygons [[[lat,lon],...],...]')
    ap.add_argument('--html-out',    type=str,   default=None, metavar='FILE',
                    help='Write simulation result to a Leaflet HTML file')
    ap.add_argument('--debug', action='store_true', help='Print per-step controller state')
    ap.add_argument('waypoints',     help='CSV file with lat,lon[,speed,hold_secs] columns')
    args = ap.parse_args()

    wps = _load_csv(args.waypoints)
    print(f'Loaded {len(wps)} waypoints')

    obstacles = _load_obstacles(args.obstacles) if args.obstacles else None
    if obstacles:
        print(f'Loaded {len(obstacles)} obstacle polygon(s)')

    result = simulate(wps, args.lat, args.lon, args.heading,
                      nav_params={},
                      phys_params={'wheelbase': args.wheelbase,
                                   'turn_scale': args.turn_scale},
                      obstacles=obstacles,
                      verbose=args.debug)

    print(f'Complete : {result.complete}  ({result.total_steps} steps, '
          f'{result.total_steps / DEFAULT_NAV["control_rate"]:.1f} s simulated)')
    print(f'Reached  : {len(result.waypoints_reached)}/{len(wps)} waypoints')
    print(f'XTE RMS  : {result.rms_xte:.3f} m')
    print(f'XTE max  : {result.max_xte:.3f} m')
    print(f'XTE avg  : {result.avg_xte:.3f} m')
    print(f'Path pts : {len(result.path)}')
    if result.obstacle_polygons:
        print(f'Obstacles: {len(result.obstacle_polygons)} polygon(s) (expanded)')

    if args.html_out:
        _write_html_result(result, wps, args.lat, args.lon,
                           {**DEFAULT_NAV}, args.html_out)
