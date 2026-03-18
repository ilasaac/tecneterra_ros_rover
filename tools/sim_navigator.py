"""
tools/sim_navigator.py — Software-in-the-loop simulation of the full-path Stanley navigator.

Replicates navigator.py (full-path Stanley + pivot turns) in pure Python with no ROS2
dependency.  Uses the dead-reckoning physics from simulator.py's RoverState.

Importable by monitor.py for automatic pre-mission simulation, or run standalone:

  python tools/sim_navigator.py --lat 20.727715 --lon -103.566782 --heading 90 \\
         --waypoints missions/field.csv

The navigator parameters default to the same values as navigator_params.yaml.
Physical parameters (wheelbase, turn_scale) can be overridden via DEFAULT_PHYS or
the --wheelbase / --turn-scale CLI args.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from dataclasses import dataclass, field

# Import RoverState (dead-reckoning physics) from sibling simulator.py
sys.path.insert(0, os.path.dirname(__file__))
from simulator import RoverState   # noqa: E402

# ── Constants ─────────────────────────────────────────────────────────────────

PPM_CENTER = 1500
EARTH_R    = 6_371_000.0

# Navigator parameter defaults — keep in sync with navigator_params.yaml
DEFAULT_NAV: dict = {
    'lookahead_distance':        3.0,
    'default_acceptance_radius': 0.3,
    'max_speed':                 1.5,
    'min_speed':                 0.3,
    'max_steering':              0.8,
    'align_threshold':           30.0,
    'heading_deadband':          3.0,
    'stanley_k':                 1.0,
    'stanley_softening':         0.3,
    'pivot_threshold':           60.0,
    'pivot_approach_dist':       4.0,
    'control_rate':              25.0,
    'max_timeout':               300.0,  # simulation hard stop (seconds)
}

# Physical rover parameters — tune to match the real rover
DEFAULT_PHYS: dict = {
    'wheelbase':  0.6,    # metres — distance between left and right wheels
    'turn_scale': 0.3,    # turn rate fraction (0.1 = slow, 1.0 = full differential)
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
        self._path_s     = self._compute_arclens()

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
        turn_angle  = self._turn_angle_at(self.path_idx)
        needs_pivot = (turn_angle >= pivot_thresh and self.path_idx < len(self._wps) - 1)

        s_nearest, best_seg = self._nearest_on_path(rlat, rlon)
        wp_s                = self._path_s[self.path_idx]
        dist_to_wp          = _haversine(rlat, rlon, wp.lat, wp.lon)

        # Waypoint advance
        reached = dist_to_wp < accept or (not needs_pivot and s_nearest > wp_s + accept)
        if reached:
            if wp.hold_secs > 0.0:
                self._holding  = True
                self._hold_end = t_mono + wp.hold_secs
                return PPM_CENTER, PPM_CENTER, best_seg, False
            elif needs_pivot:
                nxt = self._wps[self.path_idx + 1]
                self._pivot_hdg = _bearing_to(wp.lat, wp.lon, nxt.lat, nxt.lon)
                self._pivoting  = True
                return PPM_CENTER, PPM_CENTER, best_seg, False
            else:
                self._advance()
                return PPM_CENTER, PPM_CENTER, best_seg, self.path_idx >= len(self._wps)

        # Precision approach to pivot waypoint
        target_spd = wp.speed if wp.speed > 0 else max_spd
        if needs_pivot and dist_to_wp < pivot_app_dist:
            la_lat, la_lon = wp.lat, wp.lon
            target_spd = max(min_spd, target_spd * dist_to_wp / pivot_app_dist)
        else:
            la_lat, la_lon = self._point_at_s(s_nearest + lookahead)

        target_bearing = _bearing_to(rlat, rlon, la_lat, la_lon)
        heading_err    = ((target_bearing - heading + 180) % 360) - 180
        cte            = self._cte_to_seg(flat, flon, best_seg)

        if abs(heading_err) > align_thresh:
            spin_dir  = math.copysign(1.0, heading_err)
            steer_ppm = int(PPM_CENTER - spin_dir * max_steer * 500)
            return PPM_CENTER, steer_ppm, best_seg, False

        v_mps        = max(target_spd, min_spd)
        throttle_ppm = int(PPM_CENTER + (v_mps / max_spd) * 500)
        stanley_ang  = heading_err + math.degrees(
            math.atan2(k * cte, max(v_mps, softening)))
        stanley_ang  = max(-90.0, min(90.0, stanley_ang))
        steer_frac   = max(-max_steer, min(max_steer, stanley_ang / 45.0))
        steer_ppm    = int(PPM_CENTER - steer_frac * 500)

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
             phys_params:     dict | None = None) -> SimResult:
    """
    Run a software-in-the-loop simulation of the rover following *waypoints*.

    Parameters
    ----------
    waypoints     : ordered list of SimWaypoint
    start_lat/lon : rover starting position (degrees)
    start_heading : rover starting heading (degrees from north)
    nav_params    : override any key in DEFAULT_NAV
    phys_params   : override any key in DEFAULT_PHYS
    """
    nav   = {**DEFAULT_NAV,  **(nav_params  or {})}
    phys  = {**DEFAULT_PHYS, **(phys_params or {})}
    dt    = 1.0 / nav['control_rate']
    bm    = phys['baseline_m']

    rover  = RoverState(start_lat, start_lon, start_heading)
    path_n = PathNavigator(waypoints, start_lat, start_lon, nav)

    result_path: list[tuple[float, float]] = [_center_pos(rover, bm)]
    xte_log:     list[float]               = []
    wps_reached: list[int]                 = []
    t_mono = 0.0
    prev_idx = 0

    max_steps = int(nav['max_timeout'] / dt)

    for step in range(max_steps):
        rlat, rlon = _center_pos(rover, bm)
        flat, flon = _front_pos(rover, bm)
        thr, steer, best_seg, done = path_n.step(rlat, rlon, flat, flon,
                                                  rover.heading_deg, t_mono)
        # Track newly reached waypoints
        while prev_idx < path_n.path_idx and prev_idx < len(waypoints):
            wps_reached.append(waypoints[prev_idx].seq)
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

    return SimResult(
        path              = result_path,
        xte_log           = xte_log,
        waypoints_reached = wps_reached,
        rms_xte           = round(rms_xte, 4),
        max_xte           = round(max_xte, 4),
        avg_xte           = round(avg_xte, 4),
        total_steps       = step + 1,
        complete          = path_n.path_idx >= len(waypoints),
    )


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


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description='Simulate rover mission path')
    ap.add_argument('--lat',         type=float, required=True, help='Start latitude')
    ap.add_argument('--lon',         type=float, required=True, help='Start longitude')
    ap.add_argument('--heading',     type=float, default=0.0,   help='Start heading (deg N)')
    ap.add_argument('--wheelbase',   type=float, default=DEFAULT_PHYS['wheelbase'])
    ap.add_argument('--turn-scale',  type=float, default=DEFAULT_PHYS['turn_scale'])
    ap.add_argument('waypoints',     help='CSV file with lat,lon[,speed,hold_secs] columns')
    args = ap.parse_args()

    wps = _load_csv(args.waypoints)
    print(f'Loaded {len(wps)} waypoints')

    result = simulate(wps, args.lat, args.lon, args.heading,
                      phys_params={'wheelbase': args.wheelbase,
                                   'turn_scale': args.turn_scale})

    print(f'Complete : {result.complete}  ({result.total_steps} steps, '
          f'{result.total_steps / DEFAULT_NAV["control_rate"]:.1f} s simulated)')
    print(f'Reached  : {len(result.waypoints_reached)}/{len(wps)} waypoints')
    print(f'XTE RMS  : {result.rms_xte:.3f} m')
    print(f'XTE max  : {result.max_xte:.3f} m')
    print(f'XTE avg  : {result.avg_xte:.3f} m')
    print(f'Path pts : {len(result.path)}')
