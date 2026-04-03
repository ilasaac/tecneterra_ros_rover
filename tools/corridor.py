"""
tools/corridor.py — Corridor data model and turn-arc geometry.

Shared by sim_navigator.py (SIL) and navigator.py (ROS2).

A corridor mission is a list of Corridor objects (polyline centerline + width).
Corridors connect via turns at headlands.  Turn type is adaptive:
  - 'arc'  — smooth circular arc (when headland is wide enough)
  - 'spin' — stop, spin in place, continue (tight headlands)
  - 'auto' — decide from geometry (default)

corridors_to_path() converts a corridor mission into a single continuous
polyline that the Stanley controller can follow.  Arc turns become
interpolated points; spin turns become consecutive points at the same
location with different bearings (the navigator will spin naturally
from the large heading error).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

# ── Constants ────────────────────────────────────────────────────────────────

EARTH_R = 6_371_000.0


# ── Geometry helpers ─────────────────────────────────────────────────────────

def _haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Distance in metres between two GPS points."""
    rlat1, rlat2 = math.radians(lat1), math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(rlat1) * math.cos(rlat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_R * math.asin(math.sqrt(a))


def _bearing_to(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Bearing in degrees (0-360) from point 1 to point 2."""
    rlat1, rlat2 = math.radians(lat1), math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(rlat2)
    y = math.cos(rlat1) * math.sin(rlat2) - math.sin(rlat1) * math.cos(rlat2) * math.cos(dlon)
    return math.degrees(math.atan2(x, y)) % 360


def _destination(lat: float, lon: float, bearing_deg: float, dist_m: float) -> tuple[float, float]:
    """Move from (lat, lon) along bearing_deg for dist_m metres. Returns (lat2, lon2)."""
    rlat = math.radians(lat)
    rbrg = math.radians(bearing_deg)
    d = dist_m / EARTH_R
    lat2 = math.asin(math.sin(rlat) * math.cos(d) +
                      math.cos(rlat) * math.sin(d) * math.cos(rbrg))
    lon2 = math.radians(lon) + math.atan2(
        math.sin(rbrg) * math.sin(d) * math.cos(rlat),
        math.cos(d) - math.sin(rlat) * math.sin(lat2))
    return math.degrees(lat2), math.degrees(lon2)


def _normalize_angle(deg: float) -> float:
    """Normalize angle to (-180, 180]."""
    return ((deg + 180) % 360) - 180


# ── Data model ───────────────────────────────────────────────────────────────

@dataclass
class Corridor:
    corridor_id: int
    centerline: list[tuple[float, float]]   # [(lat, lon), ...]
    width: float = 1.5                       # half-width in metres
    speed: float = 0.0                       # target speed m/s (0 = use default)
    speeds: list[float] = field(default_factory=list)  # per-vertex speeds (parallel to centerline)
    next_corridor_id: int = -1               # -1 = last corridor
    turn_type: str = 'auto'                  # 'auto' | 'arc' | 'spin'
    headland_width: float = 0.0              # available turn space in metres (0 = auto)


@dataclass
class CorridorMission:
    corridors: list[Corridor] = field(default_factory=list)
    min_turn_radius: float = 3.0             # metres (rover physical minimum)
    headland_width: float = 0.0              # crossing corridor width (0 = auto: 2× row width)


# ── Turn arc computation ─────────────────────────────────────────────────────

def compute_turn_arc(
    exit_lat: float, exit_lon: float, exit_heading: float,
    entry_lat: float, entry_lon: float, entry_heading: float,
    min_turn_radius: float,
    point_spacing: float = 0.3,
) -> list[tuple[float, float]]:
    """Compute a circular arc connecting two corridor endpoints.

    The arc starts at (exit_lat, exit_lon) heading exit_heading and ends
    at (entry_lat, entry_lon) heading entry_heading.  Uses a simple
    single-arc Dubins-like approach: find the turn center, compute the
    sweep angle, and interpolate points along the arc.

    Parameters
    ----------
    exit_lat, exit_lon   : GPS position where the rover exits corridor N
    exit_heading         : heading (degrees) at corridor N exit
    entry_lat, entry_lon : GPS position where the rover enters corridor N+1
    entry_heading        : heading (degrees) at corridor N+1 entry
    min_turn_radius      : minimum turn radius in metres
    point_spacing        : distance between interpolated arc points (metres)

    Returns
    -------
    List of (lat, lon) points along the arc (includes start, excludes end).
    Empty list if the arc is degenerate.
    """
    # Heading change (signed, positive = right turn)
    delta = _normalize_angle(entry_heading - exit_heading)
    if abs(delta) < 1.0:
        # Nearly straight — just return the exit point
        return [(exit_lat, exit_lon)]

    # For ~180° turns, heading difference alone is ambiguous (could be left or right).
    # Use the entry point's position relative to the exit heading to determine
    # which side to turn: if the entry point is to the right of the exit heading,
    # turn right (clockwise); otherwise turn left.
    if abs(delta) > 170.0:
        exit_to_entry = _bearing_to(exit_lat, exit_lon, entry_lat, entry_lon)
        cross = _normalize_angle(exit_to_entry - exit_heading)
        delta = 180.0 if cross > 0 else -180.0

    # Determine turn direction: positive delta = right turn = center to the right
    # Turn center is perpendicular to heading at distance = radius
    if delta > 0:
        # Right turn: center is 90° clockwise from heading
        center_bearing = (exit_heading + 90) % 360
    else:
        # Left turn: center is 90° counter-clockwise from heading
        center_bearing = (exit_heading - 90) % 360

    # Use the midpoint between exit and entry to estimate the actual radius
    chord = _haversine(exit_lat, exit_lon, entry_lat, entry_lon)
    # For a circular arc: chord = 2 * R * sin(|delta|/2)
    half_delta_rad = math.radians(abs(delta) / 2)
    sin_half = math.sin(half_delta_rad)
    if sin_half < 0.01:
        return [(exit_lat, exit_lon)]
    radius = max(min_turn_radius, chord / (2 * sin_half))

    # Compute turn center from exit point
    center_lat, center_lon = _destination(exit_lat, exit_lon, center_bearing, radius)

    # Arc sweep angle
    sweep = abs(delta)
    arc_len = math.radians(sweep) * radius

    # Number of interpolated points
    n_points = max(2, int(arc_len / point_spacing) + 1)

    # Generate points along the arc
    # Start angle: bearing from center to exit point
    start_angle = _bearing_to(center_lat, center_lon, exit_lat, exit_lon)

    points = []
    for i in range(n_points):
        frac = i / (n_points - 1) if n_points > 1 else 0.0
        if delta > 0:
            # Right turn: center is to the right, rover moves CCW around center
            angle = start_angle + frac * sweep
        else:
            # Left turn: center is to the left, rover moves CW around center
            angle = start_angle - frac * sweep
        pt_lat, pt_lon = _destination(center_lat, center_lon, angle, radius)
        points.append((pt_lat, pt_lon))

    return points


def compute_spin_turn(
    exit_lat: float, exit_lon: float,
    entry_lat: float, entry_lon: float,
) -> list[tuple[float, float]]:
    """Generate points for a spin-in-place turn.

    Returns two points at the exit position (the rover stops there and spins)
    plus a short approach toward the entry corridor.  The Stanley controller
    will see a large heading error and spin naturally.
    """
    return [(exit_lat, exit_lon)]


# ── Corridor → path conversion ───────────────────────────────────────────────

def _corridor_exit_heading(corridor: Corridor) -> float:
    """Bearing of the last segment of the corridor centerline."""
    cl = corridor.centerline
    if len(cl) < 2:
        return 0.0
    return _bearing_to(cl[-2][0], cl[-2][1], cl[-1][0], cl[-1][1])


def _corridor_entry_heading(corridor: Corridor) -> float:
    """Bearing of the first segment of the corridor centerline."""
    cl = corridor.centerline
    if len(cl) < 2:
        return 0.0
    return _bearing_to(cl[0][0], cl[0][1], cl[1][0], cl[1][1])


def corridors_to_path(
    mission: CorridorMission,
    default_speed: float = 0.0,
) -> list[tuple[float, float, float, float]]:
    """Convert a corridor mission into a continuous polyline path.

    Returns a list of (lat, lon, speed, corridor_half_width) tuples.
    The Stanley controller follows this as a simple polyline.

    Turn arcs or spin points are inserted between consecutive corridors.
    """
    if not mission.corridors:
        return []

    # Build corridor lookup
    by_id = {c.corridor_id: c for c in mission.corridors}

    # Walk the corridor chain
    path: list[tuple[float, float, float, float]] = []
    current = mission.corridors[0]
    visited = set()

    while current and current.corridor_id not in visited:
        visited.add(current.corridor_id)
        fallback_speed = current.speed if current.speed > 0 else default_speed
        has_per_vertex = len(current.speeds) == len(current.centerline)

        # Append all centerline points of this corridor
        for i, (lat, lon) in enumerate(current.centerline):
            spd = current.speeds[i] if has_per_vertex and current.speeds[i] > 0 else fallback_speed
            path.append((lat, lon, spd, current.width))

        # Find next corridor
        next_c = by_id.get(current.next_corridor_id)
        if next_c is None:
            break

        # Determine turn type
        exit_heading = _corridor_exit_heading(current)
        entry_heading = _corridor_entry_heading(next_c)
        heading_change = abs(_normalize_angle(entry_heading - exit_heading))

        turn_type = current.turn_type
        if turn_type == 'auto':
            if heading_change < 10.0:
                turn_type = 'none'
            else:
                # Default: 3-point headland (spin + cross + spin) — safest
                turn_type = 'headland'

        exit_pt = current.centerline[-1]
        entry_pt = next_c.centerline[0]
        crossing_width = mission.headland_width if mission.headland_width > 0 else max(current.width, next_c.width) * 2

        if turn_type == 'headland':
            # 3-point headland: straight crossing segment between row ends.
            # The rover will:
            #   1. Reach row end → spin (heading error to crossing bearing > align_thresh)
            #   2. Drive straight across headland (CTE on crossing segment)
            #   3. Reach crossing end → spin (heading error to next row > align_thresh)
            # No new control code needed — align-spin handles the spins naturally.
            path.append((exit_pt[0], exit_pt[1], 0.0, crossing_width))   # crossing start
            path.append((entry_pt[0], entry_pt[1], 0.0, crossing_width)) # crossing end

        elif turn_type == 'arc':
            arc_pts = compute_turn_arc(
                exit_pt[0], exit_pt[1], exit_heading,
                entry_pt[0], entry_pt[1], entry_heading,
                mission.min_turn_radius,
            )
            turn_width = min(current.width, next_c.width)
            for lat, lon in arc_pts[1:]:
                path.append((lat, lon, 0.0, turn_width))

        elif turn_type == 'spin':
            # Legacy single-point spin (rover spins at row end, then drives to next row)
            path.append((exit_pt[0], exit_pt[1], 0.0, crossing_width))

        # else: 'none' — corridors connect directly

        current = next_c

    return path


# ── JSON serialization ───────────────────────────────────────────────────────

def corridor_mission_to_json(mission: CorridorMission) -> str:
    """Serialize a CorridorMission to JSON string."""
    import json
    data = {
        'min_turn_radius': mission.min_turn_radius,
        'headland_width': mission.headland_width,
        'corridors': [
            {
                'corridor_id': c.corridor_id,
                'centerline': [[lat, lon] for lat, lon in c.centerline],
                'width': c.width,
                'speed': c.speed,
                'speeds': c.speeds,
                'next_corridor_id': c.next_corridor_id,
                'turn_type': c.turn_type,
                'headland_width': c.headland_width,
            }
            for c in mission.corridors
        ],
    }
    return json.dumps(data, separators=(',', ':'))


def corridor_mission_from_json(json_str: str) -> CorridorMission:
    """Deserialize a CorridorMission from JSON string."""
    import json
    data = json.loads(json_str)
    corridors = []
    for c in data.get('corridors', []):
        corridors.append(Corridor(
            corridor_id=c['corridor_id'],
            centerline=[(pt[0], pt[1]) for pt in c['centerline']],
            width=c.get('width', 1.5),
            speed=c.get('speed', 0.0),
            speeds=c.get('speeds', []),
            next_corridor_id=c.get('next_corridor_id', -1),
            turn_type=c.get('turn_type', 'auto'),
            headland_width=c.get('headland_width', 0.0),
        ))
    return CorridorMission(
        corridors=corridors,
        min_turn_radius=data.get('min_turn_radius', 3.0),
        headland_width=data.get('headland_width', 0.0),
    )


# ── Auto-split & path optimization ───────────────────────────────────────────

def auto_split_corridors(
    points: list[tuple[float, float]],
    turn_threshold_deg: float = 70.0,
    min_segment_points: int = 3,
    width: float = 1.5,
    speeds: list[float] | None = None,
) -> CorridorMission:
    """Split a raw GPS polyline into corridors at sharp turns.

    Walks the polyline computing heading changes.  Where the heading
    change exceeds turn_threshold_deg, a new corridor starts.
    Per-vertex speeds are preserved and split alongside the points.

    Parameters
    ----------
    points              : raw GPS polyline [(lat, lon), ...]
    turn_threshold_deg  : heading change (degrees) that triggers a split
    min_segment_points  : minimum points per corridor (avoids tiny fragments)
    width               : corridor half-width (metres)
    speeds              : per-vertex recorded speeds (parallel to points), or None

    Returns
    -------
    CorridorMission with auto-connected corridors.
    """
    if len(points) < 2:
        return CorridorMission()

    spd_list = speeds if speeds and len(speeds) == len(points) else [0.0] * len(points)
    has_turn_markers = any(s < 0 for s in spd_list)

    corridors: list[Corridor] = []
    current_pts: list[tuple[float, float]] = [points[0]]
    current_spd: list[float] = [spd_list[0]]
    prev_heading: float | None = None

    i = 1
    while i < len(points):
        if spd_list[i] < 0:
            turn_lats = []
            turn_lons = []
            while i < len(points) and spd_list[i] < 0:
                turn_lats.append(points[i][0])
                turn_lons.append(points[i][1])
                i += 1
            clat = sum(turn_lats) / len(turn_lats)
            clon = sum(turn_lons) / len(turn_lons)
            current_pts.append((clat, clon))
            current_spd.append(0.0)
            if len(current_pts) >= 2:
                corridors.append(Corridor(
                    corridor_id=len(corridors),
                    centerline=list(current_pts),
                    width=width,
                    speeds=list(current_spd),
                    next_corridor_id=len(corridors) + 1,
                    turn_type='none',
                ))
            current_pts = [(clat, clon)]
            current_spd = [0.0]
            prev_heading = None
            continue

        heading = _bearing_to(points[i - 1][0], points[i - 1][1],
                              points[i][0], points[i][1])

        # Heading-based split — only used when no turn markers exist
        if not has_turn_markers and prev_heading is not None:
            delta = abs(_normalize_angle(heading - prev_heading))
            if delta >= turn_threshold_deg and len(current_pts) >= min_segment_points:
                corridors.append(Corridor(
                    corridor_id=len(corridors),
                    centerline=list(current_pts),
                    width=width,
                    speeds=list(current_spd),
                    next_corridor_id=len(corridors) + 1,
                ))
                current_pts = [current_pts[-1]]
                current_spd = [current_spd[-1]]

        current_pts.append((points[i][0], points[i][1]))
        current_spd.append(spd_list[i])
        prev_heading = heading
        i += 1

    # Save final corridor
    if len(current_pts) >= 2:
        corridors.append(Corridor(
            corridor_id=len(corridors),
            centerline=list(current_pts),
            width=width,
            speeds=list(current_spd),
            next_corridor_id=-1,
        ))

    # Fix chain
    for i in range(len(corridors) - 1):
        corridors[i].next_corridor_id = i + 1
    if corridors:
        corridors[-1].next_corridor_id = -1

    return CorridorMission(corridors=corridors)


def optimize_corridor_speeds(
    path: list[tuple[float, float, float, float]],
    max_speed: float = 1.5,
    min_speed: float = 0.3,
    curvature_window: int = 5,
    curvature_slowdown: float = 5.0,
) -> list[tuple[float, float, float, float]]:
    """Adjust speed based on path curvature — only reduce, never increase.

    Each point's recorded speed is a ceiling.  Curvature analysis may
    reduce it further but never above the recorded value.  Points with
    speed=0 (no recorded speed) use max_speed as ceiling.

    Parameters
    ----------
    path                : [(lat, lon, speed, width), ...]
    max_speed           : ceiling for points with no recorded speed (speed=0)
    min_speed           : absolute floor (never slower than this)
    curvature_window    : number of points for heading change window
    curvature_slowdown  : heading change (deg) per point that halves speed

    Returns
    -------
    Same path with adjusted speed values.
    """
    if len(path) < 3:
        return path

    result = list(path)
    for i in range(1, len(path) - 1):
        # Compute heading change over window
        lo = max(0, i - curvature_window)
        hi = min(len(path) - 1, i + curvature_window)
        if hi <= lo:
            continue
        h_start = _bearing_to(path[lo][0], path[lo][1], path[lo + 1][0], path[lo + 1][1])
        h_end = _bearing_to(path[hi - 1][0], path[hi - 1][1], path[hi][0], path[hi][1])
        curve = abs(_normalize_angle(h_end - h_start)) / max(1, hi - lo)

        lat, lon, recorded_spd, width = result[i]
        # Ceiling = recorded speed if available, else max_speed
        ceiling = recorded_spd if recorded_spd > 0 else max_speed
        # Reduce proportionally to curvature, but never above ceiling
        factor = max(0.0, 1.0 - curve / max(curvature_slowdown, 0.01))
        spd = max(min_speed, ceiling * factor)

        result[i] = (lat, lon, spd, width)

    return result


# ── Corridor grid generator ─────────────────────────────────────────────────

def generate_corridor_grid(
    origin_lat: float, origin_lon: float,
    heading_deg: float = 0.0,
    row_count: int = 5,
    row_length_m: float = 30.0,
    row_spacing_m: float = 2.0,
    corridor_width: float = 1.0,
    speed: float = 0.0,
    min_turn_radius: float = 3.0,
    turn_type: str = 'auto',
) -> CorridorMission:
    """Generate a serpentine corridor grid (parallel rows with headland turns).

    Parameters
    ----------
    origin_lat, origin_lon : bottom-left corner of the field
    heading_deg            : row direction (degrees from north, 0 = north)
    row_count              : number of parallel rows
    row_length_m           : length of each row
    row_spacing_m          : distance between adjacent row centerlines
    corridor_width         : half-width of each corridor (metres)
    speed                  : target speed (0 = use navigator default)
    min_turn_radius        : minimum turn radius for arc turns
    turn_type              : 'auto', 'arc', or 'spin'

    Returns
    -------
    CorridorMission with serpentine-connected corridors.
    """
    # Row direction and perpendicular
    row_brg = heading_deg % 360
    perp_brg = (row_brg + 90) % 360

    corridors = []
    for i in range(row_count):
        # Row start: offset perpendicular from origin
        row_start = _destination(origin_lat, origin_lon, perp_brg, i * row_spacing_m)

        # Alternate direction for serpentine pattern
        if i % 2 == 0:
            # Forward
            row_end = _destination(row_start[0], row_start[1], row_brg, row_length_m)
            centerline = [row_start, row_end]
        else:
            # Reverse
            row_end = _destination(row_start[0], row_start[1], row_brg, row_length_m)
            centerline = [row_end, row_start]  # reversed

        # Headland width = row spacing (space available for turning)
        headland_w = row_spacing_m

        corridors.append(Corridor(
            corridor_id=i,
            centerline=centerline,
            width=corridor_width,
            speed=speed,
            next_corridor_id=i + 1 if i < row_count - 1 else -1,
            turn_type=turn_type,
            headland_width=headland_w,
        ))

    return CorridorMission(
        corridors=corridors,
        min_turn_radius=min_turn_radius,
    )
