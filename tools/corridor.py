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
    next_corridor_id: int = -1               # -1 = last corridor
    turn_type: str = 'auto'                  # 'auto' | 'arc' | 'spin'
    headland_width: float = 0.0              # available turn space in metres (0 = auto)


@dataclass
class CorridorMission:
    corridors: list[Corridor] = field(default_factory=list)
    min_turn_radius: float = 3.0             # metres (rover physical minimum)


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
        speed = current.speed if current.speed > 0 else default_speed

        # Append all centerline points of this corridor
        for lat, lon in current.centerline:
            path.append((lat, lon, speed, current.width))

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
            # Decide based on available space and heading change
            if heading_change < 10.0:
                # Nearly straight — no turn needed
                turn_type = 'none'
            else:
                # Check if headland is wide enough for arc
                exit_pt = current.centerline[-1]
                entry_pt = next_c.centerline[0]
                headland_dist = _haversine(exit_pt[0], exit_pt[1],
                                           entry_pt[0], entry_pt[1])
                headland_w = current.headland_width if current.headland_width > 0 else headland_dist
                needed = 2 * mission.min_turn_radius * math.sin(math.radians(heading_change / 2))
                if headland_w >= needed:
                    turn_type = 'arc'
                else:
                    turn_type = 'spin'

        if turn_type == 'arc':
            exit_pt = current.centerline[-1]
            entry_pt = next_c.centerline[0]
            arc_pts = compute_turn_arc(
                exit_pt[0], exit_pt[1], exit_heading,
                entry_pt[0], entry_pt[1], entry_heading,
                mission.min_turn_radius,
            )
            # Use min speed during turns, corridor width = min of both corridors
            turn_width = min(current.width, next_c.width)
            for lat, lon in arc_pts[1:]:   # skip first (= corridor last point)
                path.append((lat, lon, 0.0, turn_width))  # speed 0 = min_speed

        elif turn_type == 'spin':
            exit_pt = current.centerline[-1]
            entry_pt = next_c.centerline[0]
            spin_pts = compute_spin_turn(
                exit_pt[0], exit_pt[1],
                entry_pt[0], entry_pt[1],
            )
            turn_width = min(current.width, next_c.width)
            for lat, lon in spin_pts:
                path.append((lat, lon, 0.0, turn_width))

        # else: 'none' — corridors connect directly

        current = next_c

    return path


# ── JSON serialization ───────────────────────────────────────────────────────

def corridor_mission_to_json(mission: CorridorMission) -> str:
    """Serialize a CorridorMission to JSON string."""
    import json
    data = {
        'min_turn_radius': mission.min_turn_radius,
        'corridors': [
            {
                'corridor_id': c.corridor_id,
                'centerline': [[lat, lon] for lat, lon in c.centerline],
                'width': c.width,
                'speed': c.speed,
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
            next_corridor_id=c.get('next_corridor_id', -1),
            turn_type=c.get('turn_type', 'auto'),
            headland_width=c.get('headland_width', 0.0),
        ))
    return CorridorMission(
        corridors=corridors,
        min_turn_radius=data.get('min_turn_radius', 3.0),
    )


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
