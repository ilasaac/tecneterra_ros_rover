"""
lane_graph.py — Directed lane graph for berry field navigation.

A lane map is a directed graph of pre-defined lane segments (rows + headlands).
Each lane has a start and end point and can only be traversed in one direction.
Connections between lanes define junctions where smooth arc turns connect them.
Obstacles prune infeasible connections (turn arc hits obstacle).

Used by:
  - navigator.py  : lane-aware return-to-base routing
  - mission_planner.py : lane map editor + mission generation
  - sim_navigator.py   : SIL testing

Pure Python, no ROS2 dependency.
"""

from __future__ import annotations

import heapq
import json
import math
from dataclasses import dataclass, field
from typing import Optional

from agri_rover_navigator.corridor import (
    compute_turn_arc, _bearing_to, _haversine, _destination, _normalize_angle,
)

# ── Constants ────────────────────────────────────────────────────────────────

M_PER_DEG_LAT = 111_320.0


# ── Data model ───────────────────────────────────────────────────────────────

@dataclass
class Lane:
    id: str
    centerline: list[tuple[float, float]]  # [(lat, lon), ...] — polyline (>=2 points)
    speed: float = 0.8                      # m/s
    lane_type: str = 'row'                  # 'row' | 'headland'

    @property
    def start(self) -> tuple[float, float]:
        return self.centerline[0]

    @property
    def end(self) -> tuple[float, float]:
        return self.centerline[-1]

    def length(self) -> float:
        total = 0.0
        for i in range(1, len(self.centerline)):
            total += _haversine(self.centerline[i - 1][0], self.centerline[i - 1][1],
                                self.centerline[i][0], self.centerline[i][1])
        return total

    def start_bearing(self) -> float:
        cl = self.centerline
        if len(cl) < 2:
            return 0.0
        return _bearing_to(cl[0][0], cl[0][1], cl[1][0], cl[1][1])

    def end_bearing(self) -> float:
        cl = self.centerline
        if len(cl) < 2:
            return 0.0
        return _bearing_to(cl[-2][0], cl[-2][1], cl[-1][0], cl[-1][1])


@dataclass
class Connection:
    from_lane: str
    to_lane: str
    turn_radius: float = 2.0
    arc_points: list[tuple[float, float]] = field(default_factory=list)
    blocked: bool = False       # True if arc hits an obstacle
    arc_length: float = 0.0     # metres


@dataclass
class LaneMap:
    lanes: dict[str, Lane] = field(default_factory=dict)
    connections: list[Connection] = field(default_factory=list)
    base_lane: str = ''
    base_position: tuple[float, float] = (0.0, 0.0)
    min_turn_radius: float = 2.0


# ── Parsing ──────────────────────────────────────────────────────────────────

def lane_map_from_json(raw: str) -> LaneMap:
    """Parse lane map JSON string into LaneMap."""
    d = json.loads(raw)
    lm = LaneMap()
    lm.min_turn_radius = d.get('min_turn_radius', 2.0)
    lm.base_lane = d.get('base_lane', '')
    bp = d.get('base_position', [0.0, 0.0])
    lm.base_position = (bp[0], bp[1])

    for ld in d.get('lanes', []):
        # Support polyline centerline or legacy start/end
        if 'centerline' in ld:
            cl = [(p[0], p[1]) for p in ld['centerline']]
        else:
            s = ld['start']
            e = ld['end']
            cl = [(s[0], s[1]), (e[0], e[1])]
        lane = Lane(
            id=ld['id'],
            centerline=cl,
            speed=ld.get('speed', 0.8),
            lane_type=ld.get('type', 'row'),
        )
        lm.lanes[lane.id] = lane

    for cd in d.get('connections', []):
        conn = Connection(
            from_lane=cd['from'],
            to_lane=cd['to'],
            turn_radius=cd.get('turn_radius', lm.min_turn_radius),
        )
        lm.connections.append(conn)

    return lm


def lane_map_to_json(lm: LaneMap) -> str:
    """Serialize LaneMap to JSON string."""
    lanes = []
    for lane in lm.lanes.values():
        lanes.append({
            'id': lane.id,
            'centerline': [list(p) for p in lane.centerline],
            'speed': lane.speed,
            'type': lane.lane_type,
        })
    conns = []
    for c in lm.connections:
        conns.append({
            'from': c.from_lane,
            'to': c.to_lane,
            'turn_radius': c.turn_radius,
        })
    return json.dumps({
        'lanes': lanes,
        'connections': conns,
        'base_lane': lm.base_lane,
        'base_position': list(lm.base_position),
        'min_turn_radius': lm.min_turn_radius,
    }, indent=2)


# ── Arc computation + obstacle check ─────────────────────────────────────────

def _point_in_polygon(lat: float, lon: float,
                      polygon: list[tuple[float, float]]) -> bool:
    """Ray-casting point-in-polygon test (flat-earth approximation)."""
    n = len(polygon)
    inside = False
    j = n - 1
    for i in range(n):
        yi, xi = polygon[i]
        yj, xj = polygon[j]
        if ((yi > lat) != (yj > lat)) and \
           (lon < (xj - xi) * (lat - yi) / (yj - yi + 1e-15) + xi):
            inside = not inside
        j = i
    return inside


def _seg_min_dist_to_polygon(p_lat: float, p_lon: float,
                             polygon: list[tuple[float, float]],
                             cos_lat: float) -> float:
    """Minimum distance from point to any edge of polygon (metres, flat-earth)."""
    px = p_lon * M_PER_DEG_LAT * cos_lat
    py = p_lat * M_PER_DEG_LAT
    min_d = float('inf')
    n = len(polygon)
    for i in range(n):
        ax = polygon[i][1] * M_PER_DEG_LAT * cos_lat
        ay = polygon[i][0] * M_PER_DEG_LAT
        bx = polygon[(i + 1) % n][1] * M_PER_DEG_LAT * cos_lat
        by = polygon[(i + 1) % n][0] * M_PER_DEG_LAT
        dx, dy = bx - ax, by - ay
        seg_len2 = dx * dx + dy * dy
        if seg_len2 < 1e-10:
            d = math.hypot(px - ax, py - ay)
        else:
            t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len2))
            d = math.hypot(px - (ax + t * dx), py - (ay + t * dy))
        if d < min_d:
            min_d = d
    return min_d


def compute_all_arcs(lm: LaneMap,
                     obstacles: list[list[tuple[float, float]]] | None = None,
                     clearance_m: float = 1.0) -> None:
    """Compute turn arcs for all connections and check against obstacles.

    Args:
        lm: Lane map (connections are modified in-place)
        obstacles: List of obstacle polygons [(lat,lon), ...] (already expanded)
        clearance_m: Minimum clearance from arc points to obstacle edges
    """
    obs = obstacles or []

    for conn in lm.connections:
        from_lane = lm.lanes.get(conn.from_lane)
        to_lane = lm.lanes.get(conn.to_lane)
        if not from_lane or not to_lane:
            conn.blocked = True
            continue

        # Compute arc from end of from_lane to start of to_lane
        exit_hdg = from_lane.end_bearing()
        entry_hdg = to_lane.start_bearing()
        arc_pts = compute_turn_arc(
            from_lane.end[0], from_lane.end[1], exit_hdg,
            to_lane.start[0], to_lane.start[1], entry_hdg,
            conn.turn_radius,
            point_spacing=0.2,
        )
        conn.arc_points = arc_pts

        # Compute arc length
        total = 0.0
        for i in range(1, len(arc_pts)):
            total += _haversine(arc_pts[i - 1][0], arc_pts[i - 1][1],
                                arc_pts[i][0], arc_pts[i][1])
        # Add distance from last arc point to start of to_lane
        if arc_pts:
            total += _haversine(arc_pts[-1][0], arc_pts[-1][1],
                                to_lane.start[0], to_lane.start[1])
        conn.arc_length = total

        # Check obstacle clearance
        conn.blocked = False
        if obs:
            cos_lat = math.cos(math.radians(from_lane.end[0]))
            for pt in arc_pts:
                for poly in obs:
                    if _point_in_polygon(pt[0], pt[1], poly):
                        conn.blocked = True
                        break
                    if _seg_min_dist_to_polygon(pt[0], pt[1], poly, cos_lat) < clearance_m:
                        conn.blocked = True
                        break
                if conn.blocked:
                    break

    # Also check lane centerline points against obstacles
    for lane in lm.lanes.values():
        lane._blocked = False  # type: ignore
        if obs:
            cos_lat = math.cos(math.radians(lane.start[0]))
            for pt in lane.centerline:
                for poly in obs:
                    if _point_in_polygon(pt[0], pt[1], poly):
                        lane._blocked = True  # type: ignore
                        break
                    if _seg_min_dist_to_polygon(pt[0], pt[1], poly, cos_lat) < clearance_m:
                        lane._blocked = True  # type: ignore
                        break
                if getattr(lane, '_blocked', False):
                    break


# ── Graph + Dijkstra ─────────────────────────────────────────────────────────

def find_nearest_lane(lm: LaneMap, lat: float, lon: float
                      ) -> tuple[Optional[str], float]:
    """Find the lane closest to (lat, lon) and the distance along it.

    Returns (lane_id, fraction) where fraction is 0..1 along the lane.
    Returns (None, 0) if no lanes.
    """
    best_id: Optional[str] = None
    best_dist = float('inf')
    best_frac = 0.0
    cos_lat = math.cos(math.radians(lat))

    for lane in lm.lanes.values():
        if getattr(lane, '_blocked', False):
            continue
        # Project point onto each segment of the polyline
        cl = lane.centerline
        total_len = lane.length()
        cum_len = 0.0
        px = lon * M_PER_DEG_LAT * cos_lat
        py = lat * M_PER_DEG_LAT
        for si in range(len(cl) - 1):
            ax = cl[si][1] * M_PER_DEG_LAT * cos_lat
            ay = cl[si][0] * M_PER_DEG_LAT
            bx = cl[si + 1][1] * M_PER_DEG_LAT * cos_lat
            by = cl[si + 1][0] * M_PER_DEG_LAT
            dx, dy = bx - ax, by - ay
            seg_len2 = dx * dx + dy * dy
            seg_len = math.sqrt(seg_len2) if seg_len2 > 1e-10 else 0.0
            if seg_len2 < 1e-10:
                d = math.hypot(px - ax, py - ay)
                t = 0.0
            else:
                t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len2))
                d = math.hypot(px - (ax + t * dx), py - (ay + t * dy))
            if d < best_dist:
                best_dist = d
                best_id = lane.id
                best_frac = (cum_len + t * seg_len) / max(total_len, 1e-6)
            cum_len += seg_len

    return best_id, best_frac


def shortest_path(lm: LaneMap, from_lane: str, to_lane: str
                  ) -> list[str]:
    """Find shortest lane sequence from from_lane to to_lane using Dijkstra.

    Returns list of lane IDs in order (includes from_lane and to_lane).
    Returns empty list if no path exists.
    """
    # Build adjacency: lane_id → [(neighbor_lane_id, cost)]
    adj: dict[str, list[tuple[str, float]]] = {lid: [] for lid in lm.lanes}

    for conn in lm.connections:
        if conn.blocked:
            continue
        fl = lm.lanes.get(conn.from_lane)
        tl = lm.lanes.get(conn.to_lane)
        if not fl or not tl:
            continue
        if getattr(fl, '_blocked', False) or getattr(tl, '_blocked', False):
            continue
        # Cost = remaining distance on from_lane (end) + arc + to_lane length
        cost = conn.arc_length + tl.length()
        adj.setdefault(conn.from_lane, []).append((conn.to_lane, cost))

    # Dijkstra
    dist: dict[str, float] = {lid: float('inf') for lid in lm.lanes}
    prev: dict[str, Optional[str]] = {lid: None for lid in lm.lanes}
    dist[from_lane] = 0.0
    pq: list[tuple[float, str]] = [(0.0, from_lane)]

    while pq:
        d, u = heapq.heappop(pq)
        if d > dist[u]:
            continue
        if u == to_lane:
            break
        for v, w in adj.get(u, []):
            nd = d + w
            if nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))

    # Reconstruct path
    if dist[to_lane] == float('inf'):
        return []

    path = []
    cur: Optional[str] = to_lane
    while cur is not None:
        path.append(cur)
        cur = prev[cur]
    path.reverse()
    return path


# ── Path generation ──────────────────────────────────────────────────────────

def _connection_between(lm: LaneMap, from_id: str, to_id: str
                        ) -> Optional[Connection]:
    """Find the connection from from_id to to_id."""
    for conn in lm.connections:
        if conn.from_lane == from_id and conn.to_lane == to_id:
            return conn
    return None


def path_to_waypoints(lm: LaneMap, lane_ids: list[str],
                      default_speed: float = 0.8
                      ) -> list[tuple[float, float, float]]:
    """Convert a lane sequence into a list of (lat, lon, speed) waypoints.

    Includes lane centerline points and arc turn points at junctions.
    """
    if not lane_ids:
        return []

    waypoints: list[tuple[float, float, float]] = []

    for i, lid in enumerate(lane_ids):
        lane = lm.lanes.get(lid)
        if not lane:
            continue

        speed = lane.speed if lane.speed > 0 else default_speed

        # Add all centerline points (polyline — supports curved lanes)
        for pt in lane.centerline:
            waypoints.append((pt[0], pt[1], speed))

        # Add arc turn to next lane (if not the last)
        if i < len(lane_ids) - 1:
            conn = _connection_between(lm, lid, lane_ids[i + 1])
            if conn and conn.arc_points:
                turn_speed = min(speed, 0.5)  # slow through turns
                for pt in conn.arc_points[1:]:  # skip first (same as lane end)
                    waypoints.append((pt[0], pt[1], turn_speed))

    return waypoints


def route_to_base(lm: LaneMap, rover_lat: float, rover_lon: float,
                  base_lat: float = 0.0, base_lon: float = 0.0
                  ) -> list[tuple[float, float, float]]:
    """Compute a lane-based route from rover position to base station.

    Returns list of (lat, lon, speed) waypoints, or empty if no route.
    """
    # Use lane map base position if not specified
    if base_lat == 0.0 and base_lon == 0.0:
        base_lat, base_lon = lm.base_position
    if base_lat == 0.0 and base_lon == 0.0:
        return []

    # Find nearest lanes
    from_id, from_frac = find_nearest_lane(lm, rover_lat, rover_lon)
    to_id, to_frac = find_nearest_lane(lm, base_lat, base_lon)

    if not from_id or not to_id:
        return []

    # If already on the base lane, just go to the base position
    if from_id == to_id:
        lane = lm.lanes[from_id]
        speed = lane.speed if lane.speed > 0 else 0.5
        return [
            (rover_lat, rover_lon, speed),
            (base_lat, base_lon, speed),
        ]

    # Find shortest lane path
    lane_path = shortest_path(lm, from_id, to_id)
    if not lane_path:
        return []

    # Generate waypoints
    wps = path_to_waypoints(lm, lane_path)

    # Prepend rover position, append base position
    if wps:
        result = [(rover_lat, rover_lon, wps[0][2])]
        result.extend(wps)
        result.append((base_lat, base_lon, wps[-1][2]))
        return result
    return []
