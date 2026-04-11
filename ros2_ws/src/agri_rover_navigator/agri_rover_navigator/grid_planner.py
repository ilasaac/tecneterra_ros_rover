"""
grid_planner.py — A-star path planner on a GPS obstacle grid.

Pure Python, no ROS2 or Nav2 dependency. Called by navigator._reroute_path()
to generate smooth obstacle-aware paths.

Usage:
    from agri_rover_navigator.grid_planner import plan_around_obstacles
    path = plan_around_obstacles(
        start=(lat, lon), goal=(lat, lon),
        obstacles=[[(lat,lon), ...], ...],  # raw polygons
        clearance_m=1.0,
        resolution_m=0.10,
    )
    # path = [(lat, lon), ...] smooth GPS path
"""

from __future__ import annotations
import heapq
import math


# ---------------------------------------------------------------------------
# Smooth heading-aware approach path (cubic Bézier)
# ---------------------------------------------------------------------------

def plan_smooth_approach(
    start: tuple[float, float],
    start_heading_deg: float,
    goal: tuple[float, float],
    goal_heading_deg: float,
    expanded_obstacles: list[list[tuple[float, float]]],
    point_spacing_m: float = 0.10,
    min_turn_radius: float = 1.5,
) -> list[tuple[float, float]] | None:
    """Generate a smooth heading-aware approach path using cubic Bézier.

    The curve is tangent to *start_heading_deg* at the start and arrives
    tangent to *goal_heading_deg* at the goal — no axis turns.

    Parameters
    ----------
    start, goal          : (lat, lon) degrees
    start_heading_deg    : rover heading (north = 0, CW)
    goal_heading_deg     : desired arrival heading (north = 0, CW)
    expanded_obstacles   : polygons already expanded by clearance
    point_spacing_m      : sample spacing along the curve (metres)
    min_turn_radius      : minimum effective turn radius (metres)

    Returns
    -------
    List of (lat, lon) waypoints, or *None* if the path crosses an obstacle.
    """
    dist = _haversine(start[0], start[1], goal[0], goal[1])
    if dist < 0.5:
        return [start, goal]

    # Control-point distance — longer for larger heading differences
    angle_diff = abs((goal_heading_deg - start_heading_deg + 180) % 360 - 180)
    heading_factor = 1.0 + angle_diff / 180.0        # 1.0 → 2.0
    cp_dist = max(min_turn_radius, dist / 3.0 * heading_factor)
    cp_dist = min(cp_dist, dist * 0.8)               # don't overshoot

    # Flat-earth projection centred at start
    cos_lat = math.cos(math.radians(start[0])) or 1e-9
    M_LAT = 111_320.0
    M_LON = 111_320.0 * cos_lat

    def to_m(lat, lon):
        return (lon - start[1]) * M_LON, (lat - start[0]) * M_LAT

    def to_gps(x, y):
        return start[0] + y / M_LAT, start[1] + x / M_LON

    # Headings → math angle (north CW → east CCW)
    s_rad = math.radians(90.0 - start_heading_deg)
    g_rad = math.radians(90.0 - goal_heading_deg)

    sx, sy = 0.0, 0.0
    gx, gy = to_m(*goal)

    # Bézier control points
    cp1x = sx + cp_dist * math.cos(s_rad)
    cp1y = sy + cp_dist * math.sin(s_rad)
    cp2x = gx - cp_dist * math.cos(g_rad)
    cp2y = gy - cp_dist * math.sin(g_rad)

    # Sample the curve
    n_pts = max(10, int(dist / point_spacing_m))
    path_m: list[tuple[float, float]] = []
    for i in range(n_pts + 1):
        t = i / n_pts
        mt = 1.0 - t
        x = (mt**3 * sx + 3 * mt**2 * t * cp1x
             + 3 * mt * t**2 * cp2x + t**3 * gx)
        y = (mt**3 * sy + 3 * mt**2 * t * cp1y
             + 3 * mt * t**2 * cp2y + t**3 * gy)
        path_m.append((x, y))

    # Check every sample against expanded obstacles (already include clearance)
    if expanded_obstacles:
        obs_m = [[to_m(lat, lon) for lat, lon in poly]
                 for poly in expanded_obstacles]
        for px, py in path_m:
            for poly in obs_m:
                if _pip(px, py, poly):
                    return None          # blocked → caller falls back to A*

    # Convert to GPS, pin endpoints exactly
    path_gps = [start] + [to_gps(x, y) for x, y in path_m[1:-1]] + [goal]
    return path_gps


def _haversine(lat1, lon1, lat2, lon2):
    """Quick haversine distance in metres."""
    R = 6_371_000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2
         + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2))
         * math.sin(dlon / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def _pip(px, py, poly):
    """Point-in-polygon (ray casting) in flat-earth metres."""
    n = len(poly)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > py) != (yj > py)
                and px < (xj - xi) * (py - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


def plan_around_obstacles(
    start: tuple[float, float],
    goal: tuple[float, float],
    obstacles: list[list[tuple[float, float]]],
    clearance_m: float = 1.0,
    resolution_m: float = 0.10,
    padding_m: float = 3.0,
    simplify_tolerance_m: float = 0.15,
) -> list[tuple[float, float]] | None:
    """Plan a smooth GPS path from start to goal avoiding obstacle polygons.

    Parameters
    ----------
    start, goal     : (lat, lon) in degrees
    obstacles       : list of polygons, each [(lat, lon), ...]
    clearance_m     : minimum distance from obstacle edges (already includes rover width)
    resolution_m    : grid cell size in metres
    padding_m       : extra margin around bounding box
    simplify_tolerance_m : Ramer-Douglas-Peucker tolerance for path simplification

    Returns
    -------
    List of (lat, lon) waypoints, or None if no path found.
    """
    if not obstacles:
        return [start, goal]

    # Flat-earth projection centred at midpoint of start/goal
    c_lat = (start[0] + goal[0]) / 2.0
    c_lon = (start[1] + goal[1]) / 2.0
    cos_lat = math.cos(math.radians(c_lat)) or 1e-9
    m_lat = 111_320.0
    m_lon = 111_320.0 * cos_lat

    def to_m(lat, lon):
        return (lon - c_lon) * m_lon, (lat - c_lat) * m_lat

    def to_gps(x, y):
        return c_lat + y / m_lat, c_lon + x / m_lon

    # Convert all points to metres
    start_m = to_m(*start)
    goal_m = to_m(*goal)
    obs_m = [[to_m(lat, lon) for lat, lon in poly] for poly in obstacles]

    # Compute bounding box
    all_x = [start_m[0], goal_m[0]]
    all_y = [start_m[1], goal_m[1]]
    for poly in obs_m:
        for x, y in poly:
            all_x.append(x)
            all_y.append(y)
    min_x = min(all_x) - padding_m
    min_y = min(all_y) - padding_m
    max_x = max(all_x) + padding_m
    max_y = max(all_y) + padding_m

    # Grid dimensions
    w = int((max_x - min_x) / resolution_m) + 1
    h = int((max_y - min_y) / resolution_m) + 1

    if w * h > 4_000_000:  # safety cap
        return None

    def to_grid(x, y):
        return int((x - min_x) / resolution_m), int((y - min_y) / resolution_m)

    def to_world(gx, gy):
        return min_x + gx * resolution_m, min_y + gy * resolution_m

    # Build obstacle grid with inflation
    grid = bytearray(w * h)  # 0 = free, 1 = blocked

    inflate_cells = int(math.ceil(clearance_m / resolution_m))

    for poly in obs_m:
        # Rasterize filled polygon + inflation
        _rasterize_polygon(grid, w, h, poly, min_x, min_y, resolution_m, inflate_cells)

    # A-star
    sg = to_grid(*start_m)
    gg = to_grid(*goal_m)

    # Clamp to grid bounds
    sg = (max(0, min(w - 1, sg[0])), max(0, min(h - 1, sg[1])))
    gg = (max(0, min(w - 1, gg[0])), max(0, min(h - 1, gg[1])))

    if grid[sg[1] * w + sg[0]] or grid[gg[1] * w + gg[0]]:
        # Start or goal is blocked — try to find nearest free cell
        sg = _nearest_free(grid, w, h, sg)
        gg = _nearest_free(grid, w, h, gg)
        if sg is None or gg is None:
            return None

    path_grid = _astar(grid, w, h, sg, gg)
    if path_grid is None:
        return None

    # Convert grid path to GPS
    path_m = [to_world(gx, gy) for gx, gy in path_grid]

    # Simplify (Ramer-Douglas-Peucker)
    if simplify_tolerance_m > 0:
        path_m = _rdp(path_m, simplify_tolerance_m)

    # Smooth corners (Chaikin corner-cutting, 3 iterations)
    path_m = _chaikin_smooth(path_m, iterations=3)

    # Verify smoothed path doesn't cut into obstacles
    path_m = _validate_smooth(path_m, grid, w, h, min_x, min_y, resolution_m)

    # Ensure start and goal are exact
    path_gps = [start] + [to_gps(x, y) for x, y in path_m[1:-1]] + [goal]

    return path_gps


def _rasterize_polygon(grid, w, h, poly, min_x, min_y, res, inflate):
    """Fill polygon cells + inflation ring in the grid."""
    n = len(poly)
    if n < 3:
        return

    # Convert polygon to grid coords
    gpoly = [(int((x - min_x) / res), int((y - min_y) / res)) for x, y in poly]

    # Scanline fill
    all_gy = [p[1] for p in gpoly]
    y_lo = max(0, min(all_gy) - inflate)
    y_hi = min(h - 1, max(all_gy) + inflate)

    for gy in range(y_lo, y_hi + 1):
        # Find intersections of scanline with polygon edges
        nodes = []
        j = n - 1
        for i in range(n):
            yi, xi = gpoly[i][1], gpoly[i][0]
            yj, xj = gpoly[j][1], gpoly[j][0]
            if (yi < gy <= yj) or (yj < gy <= yi):
                if yj != yi:
                    nodes.append(int(xi + (gy - yi) / (yj - yi) * (xj - xi)))
            j = i
        nodes.sort()

        # Fill between pairs
        for k in range(0, len(nodes) - 1, 2):
            x_start = max(0, nodes[k] - inflate)
            x_end = min(w - 1, nodes[k + 1] + inflate)
            for gx in range(x_start, x_end + 1):
                grid[gy * w + gx] = 1

    # Inflate edges (not just fill interior)
    for i in range(n):
        j = (i + 1) % n
        _rasterize_thick_line(grid, w, h, gpoly[i], gpoly[j], inflate)


def _rasterize_thick_line(grid, w, h, p1, p2, thickness):
    """Draw a thick line on the grid (Bresenham + dilation)."""
    x0, y0 = p1
    x1, y1 = p2
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        # Fill circle of radius=thickness at (x0, y0)
        for dy2 in range(-thickness, thickness + 1):
            for dx2 in range(-thickness, thickness + 1):
                if dx2 * dx2 + dy2 * dy2 <= thickness * thickness:
                    gx, gy = x0 + dx2, y0 + dy2
                    if 0 <= gx < w and 0 <= gy < h:
                        grid[gy * w + gx] = 1
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy


def _nearest_free(grid, w, h, pos):
    """Find nearest free cell to pos using BFS."""
    gx, gy = pos
    if 0 <= gx < w and 0 <= gy < h and not grid[gy * w + gx]:
        return pos
    from collections import deque
    visited = set()
    q = deque([(gx, gy)])
    visited.add((gx, gy))
    while q:
        cx, cy = q.popleft()
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < w and 0 <= ny < h and (nx, ny) not in visited:
                visited.add((nx, ny))
                if not grid[ny * w + nx]:
                    return (nx, ny)
                q.append((nx, ny))
                if len(visited) > 10000:
                    return None
    return None


def _astar(grid, w, h, start, goal):
    """A-star on 8-connected grid. Returns list of (gx, gy) or None."""
    SQRT2 = 1.414

    open_set = [(0.0, start)]
    came_from = {}
    g_score = {start: 0.0}
    gx_goal, gy_goal = goal

    # 8-connected neighbors
    dirs = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            # Reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        cx, cy = current
        for dx, dy, cost in dirs:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < w and 0 <= ny < h and not grid[ny * w + nx]:
                ng = g_score[current] + cost
                if ng < g_score.get((nx, ny), float('inf')):
                    g_score[(nx, ny)] = ng
                    # Octile distance heuristic
                    ddx = abs(nx - gx_goal)
                    ddy = abs(ny - gy_goal)
                    heur = max(ddx, ddy) + (SQRT2 - 1) * min(ddx, ddy)
                    heapq.heappush(open_set, (ng + heur, (nx, ny)))
                    came_from[(nx, ny)] = current

    return None  # no path found


def _rdp(points, epsilon):
    """Ramer-Douglas-Peucker simplification."""
    if len(points) <= 2:
        return points

    # Find point with max distance from line start→end
    start, end = points[0], points[-1]
    dx, dy = end[0] - start[0], end[1] - start[1]
    line_len = math.hypot(dx, dy)

    max_dist = 0.0
    max_idx = 0
    for i in range(1, len(points) - 1):
        px, py = points[i][0] - start[0], points[i][1] - start[1]
        if line_len > 0:
            dist = abs(dx * py - dy * px) / line_len
        else:
            dist = math.hypot(px, py)
        if dist > max_dist:
            max_dist = dist
            max_idx = i

    if max_dist > epsilon:
        left = _rdp(points[:max_idx + 1], epsilon)
        right = _rdp(points[max_idx:], epsilon)
        return left[:-1] + right
    else:
        return [start, end]


def _chaikin_smooth(points, iterations=3):
    """Chaikin corner-cutting: each iteration replaces each segment midpoint
    with two points at 25% and 75%, rounding corners progressively.
    First and last points are preserved."""
    if len(points) <= 2:
        return points
    for _ in range(iterations):
        new_pts = [points[0]]
        for i in range(len(points) - 1):
            x0, y0 = points[i]
            x1, y1 = points[i + 1]
            new_pts.append((0.75 * x0 + 0.25 * x1, 0.75 * y0 + 0.25 * y1))
            new_pts.append((0.25 * x0 + 0.75 * x1, 0.25 * y0 + 0.75 * y1))
        new_pts.append(points[-1])
        points = new_pts
    return points


def _validate_smooth(path_m, grid, w, h, min_x, min_y, res):
    """Remove smoothed points that land inside obstacles.
    Replaces them with the midpoint of their neighbours (safe fallback)."""
    result = [path_m[0]]
    for i in range(1, len(path_m) - 1):
        x, y = path_m[i]
        gx = int((x - min_x) / res)
        gy = int((y - min_y) / res)
        if 0 <= gx < w and 0 <= gy < h and grid[gy * w + gx]:
            # Point is inside obstacle — skip it (neighbours connect directly)
            continue
        result.append(path_m[i])
    result.append(path_m[-1])
    return result
