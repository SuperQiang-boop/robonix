# SPDX-License-Identifier: MulanPSL-2.0
"""Select inset frontier goals in connected, footprint-clear known space.

Boundary clusters are split into bounded spatial segments. Goals come from
actual boundary cells, inset toward known space, rather than cluster means.
The conservative clearance disk covers Scout2's padded footprint at any yaw;
Nav2 remains responsible for exact footprint planning and collision avoidance.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import time
from typing import List, Optional, Tuple

import numpy as np


OCC_THRESH = 50  # ≥ this counts as obstacle (matches nav2 convention)
# Covers Scout2's octagonal footprint plus padding (outer radius about 0.63 m).
MIN_SAFE_RADIUS_M = 0.65
FRONTIER_SEGMENT_SPAN_M = 1.5
MIN_TARGET_DISTANCE_M = 0.60
MAX_VIEWPOINT_DISTANCE_M = 2.5


def _nearest_obstacle_distance(gv: GridView, wx: float, wy: float) -> float:
    """Approximate distance from (wx, wy) to nearest obstacle cell.

    Scans a bounded 4m × 4m cell patch around the point for occupied cells.
    Returns a large number if no obstacle is nearby — this is a soft
    heuristic only, used to penalise frontier scores, not to reject goals.

    Performance note: the patch radius is limited to 20 cells so the scan
    remains bounded even when the map resolution is fine.
    """
    cx, cy = gv.world_to_cell(wx, wy)
    r = 20  # search radius in cells (~2m at 0.1m resolution)
    y0, y1 = max(0, cy - r), min(gv.height, cy + r + 1)
    x0, x1 = max(0, cx - r), min(gv.width, cx + r + 1)
    patch = gv.data[y0:y1, x0:x1]
    obstacle_indices = np.argwhere(patch >= OCC_THRESH)
    if obstacle_indices.size == 0:
        return 10.0  # large number if no obstacle nearby
    # Find nearest cell
    dx = obstacle_indices[:, 1] + x0 - cx
    dy = obstacle_indices[:, 0] + y0 - cy
    return float(np.hypot(dx, dy).min()) * gv.resolution


@dataclass
class GridView:
    """Numpy-friendly view of nav_msgs/OccupancyGrid."""
    data: np.ndarray         # shape (h, w), int8 in [-1, 100]
    resolution: float        # m / cell
    origin_x: float
    origin_y: float
    width: int
    height: int

    @classmethod
    def from_msg(cls, msg) -> "GridView":
        h, w = int(msg.info.height), int(msg.info.width)
        # ROS uses array('b'); lists of signed values are also accepted in tests.
        arr = np.asarray(msg.data, dtype=np.int8).reshape(h, w)
        if h <= 0 or w <= 0 or not math.isfinite(msg.info.resolution) or msg.info.resolution <= 0:
            raise ValueError("occupancy grid dimensions and resolution must be positive")
        orientation = msg.info.origin.orientation
        if (abs(orientation.x) > 1e-6 or abs(orientation.y) > 1e-6
                or abs(orientation.z) > 1e-6 or abs(abs(orientation.w) - 1.0) > 1e-6):
            raise ValueError("rotated occupancy grids are not supported")
        return cls(
            data=arr,
            resolution=float(msg.info.resolution),
            origin_x=float(msg.info.origin.position.x),
            origin_y=float(msg.info.origin.position.y),
            width=w, height=h,
        )

    def cell_to_world(self, cx: int, cy: int) -> Tuple[float, float]:
        return (self.origin_x + (cx + 0.5) * self.resolution,
                self.origin_y + (cy + 0.5) * self.resolution)

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        return (math.floor((x - self.origin_x) / self.resolution),
                math.floor((y - self.origin_y) / self.resolution))

    def in_bounds(self, cx: int, cy: int) -> bool:
        return 0 <= cx < self.width and 0 <= cy < self.height


@dataclass
class FrontierCluster:
    centroid_xy: Tuple[float, float]   # cluster mean in cells; selected goal in world metres
    size: int                          # cell count
    cell_indices: np.ndarray           # (N, 2) int — for debugging / viz
    travel_distance_m: float = 0.0     # known-free route length for budget estimation
    frontier_goal_xy: Optional[Tuple[float, float]] = None  # destination beyond a waypoint


def find_frontier_cells(gv: GridView) -> np.ndarray:
    """Return (N, 2) array of (cx, cy) for cells that are free AND
    have at least one unknown 4-neighbour. Vectorised via shifted
    masks to avoid per-cell python loops."""
    g = gv.data
    free    = (g == 0)
    unknown = (g == -1)

    # Pad unknown by 1 in each direction; OR them and intersect with free.
    h, w = g.shape
    has_unknown_neighbour = np.zeros_like(free, dtype=bool)
    has_unknown_neighbour[1:, :]   |= unknown[:-1, :]   # neighbour above
    has_unknown_neighbour[:-1, :]  |= unknown[1:, :]    # below
    has_unknown_neighbour[:, 1:]   |= unknown[:, :-1]   # left
    has_unknown_neighbour[:, :-1]  |= unknown[:, 1:]    # right

    frontier_mask = free & has_unknown_neighbour
    yy, xx = np.where(frontier_mask)
    return np.stack([xx, yy], axis=1)  # (N, 2) as (cx, cy)


def cluster_frontiers(cells: np.ndarray, min_size: int = 3,
                       max_link_cells: int = 1) -> List[FrontierCluster]:
    """Connected-components style clustering with 8-neighbour adjacency
    extended by `max_link_cells` (cells within this Chebyshev distance
    are merged into the same cluster). This is cheaper than a real
    DBSCAN since we already have integer grid coords.

    Drops clusters smaller than `min_size` cells — those are usually
    noise from boundary cells against partially-mapped obstacles.
    """
    if cells.size == 0:
        return []

    # Bucket into a sparse grid for fast neighbour lookup.
    cell_set = {(int(c[0]), int(c[1])): i for i, c in enumerate(cells)}
    parent = list(range(len(cells)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    r = max_link_cells
    for (cx, cy), idx in cell_set.items():
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx == 0 and dy == 0:
                    continue
                nb = (cx + dx, cy + dy)
                j = cell_set.get(nb)
                if j is not None:
                    union(idx, j)

    groups: dict[int, list[int]] = {}
    for i in range(len(cells)):
        groups.setdefault(find(i), []).append(i)

    clusters: List[FrontierCluster] = []
    for _, members in groups.items():
        if len(members) < min_size:
            continue
        member_arr = cells[members]
        # Centroid in world frame computed by caller (needs GridView);
        # here we just produce cell-space mean and let the caller
        # convert.
        clusters.append(FrontierCluster(
            centroid_xy=(float(member_arr[:, 0].mean()),
                         float(member_arr[:, 1].mean())),  # cell-space, will convert
            size=len(members),
            cell_indices=member_arr,
        ))
    return clusters


def _clearance_mask(gv: GridView, safe_radius_m: float) -> np.ndarray:
    """Erode known-free space by a conservative all-yaw footprint disk.

    Unknown, occupied, partially occupied and outside-map cells all block the
    disk. Include cells intersecting its edge using a half-cell-diagonal margin.
    """
    radius_m = max(MIN_SAFE_RADIUS_M, safe_radius_m)
    if not math.isfinite(radius_m):
        raise ValueError("safe radius must be finite")
    radius_cells = radius_m / gv.resolution + math.sqrt(2.0) / 2.0
    r = int(math.ceil(radius_cells))
    free = np.pad(gv.data == 0, r, constant_values=False)
    safe = np.ones((gv.height, gv.width), dtype=bool)
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy <= radius_cells * radius_cells:
                safe &= free[r + dy:r + dy + gv.height,
                             r + dx:r + dx + gv.width]
    return safe


def is_target_safe(gv: GridView, wx: float, wy: float,
                   safe_radius_m: float = MIN_SAFE_RADIUS_M) -> bool:
    """Require the entire clearance disk to be inside known-free space."""
    cx, cy = gv.world_to_cell(wx, wy)
    if not gv.in_bounds(cx, cy):
        return False
    return bool(_clearance_mask(gv, safe_radius_m)[cy, cx])


def _travel_distances(safe: np.ndarray, start: Tuple[int, int], resolution: float) -> np.ndarray:
    """Measure four-connected known-free route lengths without cutting corners.

    The Manhattan grid routes are conservative estimates, not a Nav2 plan.
    Unsafe/outside starts fail closed rather than snapping across obstacles.
    """
    distances = np.full(safe.shape, np.inf)
    h, w = safe.shape
    sx, sy = start
    if not (0 <= sx < w and 0 <= sy < h) or not safe[sy, sx]:
        return distances
    pending = deque([(sx, sy)])
    distances[sy, sx] = 0.0
    while pending:
        cx, cy = pending.popleft()
        for nx, ny in ((cx - 1, cy), (cx + 1, cy), (cx, cy - 1), (cx, cy + 1)):
            if 0 <= nx < w and 0 <= ny < h and safe[ny, nx] and not np.isfinite(distances[ny, nx]):
                distances[ny, nx] = distances[cy, cx] + resolution
                pending.append((nx, ny))
    return distances


def _reachable_cells(safe: np.ndarray, start: Tuple[int, int]) -> np.ndarray:
    """Return the robot's connected safe component, without corner cutting."""
    return np.isfinite(_travel_distances(safe, start, 1.0))


def _split_clusters(clusters: List[FrontierCluster], gv: GridView) -> List[FrontierCluster]:
    """Split large boundary components into bounded, connected spatial segments."""
    span = max(1, int(FRONTIER_SEGMENT_SPAN_M / gv.resolution))
    segments = []
    for cluster in clusters:
        tiles = cluster.cell_indices // span
        for tile in np.unique(tiles, axis=0):
            members = cluster.cell_indices[np.all(tiles == tile, axis=1)]
            # The original component already passed the noise threshold. Keep
            # small tile fragments so crossing a tile edge cannot erase a frontier.
            segments.extend(cluster_frontiers(members, min_size=1))
    return segments


def _known_free_line(gv: GridView, start: Tuple[int, int],
                     end: Tuple[int, int]) -> bool:
    """Check a boundary sight line including both cells touched at grid corners.

    Unknown and occupied cells block the view. This is only an observation
    test, never a drivable path: robot motion uses the clearance-safe flood.
    """
    x, y = start
    ex, ey = end
    if not gv.in_bounds(x, y) or not gv.in_bounds(ex, ey) or gv.data[y, x] != 0:
        return False
    nx, ny = abs(ex - x), abs(ey - y)
    sx, sy = (1 if ex > x else -1), (1 if ey > y else -1)
    ix = iy = 0
    while ix < nx or iy < ny:
        decision = (1 + 2 * ix) * ny - (1 + 2 * iy) * nx
        if decision == 0:
            if gv.data[y, x + sx] != 0 or gv.data[y + sy, x] != 0:
                return False
            x, y, ix, iy = x + sx, y + sy, ix + 1, iy + 1
        elif decision < 0:
            x, ix = x + sx, ix + 1
        else:
            y, iy = y + sy, iy + 1
        if gv.data[y, x] != 0:
            return False
    return True


def _frontier_goal_candidates(gv: GridView, cells: np.ndarray,
                              reachable: np.ndarray,
                              safe_goal_mask: np.ndarray,
                              diagnostics: Optional[dict] = None,
                              robot_xy: Optional[Tuple[float, float]] = None
                              ) -> list[Tuple[int, int]]:
    """Search two-dimensional known space for safe views of actual boundaries.

    Seed a bounded multi-source flood from the boundary segment. A viewpoint
    must see its source boundary through known-free cells and already belong
    to the robot's clearance-safe component. Stop expanding a branch at its
    first valid view, rather than filling the room or selecting its centroid.
    The observation search is limited to 2.5 m; the robot never follows the
    point-sized observation flood through narrow or unknown space.
    """
    max_steps = int(math.floor(MAX_VIEWPOINT_DISTANCE_M / gv.resolution))
    pending = deque()
    seen = set()
    candidates = set()
    stats = diagnostics if diagnostics is not None else {}
    stats.setdefault('rejected_goal_clearance', 0)
    stats.setdefault('rejected_route_reachability', 0)
    stats.setdefault('rejected_visibility', 0)
    stats.setdefault('rejected_view_distance', 0)
    stats.setdefault('rejected_near_robot', 0)
    stats.setdefault('viewpoint_cells_checked', 0)
    for raw_x, raw_y in cells:
        cell = int(raw_x), int(raw_y)
        if gv.in_bounds(*cell) and gv.data[cell[1], cell[0]] == 0:
            seen.add(cell)
            pending.append((*cell, 0, *cell))
    while pending:
        cx, cy, steps, source_x, source_y = pending.popleft()
        stats['viewpoint_cells_checked'] += 1
        if steps > max_steps:
            stats['rejected_view_distance'] += 1
            continue
        if not safe_goal_mask[cy, cx]:
            stats['rejected_goal_clearance'] += 1
        elif not reachable[cy, cx]:
            stats['rejected_route_reachability'] += 1
        elif not _known_free_line(gv, (cx, cy), (source_x, source_y)):
            stats['rejected_visibility'] += 1
        else:
            wx, wy = gv.cell_to_world(cx, cy)
            if (robot_xy is not None
                    and math.dist((wx, wy), robot_xy) < MIN_TARGET_DISTANCE_M):
                # Do not let a nearby but unusable view hide a farther safe
                # observation point on this same boundary branch.
                stats['rejected_near_robot'] += 1
            else:
                candidates.add((cx, cy))
                continue
        # Expand one extra ring so the distance counter includes branches
        # whose first otherwise-checkable viewpoint lies just beyond range.
        for nx, ny in ((cx - 1, cy), (cx + 1, cy), (cx, cy - 1), (cx, cy + 1)):
            if gv.in_bounds(nx, ny) and gv.data[ny, nx] == 0 and (nx, ny) not in seen:
                seen.add((nx, ny))
                pending.append((nx, ny, steps + 1, source_x, source_y))
    return sorted(candidates)


def score_clusters(clusters: List[FrontierCluster], gv: GridView,
                   robot_xy: Tuple[float, float], *,
                   max_distance_m: float = 8.0,
                   visited_cells: Optional[set] = None,
                   visited_penalty_m: float = 1.5,
                   failed_goals: Optional[dict] = None,
                   failed_goal_radius_m: float = 0.5,
                   failed_goal_cooldown_s: float = 120.0,
                   safe_radius_m: float = MIN_SAFE_RADIUS_M,
                   arrival_tolerance_m: float = 0.0,
                   glass_doors: Optional[list] = None,
                   glass_door_radius_m: float = 1.0,
                   diagnostics: Optional[dict] = None
                   ) -> List[Tuple[float, FrontierCluster]]:
    """Rank inset goals by segment size over travel plus observation penalties.

    Every candidate belongs to the robot's footprint-clear known component.
    Recently failed goal neighborhoods are temporarily excluded. Visited
    cells and nearby obstacles/scene glass-door detections add soft penalties.
    Returned centroid_xy is the chosen navigation point, not a boundary mean.
    """
    stats = diagnostics if diagnostics is not None else {}
    stats.update(frontier_clusters=len(clusters), generated_viewpoints=0,
                 rejected_distance=0, rejected_near_robot=0, rejected_cooldown=0,
                 rejected_goal_clearance=0, rejected_route_reachability=0,
                 rejected_visibility=0, rejected_view_distance=0,
                 viewpoint_cells_checked=0,
                 ranked_goals=0, min_route_distance_m=None)
    if not clusters:
        return []
    if not math.isfinite(arrival_tolerance_m) or arrival_tolerance_m < 0:
        raise ValueError("arrival tolerance must be finite and nonnegative")
    # The route uses chassis clearance; the endpoint reserves additional room
    # for Nav2 to stop anywhere inside its XY tolerance.
    safe = _clearance_mask(gv, safe_radius_m)
    goal_clearance_radius_m = max(MIN_SAFE_RADIUS_M, safe_radius_m) + arrival_tolerance_m
    safe_goal = _clearance_mask(gv, goal_clearance_radius_m)
    distances = _travel_distances(safe, gv.world_to_cell(*robot_xy), gv.resolution)
    reachable = np.isfinite(distances)
    sx, sy = gv.world_to_cell(*robot_xy)
    stats.update(safe_cells=int(safe.sum()), reachable_cells=int(reachable.sum()),
                 robot_clearance_safe=bool(gv.in_bounds(sx, sy) and safe[sy, sx]),
                 goal_clearance_radius_m=goal_clearance_radius_m,
                 goal_clearance_candidates=0)
    if not reachable.any():
        return []
    scored = []
    now = time.monotonic()
    for segment in _split_clusters(clusters, gv):
        for tx, ty in _frontier_goal_candidates(
                gv, segment.cell_indices, reachable, safe_goal,
                diagnostics=stats, robot_xy=robot_xy):
            stats['generated_viewpoints'] += 1
            stats['goal_clearance_candidates'] += 1
            wx, wy = gv.cell_to_world(tx, ty)
            travel = math.hypot(wx - robot_xy[0], wy - robot_xy[1])
            if travel > max_distance_m:
                stats['rejected_distance'] += 1
                continue
            if travel < MIN_TARGET_DISTANCE_M:
                # Defensive check for candidates supplied by alternate search
                # implementations; the bounded viewpoint flood filters these.
                stats['rejected_near_robot'] += 1
                continue
            if failed_goals and any(
                    now - failed_at < failed_goal_cooldown_s
                    and math.hypot(wx - fx, wy - fy) < failed_goal_radius_m
                    for (fx, fy), failed_at in failed_goals.items()):
                stats["rejected_cooldown"] += 1
                continue
            visited_penalty = 0.0
            if visited_cells:
                radius = int(math.ceil(visited_penalty_m / gv.resolution))
                if any((tx + dx, ty + dy) in visited_cells
                       for dy in range(-radius, radius + 1)
                       for dx in range(-radius, radius + 1)):
                    visited_penalty = visited_penalty_m
            obstacle_penalty = max(0.0, 1.0 - _nearest_obstacle_distance(gv, wx, wy))
            glass_penalty = 0.0
            if glass_doors:
                for gx, gy in glass_doors:
                    distance = math.hypot(wx - gx, wy - gy)
                    if distance < glass_door_radius_m:
                        glass_penalty = max(glass_penalty,
                            10.0 * (1.0 - distance / glass_door_radius_m))
            route_distance = float(distances[ty, tx])
            score = segment.size / (route_distance + visited_penalty + obstacle_penalty + glass_penalty + 1.0)
            goal = FrontierCluster((wx, wy), segment.size, segment.cell_indices, route_distance)
            scored.append((score, goal))
    scored.sort(key=lambda item: item[0], reverse=True)
    stats['ranked_goals'] = len(scored)
    stats['min_route_distance_m'] = min((g.travel_distance_m for _, g in scored), default=None)
    return scored


def _route_waypoint(gv: GridView, robot_xy: Tuple[float, float],
                    goals: List[FrontierCluster], max_route_m: float, *,
                    safe_radius_m: float, arrival_tolerance_m: float,
                    failed_goals: Optional[dict], failed_goal_radius_m: float,
                    failed_goal_cooldown_s: float) -> Optional[FrontierCluster]:
    """Stop on a safe prefix of a four-connected route to an actual frontier goal.

    Reconstruct shortest grid routes from decreasing flood distances, never
    interpolating XY across unknown cells, walls or corners. The farthest stop
    within this leg's allowance must also reserve arrival uncertainty and make
    meaningful translation. Other routes are tried if one has no safe stop.
    Nav2 still validates the actual motion; the next leg is selected anew.
    """
    safe = _clearance_mask(gv, safe_radius_m)
    safe_goal = _clearance_mask(gv, max(MIN_SAFE_RADIUS_M, safe_radius_m) + arrival_tolerance_m)
    start = gv.world_to_cell(*robot_xy)
    distances = _travel_distances(safe, start, gv.resolution)
    now = time.monotonic()
    minimum_progress = max(MIN_TARGET_DISTANCE_M, 2 * arrival_tolerance_m + gv.resolution)
    for goal in goals:
        cell = gv.world_to_cell(*goal.centroid_xy)
        if not gv.in_bounds(*cell) or not math.isfinite(distances[cell[1], cell[0]]):
            continue
        while cell != start:
            cx, cy = cell
            route_m = float(distances[cy, cx])
            if route_m <= max_route_m + 1e-9 and safe_goal[cy, cx]:
                wx, wy = gv.cell_to_world(cx, cy)
                in_cooldown = failed_goals and any(
                    now - failed_at < failed_goal_cooldown_s
                    and math.hypot(wx - fx, wy - fy) < failed_goal_radius_m
                    for (fx, fy), failed_at in failed_goals.items())
                if math.dist((wx, wy), robot_xy) >= minimum_progress and not in_cooldown:
                    return FrontierCluster((wx, wy), goal.size, goal.cell_indices,
                                           route_m, goal.centroid_xy)
            neighbors = [(nx, ny) for nx, ny in
                         ((cx - 1, cy), (cx + 1, cy), (cx, cy - 1), (cx, cy + 1))
                         if gv.in_bounds(nx, ny) and distances[ny, nx] < route_m - 1e-9]
            if not neighbors:
                break
            cell = min(neighbors, key=lambda xy: distances[xy[1], xy[0]])
    return None


def pick_target(gv: GridView, robot_xy: Tuple[float, float], *,
                 min_size: int = 3,
                 max_distance_m: float = 8.0,
                 visited_cells: Optional[set] = None,
                 failed_goals: Optional[dict] = None,
                 failed_goal_radius_m: float = 0.5,
                 failed_goal_cooldown_s: float = 120.0,
                 safe_radius_m: float = MIN_SAFE_RADIUS_M,
                 arrival_tolerance_m: float = 0.0,
                 glass_doors: Optional[list] = None,
                 glass_door_radius_m: float = 1.0,
                 max_travel_distance_m: float = math.inf,
                 preferred_travel_distance_m: float = math.inf,
                 allow_route_splitting: bool = False,
                 diagnostics: Optional[dict] = None,
                 ) -> Optional[FrontierCluster]:
    """Prefer shorter safe goals, allowing reference overruns within the hard cap.

    Opt-in route splitting returns a safe stop on a known-space route toward
    an actual boundary observation goal. This stop need not yet see that
    boundary. No candidate does not imply complete coverage: remaining
    frontiers and route/clearance limitations must be reported separately.
    """
    stats = diagnostics if diagnostics is not None else {}
    stats.clear()
    cells = find_frontier_cells(gv)
    stats['raw_frontier_cells'] = len(cells)
    clusters = cluster_frontiers(cells, min_size=min_size)
    scored = score_clusters(clusters, gv, robot_xy,
                             max_distance_m=max_distance_m,
                             visited_cells=visited_cells,
                             failed_goals=failed_goals,
                             failed_goal_radius_m=failed_goal_radius_m,
                             failed_goal_cooldown_s=failed_goal_cooldown_s,
                             safe_radius_m=safe_radius_m,
                             arrival_tolerance_m=arrival_tolerance_m,
                             glass_doors=glass_doors,
                             glass_door_radius_m=glass_door_radius_m,
                             diagnostics=stats)
    # Prefer shorter safe routes; the advisory reference never rejects a goal.
    scored.sort(key=lambda item: (item[1].travel_distance_m, -item[0]))
    feasible = [goal for _, goal in scored
                if goal.travel_distance_m <= max_travel_distance_m + 1e-9]
    preferred = [goal for goal in feasible
                 if goal.travel_distance_m <= preferred_travel_distance_m + 1e-9]
    stats['rejected_budget'] = len(scored) - len(feasible)
    stats['feasible_goals'] = len(feasible)
    stats['reference_budget_exceeded'] = False
    stats['reference_budget_candidates'] = len(preferred)
    stats['intermediate_waypoint'] = False
    stats['route_split_unavailable'] = False
    target = feasible[0] if feasible else None
    if target is None and scored and allow_route_splitting and max_travel_distance_m > 0:
        target = _route_waypoint(
            gv, robot_xy, [goal for _, goal in scored], max_travel_distance_m,
            safe_radius_m=safe_radius_m, arrival_tolerance_m=arrival_tolerance_m,
            failed_goals=failed_goals, failed_goal_radius_m=failed_goal_radius_m,
            failed_goal_cooldown_s=failed_goal_cooldown_s)
        stats['route_split_unavailable'] = target is None
        stats['intermediate_waypoint'] = target is not None
        stats['feasible_goals'] = int(target is not None)
    if target is not None:
        stats['reference_budget_exceeded'] = target.travel_distance_m > preferred_travel_distance_m + 1e-9
        stats['selected_route_distance_m'] = target.travel_distance_m
        stats['frontier_goal_xy'] = target.frontier_goal_xy or target.centroid_xy
    return target


def total_frontier_count(gv: GridView, min_size: int = 3) -> int:
    """For status reporting: how many frontier clusters remain that
    are large enough to warrant another exploration round."""
    cells = find_frontier_cells(gv)
    if cells.size == 0:
        return 0
    return len(cluster_frontiers(cells, min_size=min_size))


def mapped_free_area_m2(gv: GridView) -> float:
    """How much area (m²) is currently mapped as free."""
    free_cells = int(np.sum(gv.data == 0))
    return free_cells * (gv.resolution ** 2)
