# SPDX-License-Identifier: MulanPSL-2.0
"""Frontier extraction + scoring on nav_msgs/OccupancyGrid.

Standard formulation: a "frontier cell" is a free cell (occupancy=0)
adjacent to at least one unknown cell (occupancy=-1). Cells are
clustered into frontier *groups*, each group represented by its
centroid + size. The controller picks the highest-scoring group as
the next exploration target.

Scoring trades off information gain (cluster size — bigger frontier
= more unknown will become known if visited) against travel cost
(Euclidean distance from robot to centroid as a cheap proxy for the
true planner cost). nav2 / RRT-based explorers use the actual
costmap-aware planner cost, but for the dev demo a Euclidean proxy is
fine and avoids re-implementing A*.

We deliberately don't filter against an inflation halo here — that's
the navigation service's costmap layer's job. If the chosen frontier
is technically unreachable due to obstacle inflation, the nav RPC
will fail with a planning error and the controller picks the next
candidate. Re-doing inflation in this module would duplicate state
and violate the layering rule (frontier finder has no contract
dependency on inflation).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np


OCC_THRESH = 50  # ≥ this counts as obstacle (matches nav2 convention)


def _nearest_obstacle_distance(gv: GridView, wx: float, wy: float) -> float:
    """Approximate distance from (wx, wy) to nearest obstacle cell.

    Scans a 2m × 2m patch around the point for occupied cells (≥ OCC_THRESH).
    Returns a large number if no obstacle is nearby — this is a soft
    heuristic only, used to penalise frontier scores, not to reject goals.

    Performance note: the patch is limited to 20 cells (~2m at 0.1m
    resolution) so the scan is fast even at 1 Hz frontier evaluation.
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
    min_dist = float("inf")
    for oy, ox in obstacle_indices:
        d = ((ox - cx) ** 2 + (oy - cy) ** 2) ** 0.5
        min_dist = min(min_dist, d)
    return min_dist * gv.resolution  # convert to meters


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
        arr = np.frombuffer(bytes(msg.data), dtype=np.int8).reshape(h, w)
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
        return (int((x - self.origin_x) / self.resolution),
                int((y - self.origin_y) / self.resolution))

    def in_bounds(self, cx: int, cy: int) -> bool:
        return 0 <= cx < self.width and 0 <= cy < self.height


@dataclass
class FrontierCluster:
    centroid_xy: Tuple[float, float]   # world coords
    size: int                          # cell count
    cell_indices: np.ndarray           # (N, 2) int — for debugging / viz


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
                       max_link_cells: int = 2) -> List[FrontierCluster]:
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


def is_target_safe(gv: GridView, wx: float, wy: float,
                    safe_radius_m: float = 0.15) -> bool:
    """Reject targets that sit inside or near an obstacle. Single
    check: every cell in a `safe_radius_m`-radius patch around the
    target must be NON-OCCUPIED (g < OCC_THRESH). Unknown cells (-1)
    are allowed — frontier centroids sit on the free/unknown boundary
    by construction, so requiring "mostly known" at the exact centroid
    deadlocks exploration ("no safe frontier" forever even when 7+
    legitimate clusters exist; observed on webots tiago).

    The nav service's costmap layer is the second line of defence
    against inflation-halo violations — if the actual planner can't
    route to the chosen frontier, the goal aborts and the controller
    falls through to the next candidate.
    """
    cx, cy = gv.world_to_cell(wx, wy)
    if not gv.in_bounds(cx, cy):
        return False
    r = max(1, int(round(safe_radius_m / gv.resolution)))
    y0, y1 = max(0, cy - r), min(gv.height, cy + r + 1)
    x0, x1 = max(0, cx - r), min(gv.width,  cx + r + 1)
    patch = gv.data[y0:y1, x0:x1]
    return bool(np.all(patch < OCC_THRESH))


def score_clusters(clusters: List[FrontierCluster], gv: GridView,
                    robot_xy: Tuple[float, float], *,
                    max_distance_m: float = 8.0,
                    visited_cells: Optional[set] = None,
                    visited_penalty_m: float = 1.5,
                    failed_goals: Optional[set] = None,
                    failed_goal_radius_m: float = 0.5,
                    safe_radius_m: float = 0.15,
                    glass_doors: Optional[list] = None,
                    glass_door_radius_m: float = 1.0
                    ) -> List[Tuple[float, FrontierCluster]]:
    """Score frontiers and rank descending. Score formula:

        score = info_gain / (travel + visited_penalty + obstacle_penalty + glass_penalty + 1)

    With these guards:
      - travel > max_distance_m → cluster dropped entirely (local
        preference: don't try to teleport across a multi-room map).
      - centroid inside lethal halo → dropped (is_target_safe()).
      - centroid in/near a visited cell → travel penalty added so
        re-visiting unexplored fringes is preferred.
      - centroid near a failed goal → score heavily penalized.
      - centroid near obstacle → obstacle_penalty added so the planner
        avoids destinations dangerously close to walls.
      - centroid near glass door → glass_penalty added to avoid
        planning paths through glass doors.

    visited_cells is a set of (cx, cy) cell-space coordinates the
    skill has already driven through; the controller maintains it.
    failed_goals is a set of (x, y) world coords that caused nav failures.
    safe_radius_m is the dynamic radius for safety checks.
    glass_doors is a list of (x, y) world coords of detected glass doors.
    """
    scored = []
    for c in clusters:
        cx, cy = c.centroid_xy
        wx, wy = gv.cell_to_world(int(round(cx)), int(round(cy)))
        c_world = FrontierCluster(centroid_xy=(wx, wy),
                                  size=c.size,
                                  cell_indices=c.cell_indices)
        travel = ((wx - robot_xy[0]) ** 2 + (wy - robot_xy[1]) ** 2) ** 0.5
        if travel > max_distance_m:
            continue                             # too far — skip
        if not is_target_safe(gv, wx, wy, safe_radius_m=safe_radius_m):
            continue                             # would crash — skip

        penalty = 0.0
        if visited_cells:
            tcx, tcy = gv.world_to_cell(wx, wy)
            radius = max(1, int(round(visited_penalty_m / gv.resolution)))
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if (tcx + dx, tcy + dy) in visited_cells:
                        penalty = visited_penalty_m
                        break
                if penalty:
                    break

        # Penalize targets near failed navigation goals
        fail_penalty = 0.0
        if failed_goals:
            for fg_x, fg_y in failed_goals:
                dist = ((wx - fg_x) ** 2 + (wy - fg_y) ** 2) ** 0.5
                if dist < failed_goal_radius_m:
                    fail_penalty = 10.0  # heavy penalty, not skip
                    break

        # Obstacle proximity penalty: prefer frontiers farther from walls.
        # Frontier cells sit on the free/unknown boundary, but some are
        # right next to mapped obstacles (walls). The inflation layer will
        # push the actual endpoint inward, potentially causing planning
        # failures. This penalty biases selection toward safer, more open
        # frontiers.
        obstacle_penalty = 0.0
        dist_to_obstacle = _nearest_obstacle_distance(gv, wx, wy)
        if dist_to_obstacle < 1.0:
            # Linear penalty: 0 at 1m, 1.0 at 0m
            obstacle_penalty = max(0.0, 1.0 - dist_to_obstacle)

        # Glass door penalty: avoid frontiers near detected glass doors.
        # Glass doors are transparent to lidar but impassable for the robot.
        # Without this penalty, the planner may route through glass doors.
        glass_penalty = 0.0
        if glass_doors:
            for gx, gy in glass_doors:
                dist = ((wx - gx) ** 2 + (wy - gy) ** 2) ** 0.5
                if dist < glass_door_radius_m:
                    # Heavy penalty proportional to closeness
                    glass_penalty = max(glass_penalty, 10.0 * (1.0 - dist / glass_door_radius_m))

        score = c_world.size / (travel + penalty + fail_penalty + obstacle_penalty + glass_penalty + 1.0)
        scored.append((score, c_world))
    scored.sort(key=lambda t: t[0], reverse=True)
    return scored


def pick_target(gv: GridView, robot_xy: Tuple[float, float], *,
                 min_size: int = 3,
                 max_distance_m: float = 8.0,
                 visited_cells: Optional[set] = None,
                 failed_goals: Optional[set] = None,
                 failed_goal_radius_m: float = 0.5,
                 safe_radius_m: float = 0.15,
                 glass_doors: Optional[list] = None,
                 glass_door_radius_m: float = 1.0
                 ) -> Optional[FrontierCluster]:
    """End-to-end convenience. Returns None if no SAFE frontier in
    range — caller may declare done."""
    cells = find_frontier_cells(gv)
    if cells.size == 0:
        return None
    clusters = cluster_frontiers(cells, min_size=min_size)
    if not clusters:
        return None
    scored = score_clusters(clusters, gv, robot_xy,
                             max_distance_m=max_distance_m,
                             visited_cells=visited_cells,
                             failed_goals=failed_goals,
                             failed_goal_radius_m=failed_goal_radius_m,
                             safe_radius_m=safe_radius_m,
                             glass_doors=glass_doors,
                             glass_door_radius_m=glass_door_radius_m)
    return scored[0][1] if scored else None


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
