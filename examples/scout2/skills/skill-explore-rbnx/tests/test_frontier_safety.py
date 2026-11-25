import unittest
import math
import time
from unittest.mock import patch

import numpy as np

from explore_skill.frontier import (
    FRONTIER_SEGMENT_SPAN_M,
    MIN_SAFE_RADIUS_M,
    FrontierCluster,
    GridView,
    _clearance_mask,
    _frontier_goal_candidates,
    _known_free_line,
    MAX_VIEWPOINT_DISTANCE_M,
    find_frontier_cells,
    pick_target,
)


class FrontierGoalTest(unittest.TestCase):
    def _grid(self, data, resolution=0.1):
        return GridView(data, resolution, 0.0, 0.0, data.shape[1], data.shape[0])

    def test_enclosing_frontier_selects_boundary_goal_not_component_mean(self):
        data = np.full((120, 120), -1, dtype=np.int8)
        data[10:110, 10:110] = 0
        grid = self._grid(data)
        robot = grid.cell_to_world(60, 60)

        target = pick_target(grid, robot, max_distance_m=15.0)

        self.assertIsNotNone(target)
        tx, ty = target.centroid_xy
        cx, cy = grid.world_to_cell(tx, ty)
        self.assertGreater(np.linalg.norm(np.array([cx, cy]) - [60, 60]), 30)
        self.assertEqual(int(grid.data[cy, cx]), 0)
        self.assertFalse(any(np.array_equal([cx, cy], cell) for cell in find_frontier_cells(grid)))
        self.assertTrue(_clearance_mask(grid, MIN_SAFE_RADIUS_M)[cy, cx])

    def test_candidate_requires_full_known_free_clearance(self):
        data = np.full((80, 80), -1, dtype=np.int8)
        data[10:70, 10:70] = 0
        data[38, 60] = 100
        grid = self._grid(data)
        mask = _clearance_mask(grid, MIN_SAFE_RADIUS_M)

        self.assertFalse(mask[38, 59])
        self.assertFalse(mask[10, 30])
        self.assertTrue(mask[30, 30])

    def test_goal_clearance_includes_nav2_arrival_tolerance(self):
        """Reserve chassis clearance plus XY arrival uncertainty without relaxing route clearance."""
        data = np.full((100, 140), -1, dtype=np.int8)
        data[10:90, 10:130] = 0
        grid = self._grid(data, resolution=.05)
        robot = grid.cell_to_world(70, 50)
        base_only = pick_target(grid, robot, max_distance_m=15.,
                                arrival_tolerance_m=0.)
        tolerance = pick_target(grid, robot, max_distance_m=15.,
                                arrival_tolerance_m=.45)
        self.assertIsNotNone(base_only)
        self.assertIsNotNone(tolerance)
        self.assertTrue(_clearance_mask(grid, MIN_SAFE_RADIUS_M)[
            grid.world_to_cell(*base_only.centroid_xy)[1],
            grid.world_to_cell(*base_only.centroid_xy)[0]])
        endpoint_mask = _clearance_mask(grid, MIN_SAFE_RADIUS_M + .45)
        tx, ty = grid.world_to_cell(*tolerance.centroid_xy)
        self.assertTrue(endpoint_mask[ty, tx])
        self.assertGreaterEqual(MIN_SAFE_RADIUS_M + .45, 1.10)

    def test_frontier_in_narrow_known_corridor_is_not_declared_reachable(self):
        data = np.full((60, 100), -1, dtype=np.int8)
        data[26:34, 5:65] = 0
        grid = self._grid(data)
        robot = grid.cell_to_world(30, 29)

        self.assertIsNone(pick_target(grid, robot, max_distance_m=15.0))

    def test_frontier_behind_obstacle_wall_is_not_reachable(self):
        data = np.full((80, 120), -1, dtype=np.int8)
        data[10:70, 5:55] = 0
        data[10:70, 65:115] = 0
        data[10:70, 55:65] = 100
        grid = self._grid(data)
        robot = grid.cell_to_world(30, 40)

        target = pick_target(grid, robot, max_distance_m=15.0)

        self.assertIsNotNone(target)
        self.assertLess(grid.world_to_cell(*target.centroid_xy)[0], 55)

    def test_frontier_in_second_room_is_reachable_through_known_clear_doorway(self):
        """Cross a mapped doorway only when its known-free corridor fits clearance."""
        data = np.full((100, 170), 100, dtype=np.int8)
        data[10:90, 10:60] = 0
        data[40:60, 60:85] = 0  # 2.0 m wide connector
        data[10:90, 85:145] = 0
        data[10:90, 145:170] = -1
        grid = self._grid(data, resolution=.1)
        robot = grid.cell_to_world(35, 50)

        target = pick_target(grid, robot, max_distance_m=15.)

        self.assertIsNotNone(target)
        self.assertGreater(grid.world_to_cell(*target.centroid_xy)[0], 85)
        route_clear = _clearance_mask(grid, MIN_SAFE_RADIUS_M)
        self.assertTrue(route_clear[grid.world_to_cell(*robot)[1],
                                    grid.world_to_cell(*robot)[0]])
        self.assertTrue(route_clear[grid.world_to_cell(*target.centroid_xy)[1],
                                    grid.world_to_cell(*target.centroid_xy)[0]])

    def test_frontier_behind_doorway_narrower_than_clearance_is_not_reachable(self):
        """Do not enter another room through a passage that fails the chassis disk."""
        data = np.full((100, 170), 100, dtype=np.int8)
        data[10:90, 10:60] = 0
        data[45:55, 60:85] = 0  # 1.0 m wide, below the 1.30 m clearance diameter
        data[10:90, 85:145] = 0
        data[10:90, 145:170] = -1
        grid = self._grid(data, resolution=.1)
        robot = grid.cell_to_world(35, 50)
        diagnostics = {}

        target = pick_target(grid, robot, max_distance_m=15., diagnostics=diagnostics)

        self.assertIsNone(target)
        self.assertTrue(diagnostics['robot_clearance_safe'])
        self.assertGreater(diagnostics['rejected_route_reachability'], 0)

    def test_failed_goal_is_temporarily_excluded(self):
        data = np.full((120, 120), -1, dtype=np.int8)
        data[10:110, 10:110] = 0
        grid = self._grid(data)
        robot = grid.cell_to_world(60, 60)
        first = pick_target(grid, robot, max_distance_m=15.0)
        self.assertIsNotNone(first)

        retry = pick_target(grid, robot, max_distance_m=15.0,
                            failed_goals={first.centroid_xy: time.monotonic()},
                            failed_goal_cooldown_s=120.0)
        self.assertIsNotNone(retry)
        self.assertGreater(np.linalg.norm(np.subtract(retry.centroid_xy, first.centroid_xy)), 0.5)

    def test_large_frontier_is_split_into_local_segments(self):
        data = np.full((120, 120), -1, dtype=np.int8)
        data[10:110, 10:110] = 0
        grid = self._grid(data)
        target = pick_target(grid, grid.cell_to_world(60, 60), max_distance_m=15.0)

        self.assertIsNotNone(target)
        self.assertLessEqual(target.size, int(FRONTIER_SEGMENT_SPAN_M / grid.resolution) * 4)

    def test_narrow_boundary_can_be_observed_from_safe_room_without_driving_into_it(self):
        """A short point-free opening need not fit the robot to be observable."""
        data = np.full((120, 140), 100, dtype=np.int8)
        data[10:110, 10:70] = 0
        data[50:70, 70:100] = 0
        data[50:70, 100:130] = -1
        grid = self._grid(data, resolution=.05)
        robot = grid.cell_to_world(40, 60)
        diagnostics = {}
        target = pick_target(grid, robot, max_distance_m=15., diagnostics=diagnostics)
        self.assertIsNotNone(target)
        gx, gy = grid.world_to_cell(*target.centroid_xy)
        self.assertLess(gx, 70)  # Observe from the room, never enter the 1m opening.
        self.assertTrue(_clearance_mask(grid, MIN_SAFE_RADIUS_M)[gy, gx])
        boundary = find_frontier_cells(grid)
        view_distance = min(np.linalg.norm(np.array([gx, gy]) - cell)
                            for cell in boundary) * grid.resolution
        self.assertGreater(view_distance, .95)  # Previous short-ray search missed all views.
        self.assertLessEqual(view_distance, MAX_VIEWPOINT_DISTANCE_M)
        self.assertGreater(diagnostics['generated_viewpoints'], 0)
        self.assertTrue(diagnostics['robot_clearance_safe'])

    def test_visibility_does_not_look_through_walls_or_diagonal_corners(self):
        """A point-free search route alone is not proof that a sensor sees the boundary."""
        data = np.zeros((20, 20), dtype=np.int8)
        grid = self._grid(data)
        self.assertTrue(_known_free_line(grid, (1, 1), (15, 15)))
        data[4, 5] = 100
        self.assertFalse(_known_free_line(grid, (1, 1), (15, 15)))
        data[4, 5] = -1
        self.assertFalse(_known_free_line(grid, (1, 1), (15, 15)))
        self.assertFalse(_known_free_line(grid, (-1, 1), (15, 15)))

    def test_viewpoint_rejections_are_classified_by_first_failed_condition(self):
        """Separate endpoint clearance, route, visibility and distance failures."""
        cases = (
            ('clearance', (11, 11), np.zeros((11, 11), dtype=bool),
             np.ones((11, 11), dtype=bool), None),
            ('route', (11, 11), np.ones((11, 11), dtype=bool),
             np.zeros((11, 11), dtype=bool), None),
            ('visibility', (11, 11), np.ones((11, 11), dtype=bool),
             np.ones((11, 11), dtype=bool), False),
            ('distance', (61, 61), np.ones((61, 61), dtype=bool),
             np.ones((61, 61), dtype=bool), False),
        )
        for rejected, shape, safe, reachable, visible in cases:
            with self.subTest(rejected=rejected):
                grid = self._grid(np.zeros(shape, dtype=np.int8))
                diagnostics = {}
                call = lambda: _frontier_goal_candidates(
                    grid, np.array([[shape[1] // 2, shape[0] // 2]]),
                    reachable, safe, diagnostics)
                if visible is None:
                    candidates = call()
                else:
                    with patch('explore_skill.frontier._known_free_line',
                               return_value=visible):
                        candidates = call()
                self.assertEqual(candidates, [])
                expected = {
                    'clearance': 'rejected_goal_clearance',
                    'route': 'rejected_route_reachability',
                    'visibility': 'rejected_visibility',
                    'distance': 'rejected_view_distance',
                }[rejected]
                self.assertGreater(diagnostics[expected], 0)
                allowed = ({expected, 'rejected_visibility'} if rejected == 'distance'
                           else {expected})
                for field in ('rejected_goal_clearance',
                              'rejected_route_reachability',
                              'rejected_visibility', 'rejected_view_distance'):
                    if field not in allowed:
                        self.assertEqual(diagnostics[field], 0, (rejected, field))

    def test_candidate_observation_search_remains_bounded(self):
        """A frontier beyond the observation range must not pull a goal deep into a room."""
        data = np.full((120, 200), 100, dtype=np.int8)
        data[10:110, 10:70] = 0
        data[50:70, 70:160] = 0
        data[50:70, 160:190] = -1
        grid = self._grid(data, resolution=.05)
        diagnostics = {}
        self.assertIsNone(pick_target(grid, grid.cell_to_world(40, 60), diagnostics=diagnostics))
        self.assertTrue(diagnostics['robot_clearance_safe'])
        self.assertGreater(diagnostics['reachable_cells'], 0)
        self.assertEqual(diagnostics['generated_viewpoints'], 0)

    def test_diagnostics_distinguish_budget_rejection_from_missing_viewpoints(self):
        """Preserve actionable rejection counts instead of a generic no-frontier error."""
        data = np.full((120, 120), -1, dtype=np.int8)
        data[10:110, 10:110] = 0
        grid = self._grid(data)
        diagnostics = {}
        target = pick_target(grid, grid.cell_to_world(60, 60),
                             max_travel_distance_m=.01, diagnostics=diagnostics)
        self.assertIsNone(target)
        self.assertGreater(diagnostics['ranked_goals'], 0)
        self.assertEqual(diagnostics['ranked_goals'], diagnostics['rejected_budget'])
        self.assertEqual(diagnostics['feasible_goals'], 0)

    def test_reference_budget_overrun_is_advisory_within_hard_cap(self):
        """Prefer the same safe route even if its estimate slightly exceeds reference."""
        data = np.full((100, 140), -1, dtype=np.int8)
        data[10:90, 10:130] = 0
        grid = self._grid(data)
        robot = grid.cell_to_world(70, 50)
        baseline = pick_target(grid, robot, max_distance_m=15.)
        self.assertIsNotNone(baseline)
        diagnostics = {}
        target = pick_target(grid, robot, max_distance_m=15.,
            max_travel_distance_m=baseline.travel_distance_m + .01,
            preferred_travel_distance_m=baseline.travel_distance_m - .01,
            diagnostics=diagnostics)
        self.assertIsNotNone(target)
        self.assertAlmostEqual(target.travel_distance_m, baseline.travel_distance_m)
        self.assertTrue(diagnostics['reference_budget_exceeded'])

    def test_shortest_over_reference_route_is_selected_and_reported(self):
        """When every legal candidate exceeds reference, try the nearest one."""
        data = np.full((100, 140), -1, dtype=np.int8)
        data[10:90, 10:130] = 0
        grid = self._grid(data)
        robot = grid.cell_to_world(70, 50)
        shorter = FrontierCluster((2., 2.), 4, np.zeros((1, 2)), 13.6)
        farther = FrontierCluster((3., 3.), 4, np.zeros((1, 2)), 14.0)
        diagnostics = {}
        with patch('explore_skill.frontier.score_clusters', return_value=[
                (1., farther), (100., shorter)]):
            target = pick_target(grid, robot, max_distance_m=20.,
                max_travel_distance_m=15., preferred_travel_distance_m=13.5,
                diagnostics=diagnostics)
        self.assertIs(target, shorter)
        self.assertEqual(diagnostics['reference_budget_candidates'], 0)
        self.assertTrue(diagnostics['reference_budget_exceeded'])

    def test_long_frontier_route_uses_safe_known_space_waypoint(self):
        """Split a route above its per-leg cap without cutting through unknown cells."""
        data = np.full((120, 180), 100, dtype=np.int8)
        data[10:110, 10:70] = 0
        data[20:110, 70:170] = 0
        data[20:110, 170:180] = -1
        # Force the shortest safe route around the divider's lower opening.
        data[20:80, 100] = 100
        grid = self._grid(data)
        robot = grid.cell_to_world(35, 60)
        diagnostics = {}
        target = pick_target(grid, robot, max_distance_m=15.,
            max_travel_distance_m=8., preferred_travel_distance_m=7.5,
            allow_route_splitting=True, arrival_tolerance_m=.45,
            diagnostics=diagnostics)
        self.assertIsNotNone(target)
        self.assertTrue(diagnostics['intermediate_waypoint'])
        self.assertIsNotNone(target.frontier_goal_xy)
        self.assertGreater(diagnostics['min_route_distance_m'], 8.)
        self.assertLessEqual(target.travel_distance_m, 8. + 1e-6)
        self.assertGreater(math.dist(target.centroid_xy, robot), .90)
        self.assertGreaterEqual(grid.world_to_cell(*target.centroid_xy)[1], 80)
        endpoint_mask = _clearance_mask(grid, MIN_SAFE_RADIUS_M + .45)
        x, y = grid.world_to_cell(*target.centroid_xy)
        self.assertTrue(endpoint_mask[y, x])
        self.assertGreater(math.dist(target.frontier_goal_xy, target.centroid_xy), 3.)

        # After arrival, solve again from the waypoint instead of reusing a
        # preplanned waypoint list or treating the intermediate stop as done.
        next_target = pick_target(grid, target.centroid_xy, max_distance_m=15.,
            max_travel_distance_m=8., preferred_travel_distance_m=7.5,
            allow_route_splitting=True, arrival_tolerance_m=.45)
        self.assertIsNotNone(next_target)
        self.assertGreater(math.dist(next_target.centroid_xy, target.centroid_xy), .90)
        self.assertLessEqual(next_target.travel_distance_m, 8. + 1e-6)
        nx, ny = grid.world_to_cell(*next_target.centroid_xy)
        self.assertTrue(endpoint_mask[ny, nx])

    def test_route_split_fails_closed_without_a_useful_safe_waypoint(self):
        """Do not dispatch tiny or unsafe route fragments when a full goal is over cap."""
        data = np.full((100, 140), -1, dtype=np.int8)
        data[10:90, 10:130] = 0
        grid = self._grid(data)
        diagnostics = {}
        target = pick_target(grid, grid.cell_to_world(70, 50),
            max_distance_m=15., max_travel_distance_m=.5,
            allow_route_splitting=True, arrival_tolerance_m=.45,
            diagnostics=diagnostics)
        self.assertIsNone(target)
        self.assertTrue(diagnostics['route_split_unavailable'])


if __name__ == "__main__":
    unittest.main()
