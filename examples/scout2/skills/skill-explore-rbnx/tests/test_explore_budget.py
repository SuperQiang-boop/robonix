"""Exercise real task loops and navigation calls with a virtual clock and no ROS."""
import ast
import logging
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from explore_skill.configuration import ExploreSettings
from explore_skill.controller import ExploreController, TaskHandle
from explore_skill.frontier import GridView, _travel_distances, pick_target


class Clock:
    """Advance test time without waiting or connecting to any robot."""
    def __init__(self):
        self.value = 100.0

    def now(self):
        return self.value

    def sleep(self, seconds):
        self.value += seconds


class ExploreBudgetTests(unittest.TestCase):
    def setUp(self):
        """Use the actual controller with fake RPC endpoints and deterministic time."""
        self.clock = Clock()
        self.ctrl = ExploreController(map_topic='/fixture/map',
            nav_navigate_endpoint='navigate', nav_status_endpoint='status',
            nav_cancel_endpoint='cancel', nav_get_speed_endpoint='speed')
        self.ctrl._latest_pose_xyyaw = (0.0, 0.0, 0.0)
        self.ctrl._latest_pose_received_at = 100.0
        self.handle = TaskHandle(task_id='test', started_at=0.0, timeout_s=300,
                                 started_monotonic=100.0, max_speed_m_s=0.3,
                                 last_progress_t=100.0)
        self.calls = []
        self.cancelled = False

    def rpc(self, tool, args, endpoint=None, **options):
        """Return a running action until the exact run has been cancelled."""
        self.calls.append((tool, args))
        if tool == 'get_speed_limit':
            return dict(available=True, effective_linear_speed_mps=0.14851485148514854)
        if tool == 'navigate':
            return dict(accepted=True, run_id='owned-run')
        if tool == 'cancel':
            self.cancelled = True
            return dict(ok=True)
        if tool == 'status':
            return dict(known=True, state='CANCELED' if self.cancelled else 'RUNNING')
        return {}

    def succeeded_rpc(self, tool, args, endpoint=None, **options):
        """Record calls while simulating Nav2 terminal success."""
        result = self.rpc(tool, args, endpoint=endpoint, **options)
        return dict(known=True, state='SUCCEEDED') if tool == 'status' else result

    def test_manifest_defaults_and_request_overrides_are_explicit(self):
        """Unspecified tasks use unlimited total time; explicit caps still apply."""
        self.assertEqual(ExploreSettings().timeout_s, 0.)
        self.assertEqual(ExploreSettings.parse({}).resolve_timeout(-1), 0.)
        settings = ExploreSettings.parse(dict(timeout_s=600, explore_mode='frontier'))
        self.assertEqual(settings.resolve_timeout(-1), 600)
        self.assertEqual(settings.resolve_timeout(300), 300)
        self.assertEqual(settings.resolve_timeout(0), 0)
        for invalid in (-2, math.inf, math.nan, True):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                settings.resolve_timeout(invalid)
        for config in ({'timeout_s': True}, {'timeout_s': -1}, {'bad_key': 1},
                       {'nav_time_factor': .5}, {'sweep_every_n_legs': 1.2}):
            with self.subTest(config=config), self.assertRaises(ValueError):
                ExploreSettings.parse(config)

    def test_bridge_uses_validated_config_and_reports_effective_timeout(self):
        """Execute the actual bridge handlers without importing Atlas or ROS."""
        path = Path(__file__).parents[1] / 'explore_skill/atlas_bridge.py'
        tree = ast.parse(path.read_text())
        functions = [n for n in tree.body if isinstance(n, ast.FunctionDef)
                     and n.name in ('init', 'explore')]
        for function in functions:
            function.decorator_list = []
        namespace = dict(ExploreSettings=ExploreSettings, log=logging.getLogger('test'),
                         Ok=lambda: 'ok', Err=lambda message: message,
                         Explore_Request=object, Explore_Response=lambda **args: args)
        exec(compile(ast.Module(body=functions, type_ignores=[]), str(path), 'exec'), namespace)
        self.assertEqual(namespace['init']({'timeout_s': 900}), 'ok')
        namespace['ctrl'] = Mock()
        namespace['ctrl'].start.return_value = SimpleNamespace(task_id='run', detail='timeout_s=900')
        result = namespace['explore'](SimpleNamespace(area_hint='', timeout_s=-1, max_speed_m_s=.2))
        self.assertTrue(result['accepted'])
        self.assertEqual(namespace['ctrl'].start.call_args.kwargs['timeout_s'], 900)

    def test_bridge_preserves_frontier_rejection_diagnostics_in_status(self):
        """Keep each diagnostic field on the MCP status response."""
        path = Path(__file__).parents[1] / 'explore_skill/atlas_bridge.py'
        tree = ast.parse(path.read_text())
        function = next(node for node in tree.body
                        if isinstance(node, ast.FunctionDef) and node.name == 'status')
        function.decorator_list = []
        values = {
            'frontier_regions': 25,
            'safe_observation_candidates': 0,
            'budget_rejected_candidates': 0,
            'rejected_goal_clearance': 17,
            'rejected_route_reachability': 19,
            'rejected_visibility': 23,
            'rejected_view_distance': 29,
            'rejected_local_distance': 31,
            'rejected_near_robot': 37,
            'rejected_cooldown': 41,
            'viewpoint_cells_checked': 166,
        }
        namespace = {
            'GetExploreStatus_Request': object,
            'GetExploreStatus_Response': lambda **kwargs: kwargs,
            '_canonical_state': lambda state: state,
            'ctrl': Mock(),
        }
        namespace['ctrl'].status.return_value = values
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), 'exec'), namespace)
        response = namespace['status'](SimpleNamespace(run_id='run'))
        for field, value in values.items():
            self.assertEqual(response[field], value, field)

    def test_live_speed_overrides_optimistic_request_in_estimate(self):
        """A 0.3m/s request cannot budget as though the 0.149m/s guard were faster."""
        self.ctrl._mcp_call_sync = self.rpc
        speed = self.ctrl._navigation_speed(self.handle)
        self.assertAlmostEqual(speed, .14851485148514854)
        estimate = self.ctrl._estimate_leg_seconds(14.55, speed)
        self.assertGreater(estimate, 170)
        self.assertLessEqual(estimate, 180)
        with patch('explore_skill.controller.time.monotonic', self.clock.now):
            hard_limit = self.ctrl._leg_budget(None)
            finite_limit = self.ctrl._leg_budget(253)
        reference = self.ctrl._reference_leg_budget(hard_limit)
        self.assertEqual(hard_limit, 180)
        self.assertEqual(reference, 170)
        self.assertGreater(self.ctrl._max_route_distance(hard_limit, speed), 13.8)
        self.assertEqual(finite_limit, 148)
        self.assertLessEqual(self.ctrl._estimate_leg_seconds(
            self.ctrl._max_route_distance(hard_limit, speed), speed), hard_limit)
        self.ctrl._mcp_call_sync = lambda *args: {'available': False}
        with self.assertRaisesRegex(RuntimeError, 'unavailable'):
            self.ctrl._navigation_speed(self.handle)

    def test_route_estimate_accounts_for_detours(self):
        """A wall increases route length even when the destination is nearby."""
        safe = np.ones((12, 12), dtype=bool)
        safe[:10, 6] = False
        distances = _travel_distances(safe, (4, 2), .1)
        self.assertGreater(distances[2, 8], 1.5)
        self.assertAlmostEqual(math.dist((4, 2), (8, 2)) * .1, .4)

    def test_target_selection_skips_large_segments_that_exceed_time_budget(self):
        """Only boundary goals with a feasible route survive a short budget."""
        data = np.full((100, 160), -1, dtype=np.int8)
        data[10:90, 10:150] = 0
        grid = GridView(data, .1, 0., 0., 160, 100)
        robot = grid.cell_to_world(35, 45)
        short = pick_target(grid, robot, max_distance_m=15., max_travel_distance_m=2.)
        self.assertIsNotNone(short)
        self.assertLessEqual(short.travel_distance_m, 2.)
        self.assertIsNone(pick_target(grid, robot, max_travel_distance_m=.1))

    def test_translation_faces_travel_direction_and_stops_at_observation_position(self):
        """Fresh XY arrival cancels, then requires settled actual-pose verification."""
        self.ctrl._mcp_call_sync = self.rpc
        target = (0.0, 2.0)
        def sleep(seconds):
            self.clock.sleep(seconds)
            self.ctrl._latest_pose_xyyaw = (0., 1.6, .7)
            self.ctrl._latest_pose_received_at = self.clock.now()
        with patch('explore_skill.controller.time.monotonic', self.clock.now), \
             patch('explore_skill.controller.time.sleep', sleep), \
             patch.object(self.ctrl, '_wait_for_safe_arrival', return_value=(True, 'arrival footprint verified')) as verify:
            ok, detail = self.ctrl._nav_navigate_blocking(*target, yaw=None,
                timeout_s=30, cancel_evt=self.handle)
        self.assertTrue(ok)
        self.assertIn('observation position reached', detail)
        verify.assert_called_once_with(*target, 130.0, cancel_evt=self.handle)
        goal = next(args['goal'] for tool, args in self.calls if tool == 'navigate')
        self.assertAlmostEqual(goal['pose']['orientation']['z'], math.sin(math.pi/4))
        self.assertIn(('cancel', {'run_id': 'owned-run'}), self.calls)
        self.assertEqual(self.handle.active_nav_run_id, '')

    def test_nav2_success_still_verifies_actual_pose_clearance(self):
        """The action terminal path cannot bypass settled actual-pose validation."""
        self.ctrl._mcp_call_sync = self.succeeded_rpc
        verified = Mock(return_value=(True, 'arrival footprint verified'))
        with patch('explore_skill.controller.time.monotonic', self.clock.now), \
             patch.object(self.ctrl, '_wait_for_safe_arrival', verified):
            ok, detail = self.ctrl._nav_navigate_blocking(0., 2., yaw=None,
                timeout_s=5., cancel_evt=self.handle)
        self.assertTrue(ok)
        self.assertIn('nav terminal: SUCCEEDED', detail)
        verified.assert_called_once_with(0., 2., 105.0, cancel_evt=self.handle)
        self.assertEqual(self.handle.active_nav_run_id, '')

    def test_nav2_success_is_rejected_when_actual_pose_fails_clearance(self):
        """A SUCCESS result is downgraded if the actual footprint is unsafe."""
        self.ctrl._mcp_call_sync = self.succeeded_rpc
        with patch('explore_skill.controller.time.monotonic', self.clock.now), \
             patch.object(self.ctrl, '_wait_for_safe_arrival',
                          return_value=(False, 'unsafe arrival: actual pose lacks clearance')):
            ok, detail = self.ctrl._nav_navigate_blocking(0., 2., yaw=None,
                timeout_s=5., cancel_evt=self.handle)
        self.assertFalse(ok)
        self.assertIn('unsafe arrival', detail)
        self.assertEqual(self.handle.active_nav_run_id, '')

    def test_actual_footprint_is_checked_on_both_navigation_completion_paths(self):
        """Exercise real map clearance after Nav2 success and confirmed early stop."""
        for completion in ('nav2', 'observation'):
            for blocked in (None, 100, -1, 30):
                with self.subTest(completion=completion, blocked=blocked):
                    self.clock.value = 100.
                    self.calls.clear()
                    self.cancelled = False
                    data = np.zeros((80, 80), dtype=np.int8)
                    if blocked is not None:
                        # Target at y=2 is clear, but actual y=1.65 is too close.
                        data[50, 40] = blocked
                    self.ctrl._latest_map = SimpleNamespace(
                        data=data.ravel().tolist(),
                        info=SimpleNamespace(height=80, width=80, resolution=.1,
                            origin=SimpleNamespace(
                                position=SimpleNamespace(x=-4., y=-4.),
                                orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))))
                    self.ctrl._latest_pose_xyyaw = (0., 1.65, .7)
                    self.ctrl._latest_pose_received_at = self.clock.now()
                    self.ctrl._mcp_call_sync = (self.succeeded_rpc
                                               if completion == 'nav2' else self.rpc)

                    def observe(seconds):
                        """Keep receiving the same stationary actual pose."""
                        self.clock.sleep(seconds)
                        self.ctrl._latest_pose_received_at = self.clock.now()

                    with patch('explore_skill.controller.time.monotonic', self.clock.now), \
                         patch('explore_skill.controller.time.sleep', observe):
                        ok, detail = self.ctrl._nav_navigate_blocking(0., 2., yaw=None,
                            timeout_s=10., cancel_evt=self.handle)
                    self.assertEqual(ok, blocked is None, detail)
                    if blocked is not None:
                        self.assertIn('unsafe arrival: actual pose', detail)
                        self.assertIn('0.65m chassis clearance', detail)
                    if completion == 'observation':
                        self.assertIn(('cancel', {'run_id': 'owned-run'}), self.calls)
                        self.assertEqual(self.calls[-1], ('status', {'run_id': 'owned-run'}))
                    self.assertEqual(self.handle.active_nav_run_id, '')

    def test_arrival_requires_map_fresh_pose_tolerance_and_stability(self):
        """Do not turn missing data, stale TF, drift or out-of-tolerance poses into success."""
        grid = GridView(np.zeros((80, 80), dtype=np.int8), .1, -4., -4., 80, 80)
        for problem in ('no_map', 'stale_tf', 'outside_tolerance', 'moving'):
            with self.subTest(problem=problem):
                self.clock.value = 100.
                self.ctrl._latest_map = None if problem == 'no_map' else object()
                self.ctrl._latest_pose_xyyaw = (0., 2., 0.)
                self.ctrl._latest_pose_received_at = 90. if problem == 'stale_tf' else 100.
                if problem == 'outside_tolerance':
                    self.ctrl._latest_pose_xyyaw = (0., 1., 0.)

                def observe(seconds):
                    """Model either unavailable TF or continuing post-stop translation."""
                    self.clock.sleep(seconds)
                    if problem != 'stale_tf':
                        self.ctrl._latest_pose_received_at = self.clock.now()
                    if problem == 'moving':
                        offset = .1 if int(self.clock.now()) % 2 else 0.
                        self.ctrl._latest_pose_xyyaw = (offset, 2., 0.)

                with patch('explore_skill.controller.time.monotonic', self.clock.now), \
                     patch('explore_skill.controller.time.sleep', observe), \
                     patch('explore_skill.frontier.GridView.from_msg', return_value=grid):
                    ok, detail = self.ctrl._wait_for_safe_arrival(
                        0., 2., 105., cancel_evt=self.handle)
                self.assertFalse(ok)
                self.assertIn('unsafe arrival:', detail)
                self.assertEqual(self.clock.now(), 105.)

    def test_arrival_stability_window_restarts_after_stale_tf(self):
        """A fresh pose after a TF gap needs a complete new settling interval."""
        self.ctrl._latest_map = object()
        self.ctrl._latest_pose_xyyaw = (0., 2., 0.)
        grid = GridView(np.zeros((80, 80), dtype=np.int8), .1, -4., -4., 80, 80)

        def observe(seconds):
            """Interrupt the first stability interval with one stale observation."""
            self.clock.sleep(seconds)
            self.ctrl._latest_pose_received_at = (90. if self.clock.now() == 101.
                                                  else self.clock.now())

        with patch('explore_skill.controller.time.monotonic', self.clock.now), \
             patch('explore_skill.controller.time.sleep', observe), \
             patch('explore_skill.frontier.GridView.from_msg', return_value=grid):
            ok, detail = self.ctrl._wait_for_safe_arrival(
                0., 2., 106., cancel_evt=self.handle)
        self.assertTrue(ok, detail)
        self.assertEqual(self.clock.now(), 104.)

    def test_cancel_during_arrival_verification_returns_promptly(self):
        """Verification remains cancelable after the navigation action is already stopped."""
        def cancel(seconds):
            """Cancel on the first wait for map or pose verification."""
            self.clock.sleep(seconds)
            self.handle.cancel_requested = True

        with patch('explore_skill.controller.time.monotonic', self.clock.now), \
             patch('explore_skill.controller.time.sleep', cancel):
            ok, detail = self.ctrl._wait_for_safe_arrival(
                0., 2., 180., cancel_evt=self.handle)
        self.assertFalse(ok)
        self.assertIn('canceled', detail)
        self.assertEqual(self.clock.now(), 101.)

    def test_exploration_classifies_unsafe_arrival_and_reselects_instead_of_success(self):
        """Reject unsafe arrival, cool down its target, and select a different one."""
        grid = GridView(np.zeros((40, 40), dtype=np.int8), .1, 0., 0., 40, 40)
        self.ctrl._latest_map = object()
        self.ctrl._mcp_call_sync = self.rpc

        def navigate(*args, **kwargs):
            """Stop only after the second dispatch, so reselection actually executes."""
            if self.ctrl._nav_navigate_blocking.call_count == 1:
                return False, 'unsafe arrival: actual pose failed footprint clearance'
            self.handle.cancel_requested = True
            return False, 'canceled during nav'

        self.ctrl._nav_navigate_blocking = Mock(side_effect=navigate)
        with patch('explore_skill.controller.time.monotonic', self.clock.now), \
             patch('explore_skill.frontier.GridView.from_msg', return_value=grid), \
             patch('explore_skill.frontier.total_frontier_count', return_value=4), \
             patch('explore_skill.frontier.pick_target', side_effect=[
                 SimpleNamespace(centroid_xy=(1., 0.), size=12, travel_distance_m=.7),
                 SimpleNamespace(centroid_xy=(2., 0.), size=10, travel_distance_m=1.),
             ]) as select, \
             patch.object(self.ctrl, '_record_progress', return_value=False), \
             patch.object(self.ctrl, '_progress_stale', return_value=False), \
             patch.object(self.ctrl, '_navigation_speed', return_value=.148), \
             patch.object(self.ctrl, '_leg_budget', return_value=170.), \
             patch.object(self.ctrl, '_estimate_leg_seconds', return_value=60.), \
             patch('explore_skill.controller.time.sleep', self.clock.sleep):
            self.ctrl._run_task(self.handle)
        self.assertEqual(self.ctrl._nav_navigate_blocking.call_count, 2)
        self.assertIn((1., 0.), self.ctrl._failed_goals)
        self.assertIn((1., 0.), select.call_args_list[1].kwargs['failed_goals'])
        self.assertEqual(select.call_args_list[0].kwargs['arrival_tolerance_m'], .45)
        self.assertEqual(self.handle.legs_completed, 0)
        self.assertEqual(self.handle.state, 'canceled')

    def test_sweep_never_completes_merely_from_xy_arrival(self):
        """Yaw-only actions still need Nav2 terminal success."""
        self.ctrl._mcp_call_sync = self.rpc
        with patch('explore_skill.controller.time.monotonic', self.clock.now), \
             patch('explore_skill.controller.time.sleep', self.clock.sleep):
            ok, detail = self.ctrl._nav_navigate_blocking(0., 0., yaw=math.pi/2,
                timeout_s=3, cancel_evt=self.handle)
        self.assertFalse(ok)
        self.assertEqual(detail, 'leg timeout')

    def test_stale_pose_does_not_claim_arrival(self):
        """An old TF sample cannot turn a running action into an arrival."""
        self.ctrl._mcp_call_sync = self.rpc
        self.ctrl._latest_pose_received_at = 90.
        with patch('explore_skill.controller.time.monotonic', self.clock.now), \
             patch('explore_skill.controller.time.sleep', self.clock.sleep):
            ok, _ = self.ctrl._nav_navigate_blocking(.1, 0., yaw=None,
                timeout_s=4, cancel_evt=self.handle)
        self.assertFalse(ok)

    def test_unconfirmed_cancel_is_a_terminal_error(self):
        """Do not send the next frontier while the old action might still move."""
        def rpc(tool, args, **options):
            if tool == 'status':
                return {'known': True, 'state': 'RUNNING'}
            return self.rpc(tool, args)
        self.ctrl._mcp_call_sync = rpc
        with patch('explore_skill.controller.time.monotonic', self.clock.now), \
             patch('explore_skill.controller.time.sleep', self.clock.sleep):
            ok, detail = self.ctrl._nav_navigate_blocking(2., 0., yaw=None,
                timeout_s=2, cancel_evt=self.handle)
        self.assertFalse(ok)
        self.assertEqual(self.handle.state, 'error')
        self.assertIn('cancellation was not confirmed', detail)

    def test_budget_too_small_never_dispatches_an_infeasible_goal(self):
        """Replay the end-of-run budget failure using the real selection loop."""
        data = np.full((100, 160), -1, dtype=np.int8)
        data[10:90, 10:150] = 0
        grid = GridView(data, .1, 0., 0., 160, 100)
        self.ctrl._latest_pose_xyyaw = (*grid.cell_to_world(80, 50), 0.)
        self.ctrl._latest_map = object()
        self.ctrl._mcp_call_sync = self.rpc
        self.handle.timeout_s = 5
        with patch('explore_skill.controller.time.monotonic', self.clock.now), \
             patch('explore_skill.frontier.GridView.from_msg', return_value=grid):
            self.ctrl._run_task(self.handle)
        self.assertEqual(self.handle.state, 'timeout')
        self.assertIn('hard leg limit', self.handle.detail)
        self.assertIn('reference', self.handle.detail)
        self.assertIn('safe frontiers remain', self.handle.detail)
        self.assertFalse(any(tool == 'navigate' for tool, _ in self.calls))

    def test_estimate_above_reference_is_dispatched_within_hard_leg_cap(self):
        """An uncertain route estimate over 170s is tried below the 180s action cap."""
        grid = GridView(np.zeros((40, 40), dtype=np.int8), .1, 0., 0., 40, 40)
        self.ctrl._latest_map = object()
        self.ctrl._mcp_call_sync = self.rpc
        route_m = 14.0
        estimate_s = self.ctrl._estimate_leg_seconds(
            route_m, .14851485148514854)
        self.assertGreater(estimate_s, 170.)
        self.assertLess(estimate_s, 180.)
        target = SimpleNamespace(centroid_xy=(3., 0.), size=8,
            travel_distance_m=route_m, frontier_goal_xy=None)
        progress = []

        def select(*_args, diagnostics, **_kwargs):
            """Provide one hard-cap-feasible goal that exceeds only the reference."""
            diagnostics.update(ranked_goals=1, rejected_budget=0,
                min_route_distance_m=route_m, reference_budget_exceeded=True,
                intermediate_waypoint=False, route_split_unavailable=False)
            return target

        def stop_after_dispatch(*_args, **_kwargs):
            progress.append(self.handle.detail)
            self.handle.cancel_requested = True
            return False, 'canceled during nav'

        self.ctrl._nav_navigate_blocking = Mock(side_effect=stop_after_dispatch)
        with patch('explore_skill.controller.time.monotonic', self.clock.now), \
             patch('explore_skill.frontier.GridView.from_msg', return_value=grid), \
             patch('explore_skill.frontier.mapped_free_area_m2', return_value=2.), \
             patch('explore_skill.frontier.total_frontier_count', return_value=1), \
             patch('explore_skill.frontier.pick_target', side_effect=select), \
             patch.object(self.ctrl, '_record_progress', return_value=False), \
             patch.object(self.ctrl, '_progress_stale', return_value=False), \
             patch.object(self.ctrl, '_update_glass_doors'), \
             patch.object(self.ctrl, '_navigation_speed', return_value=.14851485148514854):
            self.ctrl._run_task(self.handle)
        self.ctrl._nav_navigate_blocking.assert_called_once()
        self.assertAlmostEqual(
            self.ctrl._nav_navigate_blocking.call_args.kwargs['timeout_s'], 180.)
        self.assertLessEqual(estimate_s, 180.)
        self.assertIn('reference exceeded', progress[0])
        self.assertIn('actual travel time uncertain', progress[0])
        self.assertEqual(self.handle.legs_completed, 0)
        self.assertEqual(self.handle.state, 'canceled')

    def test_real_long_frontier_route_reselects_after_safe_waypoint(self):
        """Reach one known-space checkpoint, then select again from its fresh pose."""
        data = np.full((120, 180), 100, dtype=np.int8)
        data[10:110, 10:70] = 0
        data[20:110, 70:170] = 0
        data[20:110, 170:180] = -1
        data[20:80, 100] = 100
        grid = GridView(data, .1, 0., 0., 180, 120)
        self.ctrl._latest_map = object()
        self.ctrl._latest_pose_xyyaw = (*grid.cell_to_world(35, 60), 0.)
        self.ctrl._latest_pose_received_at = self.clock.now()
        dispatches = []

        def navigate(x, y, **_kwargs):
            dispatches.append((x, y, self.handle.detail))
            if len(dispatches) == 1:
                self.ctrl._latest_pose_xyyaw = (x, y, .4)
                self.ctrl._latest_pose_received_at = self.clock.now()
                return True, 'arrival footprint verified'
            self.handle.cancel_requested = True
            return False, 'canceled during nav'

        self.ctrl._nav_navigate_blocking = Mock(side_effect=navigate)
        with patch('explore_skill.controller.time.monotonic', self.clock.now), \
             patch('explore_skill.frontier.GridView.from_msg', return_value=grid), \
             patch('explore_skill.frontier.mapped_free_area_m2', return_value=10.), \
             patch.object(self.ctrl, '_navigation_speed', return_value=.14851485148514854), \
             patch.object(self.ctrl, '_update_glass_doors'), \
             patch.object(self.ctrl, '_record_progress', return_value=False), \
             patch.object(self.ctrl, '_progress_stale', return_value=False), \
             patch('explore_skill.controller.time.sleep', lambda _seconds: None):
            self.ctrl._run_task(self.handle)

        self.assertEqual(len(dispatches), 2)
        self.assertIn('safe waypoint', dispatches[0][2])
        self.assertIn('intermediate waypoint toward', dispatches[0][2])
        self.assertIn('frontier observation target', dispatches[1][2])
        self.assertNotIn('intermediate waypoint toward', dispatches[1][2])
        self.assertGreater(math.dist(dispatches[0][:2], dispatches[1][:2]), .5)
        self.assertEqual(self.handle.legs_completed, 1)
        self.assertEqual(self.handle.state, 'canceled')

    def test_default_and_short_deadlines_do_not_spend_time_on_full_sweeps(self):
        """Scout2 opts out; an opted-in sweep also needs adequate total budget."""
        self.assertFalse(self.ctrl._should_sweep(self.handle))
        self.ctrl.settings = ExploreSettings(sweep_every_n_legs=3)
        self.ctrl._nav_navigate_blocking = Mock()
        with patch('explore_skill.controller.time.monotonic', self.clock.now):
            self.ctrl._sweep_360(self.handle, deadline=153.)
        self.ctrl._nav_navigate_blocking.assert_not_called()

    def test_motionless_translation_is_cancelled_after_turn_allowance_and_stall_window(self):
        """Reject a stationary goal without falsely stalling useful detour travel."""
        self.ctrl._mcp_call_sync = self.rpc
        def sleep(seconds):
            self.clock.sleep(seconds)
            self.ctrl._latest_pose_received_at = self.clock.now()
        with patch('explore_skill.controller.time.monotonic', self.clock.now), \
             patch('explore_skill.controller.time.sleep', sleep):
            ok, detail = self.ctrl._nav_navigate_blocking(2., 0., yaw=None,
                timeout_s=180, cancel_evt=self.handle)
        self.assertFalse(ok)
        self.assertIn('no translation progress', detail)
        self.assertLess(self.clock.now(), 160)

    def test_unproductive_opted_in_sweep_disables_future_sweeps(self):
        """A completed yaw sequence alone does not justify more rotation time."""
        self.ctrl.settings = ExploreSettings(sweep_every_n_legs=3)
        self.handle.timeout_s = 600.
        self.ctrl._latest_map = object()
        self.ctrl._nav_navigate_blocking = Mock(return_value=(True, 'done'))
        self.handle.last_area_m2 = 10.
        with patch('explore_skill.controller.time.monotonic', self.clock.now), \
             patch('explore_skill.controller.time.sleep', self.clock.sleep), \
             patch('explore_skill.frontier.GridView.from_msg', return_value=object()), \
             patch('explore_skill.frontier.mapped_free_area_m2', return_value=10.):
            self.ctrl._sweep_360(self.handle, deadline=700.)
        self.assertTrue(self.handle.sweep_without_growth)
        self.assertEqual(self.ctrl._nav_navigate_blocking.call_count, 4)
        self.ctrl._sweep_360(self.handle, deadline=700.)
        self.assertEqual(self.ctrl._nav_navigate_blocking.call_count, 4)

    def test_rpc_exception_finishes_task_and_cleans_up_its_owned_goal(self):
        """A worker exception cannot leave a RUNNING task or unowned action."""
        self.handle.active_nav_run_id = 'owned-run'
        self.ctrl._run_task_loop = Mock(side_effect=ValueError('invalid sample'))
        self.ctrl._cancel_navigation = Mock(return_value=True)
        with self.assertLogs('explore_rbnx.controller', level='ERROR'):
            self.ctrl._run_task(self.handle)
        self.assertEqual(self.handle.state, 'error')
        self.ctrl._cancel_navigation.assert_called_once_with('owned-run')
        self.assertEqual(self.handle.active_nav_run_id, '')

    def test_speed_cap_below_live_navigation_limit_fails_instead_of_guessing(self):
        self.ctrl._mcp_call_sync = self.rpc
        self.handle.max_speed_m_s = .1
        with self.assertRaisesRegex(RuntimeError, 'exceeds requested exploration cap'):
            self.ctrl._navigation_speed(self.handle)

    def test_pending_cancel_blocks_new_tasks_and_can_be_retried(self):
        """A failed stop cannot silently release ownership for another task."""
        self.handle.state = 'error'
        self.handle.active_nav_run_id = 'owned-run'
        self.ctrl._task = self.handle
        with self.assertRaises(RuntimeError):
            self.ctrl.start(area_hint='', timeout_s=300., max_speed_m_s=.3)
        self.ctrl._cancel_navigation = Mock(return_value=True)
        accepted, _ = self.ctrl.cancel('test')
        self.assertTrue(accepted)
        self.assertEqual(self.handle.active_nav_run_id, '')

    def _empty_selection(self, *args, **kwargs):
        """Model known boundaries with no currently valid observation position."""
        kwargs['diagnostics'].update(ranked_goals=0, robot_clearance_safe=True,
            reachable_cells=200, generated_viewpoints=0, feasible_goals=0)
        return None

    def test_sweeps_disabled_waits_for_real_quiet_window_not_five_fast_polls(self):
        """Replay the 2026-11-16 failure without polling one snapshot to death."""
        grid = GridView(np.zeros((40, 40), dtype=np.int8), .1, 0., 0., 40, 40)
        self.ctrl._latest_map = object()
        self.ctrl._mcp_call_sync = self.rpc
        self.ctrl._sweep_360 = Mock()
        with patch('explore_skill.controller.time.monotonic', self.clock.now), \
             patch('explore_skill.controller.time.sleep', self.clock.sleep), \
             patch('explore_skill.frontier.GridView.from_msg', return_value=grid), \
             patch('explore_skill.frontier.total_frontier_count', return_value=30), \
             patch('explore_skill.frontier.pick_target', side_effect=self._empty_selection) as select, \
             self.assertLogs('explore_rbnx.controller', level='INFO') as logs:
            self.ctrl._run_task(self.handle)
        self.assertEqual(self.handle.state, 'error')
        self.assertGreaterEqual(self.clock.now() - 100., 30.)
        self.assertGreater(select.call_count, 5)
        self.ctrl._sweep_360.assert_not_called()
        self.assertIn('viewpoints=0', self.handle.detail)
        self.assertIn('30 frontiers remain', self.handle.detail)
        self.assertFalse(any('spinning to expand FOV' in line for line in logs.output))
        self.assertTrue(any('sweeps disabled' in line for line in logs.output))

    def test_candidate_appearing_after_five_polls_still_resumes_navigation(self):
        """Slow occupancy updates can restore candidates without resetting the map."""
        grid = GridView(np.zeros((40, 40), dtype=np.int8), .1, 0., 0., 40, 40)
        self.ctrl._latest_map = object()
        self.ctrl._mcp_call_sync = self.rpc
        attempts = []
        def select(*args, **kwargs):
            attempts.append(self.clock.now())
            if len(attempts) <= 6:
                return self._empty_selection(*args, **kwargs)
            kwargs['diagnostics'].update(ranked_goals=1, feasible_goals=1)
            return SimpleNamespace(centroid_xy=(2., 0.), size=10, travel_distance_m=2.)
        def navigate(*args, **kwargs):
            self.handle.cancel_requested = True
            return True, 'observation position reached'
        self.ctrl._nav_navigate_blocking = Mock(side_effect=navigate)
        with patch('explore_skill.controller.time.monotonic', self.clock.now), \
             patch('explore_skill.controller.time.sleep', self.clock.sleep), \
             patch('explore_skill.frontier.GridView.from_msg', return_value=grid), \
             patch('explore_skill.frontier.total_frontier_count', return_value=30), \
             patch('explore_skill.frontier.pick_target', side_effect=select):
            self.ctrl._run_task(self.handle)
        self.ctrl._nav_navigate_blocking.assert_called_once()
        self.assertGreaterEqual(len(attempts), 7)
        self.assertEqual(self.handle.state, 'canceled')

    def test_terminal_elapsed_time_does_not_keep_increasing(self):
        """A late status query must not make an already failed run appear active."""
        self.ctrl._task = self.handle
        self.clock.value = 115.
        with patch('explore_skill.controller.time.monotonic', self.clock.now):
            self.ctrl._terminate(self.handle, 'error', 'no safe observable goal')
            self.clock.value = 500.
            status = self.ctrl.status('test')
        self.assertEqual(status['elapsed_s'], 15.)
        self.assertEqual(status['state'], 'error')

    def test_no_target_detail_names_actual_rejection_counts_and_disabled_sweep(self):
        """A no-candidate RUNNING status explains counts and never claims a sweep."""
        detail = self.ctrl._selection_detail({
            'robot_clearance_safe': False,
            'reachable_cells': 0,
            'generated_viewpoints': 0,
            'rejected_distance': 0,
            'rejected_near_robot': 0,
            'rejected_cooldown': 0,
            'rejected_budget': 0,
            'rejected_goal_clearance': 0,
            'rejected_route_reachability': 0,
            'rejected_visibility': 0,
            'rejected_view_distance': 0,
            'rejected_near_robot': 0,
            'viewpoint_cells_checked': 0,
            'feasible_goals': 0,
            'goal_clearance_radius_m': 1.10,
            'goal_clearance_candidates': 0,
        })
        self.assertIn('robot_clearance_safe=False', detail)
        self.assertIn('reachable_cells=0', detail)
        self.assertIn('viewpoints=0', detail)
        self.assertIn('rejected_goal_clearance=0', detail)
        self.assertIn('rejected_route_reachability=0', detail)
        self.assertIn('rejected_visibility=0', detail)
        self.assertIn('rejected_view_distance=0', detail)
        self.assertIn('rejected_budget=0', detail)
        self.assertIn('goal_clearance_radius_m=1.10', detail)
        self.ctrl._task = self.handle
        status = self.ctrl.status('test')
        self.assertIn('frontier_regions', status)
        self.assertIn('safe_observation_candidates', status)
        self.assertIn('rejected_visibility', status)
        self.assertIn('viewpoint_cells_checked', status)
        self.assertEqual(status['current_target_xy'], [])


if __name__ == '__main__':
    unittest.main()
