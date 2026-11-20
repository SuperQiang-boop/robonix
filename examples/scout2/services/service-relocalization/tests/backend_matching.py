"""Offline BBS/ICP/held-out-frame integration with synthetic snapshot data."""
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np
import _relocalization_bbs as native

from relocalization_rbnx.map_index import ARTIFACTS, build_index, sha256
from relocalization_rbnx.matching import (MatchRejected, MatchingOptions, Observation,
                                        OfflineMatcher)


class MatchingTests(unittest.TestCase):
    def setUp(self):
        """Prepare an asymmetric synthetic scan, saved grid and matching 3D cloud."""
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        rng = np.random.default_rng(913)
        angles = np.linspace(-math.pi, math.pi, 61, endpoint=False)
        radii = rng.uniform(2.5, 4.5, len(angles))
        self.scan = np.column_stack([np.round(radii * np.cos(angles), 1),
                                     np.round(radii * np.sin(angles), 1),
                                     rng.uniform(0.5, 1.2, len(angles))])
        self.pose = np.array([0.05, 0.05, 0.0])
        self.fit = Observation(self.scan, (0, 0, 0), 1_000_000_000, 1_100_000_000)
        self.check = Observation(self.scan + rng.normal(0, 0.001, self.scan.shape),
                                 (0, 0, 0), 1_200_000_000, 1_300_000_000)
        self.search = native.SearchOptions()
        self.search.yaw_step = math.pi / 2
        self.search.min_score = 0.98
        self.search.max_candidates = 10
        self.registration = native.RegistrationOptions()
        self.registration.max_rmse_m = 0.04
        self.options = MatchingOptions(max_blocked_ray_fraction=0.2)
        self.matcher = self.make_matcher()

    def make_matcher(self, repeated=False):
        """Publish one snapshot; an optional second identical area tests ambiguity."""
        directory = self.root / f'index-{len(list(self.root.iterdir()))}'
        directory.mkdir()
        cloud = self.scan + [*self.pose[:2], 0]
        if repeated:
            cloud = np.concatenate([cloud, cloud + [12, 0, 0]])
        grid = np.zeros((160, 320), dtype='i1')
        cells = np.floor((cloud[:, :2] + 8) / 0.1).astype(int)
        grid[cells[:, 1], cells[:, 0]] = 100
        grid.tofile(directory / 'grid.bin')
        cloud.astype('<f4').tofile(directory / 'cloud.bin')
        cloud.astype('<f4').tofile(directory / 'scans.bin')
        transform = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]
        report = dict(format_version=1, database_version='0.23.7', point_count=len(cloud),
            optimized_components=[[1]], graph_components=[[1]], excluded_ids=[],
            grid=dict(width=320, height=160, origin_x=-8, origin_y=-8, resolution=0.1),
            nodes=[dict(id=1, offset=0, count=len(cloud), map_from_base=transform,
                        base_from_sensor=transform)])
        (directory / 'export.json').write_text(json.dumps(report))
        (directory / 'coverage.json').write_text('{}')
        identity = dict(map_id='synthetic', generation=1, database_sha256='a' * 64,
                        voxel_size=0.05, pyramid_levels=8)
        manifest = dict(format_version=1, identity=identity,
            summary=build_index(directory, 0.05, 8),
            files={name: sha256(directory / name) for name in ARTIFACTS})
        (directory / 'index.json').write_text(json.dumps(manifest))
        return OfflineMatcher(directory, identity)

    def run_match(self, **kwargs):
        """Run the native offline chain with defaults and targeted overrides."""
        args = dict(fitting=self.fit, validation=self.check, search_options=self.search,
                    registration_options=self.registration, options=self.options,
                    timeout_s=3, cancelled=native.CancellationToken())
        args.update(kwargs)
        return self.matcher.match(**args)

    def test_native_chain_recovers_unique_pose(self):
        """Search, fit and evaluate a separate noisy frame at a known pose."""
        result = self.run_match()
        np.testing.assert_allclose([result.x, result.y, result.yaw], self.pose, atol=1e-6)
        self.assertLess(result.rmse_m, 0.005)
        self.assertEqual(result.observation_stamp_ns, self.check.end_stamp_ns)
        self.assertEqual(result.overlap, 1)

    def test_repeated_areas_are_ambiguous(self):
        self.matcher = self.make_matcher(repeated=True)
        with self.assertRaisesRegex(MatchRejected, 'ambiguous'):
            self.run_match()

    def test_same_projection_with_wrong_height_is_filtered(self):
        """A second BBS candidate cannot pass on planar resemblance alone."""
        self.matcher = self.make_matcher(repeated=True)
        wrong = self.scan + [12.05, 0.05, 2]
        correct = self.scan + [0.05, 0.05, 0]
        self.matcher._registration = native.PlanarRegistration(np.concatenate([wrong, correct]))
        result = self.run_match()
        np.testing.assert_allclose([result.x, result.y, result.yaw], self.pose, atol=1e-6)
        self.assertTrue(any('overlap' in reason for reason in result.rejected))

    def test_tilt_and_sensor_origin_share_projection_convention(self):
        """Undo fixed tilt in input while keeping the same map geometry."""
        from backend_registration import transform
        self.registration.roll, self.registration.pitch = 0.1, -0.08
        tilted_axes = transform(np.eye(3), [0, 0, 0], self.registration)
        raw = self.scan @ tilted_axes.T
        validation = self.check.points @ tilted_axes.T
        fit = Observation(raw, (0.12, 0.03, 0.2), self.fit.start_stamp_ns, self.fit.end_stamp_ns)
        check = Observation(validation, fit.sensor_origin, self.check.start_stamp_ns, self.check.end_stamp_ns)
        result = self.run_match(fitting=fit, validation=check)
        np.testing.assert_allclose([result.x, result.y, result.yaw], self.pose, atol=1e-6)

    def test_later_frame_cannot_be_fitted_to_hide_disagreement(self):
        check = Observation(self.scan + [0, 0, 0.25], (0, 0, 0),
                            self.check.start_stamp_ns, self.check.end_stamp_ns)
        with self.assertRaisesRegex(MatchRejected, 'no candidate.*residual'):
            self.run_match(validation=check)

    def test_capture_overlap_and_large_gap_are_rejected(self):
        """A frame in or before the fitting window cannot count as held-out data."""
        for start, end in ((self.fit.end_stamp_ns, self.check.end_stamp_ns),
                           (3_100_000_000, 3_200_000_000)):
            with self.assertRaisesRegex(ValueError, 'capture window'):
                self.run_match(validation=Observation(self.scan, (0, 0, 0), start, end))

    def test_invalid_observation_and_options_are_rejected(self):
        """Reject nonfinite scans and settings before any usable result exists."""
        with self.assertRaises(ValueError):
            self.run_match(fitting=Observation(np.full((30, 3), np.nan), (0, 0, 0), 1, 2))
        for options in (MatchingOptions(max_blocked_ray_fraction=1),
                        MatchingOptions(min_known_fraction=0),
                        MatchingOptions(ray_margin_m=float('nan'))):
            with self.assertRaises(ValueError):
                self.run_match(options=options)
        self.search.max_candidates = 1
        with self.assertRaisesRegex(ValueError, 'two BBS'):
            self.run_match()

    def test_cancel_and_deadline_discard_all_results(self):
        token = native.CancellationToken()
        token.cancel()
        with self.assertRaises(native.SearchCancelled):
            self.run_match(cancelled=token)
        with self.assertRaises(native.SearchTimeout):
            self.run_match(timeout_s=1e-12)

    def test_whole_operation_budget_is_shared(self):
        """Elapsed time during search cannot be renewed for later stages."""
        def slow_search(*args):
            """Return a candidate only after consuming the entire mock clock budget."""
            clock[0] += 4
            return SimpleNamespace(candidates=[SimpleNamespace(x=0.05, y=0.05, yaw=0)])
        clock = [10.]
        with patch('relocalization_rbnx.matching.time.monotonic', side_effect=lambda: clock[0]):
            self.matcher._search = SimpleNamespace(search=slow_search)
            with self.assertRaises(native.SearchTimeout):
                self.run_match(timeout_s=3)

    def test_unexpected_backend_fault_propagates(self):
        """An internal error must not become a benign rejected candidate."""
        with patch.object(self.matcher, '_registration') as registration:
            registration.refine.side_effect = RuntimeError('PCL query failed')
            with self.assertRaisesRegex(RuntimeError, '^PCL query failed$'):
                self.run_match()

    def test_cancellation_during_candidate_discards_result(self):
        """A late cancellation is terminal even if another candidate might pass."""
        token = native.CancellationToken()

        def cancel(*args):
            """Simulate cancellation while native fitting is active."""
            token.cancel()
            raise native.RegistrationCancelled('cancelled during fitting')

        with patch.object(self.matcher, '_registration') as registration:
            registration.refine.side_effect = cancel
            with self.assertRaises(native.RegistrationCancelled):
                self.run_match(cancelled=token)
            registration.evaluate.assert_not_called()

    def test_no_coarse_candidates_is_rejected(self):
        self.matcher._search = SimpleNamespace(search=lambda *args: SimpleNamespace(candidates=[]))
        with self.assertRaisesRegex(MatchRejected, 'no candidate'):
            self.run_match()

    def test_candidate_budget_saturation_rejects(self):
        self.matcher = self.make_matcher(repeated=True)
        self.search.max_candidates = 2
        with self.assertRaisesRegex(MatchRejected, 'limit reached'):
            self.run_match()

    def test_unknown_outside_and_free_endpoints_are_not_evidence(self):
        """Unknown/outside endpoints cannot inflate known coverage; free is conflict."""
        for fill in (-1, 0):
            grid = np.full(self.matcher._grid.shape, fill, dtype='i1')
            grid[80, 80] = 0
            self.matcher._grid = grid
            with self.assertRaisesRegex(MatchRejected, 'coverage'):
                self.run_match()
        self.matcher._grid = np.zeros_like(self.matcher._grid)
        with self.assertRaisesRegex(MatchRejected, 'coverage'):
            self.matcher._check_grid(self.scan + [100, 0, 0], np.zeros(3), self.pose,
                                     self.options, float('inf'), native.CancellationToken())

    def test_occupied_base_is_rejected(self):
        grid = self.matcher._grid.copy()
        grid[80, 80] = 100
        self.matcher._grid = grid
        with self.assertRaisesRegex(MatchRejected, 'base cell'):
            self.run_match()

    def test_projected_grid_rays_are_diagnostic_for_three_dimensional_scans(self):
        """A 2D projection cannot reject a valid 3D point-cloud match by rays."""
        grid = self.matcher._grid.copy()
        grid[70, 70:91] = grid[90, 70:91] = 100
        grid[70:91, 70] = grid[70:91, 90] = 100
        self.matcher._grid = grid
        evidence = self.matcher._grid_evidence(
            self.scan, np.zeros(3), self.pose, self.options, float('inf'),
            native.CancellationToken())
        self.assertGreater(evidence['blocked_ray_fraction'],
                           self.options.max_blocked_ray_fraction)
        result = self.run_match()
        np.testing.assert_allclose([result.x, result.y, result.yaw], self.pose, atol=1e-6)

    def test_free_endpoint_fraction_does_not_override_valid_3d_registration(self):
        """A 2D height collapse cannot reject matching 3D surfaces."""
        grid = self.matcher._grid.copy()
        cells = np.floor((self.scan[:, :2] + self.pose[:2] + 8) / 0.1).astype(int)
        unique = np.unique(cells, axis=0)
        free_count = math.ceil(len(unique) * 0.3)
        grid[unique[:free_count, 1], unique[:free_count, 0]] = 0
        self.matcher._grid = grid
        evidence = self.matcher._grid_evidence(
            self.scan, np.zeros(3), self.pose, self.options, float('inf'),
            native.CancellationToken())
        self.assertGreater(evidence['free_fraction'], 0.25)
        self.assertGreaterEqual(evidence['occupied_fraction'], self.options.min_occupied_fraction)
        result = self.run_match()
        np.testing.assert_allclose([result.x, result.y, result.yaw], self.pose, atol=1e-6)


class ComputationBudgetTests(unittest.TestCase):
    def setUp(self):
        """Isolate orchestration timing while native algorithms run in other tests."""
        from relocalization_rbnx.computation import compute_candidate
        self.compute = compute_candidate
        self.collector, self.matcher = Mock(), Mock()
        self.observations = (object(), object())
        self.collector.collect.return_value = self.observations
        self.args = dict(search_options=native.SearchOptions(),
            registration_options=native.RegistrationOptions(), options=MatchingOptions(),
            timeout_s=3, cancelled=native.CancellationToken())

    def test_all_stages_receive_only_remaining_time(self):
        """Capture, matching and final recheck share one decreasing budget and token."""
        with patch('relocalization_rbnx.computation.time.monotonic',
                   side_effect=[10, 10, 10.2, 11, 12, 12.5]):
            result = self.compute(self.collector, self.matcher, **self.args)
        self.assertIs(result, self.matcher.match.return_value)
        self.assertAlmostEqual(self.collector.collect.call_args.args[0], 2.8)
        self.assertEqual(self.matcher.match.call_args.args[-2], 2)
        self.assertEqual(self.collector.validate_after_matching.call_args.args[1], 1)
        self.assertIs(self.matcher.match.call_args.args[-1], self.args['cancelled'])
        self.assertIsNot(self.matcher.match.call_args.args[2], self.args['search_options'])
        self.assertIsNot(self.matcher.match.call_args.args[3], self.args['registration_options'])

    def test_expired_capture_cannot_start_matching(self):
        """A capture consuming the full deadline cannot renew the matching budget."""
        with patch('relocalization_rbnx.computation.time.monotonic', side_effect=[10, 10, 10, 14]):
            with self.assertRaises(native.SearchTimeout):
                self.compute(self.collector, self.matcher, **self.args)
        self.matcher.match.assert_not_called()
        self.collector.validate_after_matching.assert_not_called()

    def test_internal_and_validation_errors_propagate(self):
        """Backend faults and live validation failure never become candidates."""
        self.matcher.match.side_effect = RuntimeError('native backend fault')
        with self.assertRaisesRegex(RuntimeError, '^native backend fault$'):
            self.compute(self.collector, self.matcher, **self.args)
        self.collector.validate_after_matching.assert_not_called()
        self.matcher.match.side_effect = None
        self.collector.validate_after_matching.side_effect = ValueError('robot moved')
        with self.assertRaisesRegex(ValueError, '^robot moved$'):
            self.compute(self.collector, self.matcher, **self.args)

    def test_cancel_after_validation_discards_candidate(self):
        """Even the last validation callback cannot race cancellation into a result."""
        self.collector.validate_after_matching.side_effect = lambda *args: self.args['cancelled'].cancel()
        with self.assertRaises(native.SearchCancelled):
            self.compute(self.collector, self.matcher, **self.args)

    def test_loaded_identity_mismatch_prevents_capture(self):
        """Request and live mapping identity must both match before any ROS work."""
        from relocalization_rbnx.computation import make_task_runner
        expected = dict(map_id='synthetic', generation=1, database_sha256='a'*64)
        self.matcher.identity = expected
        reader = Mock(return_value=dict(expected, generation=2))
        runner = make_task_runner(self.collector, self.matcher, reader,
            search_options=self.args['search_options'], registration_options=self.args['registration_options'],
            options=self.args['options'])
        with self.assertRaisesRegex(ValueError, 'requested map identity'):
            runner('synthetic', 2, 'a'*64, 3, self.args['cancelled'])
        reader.assert_not_called()
        with self.assertRaisesRegex(ValueError, 'actual loaded mapping snapshot'):
            runner('synthetic', 1, 'a'*64, 3, self.args['cancelled'])
        self.collector.collect.assert_not_called()


if __name__ == '__main__':
    unittest.main(verbosity=2)
