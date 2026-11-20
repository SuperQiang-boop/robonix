"""Real ROS capture through native BBS/ICP and final observation validation."""
import threading
from types import SimpleNamespace
import unittest

import numpy as np
import _relocalization_bbs as native

import backend_matching as synthetic
import e2e_observations as ros_fixture
from relocalization_rbnx.computation import compute_candidate
from relocalization_rbnx.computation import make_task_runner
from relocalization_rbnx.runtime import Runtime
from relocalization_rbnx.settings import Settings


class ComputationTests(ros_fixture.RosObservationTests):
    def setUp(self):
        """Reuse verified snapshot and ROS fixtures with a calibrated sensor offset."""
        super().setUp()
        self.snapshot = synthetic.MatchingTests()
        self.snapshot.setUp()
        self.addCleanup(self.snapshot.doCleanups)
        self.points = self.snapshot.scan - [0.25, 0, 0]
        self.matcher = self.snapshot.matcher
        self.token = native.CancellationToken()
        self.release = threading.Event()
        # Release a deliberately delayed matcher before joining its worker.
        self.addCleanup(self.release.set)

    def compute(self, timeout=2):
        return self.pool.submit(compute_candidate, self.collector, self.matcher,
            search_options=self.snapshot.search, registration_options=self.snapshot.registration,
            options=self.snapshot.options, timeout_s=timeout, cancelled=self.token)

    def delay_result(self):
        """Pause after native matching so real ROS input can change before recheck."""
        entered = threading.Event()
        matcher = self.matcher

        def delayed(*args):
            """Preserve actual native result; wait for an explicit test-thread release."""
            result = matcher.match(*args)
            entered.set()
            if not self.release.wait(timeout=3):
                raise TimeoutError('fixture matcher release timed out')
            return result

        self.matcher = SimpleNamespace(match=delayed)
        future = self.compute()
        self.spin_until(entered.is_set)
        return future

    def test_ros_to_native_chain_recovers_known_pose(self):
        """Transmit actual ROS messages and recover the prepared snapshot pose."""
        future = self.compute()
        self.spin_until(future.done)
        result = future.result()
        np.testing.assert_allclose([result.x, result.y, result.yaw], self.snapshot.pose, atol=1e-6)
        self.assertLess(result.rmse_m, 1e-5)
        self.assertEqual(result.overlap, 1)
        self.assertGreater(result.observation_stamp_ns, 0)

    def test_motion_during_computation_discards_candidate(self):
        """A successful fit becomes unusable when odometry moves during computation."""
        future = self.delay_result()
        self.motion_x = 0.1
        self.spin_for(0.06)
        self.release.set()
        self.spin_until(future.done)
        with self.assertRaisesRegex(ValueError, 'moved'):
            future.result()

    def test_expiry_during_computation_discards_candidate(self):
        """A geometrically good candidate cannot survive stale input timestamps."""
        future = self.delay_result()
        self.cloud_timer.cancel()
        self.spin_for(0.55)
        self.release.set()
        self.spin_until(future.done)
        with self.assertRaisesRegex(ValueError, 'stale'):
            future.result()

    def test_late_cancellation_discards_candidate(self):
        """Cancellation after native matching cannot leak a result to the caller."""
        future = self.delay_result()
        self.token.cancel()
        self.release.set()
        self.spin_until(future.done)
        with self.assertRaises(native.SearchCancelled):
            future.result()

    def start_runtime(self, identity_reader=None):
        """Schedule the actual ROS/native computation with isolated loaded-map identity."""
        self.runtime = Runtime()
        settings = Settings.parse(dict(map_id='synthetic', mapping_provider_id='fixture',
            lidar_provider_id='lidar_fixture', odom_provider_id='odom_fixture',
            maps_dir='/tmp/relocalization-fixture/maps', index_dir='/tmp/relocalization-fixture/index'))
        runner = make_task_runner(self.collector, self.matcher,
            identity_reader or (lambda remaining, token: dict(map_id='synthetic',
                generation=1, database_sha256='a'*64)),
            search_options=self.snapshot.search, registration_options=self.snapshot.registration,
            options=self.snapshot.options)
        self.runtime.activate(settings, runner)
        self.addCleanup(self.runtime.deactivate)
        return self.runtime.relocalize('synthetic', 1, 'a'*64, 2)

    def test_runtime_schedules_native_chain_without_granting_validity(self):
        """A real accepted computation ends as an unverified geometric candidate."""
        accepted = self.start_runtime()
        self.assertTrue(accepted['accepted'])
        self.spin_until(lambda: not self.runtime.status()['busy'])
        status = self.runtime.status()
        self.assertEqual(status['request_id'], accepted['request_id'])
        self.assertIn('geometric candidate computed', status['detail'])
        self.assertFalse(status['localization_valid'])
        self.assertEqual(status['observation_stamp_ns'], 0)
        stored = self.runtime._candidate_for_handoff(accepted['request_id'])
        self.assertIsNotNone(stored)
        self.assertEqual(stored['identity'], {name: self.matcher.identity[name]
            for name in ('map_id', 'generation', 'database_sha256')})
        self.assertAlmostEqual(stored['result'].x, self.snapshot.pose[0], delta=0.04)
        self.assertTrue(self.runtime.cancel(accepted['request_id'])['ok'])
        self.assertIsNone(self.runtime._candidate_for_handoff(accepted['request_id']))
        self.assertFalse(self.runtime.status()['localization_valid'])

    def test_mapping_invalidation_removes_real_native_candidate(self):
        """An internal lifecycle notification revokes retained real computation."""
        accepted = self.start_runtime()
        self.spin_until(lambda: not self.runtime.status()['busy'])
        self.assertIsNotNone(self.runtime._candidate_for_handoff(accepted['request_id']))
        self.runtime._invalidate('mapping reload started')
        self.assertIsNone(self.runtime._candidate_for_handoff(accepted['request_id']))
        self.assertEqual(self.runtime.status()['detail'], 'mapping reload started')
        self.assertFalse(self.runtime.status()['localization_valid'])

    def test_runtime_cancel_while_waiting_for_scan(self):
        """Request-scoped native cancellation terminates live ROS capture."""
        self.cloud_timer.cancel()
        accepted = self.start_runtime()
        self.assertTrue(self.runtime.cancel(accepted['request_id'])['ok'])
        self.spin_until(lambda: not self.runtime.status()['busy'])
        self.assertIn('cancelled', self.runtime.status()['detail'])
        self.assertFalse(self.runtime.status()['localization_valid'])

    def test_mapping_identity_switch_discards_completed_candidate(self):
        """An actual computed candidate is discarded when mapping changes its snapshot."""
        calls = []
        def read_identity(remaining, token):
            """Model mapping changing its loaded snapshot while computation executes."""
            calls.append(remaining)
            return dict(map_id='synthetic', generation=len(calls), database_sha256='a'*64)
        self.start_runtime(read_identity)
        self.spin_until(lambda: not self.runtime.status()['busy'])
        self.assertEqual(len(calls), 2)
        self.assertLess(calls[1], calls[0])
        self.assertIn('snapshot changed', self.runtime.status()['detail'])
        self.assertFalse(self.runtime.status()['localization_valid'])


def load_tests(loader, tests, pattern):
    """Run only the integrated tests; base ROS tests have their own entry point."""
    return unittest.TestSuite(ComputationTests(name) for name in ComputationTests.__dict__
                             if name.startswith('test_'))


if __name__ == '__main__':
    unittest.main(verbosity=2)
