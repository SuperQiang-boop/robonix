"""Check that the unfinished backend cannot grant localization or accept work."""
import math
import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from relocalization_rbnx.runtime import Runtime
from relocalization_rbnx.settings import Settings


def configuration(**overrides):
    """Supply independent source/index paths without requiring robot resources."""
    values = dict(map_id="scout2_main", mapping_provider_id="mapping",
                  lidar_provider_id="scout2_lidar", odom_provider_id="scout2_chassis",
                  maps_dir="maps", index_dir="indices")
    values.update(overrides)
    return values


class SettingsTests(unittest.TestCase):
    def test_paths_require_deployment_base(self):
        """Resolve relative paths explicitly and keep writable indices separate."""
        settings = Settings.parse(configuration(), deployment_dir=Path("/tmp/deploy"))
        self.assertEqual(settings.maps_dir, Path("/tmp/deploy/maps"))
        self.assertEqual(settings.index_dir, Path("/tmp/deploy/indices"))
        self.assertTrue(settings.auto_relocalize)
        manual = Settings.parse(configuration(auto_relocalize=False), deployment_dir=Path("/tmp/deploy"))
        self.assertFalse(manual.auto_relocalize)
        with self.assertRaises(ValueError):
            Settings.parse(configuration(index_dir="maps/index"), deployment_dir=Path("/tmp/deploy"))

    def test_invalid_configuration(self):
        """Reject misspellings, nonfinite deadlines and ambiguous map identifiers."""
        for overrides in ({"typo": True}, {"timeout_s": math.nan}, {"timeout_s": True},
                          {"status_period_s": 0}, {"use_sim_time": "false"},
                          {"auto_relocalize": "false"}, {"auto_relocalize": 0},
                          {"map_id": "../other"}, {"base_frame": "odom"},
                          {"mapping_provider_id": ""}, {"maps_dir": None}):
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                Settings.parse(configuration(**overrides), deployment_dir=Path("/tmp/deploy"))

    def test_map_identity_can_be_resolved_from_mapping(self):
        """Localization config may leave map selection to mapping's loaded snapshot."""
        values = configuration()
        values.pop('map_id')
        settings = Settings.parse(values, deployment_dir=Path('/tmp/deploy'))
        self.assertIsNone(settings.map_id)


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.runtime = Runtime()
        self.settings = Settings.parse(configuration(), deployment_dir=Path("/tmp/deploy"))
        self.addCleanup(self.runtime.deactivate)

    def start_backend(self, runner):
        """Inject independent cancellation tokens without requiring native modules."""
        def token():
            """Give every request its own mutable cancellation flag."""
            value = SimpleNamespace(cancelled=False)
            value.cancel = lambda: setattr(value, 'cancelled', True)
            return value
        native = SimpleNamespace(CancellationToken=token)
        patcher = patch.dict(sys.modules, {'_relocalization_bbs': native})
        patcher.start()
        self.addCleanup(patcher.stop)
        self.runtime.activate(self.settings, runner)

    def finish_request(self, timeout=30):
        """Await scheduling completion without waiting for retained-candidate expiry."""
        response = self.runtime.relocalize('scout2_main', 1, 'a'*64, timeout)
        self.assertTrue(response['accepted'])
        deadline = time.monotonic()+1
        while self.runtime.status()['busy'] and time.monotonic() < deadline:
            time.sleep(0.001)
        self.assertFalse(self.runtime.status()['busy'])
        return response['request_id']

    def test_candidate_storage_copies_result_and_identity(self):
        """Internal geometry cannot be mutated through backend or reader aliases."""
        result = {'x': [1.0]}
        self.start_backend(lambda *args: result)
        request_id = self.finish_request()
        result['x'][0] = 99
        saved = self.runtime._candidate_for_handoff(request_id)
        self.assertEqual(saved['result'], {'x': [1.0]})
        self.assertEqual(saved['identity'], dict(map_id='scout2_main',
            generation=1, database_sha256='a'*64))
        saved['result']['x'][0] = 88
        saved['identity']['generation'] = 42
        again = self.runtime._candidate_for_handoff(request_id)
        self.assertEqual(again['result'], {'x': [1.0]})
        self.assertEqual(again['identity']['generation'], 1)
        self.assertIsNone(self.runtime._candidate_for_handoff('stale'))
        self.assertFalse(self.runtime.status()['localization_valid'])
        self.assertEqual(self.runtime.status()['map_id'], '')

    def test_cancel_completed_candidate_is_idempotent(self):
        """A completed computation can still be cancelled before handoff."""
        self.start_backend(lambda *args: {'x': 1})
        request_id = self.finish_request()
        self.assertFalse(self.runtime.cancel('stale')['ok'])
        self.assertIsNotNone(self.runtime._candidate_for_handoff(request_id))
        self.assertTrue(self.runtime.cancel(request_id)['ok'])
        self.assertTrue(self.runtime.cancel(request_id)['ok'])
        self.assertIsNone(self.runtime._candidate_for_handoff(request_id))
        self.assertIn('candidate discarded', self.runtime.status()['detail'])

    def test_backend_cleanup_cancellation_preserves_matching_failure(self):
        """Internal token cleanup must not hide the backend's matching error."""
        def fail_after_cleanup(*args):
            """Model a backend that cancels its token while unwinding a failure."""
            args[-1].cancel()
            raise RuntimeError('BBS candidate limit reached')
        self.start_backend(fail_after_cleanup)
        self.finish_request()
        detail = self.runtime.status()['detail']
        self.assertIn('BBS candidate limit reached', detail)
        self.assertNotIn('request cancelled', detail)

    def test_retained_candidate_deadline_is_checked_on_read(self):
        """A delayed timer cannot allow expired geometry to be retrieved."""
        self.start_backend(lambda *args: {'x': 1})
        request_id = self.finish_request()
        deadline = self.runtime._candidate_for_handoff(request_id)['deadline']
        with patch('relocalization_rbnx.runtime.time.monotonic', return_value=deadline):
            self.assertIsNone(self.runtime._candidate_for_handoff(request_id))
        self.assertIn('deadline exceeded', self.runtime.status()['detail'])

    def test_retained_candidate_timer_expires_without_reader(self):
        """Retained geometry is removed even when no handoff consumer reads it."""
        self.start_backend(lambda *args: {'x': 1})
        request_id = self.finish_request(timeout=0.1)
        deadline = time.monotonic()+1
        while self.runtime._candidate is not None and time.monotonic() < deadline:
            time.sleep(0.001)
        self.assertIsNone(self.runtime._candidate_for_handoff(request_id))
        self.assertIsNone(self.runtime._candidate)
        self.assertIn('deadline exceeded', self.runtime.status()['detail'])

    def test_new_request_discards_old_geometry_and_ignores_old_timer(self):
        """A stale expiry callback cannot remove the next request's candidate."""
        self.start_backend(lambda *args: {'x': 1})
        first = self.finish_request()
        old_task = self.runtime._candidate['task']
        second = self.finish_request()
        self.runtime._expire(old_task)
        self.assertIsNone(self.runtime._candidate_for_handoff(first))
        self.assertIsNotNone(self.runtime._candidate_for_handoff(second))
        self.assertEqual(self.runtime.status()['request_id'], second)
        self.assertFalse(self.runtime.cancel(first)['ok'])

    def test_invalidation_discards_candidate_without_restoration(self):
        """Same-map restoration does not revive a revoked geometric result."""
        self.start_backend(lambda *args: {'x': 1})
        request_id = self.finish_request()
        self.runtime._invalidate('mapping reload started')
        self.runtime._invalidate('same map loaded again')
        self.assertIsNone(self.runtime._candidate_for_handoff(request_id))
        self.assertEqual(self.runtime.status()['detail'], 'same map loaded again')
        self.assertFalse(self.runtime.status()['localization_valid'])

    def test_invalidation_cancels_running_work_and_rejects_late_candidate(self):
        """Dependency loss stays visible even if a backend returns afterwards."""
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        def runner(*args):
            """Return a late candidate after an observed map transition."""
            entered.set()
            release.wait(2)
            return {'x': 1}
        self.start_backend(runner)
        response = self.runtime.relocalize('scout2_main', 1, 'a'*64, 30)
        self.assertTrue(entered.wait(1))
        self.runtime._invalidate('mapping reload started')
        self.assertTrue(self.runtime._task['token'].cancelled)
        self.assertTrue(self.runtime.status()['busy'])
        worker = self.runtime._task['thread']
        release.set()
        worker.join(timeout=1)
        self.assertFalse(self.runtime.status()['busy'])
        self.assertEqual(self.runtime.status()['detail'], 'mapping reload started')
        self.assertIsNone(self.runtime._candidate_for_handoff(response['request_id']))

    def test_deactivation_discards_retained_candidate(self):
        """A service activation cannot inherit geometry from its previous lifecycle."""
        self.start_backend(lambda *args: {'x': 1})
        request_id = self.finish_request()
        self.runtime.deactivate()
        self.assertIsNone(self.runtime._candidate_for_handoff(request_id))
        self.assertIn('candidate discarded', self.runtime.status()['detail'])
        self.runtime.activate(self.settings, lambda *args: {'x': 2})
        self.assertIsNone(self.runtime._candidate_for_handoff(request_id))
        self.assertEqual(self.runtime.status()['request_id'], '')

    def test_inactive_request_is_not_processed(self):
        with self.assertRaisesRegex(RuntimeError, "not active"):
            self.runtime.relocalize("scout2_main", 0, "a" * 64, 15)

    def test_active_never_means_localized(self):
        """Even valid requests cannot issue IDs, observations or a verified map."""
        self.runtime.activate(self.settings)
        result = self.runtime.relocalize("scout2_main", 1, "a" * 64, 0)
        self.assertFalse(result["accepted"])
        self.assertEqual(result["request_id"], "")
        self.assertIn("not ready", result["detail"])
        status = self.runtime.status(123)
        self.assertTrue(status["active"])
        self.assertFalse(status["busy"])
        self.assertFalse(status["localization_valid"])
        self.assertEqual(status["map_id"], "")
        self.assertEqual(status["database_sha256"], "")
        self.assertEqual(status["observation_stamp_ns"], 0)
        self.assertEqual(status["published_stamp_ns"], 123)
        self.assertEqual(status["residual_m"], -1)

    def test_deadline_cancels_and_discards_late_result(self):
        """Expiration signals the backend and a late candidate cannot escape."""
        token = SimpleNamespace(cancelled=False)
        token.cancel = lambda: setattr(token, "cancelled", True)
        native = SimpleNamespace(CancellationToken=lambda: token)
        release = threading.Event()
        self.addCleanup(release.set)
        def runner(*args):
            """Hold a candidate beyond the request deadline until test release."""
            release.wait(2)
            return object()
        with patch.dict(sys.modules, {"_relocalization_bbs": native}):
            self.runtime.activate(self.settings, runner)
            accepted = self.runtime.relocalize("scout2_main", 1, "e"*64, 0.05)
            deadline = time.monotonic()+1
            while not token.cancelled and time.monotonic() < deadline:
                time.sleep(0.001)
            self.assertTrue(token.cancelled)
            self.assertTrue(self.runtime.status()["busy"])
            release.set()
            self.runtime.deactivate()
        self.assertIn("deadline exceeded", self.runtime.status()["detail"])
        self.assertEqual(self.runtime.status()["request_id"], accepted["request_id"])

    def test_failed_join_keeps_worker_and_prevents_reactivation(self):
        """An unresponsive worker remains owned for a later cleanup attempt."""
        token = SimpleNamespace(cancelled=False)
        token.cancel = lambda: setattr(token, "cancelled", True)
        native = SimpleNamespace(CancellationToken=lambda: token)
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        def runner(*args):
            """Keep the worker alive until shutdown retry is ready."""
            entered.set()
            release.wait(2)
            return object()
        with patch.dict(sys.modules, {"_relocalization_bbs": native}):
            self.runtime.activate(self.settings, runner)
            self.runtime.relocalize("scout2_main", 1, "e"*64, 1)
            self.assertTrue(entered.wait(1))
            with patch.object(threading.Thread, "join"):
                with self.assertRaisesRegex(RuntimeError, "did not stop"):
                    self.runtime.deactivate()
            self.assertTrue(self.runtime.status()["busy"])
            self.assertFalse(self.runtime.status()["active"])
            with self.assertRaises(RuntimeError):
                self.runtime.activate(self.settings)
            release.set()
            self.runtime.deactivate()
        self.assertFalse(self.runtime.status()["busy"])

    def test_bad_snapshot_and_deadlines_are_rejected(self):
        """A malformed identity cannot silently fall through as a valid task."""
        self.runtime.activate(self.settings)
        for args in (("other", 0, "a" * 64, 15), ("scout2_main", 0, "", 15),
                     ("scout2_main", -1, "a" * 64, 15),
                     ("scout2_main", 0, "a" * 64, math.inf)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                self.runtime.relocalize(*args)

    def test_cancel_and_repeat_lifecycle(self):
        """Cancellation and deactivation never manufacture completed requests."""
        for _ in range(2):
            self.runtime.activate(self.settings)
            self.assertFalse(self.runtime.cancel("unknown")["ok"])
            self.runtime.deactivate()
            self.runtime.deactivate()
            self.assertFalse(self.runtime.status()["active"])
        with self.assertRaises(ValueError):
            self.runtime.cancel("")
        self.assertNotEqual(self.runtime.process_id, Runtime().process_id)

    def test_backend_absence_still_rejects_without_task(self):
        """A task ID is issued only when activation injects a ready backend."""
        self.runtime.activate(self.settings)
        result = self.runtime.relocalize("scout2_main", 1, "a"*64, 1)
        self.assertFalse(result["accepted"])
        self.assertEqual(result["request_id"], "")

    def test_fast_backend_completion_is_not_lost(self):
        """Register work before the worker runs, including immediately finished work."""
        token = SimpleNamespace(cancelled=False, cancel=lambda: None)
        native = SimpleNamespace(CancellationToken=lambda: token)
        results = []
        with patch.dict(sys.modules, {"_relocalization_bbs": native}):
            self.runtime.activate(self.settings,
                lambda *args: results.append(args) or object())
            response = self.runtime.relocalize("scout2_main", 1, "a"*64, 0)
            deadline = time.monotonic()+1
            while self.runtime.status()["busy"] and time.monotonic() < deadline:
                time.sleep(0.001)
        self.assertTrue(response["accepted"])
        self.assertEqual(len(results), 1)
        status = self.runtime.status()
        self.assertFalse(status["busy"])
        self.assertEqual(status["request_id"], response["request_id"])
        self.assertIn("pose handoff", status["detail"])
        self.assertFalse(status["localization_valid"])

    def test_one_active_task_and_request_scoped_cancellation(self):
        """Reject overlap and make cancellation target only its active request."""
        entered, release = threading.Event(), threading.Event()
        token = SimpleNamespace(cancelled=False)
        token.cancel = lambda: setattr(token, "cancelled", True)
        native = SimpleNamespace(CancellationToken=lambda: token)
        def runner(*args):
            """Hold work until cancellation is observed and the test releases it."""
            entered.set()
            release.wait(2)
            return None
        with patch.dict(sys.modules, {"_relocalization_bbs": native}):
            self.runtime.activate(self.settings, runner)
            accepted = self.runtime.relocalize("scout2_main", 1, "b"*64, 2)
            self.assertTrue(entered.wait(1))
            overlap = self.runtime.relocalize("scout2_main", 1, "b"*64, 2)
            self.assertFalse(overlap["accepted"])
            self.assertFalse(self.runtime.cancel("stale")["ok"])
            self.assertTrue(self.runtime.cancel(accepted["request_id"])["ok"])
            self.assertTrue(self.runtime._task["cancel_requested"])
            self.assertTrue(self.runtime.status()["busy"])
            self.assertIn("cancelled", self.runtime.status()["detail"])
            release.set()
            deadline = time.monotonic()+1
            while self.runtime.status()["busy"] and time.monotonic() < deadline:
                time.sleep(0.001)
        self.assertFalse(self.runtime.status()["busy"])
        self.assertFalse(self.runtime.status()["localization_valid"])
        self.assertTrue(self.runtime.cancel(accepted["request_id"])["ok"])

    def test_thread_start_failure_does_not_leave_busy_task(self):
        """A scheduling failure issues no accepted task and cancels its timer."""
        native = SimpleNamespace(CancellationToken=lambda: SimpleNamespace(cancelled=False))
        self.runtime.activate(self.settings, lambda *args: None)
        with patch.dict(sys.modules, {"_relocalization_bbs": native}), \
                patch('relocalization_rbnx.runtime.threading.Thread.start',
                      side_effect=RuntimeError('cannot start thread')):
            with self.assertRaisesRegex(RuntimeError, 'cannot start thread'):
                self.runtime.relocalize('scout2_main', 1, 'a'*64, 2)
        self.assertFalse(self.runtime.status()['busy'])
        self.assertEqual(self.runtime.status()['request_id'], '')

    def test_deactivation_cancels_and_joins_worker(self):
        """Lifecycle deactivation waits for a cooperative backend to release work."""
        entered, exited = threading.Event(), threading.Event()
        token = SimpleNamespace(cancelled=False)
        token.cancel = lambda: setattr(token, "cancelled", True)
        native = SimpleNamespace(CancellationToken=lambda: token)
        def runner(*args):
            """Wait cooperatively until lifecycle cancellation reaches the backend."""
            entered.set()
            while not token.cancelled:
                time.sleep(0.001)
            exited.set()
            return None
        with patch.dict(sys.modules, {"_relocalization_bbs": native}):
            self.runtime.activate(self.settings, runner)
            self.runtime.relocalize("scout2_main", 1, "c"*64, 2)
            self.assertTrue(entered.wait(1))
            self.runtime.deactivate()
        self.assertTrue(exited.is_set())
        self.assertFalse(self.runtime.status()["active"])
        self.assertFalse(self.runtime.status()["busy"])

    def test_backend_fault_is_recorded_without_validity(self):
        """Unexpected failures remain visible and never publish a valid pose."""
        token = SimpleNamespace(cancelled=False)
        native = SimpleNamespace(CancellationToken=lambda: token)
        def fail(*args):
            """Represent an unexpected internal backend fault."""
            raise RuntimeError("fault")
        with patch.dict(sys.modules, {"_relocalization_bbs": native}):
            self.runtime.activate(self.settings, fail)
            accepted = self.runtime.relocalize("scout2_main", 1, "d"*64, 2)
            deadline = time.monotonic()+1
            while self.runtime.status()["busy"] and time.monotonic() < deadline:
                time.sleep(0.001)
        status = self.runtime.status()
        self.assertIn("RuntimeError: fault", status["detail"])
        self.assertEqual(status["request_id"], accepted["request_id"])
        self.assertFalse(status["localization_valid"])
