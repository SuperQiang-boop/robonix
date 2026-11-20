"""Regression tests for automatic map-load supervision."""
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from relocalization_rbnx import ros_status


class FakeRuntime:
    """Model two failed/completed task cycles without ROS or Atlas."""

    def __init__(self, succeeds_on=2):
        self.calls = 0
        self.succeeds_on = succeeds_on
        self.busy = False
        self.valid = False
        self.request_id = ""
        self.finished_at = 0.0
        self.detail = "test"

    def status(self):
        """Publish a failed first task and a valid second task."""
        if self.busy and time.monotonic() >= self.finished_at:
            self.busy = False
            self.valid = self.calls >= self.succeeds_on
        return dict(active=True, busy=self.busy, localization_valid=self.valid,
                    request_id=self.request_id, detail=self.detail)

    def relocalize(self, map_id, generation, digest, timeout_s):
        """Accept an automatic request and complete it shortly afterwards."""
        self.calls += 1
        self.request_id = f"request-{self.calls}"
        self.busy = True
        self.finished_at = time.monotonic() + 0.01
        return dict(accepted=True, request_id=self.request_id, detail="accepted")

    def _invalidate(self, detail):
        """Keep the fake active while exposing an invalidated result."""
        self.valid = False

    def set_detail(self, detail):
        """Accept readiness detail updates made by the supervisor."""
        self.detail = detail


class StatusSupervisorTests(unittest.TestCase):
    def test_mapping_mode_waits_without_preparing_index_or_requesting(self):
        """Mapping heartbeat is an expected idle state for the combined manifest."""
        runtime = FakeRuntime()
        publisher = ros_status.StatusPublisher(runtime,
            SimpleNamespace(coordination_dir=Path('/tmp/coord'),
                             mapping_provider_id='mapping', timeout_s=1,
                             auto_relocalize=True), 'relocalization')
        prepared = []
        publisher.backend = SimpleNamespace(prepare_map_index=lambda identity: prepared.append(identity))
        mapping = dict(process_id='process', load_id='load', ready=True,
                       mode='mapping', provider_id='mapping', map_id='scout2_main')
        published = []
        supervisor = threading.Thread(target=publisher.supervise, daemon=True)
        with patch.object(ros_status, 'MapReader', return_value=SimpleNamespace(state=lambda: mapping)), \
                patch.object(ros_status, 'write_record', side_effect=lambda path, value: published.append(value)):
            supervisor.start()
            deadline = time.monotonic() + 1
            while len(published) < 3 and time.monotonic() < deadline:
                time.sleep(0.005)
            publisher._stop.set()
            supervisor.join(1)
        self.assertFalse(supervisor.is_alive())
        self.assertEqual(prepared, [])
        self.assertEqual(runtime.calls, 0)
        self.assertGreaterEqual(len(published), 3)
        self.assertIn('waiting for a localization map', published[-1]['detail'])

    def test_failed_automatic_request_is_retried_after_stable_window(self):
        """Retry one failed load without creating an unbounded request loop."""
        runtime = FakeRuntime()
        publisher = ros_status.StatusPublisher(runtime,
            SimpleNamespace(coordination_dir=Path('/tmp/coord'),
                             mapping_provider_id='mapping', timeout_s=1,
                             auto_relocalize=True), 'relocalization')
        publisher.backend = SimpleNamespace(prepare_map_index=lambda _identity: None)
        mapping = dict(process_id='process', load_id='load', ready=True,
                       mode='localization', provider_id='mapping', map_id='scout2_main',
                       generation=1, database_sha256='a' * 64)
        stop = threading.Thread(target=publisher.supervise, daemon=True)
        with patch.object(ros_status, 'MapReader', return_value=SimpleNamespace(state=lambda: mapping)), \
                patch.object(ros_status, 'write_record'), \
                patch.object(ros_status, '_AUTO_READY_STABLE_S', 0.02), \
                patch.object(ros_status, '_AUTO_RETRY_DELAY_S', 0.02), \
                patch.object(ros_status, '_AUTO_RETRY_LIMIT', 3):
            stop.start()
            deadline = time.monotonic() + 1
            while runtime.calls < 2 and time.monotonic() < deadline:
                time.sleep(0.005)
            self.assertEqual(runtime.calls, 2)
            deadline = time.monotonic() + 1
            while not runtime.valid and time.monotonic() < deadline:
                time.sleep(0.005)
            publisher._stop.set()
            stop.join(1)
        self.assertTrue(runtime.valid)
        self.assertFalse(stop.is_alive())

    def test_automatic_retries_stop_at_the_configured_limit(self):
        """A load that never matches cannot trigger an endless task loop."""
        runtime = FakeRuntime(succeeds_on=99)
        publisher = ros_status.StatusPublisher(runtime,
            SimpleNamespace(coordination_dir=Path('/tmp/coord'),
                             mapping_provider_id='mapping', timeout_s=1,
                             auto_relocalize=True), 'relocalization')
        publisher.backend = SimpleNamespace(prepare_map_index=lambda _identity: None)
        mapping = dict(process_id='process', load_id='load', ready=True,
                       mode='localization', provider_id='mapping', map_id='scout2_main',
                       generation=1, database_sha256='a' * 64)
        supervisor = threading.Thread(target=publisher.supervise, daemon=True)
        with patch.object(ros_status, 'MapReader', return_value=SimpleNamespace(state=lambda: mapping)), \
                patch.object(ros_status, 'write_record'), \
                patch.object(ros_status, '_AUTO_READY_STABLE_S', 0.01), \
                patch.object(ros_status, '_AUTO_RETRY_DELAY_S', 0.01), \
                patch.object(ros_status, '_AUTO_RETRY_LIMIT', 3):
            supervisor.start()
            deadline = time.monotonic() + 1
            while runtime.calls < 3 and time.monotonic() < deadline:
                time.sleep(0.005)
            time.sleep(0.05)
            publisher._stop.set()
            supervisor.join(1)
        self.assertEqual(runtime.calls, 3)
        self.assertFalse(runtime.valid)
        self.assertFalse(supervisor.is_alive())

    def test_manual_mode_keeps_publishing_status_without_auto_request(self):
        """Disabling automatic localization preserves map and status supervision."""
        runtime = FakeRuntime()
        publisher = ros_status.StatusPublisher(runtime,
            SimpleNamespace(coordination_dir=Path('/tmp/coord'),
                             mapping_provider_id='mapping', timeout_s=1,
                             auto_relocalize=False), 'relocalization')
        publisher.backend = SimpleNamespace(prepare_map_index=lambda _identity: None)
        mapping = dict(process_id='process', load_id='load', ready=True,
                       mode='localization', provider_id='mapping', map_id='scout2_main',
                       generation=1, database_sha256='a' * 64)
        published = []
        supervisor = threading.Thread(target=publisher.supervise, daemon=True)
        with patch.object(ros_status, 'MapReader', return_value=SimpleNamespace(state=lambda: mapping)), \
                patch.object(ros_status, 'write_record', side_effect=lambda path, value: published.append(value)), \
                patch.object(ros_status, '_AUTO_READY_STABLE_S', 0.01):
            supervisor.start()
            deadline = time.monotonic() + 1
            while len(published) < 3 and time.monotonic() < deadline:
                time.sleep(0.005)
            publisher._stop.set()
            supervisor.join(1)
        self.assertFalse(supervisor.is_alive())
        self.assertEqual(runtime.calls, 0)
        self.assertGreaterEqual(len(published), 3)
        self.assertTrue(published[-1]['active'])
        self.assertFalse(published[-1]['localization_valid'])
        self.assertEqual(published[-1]['mapping_process_id'], 'process')

    def test_index_preparation_retries_after_transient_failures_in_manual_mode(self):
        """Keep manual localization available when index preparation recovers."""
        runtime = FakeRuntime()
        publisher = ros_status.StatusPublisher(runtime,
            SimpleNamespace(coordination_dir=Path('/tmp/coord'),
                             mapping_provider_id='mapping', timeout_s=1,
                             auto_relocalize=False), 'relocalization')
        attempts = []

        def prepare(_identity):
            attempts.append(None)
            if len(attempts) < 4:
                raise OSError('temporary index export failure')

        publisher.backend = SimpleNamespace(prepare_map_index=prepare)
        mapping = dict(process_id='process', load_id='load', ready=True,
                       mode='localization', provider_id='mapping', map_id='scout2_main',
                       generation=1, database_sha256='a' * 64)
        supervisor = threading.Thread(target=publisher.supervise, daemon=True)
        with patch.object(ros_status, 'MapReader', return_value=SimpleNamespace(state=lambda: mapping)), \
                patch.object(ros_status, 'write_record'), \
                patch.object(ros_status, '_AUTO_RETRY_DELAY_S', 0.01), \
                patch.object(ros_status, '_INDEX_RETRY_MAX_DELAY_S', 0.02):
            supervisor.start()
            deadline = time.monotonic() + 1
            while len(attempts) < 4 and time.monotonic() < deadline:
                time.sleep(0.005)
            time.sleep(0.04)
            publisher._stop.set()
            supervisor.join(1)
        self.assertEqual(len(attempts), 4)
        self.assertEqual(runtime.calls, 0)
        self.assertFalse(supervisor.is_alive())

    def test_index_backoff_starts_after_failure_and_preserves_request_budget(self):
        """Cap retries at 30 seconds and retain all three automatic requests."""
        for automatic in (False, True):
            with self.subTest(auto_relocalize=automatic):
                clock = SimpleNamespace(now=0.0)
                runtime = FakeRuntime(succeeds_on=99)
                publisher = ros_status.StatusPublisher(runtime,
                    SimpleNamespace(coordination_dir=Path('/tmp/coord'),
                                     mapping_provider_id='mapping', timeout_s=1,
                                     auto_relocalize=automatic), 'relocalization')
                started, failed_at, published = [], [], []

                def advance(delay):
                    clock.now += delay

                def prepare(_identity):
                    """Spend two seconds preparing before each transient failure."""
                    started.append(clock.now)
                    advance(2.0)
                    if len(started) <= 8:
                        failed_at.append(clock.now)
                        raise OSError('temporary index export failure')

                publisher._stop = SimpleNamespace(is_set=lambda: clock.now >= 200,
                                                  wait=advance)
                publisher.backend = SimpleNamespace(prepare_map_index=prepare)
                mapping = dict(process_id='process', load_id='load', ready=True,
                               mode='localization', provider_id='mapping', map_id='scout2_main',
                               generation=1, database_sha256='a' * 64)
                with patch.object(ros_status, 'MapReader',
                                  return_value=SimpleNamespace(state=lambda: mapping)), \
                        patch.object(ros_status, 'write_record',
                                     side_effect=lambda path, value: published.append(dict(value))), \
                        patch.object(ros_status.time, 'monotonic', side_effect=lambda: clock.now), \
                        self.assertLogs(level='ERROR') as logs:
                    publisher.supervise()

                self.assertEqual(len(started), 9)
                for i, delay in enumerate((1, 2, 4, 8, 16, 30, 30, 30)):
                    actual_delay = started[i + 1] - failed_at[i]
                    self.assertGreaterEqual(actual_delay, delay - 1e-9)
                    self.assertLessEqual(actual_delay, delay + 0.100001)
                self.assertEqual(runtime.calls, 3 if automatic else 0)
                self.assertEqual(len(logs.records), 8)
                self.assertTrue(all(record.exc_info for record in logs.records))
                self.assertTrue(any('map index preparation failed' in value['detail']
                                    for value in published))
                self.assertEqual(published[-1]['detail'], '')


if __name__ == '__main__':
    unittest.main()
