"""Verify resource rollback and real generated protobuf responses without a robot."""
import unittest
from dataclasses import replace
from types import SimpleNamespace
from pathlib import Path
from unittest.mock import Mock, patch

import grpc
from robonix_api import Err, Ok
from relocalization_rbnx import main
from relocalization_rbnx.backend import Backend
from relocalization_rbnx.runtime import Runtime
from relocalization_rbnx.settings import Settings
from test_runtime import configuration


class Abort(Exception):
    def __init__(self, code, detail):
        super().__init__(detail)
        self.code = code


class Context:
    def abort(self, code, detail):
        raise Abort(code, detail)


class LifecycleTests(unittest.TestCase):
    def setUp(self):
        main._settings = Settings.parse(configuration(), deployment_dir=Path("/tmp/deploy"))
        main._runtime = Runtime()
        main._publisher = None
        main._status_topic = None
        main._channels.clear()

    def tearDown(self):
        main.deactivate()
        main._settings = None
        main._status_topic = None

    def test_partial_dependency_failure_closes_channels(self):
        """Do not leak earlier connections when another dependency is missing."""
        channel = Mock()
        with patch.object(main.ATLAS, "find_unique_capability", side_effect=[Mock(), RuntimeError("no odom")]), \
                patch.object(main.service, "connect_capability", return_value=channel):
            self.assertIsInstance(main.activate(), Err)
        channel.close.assert_called_once()
        self.assertEqual(main._channels, [])
        self.assertFalse(main._runtime.status()["active"])

    def test_status_start_failure_and_repeat_activation(self):
        """Failed ROS startup rolls back; successful retries can be shut down twice."""
        main._settings = replace(main._settings, profile_file=Path('/tmp/profile.json'),
                                 coordination_dir=Path('/tmp/coordination'))
        with patch.object(main.ATLAS, "find_unique_capability"), \
                patch.object(main.service, "connect_capability", side_effect=lambda cap, cid, transport:
                    Mock(contract_id=cid, endpoint='/fixture')), \
                patch.object(main, 'Backend'), \
                patch.object(main.service, "declare_ros2_topic", return_value="/atlas/status") as declare, \
                patch.object(main, "StatusPublisher") as factory:
            publisher = factory.return_value
            publisher.start.side_effect = RuntimeError("ROS startup failed")
            self.assertIsInstance(main.activate(), Err)
            publisher.close.assert_called_once()
            self.assertEqual(main._channels, [])
            publisher.start.side_effect = None
            for _ in range(2):
                self.assertIsInstance(main.activate(), Ok)
                self.assertTrue(main._runtime.status()["active"])
                self.assertIsInstance(main.deactivate(), Ok)
                self.assertIsInstance(main.deactivate(), Ok)
            self.assertFalse(main._runtime.status()["localization_valid"])
            declare.assert_called_once()
            self.assertEqual(publisher.topic, "/atlas/status")

    def test_failed_cleanup_is_retained_for_retry(self):
        """A resource whose close fails remains reachable and availability is false."""
        publisher = main._publisher = Mock()
        publisher.close.side_effect = RuntimeError("cannot join")
        self.assertIsInstance(main.deactivate(), Err)
        self.assertIs(main._publisher, publisher)
        self.assertFalse(main._runtime.status()["active"])
        publisher.close.side_effect = None
        self.assertIsInstance(main.deactivate(), Ok)
        self.assertIsNone(main._publisher)

    def test_generated_grpc_responses_do_not_claim_success(self):
        """Generated wire types preserve rejection and zero observation identity."""
        request = main.pb.Relocalize_Request(map_id="scout2_main", generation=1,
                                             database_sha256="a" * 64, timeout_s=15)
        with self.assertRaises(Abort) as caught:
            main.relocalize(request, Context())
        self.assertEqual(caught.exception.code, grpc.StatusCode.FAILED_PRECONDITION)
        main._runtime.activate(main._settings)
        response = main.relocalize(request, Context())
        restored = main.pb.Relocalize_Response.FromString(response.SerializeToString())
        self.assertFalse(restored.accepted)
        self.assertEqual(restored.request_id, "")
        status = main.get_status(main.pb.GetStatus_Request(), Context()).status
        self.assertTrue(status.active)
        self.assertFalse(status.localization_valid)
        self.assertEqual(status.map_id, "")

    def test_relocalize_by_map_resolves_loaded_snapshot(self):
        """The convenience RPC binds only to the currently loaded map identity."""
        backend = Mock()
        backend.current_map_identity.return_value = dict(
            map_id="scout2_main", generation=7, database_sha256="b" * 64)
        main._backend = backend
        main._runtime = Mock()
        main._runtime.relocalize.return_value = dict(
            accepted=True, request_id="request-1", detail="accepted")
        request = main.pb.RelocalizeByMap_Request(map_id="scout2_main")
        response = main.relocalize_by_map(request, Context())
        self.assertTrue(response.accepted)
        self.assertEqual(response.request_id, "request-1")
        backend.current_map_identity.assert_called_once_with("scout2_main")
        main._runtime.relocalize.assert_called_once_with(
            map_id="scout2_main", generation=7, database_sha256="b" * 64, timeout_s=0)

    def test_current_map_identity_validates_loaded_snapshot(self):
        """Resolve only fresh mapping identity and reject mismatches or malformed values."""
        backend = Backend.__new__(Backend)
        backend.settings = main._settings
        backend.reader = Mock()
        backend._matcher_for = Mock()
        backend.reader.current.return_value = dict(map_id="scout2_main", generation=3,
                                                   database_sha256="c" * 64)
        self.assertEqual(backend.current_map_identity("scout2_main"), dict(
            map_id="scout2_main", generation=3, database_sha256="c" * 64))
        with self.assertRaisesRegex(ValueError, 'configured map'):
            backend.current_map_identity("other_map")
        backend.reader.current.return_value = dict(map_id="scout2_main", generation=True,
                                                    database_sha256="c" * 64)
        with self.assertRaisesRegex(RuntimeError, 'generation'):
            backend.current_map_identity("scout2_main")
        backend.reader.current.return_value = dict(map_id="scout2_main", generation=3,
                                                    database_sha256="C" * 64)
        with self.assertRaisesRegex(RuntimeError, 'digest'):
            backend.current_map_identity("scout2_main")

    def test_worker_cleanup_failure_retains_ros_and_channels(self):
        """Never destroy inputs a still-running computation may be using."""
        publisher = main._publisher = Mock()
        channel = Mock()
        main._channels.append(channel)
        with patch.object(main._runtime, 'deactivate', side_effect=RuntimeError('worker alive')):
            self.assertIsInstance(main.deactivate(), Err)
        publisher.close.assert_not_called()
        channel.close.assert_not_called()
        self.assertIs(main._publisher, publisher)
        self.assertIsInstance(main.deactivate(), Ok)
        publisher.close.assert_called_once()
        channel.close.assert_called_once()
