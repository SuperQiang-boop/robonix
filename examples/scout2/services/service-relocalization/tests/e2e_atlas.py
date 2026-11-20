"""Exercise real Atlas, service gRPC and ROS status without robot dependencies."""
import json
import os
from pathlib import Path
import random
import shutil
import socket
import subprocess
import tempfile
import time
import unittest

import grpc
import atlas_pb2 as atlas_pb
import atlas_pb2_grpc as atlas_grpc
import lifecycle_pb2
import relocalization_pb2 as pb
import robonix_contracts_pb2_grpc as contracts

PKG = Path(__file__).resolve().parents[1]
PREFIX = "robonix/service/relocalization/"


def wait_for(check, timeout=15):
    """Bound startup/discovery waits and preserve the last transport error."""
    deadline = time.monotonic() + timeout
    last_error = None
    while time.monotonic() < deadline:
        try:
            value = check()
            if value:
                return value
        except grpc.RpcError as exc:
            last_error = exc
        time.sleep(0.05)
    raise AssertionError(f"timed out waiting for readiness: {last_error}")


class AtlasIntegrationTests(unittest.TestCase):
    def setUp(self):
        """Start an owned Atlas and service using private ports, domain and logs."""
        self.processes, self.logs, self.grpc_channels = [], [], []
        self.temp = tempfile.TemporaryDirectory(prefix="relocalization-e2e-")
        self.addCleanup(self.cleanup)
        root = Path(self.temp.name)
        source = Path(os.environ.get("ROBONIX_SOURCE_PATH", PKG.parents[3]))
        binary = shutil.which("robonix-atlas")
        self.assertIsNotNone(binary, "robonix-atlas must be installed")
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            endpoint = f"127.0.0.1:{sock.getsockname()[1]}"
        self.env = dict(os.environ, ROBONIX_ATLAS=endpoint,
            ROBONIX_PROVIDER_BIND_HOST="127.0.0.1",
            RBNX_INSTANCE_NAME="relocalization_e2e",
            RBNX_PACKAGE_ROOT=str(PKG), RBNX_INVOCATION_CWD=str(root),
            ROS_DOMAIN_ID=str(random.randint(101, 200)), ROS_LOCALHOST_ONLY="1",
            RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
            ROS_LOG_DIR=str(root / "ros-logs"), SCRIBE_LOG_DIR=str(root / "logs"))
        self.spawn([binary, "--listen", endpoint, "--capabilities",
                    f"{source / 'capabilities'},{PKG / 'capabilities'}"], "atlas")
        channel = self.channel(endpoint)
        grpc.channel_ready_future(channel).result(timeout=10)
        self.atlas = atlas_grpc.AtlasStub(channel)
        for suffix in ("driver", "relocalize", "relocalize_by_map", "get_status", "cancel", "status"):
            result = self.atlas.QueryContract(
                atlas_pb.QueryContractRequest(contract_id=PREFIX + suffix), timeout=3)
            self.assertTrue(result.found, suffix)
        self.spawn(["bash", str(PKG / "scripts/start.sh")], "service")
        wait_for(lambda: len(self.provider().capabilities) == 5 if self.provider() else False)
        channel = self.channel(self.connect("driver").endpoint)
        self.driver = contracts.RobonixServiceRelocalizationDriverStub(channel).Driver
        self.status = contracts.RobonixServiceRelocalizationGetStatusStub(
            self.channel(self.connect("get_status").endpoint)).GetStatus
        self.relocalize = contracts.RobonixServiceRelocalizationRelocalizeStub(
            self.channel(self.connect("relocalize").endpoint)).Relocalize
        self.cancel = contracts.RobonixServiceRelocalizationCancelStub(
            self.channel(self.connect("cancel").endpoint)).Cancel

    def spawn(self, argv, name):
        """Capture output and retain process handles for deterministic cleanup."""
        path = Path(self.temp.name) / f"{name}.log"
        stream = path.open("w")
        self.logs.append((path, stream))
        self.processes.append(subprocess.Popen(argv, env=self.env,
            cwd=self.temp.name, stdout=stream, stderr=subprocess.STDOUT))

    def channel(self, endpoint):
        channel = grpc.insecure_channel(endpoint, options=[("grpc.enable_http_proxy", 0)])
        self.grpc_channels.append(channel)
        return channel

    def cleanup(self):
        """Stop only owned processes and expose captured logs on a failing test."""
        for process in reversed(self.processes):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
        for channel in self.grpc_channels:
            channel.close()
        for path, stream in self.logs:
            stream.close()
            # Logs are small and contain only isolated fixture configuration.
            if any(error for _, error in self._outcome.result.errors + self._outcome.result.failures):
                print(f"{path.name}:\n{path.read_text()}")
        self.temp.cleanup()

    def provider(self):
        result = self.atlas.Query(atlas_pb.QueryRequest(id="relocalization_e2e"), timeout=3)
        return result.providers[0] if result.providers else None

    def connect(self, suffix, transport=atlas_pb.TRANSPORT_GRPC):
        return self.atlas.ConnectCapability(atlas_pb.ConnectCapabilityRequest(
            consumer_id="e2e_client", provider_id="relocalization_e2e",
            contract_id=PREFIX + suffix, transport=transport), timeout=3)

    def command(self, command):
        """Send actual Driver requests with explicit snapshot/provider configuration."""
        config = dict(map_id="scout2_main", mapping_provider_id="mapping_e2e",
            lidar_provider_id="lidar_e2e", odom_provider_id="odom_e2e",
            maps_dir="maps", index_dir="indices", status_period_s=0.1)
        return self.driver(lifecycle_pb2.Driver_Request(
            command=command, config_json=json.dumps(config)), timeout=10)

    def dependency(self, provider, namespace, suffix, transport=atlas_pb.TRANSPORT_ROS2):
        """Register discovery-only fixtures; no sensor or motion data is emitted."""
        if not self.atlas.Query(atlas_pb.QueryRequest(id=provider), timeout=3).providers:
            register = self.atlas.RegisterPrimitive if "/primitive/" in namespace else self.atlas.RegisterService
            register(atlas_pb.RegisterRequest(id=provider, namespace=namespace), timeout=3)
        params = atlas_pb.TransportParams()
        if transport == atlas_pb.TRANSPORT_ROS2:
            params.ros2.qos_profile = "reliable"
            endpoint = f"/e2e/{provider}/{suffix}"
        else:
            params.grpc.service_name = "discovery_only"
            endpoint = "127.0.0.1:1"
        self.atlas.DeclareCapability(atlas_pb.DeclareCapabilityRequest(
            provider_id=provider, contract_id=f"{namespace}/{suffix}",
            transport=transport, endpoint=endpoint, params=params), timeout=3)
        self.atlas.SetLifecycleState(atlas_pb.SetLifecycleStateRequest(
            id=provider, state=atlas_pb.STATE_ACTIVE), timeout=3)

    def own_channels(self):
        result = self.atlas.InspectAtlas(atlas_pb.InspectAtlasRequest(), timeout=3)
        return [channel for channel in json.loads(result.json)["channels"].values()
                if channel["consumer_id"] == "relocalization_e2e"]

    def test_missing_dependency_rolls_back(self):
        """One connected lidar is released when odom discovery subsequently fails."""
        self.dependency("lidar_e2e", "robonix/primitive/lidar", "lidar3d")
        self.assertTrue(self.command(0).ok)
        result = self.command(1)
        self.assertFalse(result.ok)
        self.assertIn("odom", result.error)
        self.assertEqual(self.provider().state, atlas_pb.STATE_ERROR)
        self.assertEqual(self.own_channels(), [])
        status = self.status(pb.GetStatus_Request(), timeout=3).status
        self.assertFalse(status.active)
        self.assertFalse(status.localization_valid)
        with self.assertRaises(grpc.RpcError) as caught:
            self.relocalize(pb.Relocalize_Request(), timeout=3)
        self.assertEqual(caught.exception.code(), grpc.StatusCode.FAILED_PRECONDITION)
        self.assertTrue(self.command(3).ok)

    def test_missing_production_profile_rolls_back(self):
        """Missing calibration/index configuration cannot activate the real backend."""
        self.dependency("lidar_e2e", "robonix/primitive/lidar", "lidar3d")
        self.dependency("odom_e2e", "robonix/primitive/chassis", "odom")
        self.dependency("mapping_e2e", "robonix/service/map", "pose_estimate", atlas_pb.TRANSPORT_GRPC)
        self.dependency("mapping_e2e", "robonix/service/map", "lifecycle")
        self.dependency("mapping_e2e", "robonix/service/map", "pose")
        self.assertTrue(self.command(0).ok)
        result = self.command(1)
        self.assertFalse(result.ok)
        self.assertIn('profile_file', result.error)
        self.assertEqual(self.own_channels(), [])
        self.assertFalse(self.status(pb.GetStatus_Request(), timeout=3).status.active)
        self.assertTrue(self.command(3).ok)

    def receive_status(self, topic, process_id):
        """Subscribe in the child's isolated ROS domain and check fresh invalid status."""
        from unittest.mock import patch
        with patch.dict(os.environ, self.env):
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from relocalization.msg import Status
            context = Context()
            rclpy.init(args=[], context=context)
            try:
                node = rclpy.create_node("relocalization_e2e_client", context=context)
                executor = SingleThreadedExecutor(context=context)
                try:
                    executor.add_node(node)
                    messages = []
                    node.create_subscription(Status, topic, messages.append, 1)
                    deadline = time.monotonic() + 10
                    while not messages and time.monotonic() < deadline:
                        executor.spin_once(timeout_sec=0.1)
                    self.assertTrue(messages, "no ROS status sample received")
                    self.assertEqual(messages[-1].process_id, process_id)
                    self.assertTrue(messages[-1].active)
                    self.assertFalse(messages[-1].localization_valid)
                    self.assertGreater(messages[-1].published_stamp_ns, 0)
                    self.assertEqual(messages[-1].observation_stamp_ns, 0)
                finally:
                    executor.shutdown()
                    node.destroy_node()
            finally:
                context.shutdown()


if __name__ == "__main__":
    unittest.main(verbosity=2)
