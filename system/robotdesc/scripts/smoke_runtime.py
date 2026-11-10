"""Check isolated rbnx boot, Robotdesc RPCs and World geometry without hardware."""

import argparse
import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time


def free_port():
    """Select a currently unused loopback port for the isolated deployment."""
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        return listener.getsockname()[1]


def run_fixture():
    """Exercise Robotdesc's pre-ACTIVE body API from a real primitive INIT."""
    import grpc
    import robonix_contracts_pb2_grpc as contracts
    import robotdesc_pb2
    from robonix_api import Ok, Primitive

    provider = Primitive(
        id="robotdesc_smoke_primitive", namespace="robonix/primitive/robot_description",
    )
    marker = Path(os.environ["ROBOTDESC_SMOKE_MARKER"])

    @provider.on_init
    def init(config):
        """Read the canonical URDF using the endpoint injected by Robotdesc."""
        endpoint = os.environ["ROBONIX_ROBOTDESC_ENDPOINT"]
        with grpc.insecure_channel(endpoint) as channel:
            response = contracts.RobonixSystemRobotdescGetUrdfStub(channel).GetUrdf(
                robotdesc_pb2.GetUrdf_Request(), timeout=5,
            )
        assert response.robot_id == "test_ci_robot", response
        assert "<robot" in response.urdf_xml, response
        assert config["probe"] == "robotdesc", config
        marker.write_text(json.dumps({"endpoint": endpoint, "robot": response.robot_id}))
        return Ok()

    provider.run()


def check_rpcs(src, endpoint, root):
    """Check actual YAML, URDF, footprint, health RPCs and the World caller."""
    import grpc
    import robonix_contracts_pb2_grpc as contracts
    import robotdesc_pb2

    prefix = "robonix/system/robotdesc/"
    with grpc.insecure_channel(endpoint) as channel:
        yaml_response = contracts.RobonixSystemRobotdescGetYamlStub(channel).GetYaml(
            robotdesc_pb2.GetYaml_Request(), timeout=5,
        )
        assert yaml_response.yaml_text == (root / "examples/test_ci/robotdesc.yaml").read_text()
        urdf = contracts.RobonixSystemRobotdescGetUrdfStub(channel).GetUrdf(
            robotdesc_pb2.GetUrdf_Request(), timeout=5,
        )
        assert urdf.urdf_xml == (root / "examples/test_ci/test_ci.urdf").read_text()
        footprint = contracts.RobonixSystemRobotdescFootprintStub(channel).GetFootprint(
            robotdesc_pb2.GetFootprint_Request(), timeout=5,
        )
        assert footprint.base_frame == "base_link", footprint
        assert len(footprint.points) == 4, footprint
        health_stub = contracts.RobonixSystemRobotdescGetHealthStub(channel)
        deadline = time.monotonic() + 5
        while True:
            health = health_stub.GetHealth(robotdesc_pb2.GetHealth_Request(), timeout=5)
            if health.HasField("snapshot"):
                break
            assert time.monotonic() < deadline, "no health snapshot published"
            time.sleep(0.1)
        assert health.snapshot.robotdesc_ts_ns > 0, health
        stream = contracts.RobonixSystemRobotdescHealthStub(channel).StreamHealth(
            robotdesc_pb2.StreamHealth_Request(), timeout=5,
        )
        try:
            assert next(stream).body_id == "test_ci_robot"
        finally:
            stream.cancel()

    from world_service.robot_geometry import fetch_robotdesc_footprint

    src.register_service("world", "robonix/system/world")
    try:
        geometry = fetch_robotdesc_footprint()
        assert geometry.provider_id == "robotdesc", geometry
        assert geometry.base_frame == "base_link", geometry
        assert len(geometry.points) == 4, geometry
    finally:
        src.unregister("world")
    assert not src.query(namespace_prefix="robonix/system/soma")
    assert not src.list_contracts(namespace_prefix="robonix/system/soma")
    assert src.query_contract(prefix + "get_urdf") is not None


def main():
    """Run an isolated real boot and stop only the process groups it owns."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fixture-provider", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.fixture_provider:
        run_fixture()
        return
    root = Path(__file__).resolve().parents[3]
    binaries = root / "target/debug"
    for name in ("rbnx", "robonix-src", "robonix-robotdesc"):
        if not (binaries / name).is_file():
            parser.error("build first: cargo build -p robonix-cli -p robonix-src -p robonix-robotdesc")
    generated = root / "system/world/rbnx-build/codegen/proto_gen"
    if not (generated / "robotdesc_pb2.py").is_file():
        parser.error("generate World first: rbnx codegen -p system/world --mcp --python /usr/bin/python3")
    paths = [root / "pylib/robonix-api", root / "system/world", generated]
    sys.path[:0] = [str(path) for path in paths]
    import grpc
    from robonix_api import SRC

    artifacts = Path(tempfile.mkdtemp(prefix="robotdesc-boot-smoke-"))
    package = artifacts / "primitive"
    package.mkdir()
    (package / "rbnx-build").mkdir()
    (package / "rbnx-build/.rbnx-built").touch()
    src_endpoint = f"127.0.0.1:{free_port()}"
    endpoint = f"127.0.0.1:{free_port()}"
    start = f"exec {shlex.quote(sys.executable)} {shlex.quote(str(Path(__file__).resolve()))} --fixture-provider"
    (package / "package_manifest.yaml").write_text(
        "manifestVersion: 1\npackage:\n  name: robonix.primitive.robotdesc_smoke\n"
        "  version: 0.1.0\n  description: Isolated lifecycle fixture\n  license: MulanPSL-2.0\n"
        f"build: 'true'\nstart: {json.dumps(start)}\nstop: 'true'\n"
        "capabilities:\n  - name: robonix/primitive/robot_description/driver\n    transport: grpc\n"
    )
    for name in ("robotdesc.yaml", "test_ci.urdf"):
        shutil.copyfile(root / "examples/test_ci" / name, artifacts / name)
    manifest = artifacts / "robonix_manifest.yaml"
    manifest.write_text(json.dumps({
        "name": "robotdesc-boot-smoke",
        "system": {"src": {"listen": src_endpoint},
                   "robotdesc": {"listen": endpoint, "log": "info"}},
        "primitive": [{"name": "robotdesc_smoke_primitive", "path": "./primitive",
                       "config": {"probe": "robotdesc"}}],
        "service": [], "skill": [],
    }))
    env = os.environ.copy()
    env.update({
        "PATH": str(binaries) + os.pathsep + env.get("PATH", ""),
        "PYTHONPATH": os.pathsep.join(str(path) for path in paths),
        "PYTHONDONTWRITEBYTECODE": "1", "ROBONIX_SRC": src_endpoint,
        "ROBONIX_SOURCE_PATH": str(root), "ROBONIX_HOME": str(artifacts / "home"),
        "ROBOTDESC_SMOKE_MARKER": str(artifacts / "primitive-init.json"),
        "ROBONIX_PROVIDER_BIND_HOST": "127.0.0.1", "ROBONIX_ADVERTISE_HOST": "127.0.0.1",
    })
    for key in ("RBNX_INSTANCE_NAME", "ROBONIX_DEPLOY_DIR", "ROBONIX_ROBOTDESC_STAGE_FD",
                "RBNX_INVOCATION_CWD", "ROBONIX_CONFIG_PATH", "RBNX_CONFIG_FILE"):
        env.pop(key, None)
    os.environ["ROBONIX_SRC"] = src_endpoint
    os.environ["PYTHONDONTWRITEBYTECODE"] = "1"
    home = artifacts / "home"
    home.mkdir()
    (home / "config.yaml").write_text(
        f"robonix_source_path: {json.dumps(str(root))}\n"
        f"package_storage_path: {json.dumps(str(home / 'packages'))}\n"
    )
    print(f"Artifacts: {artifacts}", flush=True)
    with (artifacts / "boot.stdout").open("w") as output:
        process = subprocess.Popen(
            [str(binaries / "rbnx"), "boot", "-f", str(manifest), "--no-update-check"],
            cwd=artifacts, env=env, stdout=output, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    try:
        with grpc.insecure_channel(src_endpoint) as channel:
            grpc.channel_ready_future(channel).result(timeout=20)
        deadline = time.monotonic() + 60
        while True:
            assert process.poll() is None, (artifacts / "boot.stdout").read_text()
            providers = SRC.query(id="robotdesc")
            boot_log = (artifacts / "boot.stdout").read_text()
            if providers and providers[0].state.name == "ACTIVE" and "stage 2 trigger written" in boot_log:
                break
            assert time.monotonic() < deadline, boot_log
            time.sleep(0.2)
        provider = providers[0]
        assert provider.namespace == "robonix/system/robotdesc", provider
        assert len(provider.capabilities) == 5, provider
        assert SRC.query(id="robotdesc_smoke_primitive")[0].state.name == "ACTIVE"
        assert json.loads((artifacts / "primitive-init.json").read_text())["endpoint"] == endpoint
        assert not SRC.query(id="soma")
        check_rpcs(SRC, endpoint, root)
        print("PASS: boot stage 1/2, primitive INIT/ACTIVE, five RPCs and World footprint", flush=True)
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGTERM)
        try:
            process.wait(timeout=25)
        except subprocess.TimeoutExpired:
            state_path = artifacts / "rbnx-boot/state.json"
            print(f"Boot cleanup timed out; inspect {state_path}", file=sys.stderr)
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
            raise


if __name__ == "__main__":
    main()
