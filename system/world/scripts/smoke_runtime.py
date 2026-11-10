"""Check real Src registration, ROS ingestion and World MCP without robot hardware."""

import argparse
import asyncio
import base64
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time


def free_port():
    """Choose an available loopback port for an isolated test process."""
    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        return listener.getsockname()[1]


async def check_mcp(endpoint):
    """Verify the real MCP response includes the annotation created through HTTP."""
    from mcp import ClientSession
    from mcp.client.streamable_http import streamablehttp_client

    async with streamablehttp_client(endpoint, timeout=10) as streams:
        async with ClientSession(streams[0], streams[1]) as session:
            await session.initialize()
            tools = await session.list_tools()
            assert len(tools.tools) == 8, tools
            result = await session.call_tool("list_regions", {})
            assert not result.isError, result
            data = result.structuredContent
            if data and "result" in data:
                data = data["result"]
            if not data:
                data = json.loads(next(
                    item.text for item in result.content if hasattr(item, "text")
                ))
            assert any(region["name"] == "world-smoke-room"
                       for region in data["regions"]), data
            return len(tools.tools)


def stop_process(process):
    """Stop only an owned process group, with bounded escalation on timeout."""
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=12)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)


def main():
    """Run isolated real processes, preserve diagnostics and always clean up."""
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--src-bin", type=Path,
                        default=root / "target/debug/robonix-src")
    parser.add_argument("--ros-domain-id", type=int, default=218)
    args = parser.parse_args()
    if not args.src_bin.is_file():
        parser.error("build Src first: cargo build -p robonix-src")

    artifacts = Path(tempfile.mkdtemp(prefix="world-migration-smoke-"))
    endpoint = f"127.0.0.1:{free_port()}"
    web_port = free_port()
    env = os.environ.copy()
    env.update(
        ROBONIX_SRC=endpoint, ROBONIX_ADVERTISE_HOST="127.0.0.1",
        ROBONIX_PROVIDER_BIND_HOST="127.0.0.1",
        ROS_DOMAIN_ID=str(args.ros_domain_id), RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
        ROBONIX_LOG_DIR=str(artifacts / "logs"),
        RBNX_CONFIG_FILE=str(artifacts / "config.json"),
        WORLD_OBJECT_MEMORY_ENABLED="false", WORLD_GRAPH_ENABLED="false",
        WORLD_MAP_BINDING_WAIT_S="0", WORLD_RESTORE_ON_START="false",
        WORLD_GRAPH_CACHE_DIR=str(artifacts / "graph"),
        WORLD_ANNOTATIONS_DIR=str(artifacts / "annotations"),
        WORLD_MAP_META_DIR=str(artifacts / "meta"),
    )
    for key in ("RBNX_INSTANCE_NAME", "ROBONIX_DEPLOY_DIR",
                "FASTRTPS_DEFAULT_PROFILES_FILE", "CYCLONEDDS_URI"):
        env.pop(key, None)
        os.environ.pop(key, None)
    os.environ.update(env)
    package = root / "system/world"
    paths = [package, root / "pylib/robonix-api",
             package / "rbnx-build/codegen/proto_gen",
             package / "rbnx-build/codegen/robonix_mcp_types"]
    import sys

    sys.path[:0] = [str(path) for path in paths]
    env["PYTHONPATH"] = os.pathsep.join(
        [str(path) for path in paths] + [env.get("PYTHONPATH", "")]
    )
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    (artifacts / "config.json").write_text(json.dumps({
        "web_host": "127.0.0.1", "web_port": web_port,
        "observations": [{"kind": "occupancy_grid",
                          "contract": "robonix/service/map/occupancy_grid"}],
    }))
    import grpc
    import httpx
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from robonix_api import SRC

    processes = []
    node = None
    edge = None
    print(f"Artifacts: {artifacts}", flush=True)
    try:
        with (artifacts / "src.stdout").open("w") as output:
            processes.append(subprocess.Popen(
                [str(args.src_bin.resolve()), "--listen", endpoint,
                 "--capabilities", str(root / "capabilities")],
                cwd=artifacts, env=env, stdout=output, stderr=subprocess.STDOUT,
                start_new_session=True,
            ))
        with grpc.insecure_channel(endpoint) as channel:
            grpc.channel_ready_future(channel).result(timeout=15)
        SRC.register_service("world_smoke_mapping", "robonix/service/map")
        SRC.declare_capability(
            "world_smoke_mapping", "robonix/service/map/occupancy_grid",
            "ros2", "/world_migration_smoke/map",
        )
        rclpy.init(args=None)
        node = rclpy.create_node("world_migration_smoke_publisher")
        publisher = node.create_publisher(
            OccupancyGrid, "/world_migration_smoke/map",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE),
        )
        with (artifacts / "world.stdout").open("w") as output:
            processes.append(subprocess.Popen(
                [sys.executable, "-m", "world_service.service"], cwd=package,
                env=env, stdout=output, stderr=subprocess.STDOUT,
                start_new_session=True,
            ))
        message = OccupancyGrid()
        message.header.frame_id = "map"
        message.info.width, message.info.height = 4, 3
        message.info.resolution = 0.1
        message.info.origin.orientation.w = 1.0
        message.data = [0] * 12
        deadline = time.monotonic() + 50
        state = None
        with httpx.Client(timeout=2, trust_env=False) as client:
            while time.monotonic() < deadline:
                if processes[-1].poll() is not None:
                    raise RuntimeError((artifacts / "world.stdout").read_text())
                message.header.stamp = node.get_clock().now().to_msg()
                publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=0.02)
                try:
                    response = client.get(f"http://127.0.0.1:{web_port}/api/state")
                    if response.status_code == 200 and response.json().get("occupancy"):
                        state = response.json()
                        break
                except httpx.ConnectError:
                    pass
                time.sleep(0.1)
            assert state is not None, f"no World occupancy: {artifacts}"
            occupancy = state["occupancy"]
            assert (occupancy["width"], occupancy["height"]) == (4, 3), occupancy
            assert base64.b64decode(occupancy["png_b64"]).startswith(b"\x89PNG\r\n\x1a\n")
            response = client.post(f"http://127.0.0.1:{web_port}/api/annotations", json={
                "kind": "room", "name": "world-smoke-room",
                "points": [[0, 0], [1, 0], [1, 1], [0, 1]],
            })
            response.raise_for_status()
            assert response.json()["ok"], response.text
        providers = SRC.query_services(id="world")
        assert len(providers) == 1, providers
        assert providers[0].namespace == "robonix/system/world", providers
        capabilities = SRC.find_capability(
            provider_id="world", namespace_prefix="robonix/system/world", transport="mcp"
        )
        assert len(capabilities) == 8, capabilities
        assert not SRC.query_services(id="scene")
        assert not SRC.find_capability(namespace_prefix="robonix/system/scene")
        edge = SRC.connect_capability(
            consumer_id="world_migration_smoke", provider_id="world",
            contract_id="robonix/system/world/list_regions", transport="mcp",
        )
        count = asyncio.run(check_mcp(edge.endpoint))
        print(json.dumps({"ok": True, "provider": "world", "mcp_tools": count,
                          "occupancy": [4, 3], "artifacts": str(artifacts)}), flush=True)
    finally:
        if edge is not None:
            edge.close()
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()
        for process in reversed(processes):
            stop_process(process)
        print(f"Process exit codes: {[process.returncode for process in processes]}")


if __name__ == "__main__":
    main()
