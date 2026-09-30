"""Isolated ROS/RTAB-Map restart smoke test; run in a network-none container.

Uses synthetic scans/odometry only, temporary databases, and no chassis API.
The package must be mounted at /mapping with ROS Humble sourced.
"""
import json
import math
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import tempfile
import time
import yaml

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

from mapping_rbnx import map_ops


def node_ids(path):
    """Read graph node identities from a checked RTAB-Map database."""
    with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as con:
        assert con.execute("PRAGMA quick_check").fetchone()[0] == "ok"
        return {row[0] for row in con.execute("SELECT id FROM Node")}


def kill_session(proc):
    """Kill only the test wrapper and its owned launch group to simulate a crash."""
    children = Path(f"/proc/{proc.pid}/task/{proc.pid}/children").read_text().split()
    for child in children:
        try:
            child_pid = int(child)
            if os.getpgid(child_pid) == child_pid:
                os.killpg(child_pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    os.killpg(proc.pid, signal.SIGKILL)


def publish_scan(node, scan_pub, odom_pub, broadcaster, x):
    """Simulate a laser in an asymmetric room and matching odometry/TF."""
    stamp = node.get_clock().now().to_msg()
    tf = TransformStamped()
    tf.header.stamp, tf.header.frame_id, tf.child_frame_id = stamp, "odom", "base_link"
    tf.transform.translation.x = x
    tf.transform.rotation.w = 1.0
    broadcaster.sendTransform(tf)
    odom = Odometry()
    odom.header = tf.header
    odom.child_frame_id = "base_link"
    odom.pose.pose.position.x = x
    odom.pose.pose.orientation.w = 1.0
    odom_pub.publish(odom)
    scan = LaserScan()
    scan.header.stamp, scan.header.frame_id = stamp, "base_link"
    scan.angle_min, scan.angle_max = -math.pi, math.pi
    scan.angle_increment = 2.0 * math.pi / 360
    scan.range_min, scan.range_max = 0.05, 20.0
    scan.scan_time = 0.1
    for index in range(361):
        angle = scan.angle_min + index * scan.angle_increment
        dx, dy = math.cos(angle), math.sin(angle)
        distances = []
        if abs(dx) > 1e-8:
            distances.append(((8.0 if dx > 0 else -4.0) - x) / dx)
        if abs(dy) > 1e-8:
            distances.append((3.0 if dy > 0 else -2.0) / dy)
        scan.ranges.append(min(distances))
    scan_pub.publish(scan)
    time.sleep(0.1)


def run_session(root, runtime, session, start, finish, node, pubs, grids, abrupt=False):
    """Launch the production ROS launch file, feed scans and checkpoint it."""
    # Exercise the same engine wrapper and signal forwarding used by Docker.
    Path("/tmp/rtabmap_resolved.yaml").write_text(yaml.safe_dump({
        "use_sim_time": False, "scan_topic": "/persistence_test/scan",
        "odom_topic": "/persistence_test/odom", "database_path": runtime,
        "map_mode": "mapping", "reset_map": False,
        "rtabmap_overrides_file": str(root / "params.json"),
    }))
    command = ["bash", "/mapping/scripts/start_engine.sh"]
    with (root / f"session-{session}.log").open("w") as log:
        grids.clear()
        proc = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
        try:
            time.sleep(5)
            if session > 1:
                ok, detail = map_ops._publish_full_map(map_ops._get_node())
                assert ok, detail
                deadline = time.monotonic() + 10
                while not grids and time.monotonic() < deadline:
                    time.sleep(0.1)
                assert grids, "resumed database did not publish a map before new scans"
            for step in range(100):
                assert proc.poll() is None, "RTAB-Map exited unexpectedly"
                x = start + (finish - start) * max(0, step - 20) / 80
                publish_scan(node, *pubs, x)
            map_ops._ACTIVE_DB = runtime
            result = map_ops.save_map_impl("site", active_db=runtime)
            assert result["ok"], result
            saved = root / "maps/site/rtabmap.db"
            assert (saved.parent / "occupancy.png").is_file(), "missing map preview"
            ids = node_ids(saved)
            assert len(ids) >= 3, f"too few mapped nodes: {len(ids)}"
            print(f"session {session}: checkpoint contains {len(ids)} nodes", flush=True)
            return ids
        except BaseException:
            log.flush()
            print((root / f"session-{session}.log").read_text()[-12000:], flush=True)
            raise
        finally:
            if proc.poll() is None:
                if abrupt:
                    kill_session(proc)
                else:
                    proc.send_signal(signal.SIGTERM)
                try:
                    proc.wait(timeout=45)
                except subprocess.TimeoutExpired:
                    log.flush()
                    print((root / f"session-{session}.log").read_text()[-10000:], flush=True)
                    kill_session(proc)
                    proc.wait()
                    raise RuntimeError("RTAB-Map did not close gracefully")


def main():
    """Verify historical graph retention and growth across two real engines."""
    with tempfile.TemporaryDirectory(prefix="mapping-persistence-") as directory:
        root = Path(directory)
        map_ops.MAPS_DIR = str(root / "maps")
        map_ops.RUNTIME_DB_DIR = str(root / "runtime")
        # Start from the deployed Scout2 tuning and only accelerate sampling.
        params = yaml.safe_load(Path("/deploy/config/rtabmap_params.yaml").read_text())
        params.update({"Rtabmap/DetectionRate": 5.0, "RGBD/LinearUpdate": 0.05})
        (root / "params.json").write_text(json.dumps(params))
        # This also verifies that lazy ROS initialization preserves SIGTERM.
        signals_received = []
        signal.signal(signal.SIGTERM, lambda *_: signals_received.append(True))
        assert map_ops._get_node() is not None
        os.kill(os.getpid(), signal.SIGTERM)
        assert signals_received, "ROS initialization replaced the shutdown handler"
        grids = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        map_ops._get_node().create_subscription(
            OccupancyGrid, "/map", lambda msg: grids.append(msg) if msg.info.width else None, qos
        )
        node = rclpy.create_node("mapping_persistence_test")
        pubs = (node.create_publisher(LaserScan, "/persistence_test/scan", 10),
                node.create_publisher(Odometry, "/persistence_test/odom", 10),
                TransformBroadcaster(node))
        runtime = map_ops.persistent_runtime_db("site")
        first = run_session(root, runtime, 1, 0.0, 1.0, node, pubs, grids)
        assert first <= node_ids(runtime), "clean shutdown lost checkpoint nodes"
        resumed = map_ops.persistent_runtime_db("site")
        assert resumed == runtime
        second = run_session(root, resumed, 2, 0.0, 2.5, node, pubs, grids, abrupt=True)
        assert first < second, "restart did not preserve AND extend previous graph"
        recovered = map_ops.persistent_runtime_db("site")
        third = run_session(root, recovered, 3, 0.0, 3.5, node, pubs, grids)
        assert second < third, "abrupt termination lost checkpoint nodes"
        print(f"PASS: graph nodes {len(first)} -> {len(second)} -> {len(third)}; "
              "normal and abrupt restart retained the graph and restored /map", flush=True)
        node.destroy_node()
        map_ops._node._robonix_executor.shutdown()
        map_ops._node._robonix_spin.join(timeout=5)
        map_ops._node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
