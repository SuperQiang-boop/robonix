"""Isolated ROS/RTAB-Map restart smoke test; run in a network-none container.

Uses synthetic scans/odometry only, temporary databases, and no chassis API.
The package must be mounted at /mapping with ROS Humble sourced.
"""
import ast
from contextlib import nullcontext
import copy
import hashlib
import json
import logging
import math
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import tempfile
import threading
import time
from types import SimpleNamespace
import yaml

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from rcl_interfaces.srv import GetParameters, ListParameters
from rtabmap_msgs.srv import GetMap
from tf2_ros import TransformBroadcaster

from mapping_rbnx import map_ops


def node_ids(path):
    """Read graph node identities from a checked RTAB-Map database."""
    with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as con:
        assert con.execute("PRAGMA quick_check").fetchone()[0] == "ok"
        return {row[0] for row in con.execute("SELECT id FROM Node")}


def verify_snapshot(root, session, sequence):
    """Require connected optimized poses and the formal relocalization index."""
    saved = root / 'maps/site/rtabmap.db'
    with sqlite3.connect(f'file:{saved}?mode=ro', uri=True) as con:
        assert con.execute('PRAGMA quick_check').fetchone()[0] == 'ok'
        assert con.execute('SELECT opt_ids IS NOT NULL AND opt_poses IS NOT NULL AND opt_map IS NOT NULL FROM Admin').fetchone()[0]
    report = json.loads((saved.parent / 'checkpoint.json').read_text())
    assert report['physical_components'] == 1 and report['scan_points'] > 0, report
    exporter = os.environ.get('MAPPING_TEST_EXPORTER')
    if exporter:
        from relocalization_rbnx.prepare import prepare
        index = prepare(root / 'maps', 'site', root / 'indices', Path(exporter))
        exported = json.loads((index / 'export.json').read_text())
        assert len(exported['optimized_components']) == 1, exported
        assert len(exported['nodes']) == report['nodes'], exported
    print(f'session {session} checkpoint {sequence}: {report}', flush=True)
    return node_ids(saved)


def start_checkpoints(root, runtime, session, results):
    """Execute the production periodic loop while this test keeps feeding scans."""
    def save(map_id, **kwargs):
        """Record each real automatic save and verify its independent saved graph."""
        result = map_ops.save_map_impl(map_id, **kwargs)
        if result['ok']:
            try:
                ids = verify_snapshot(root, session, len(results) + 1)
                results.append(ids)
            except Exception as exc:
                results.append(exc)
        else:
            results.append(RuntimeError(result['detail']))
        return result

    source = Path('/mapping/src/mapping_rbnx/atlas_bridge.py')
    function = next(node for node in ast.parse(source.read_text()).body
                    if isinstance(node, ast.FunctionDef) and node.name == '_start_checkpoint_loop')
    namespace = dict(threading=threading, _CHECKPOINT_STOP=threading.Event(),
        _CHECKPOINT_THREAD=None, _ACTIVE={'algo': 'rtabmap', 'map_id': 'site'},
        _active_db=lambda: runtime, log=logging.getLogger('checkpoint-test'),
        lifecycle=map_ops.lifecycle,
        map_ops=SimpleNamespace(_SAVE_LOCK=map_ops._SAVE_LOCK,
            get_mode_impl=map_ops.get_mode_impl, save_map_impl=save))
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), namespace)
    namespace['_start_checkpoint_loop']({'checkpoint_interval_s': 3}, 'mapping')
    return namespace


def reject_incomplete_maps(root, runtime):
    """Replay broken service responses and ensure the published map survives."""
    node = map_ops._get_node()
    request = GetMap.Request(global_map=True, optimized=True, graph_only=False)
    ok, original = map_ops._call_service(node, GetMap, '/rtabmap/get_map_data', request, 10)
    assert ok, original
    ok, listing = map_ops._call_service(node, ListParameters, '/rtabmap/list_parameters', ListParameters.Request(), 5)
    assert ok, listing
    ok, values = map_ops._call_service(node, GetParameters, '/rtabmap/get_parameters', GetParameters.Request(names=listing.result.names), 5)
    assert ok, values
    parameters = dict(zip(listing.result.names, values.values))
    payload = [original.data]

    def get_map(request, response):
        response.data = payload[0]
        return response

    def get_parameters(request, response):
        response.values = [parameters[name] for name in request.names]
        return response

    services = [node.create_service(GetMap, '/snapshot_test/get_map_data', get_map),
                node.create_service(GetParameters, '/snapshot_test/get_parameters', get_parameters)]
    saved = root / 'maps/site/rtabmap.db'
    digest = hashlib.sha256(saved.read_bytes()).hexdigest()
    previous_ns = map_ops.RTABMAP_NS
    map_ops.RTABMAP_NS = '/snapshot_test'
    try:
        for failure in ('disconnected', 'missing_pose', 'missing_scan', 'missing_grid'):
            broken = copy.deepcopy(original.data)
            if failure == 'disconnected':
                broken.graph.links = []
            elif failure == 'missing_pose':
                broken.graph.poses_id.pop()
                broken.graph.poses.pop()
            else:
                for signature in broken.nodes:
                    if failure == 'missing_scan':
                        signature.data.laser_scan = PointCloud2()
                        signature.data.laser_scan_compressed = []
                    else:
                        signature.data.grid_cell_size = 0.0
            payload[0] = broken
            result = map_ops.save_map_impl('site', active_db=runtime)
            assert not result['ok'], (failure, result)
            assert hashlib.sha256(saved.read_bytes()).hexdigest() == digest
            print(f'PASS rejected {failure}: {result["detail"][-400:]}', flush=True)
    finally:
        map_ops.RTABMAP_NS = previous_ns
        for service in services:
            node.destroy_service(service)


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
        "map_mode": "localization" if session > 1 else "mapping", "reset_map": False,
        "rtabmap_overrides_file": str(root / "params.json"),
    }))
    command = ["bash", "/mapping/scripts/start_engine.sh"]
    with (root / f"session-{session}.log").open("w") as log:
        grids.clear()
        proc = subprocess.Popen(command, stdout=log, stderr=log, start_new_session=True)
        loop = None
        results = []
        try:
            time.sleep(5)
            if session > 1:
                ok, detail = map_ops._publish_full_map(map_ops._get_node())
                assert ok, detail
                deadline = time.monotonic() + 10
                while not grids and time.monotonic() < deadline:
                    time.sleep(0.1)
                assert grids, "resumed database did not publish a map before new scans"
                result = map_ops.pose_estimate_impl(start, 0.0, 0.0)
                assert result['ok'], result
            map_ops._ACTIVE_DB = runtime
            for step in range(220):
                assert proc.poll() is None, "RTAB-Map exited unexpectedly"
                if step == 20:
                    if session > 1:
                        ok, detail = map_ops._set_mode(map_ops._get_node(), 'mapping')
                        assert ok, detail
                    loop = start_checkpoints(root, runtime, session, results)
                x = start + (finish - start) * max(0, step - 20) / 200
                publish_scan(node, *pubs, x)
            loop['_CHECKPOINT_STOP'].set()
            loop['_CHECKPOINT_THREAD'].join(timeout=30)
            assert not loop['_CHECKPOINT_THREAD'].is_alive(), 'automatic save did not finish'
            assert len(results) >= 3, f'too few automatic saves: {results}'
            assert all(isinstance(value, set) for value in results), results
            assert results[0] < results[-1], 'mapping did not grow between automatic saves'
            result = map_ops.save_map_impl("site", active_db=runtime)
            assert result["ok"], result
            saved = root / "maps/site/rtabmap.db"
            assert (saved.parent / "occupancy.png").is_file(), "missing map preview"
            ids = node_ids(saved)
            assert len(ids) >= 3, f"too few mapped nodes: {len(ids)}"
            verify_snapshot(root, session, 'final')
            if session == 1:
                reject_incomplete_maps(root, runtime)
            print(f"session {session}: checkpoint contains {len(ids)} nodes", flush=True)
            return ids
        except BaseException:
            log.flush()
            print((root / f"session-{session}.log").read_text()[-12000:], flush=True)
            raise
        finally:
            if loop:
                loop['_CHECKPOINT_STOP'].set()
                loop['_CHECKPOINT_THREAD'].join(timeout=30)
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
            content = (root / f'session-{session}.log').read_text()
            assert 'database is locked' not in content, 'SQLite writer lock regression'
            assert '[rtabmap-1]: process has died' not in content, 'RTAB-Map crashed'


def main():
    """Verify historical graph retention and growth across two real engines."""
    output = os.environ.get('MAPPING_PERSISTENCE_TEST_DIR')
    context = nullcontext(output) if output else tempfile.TemporaryDirectory(prefix='mapping-persistence-')
    with context as directory:
        root = Path(directory)
        root.mkdir(parents=True, exist_ok=True)
        map_ops.MAPS_DIR = str(root / "maps")
        map_ops.RUNTIME_DB_DIR = str(root / "runtime")
        map_ops.set_current_mode('mapping')
        map_ops.lifecycle.MAPS_DIR = map_ops.MAPS_DIR
        map_ops.lifecycle.set_state('site', 'mapping', bump=True)
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
        second = run_session(root, resumed, 2, 1.0, 2.5, node, pubs, grids, abrupt=True)
        assert first < second, "restart did not preserve AND extend previous graph"
        recovered = map_ops.persistent_runtime_db("site")
        third = run_session(root, recovered, 3, 2.5, 3.5, node, pubs, grids)
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
