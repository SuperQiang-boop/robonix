"""Bounded receive-only G1 SDK-to-ROS check; no LocoClient, cmd publisher or TF output."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time

import rclpy
from nav_msgs.msg import Odometry


def main():
    """Measure reception-to-ROS freshness in an isolated graph, then stop owned processes."""
    if os.environ.get("ROS_DOMAIN_ID") != "217" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("Use isolated ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--seconds", type=float, default=15)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 60:
        raise ValueError("seconds must be between 1 and 60")
    package = Path(__file__).resolve().parents[1]
    daemon = package / "rbnx-build/sdk/install/bin/g1_loco_daemon"
    adapter = package / "rbnx-build/ros/install/lib/g1_chassis_adapter/g1_chassis_adapter_node"
    processes = []
    samples = []
    rclpy.init()
    node = rclpy.create_node("g1_live_odometry_readonly")

    def receive(message):
        """Record ROS arrival time and age relative to the daemon's local receive stamp."""
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        samples.append((time.monotonic(), stamp, (time.time_ns() - stamp) / 1e9, message))

    subscription = node.create_subscription(Odometry, "/g1_readonly/odom", receive, 100)
    try:
        with tempfile.TemporaryDirectory(prefix="g1-live-readonly-") as directory:
            socket_path = str(Path(directory) / "daemon.sock")
            env = {**os.environ, "G1_IPC_SOCKET": socket_path}
            with open(Path(directory) / "nodes.log", "w+") as log:
                processes.append(subprocess.Popen(
                    [str(daemon), "--socket", socket_path, "--interface", args.interface,
                     "--state-topic", "rt/odommodestate"], env=env, stdout=log, stderr=log,
                ))
                deadline = time.monotonic() + 5
                while not Path(socket_path).exists() and time.monotonic() < deadline:
                    if processes[0].poll() is not None:
                        break
                    time.sleep(0.05)
                if not Path(socket_path).exists():
                    log.seek(0)
                    raise RuntimeError(f"receive-only daemon startup failed: {log.read()}")
                processes.append(subprocess.Popen(
                    [str(adapter), "--ros-args", "-p", "publish_odom_tf:=false",
                     "-p", "twist_in_topic:=/g1_readonly/unused_cmd",
                     "-p", "odom_topic:=/g1_readonly/odom",
                     "-p", "joint_state_topic:=/g1_readonly/joint_states"],
                    env=env, stdout=log, stderr=log,
                ))
                deadline = time.monotonic() + args.seconds
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                    if any(process.poll() is not None for process in processes):
                        log.seek(0)
                        raise RuntimeError(f"receive-only pipeline exited: {log.read()}")
                if len(samples) < 2:
                    log.seek(0)
                    raise RuntimeError(f"no measured odometry: {log.read()}")
                gaps = [current[0] - prior[0] for prior, current in zip(samples, samples[1:])]
                stamps = [sample[1] for sample in samples]
                latest = samples[-1][3]
                result = {
                    "count": len(samples),
                    "rate_hz": (len(samples) - 1) / (samples[-1][0] - samples[0][0]),
                    "max_arrival_gap_s": max(gaps),
                    "max_local_reception_age_s": max(sample[2] for sample in samples),
                    "duplicate_stamps": len(stamps) - len(set(stamps)),
                    "backward_stamps": sum(current < prior for prior, current in zip(stamps, stamps[1:])),
                    "frames": [latest.header.frame_id, latest.child_frame_id],
                    "position": [latest.pose.pose.position.x, latest.pose.pose.position.y, latest.pose.pose.position.z],
                    "twist": [latest.twist.twist.linear.x, latest.twist.twist.linear.y, latest.twist.twist.angular.z],
                    "motion_enabled": False,
                    "publish_odom_tf": False,
                    "timestamp_semantics": "local DDS reception, not robot acquisition",
                }
                Path(args.output).write_text(json.dumps(result, indent=2) + "\n")
                assert result["duplicate_stamps"] == result["backward_stamps"] == 0
                assert 35 <= result["rate_hz"] <= 65, result
                print(json.dumps(result, indent=2))
    finally:
        for process in reversed(processes):
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
