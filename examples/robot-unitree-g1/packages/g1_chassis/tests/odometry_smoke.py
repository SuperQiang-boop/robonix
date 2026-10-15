"""Exercise the ROS adapter with synthetic IPC state; never access the robot SDK.

Run with ROS sourced and ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1.
The only command recipient is a temporary, in-process fake Unix socket peer.
"""
import os
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def main():
    """Check measured output, identical TF stamps, freshness guards and jump latching."""
    if os.environ.get("ROS_DOMAIN_ID") != "217" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("Use isolated ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1")
    adapter = sys.argv[1]
    rclpy.init()
    node = Node("g1_odometry_smoke")
    odometry = []
    transforms = []
    commands = []
    subscriptions = [
        node.create_subscription(Odometry, "/g1_test/odom", odometry.append, 100),
        node.create_subscription(TFMessage, "/tf", transforms.append, 100),
    ]
    publisher = node.create_publisher(Twist, "/g1_test/cmd", 10)
    process = None
    connection = None
    sequence = 0
    packet = None
    try:
        with tempfile.TemporaryDirectory(prefix="g1-odom-") as directory:
            socket_path = str(Path(directory) / "fake.sock")
            with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as server:
                server.bind(socket_path)
                server.listen(1)
                server.settimeout(10)
                with open(Path(directory) / "adapter.log", "w+") as log:
                    process = subprocess.Popen(
                        [adapter, "--ros-args", "-p", "twist_in_topic:=/g1_test/cmd",
                         "-p", "odom_topic:=/g1_test/odom", "-p", "odom_frame:=g1_test_odom",
                         "-p", "base_frame:=g1_test_base", "-p", "odom_publish_rate_hz:=50.0"],
                        env={**os.environ, "G1_IPC_SOCKET": socket_path}, stdout=log, stderr=log,
                    )
                    connection, _ = server.accept()
                    connection.setblocking(False)

                    def pump(duration, mode=None, publish_command=False):
                        """Feed bounded synthetic state and drain only fake-peer command packets."""
                        nonlocal sequence, packet
                        deadline = time.monotonic() + duration
                        next_state = next_command = 0.0
                        while time.monotonic() < deadline:
                            now = time.monotonic()
                            if mode is not None and now >= next_state:
                                if mode != "duplicate":
                                    sequence += 1
                                    timestamp = time.time_ns() - (1_000_000_000 if mode == "stale" else 0)
                                    received = time.monotonic_ns() - (1_000_000_000 if mode == "stale" else 0)
                                    packet = struct.pack(
                                        "<BBHIQQQ3f4f3ff", 3, 1, 0, int(mode == "error"), sequence,
                                        timestamp, received, 0.5 if mode == "jump" else 0,
                                        0, 0.73, 1, 0, 0, 0, 0, 0, 0, 0,
                                    )
                                connection.send(packet)
                                next_state = now + 0.01
                            if publish_command and now >= next_command:
                                command = Twist()
                                command.linear.x = 0.1
                                publisher.publish(command)
                                next_command = now + 0.05
                            rclpy.spin_once(node, timeout_sec=0.003)
                            while True:
                                try:
                                    payload = connection.recv(256)
                                except BlockingIOError:
                                    break
                                assert payload, "adapter disconnected"
                                assert len(payload) == 24
                                commands.append((time.monotonic(), struct.unpack("<BBiii10s", payload)))
                            if process.poll() is not None:
                                log.seek(0)
                                raise AssertionError(log.read())

                    pump(1.0, publish_command=True)
                    assert not odometry and not transforms, "invented odometry before state arrived"
                    assert not any(command[2] for _, command in commands), "motion without odometry"
                    pump(0.5, "valid", True)
                    odometry.clear()
                    transforms.clear()
                    start = time.monotonic()
                    pump(1.0, "valid", True)
                    elapsed = time.monotonic() - start
                    rate = len(odometry) / elapsed
                    assert 35 <= rate <= 65, f"unexpected output frequency: {rate:.1f} Hz"
                    assert any(command[2] == 1000 for _, command in commands)
                    assert all(msg.header.frame_id == "g1_test_odom" and msg.child_frame_id == "g1_test_base"
                               and msg.pose.pose.position.z == 0 and msg.pose.pose.orientation.w == 1
                               for msg in odometry)
                    stamps = [(msg.header.stamp.sec, msg.header.stamp.nanosec) for msg in odometry]
                    tf_stamps = {(tf.header.stamp.sec, tf.header.stamp.nanosec)
                                 for msg in transforms for tf in msg.transforms
                                 if tf.child_frame_id == "g1_test_base"}
                    assert len(stamps) == len(set(stamps)) and stamps == sorted(stamps)
                    assert set(stamps[:-2]).issubset(tf_stamps), "odom/TF reception stamps differ"
                    for mode in ("duplicate", None, "stale", "error"):
                        pump(0.3, mode, True)
                        count = len(odometry)
                        tf_count = len(transforms)
                        command_start = len(commands)
                        pump(0.3, mode, True)
                        assert len(odometry) == count and len(transforms) == tf_count, mode
                        assert not any(command[2] for _, command in commands[command_start:]), mode
                        pump(0.3, "valid", True)
                        assert len(odometry) > count, f"recovery failed after {mode}"
                    connection.send(b"malformed")
                    pump(0.1, "valid", True)
                    pump(0.25, "jump", True)
                    count = len(odometry)
                    command_start = len(commands)
                    pump(0.3, "valid", True)
                    assert len(odometry) == count, "pose jump fault did not latch"
                    assert not any(command[2] for _, command in commands[command_start:])
                    print(f"PASS: {rate:.1f} Hz real-state odom/TF, duplicate/stale/error guards, jump latch")
    finally:
        if connection is not None:
            connection.close()
        if process is not None:
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
