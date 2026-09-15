"""Check neutral joint coverage and TF using a fake IPC peer, without SDK access.

Run with ROS sourced, an isolated ROS_DOMAIN_ID, and adapter/URDF paths as args.
"""
import os
import math
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


def main():
    """Run only the ROS adapter and state publisher, and verify every URDF link."""
    adapter, urdf = sys.argv[1:]
    root = ET.parse(urdf).getroot()
    expected = {j.get('name') for j in root.findall('joint') if j.get('type') != 'fixed'}
    links = {link.get('name') for link in root.findall('link')} - {'base_link'}
    processes = []
    rclpy.init()
    node = Node('g1_tf_smoke')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    received = set()

    def on_joints(msg):
        """Validate published positions and collect observed joint names."""
        assert len(msg.name) == len(msg.position) == len(set(msg.name))
        assert all(value == 0.0 for value in msg.position)
        received.update(msg.name)

    subscription = node.create_subscription(JointState, '/joint_states', on_joints, 10)
    try:
        with tempfile.TemporaryDirectory(prefix='g1-tf-') as directory:
            socket_path = str(Path(directory) / 'fake.sock')
            with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as peer:
                peer.bind(socket_path)
                peer.listen(1)
                params = Path(directory) / 'robot.yaml'
                params.write_text('/**:\n  ros__parameters:\n    robot_description: |\n' +
                                  ''.join('      ' + line + '\n' for line in Path(urdf).read_text().splitlines()))
                with open(Path(directory) / 'nodes.log', 'w+') as log:
                    processes.append(subprocess.Popen([adapter], env={**os.environ, 'G1_IPC_SOCKET': socket_path}, stdout=log, stderr=log))
                    processes.append(subprocess.Popen(['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher', '--ros-args', '--params-file', str(params)], stdout=log, stderr=log))
                    deadline = time.monotonic() + 20
                    missing = links
                    while time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.1)
                        missing = {link for link in links if not buffer.can_transform('base_link', link, rclpy.time.Time())}
                        if not missing and received == expected and buffer.can_transform('odom', 'base_footprint', rclpy.time.Time()):
                            floor = buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
                            pelvis = buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
                            assert abs(floor.transform.translation.z) < 1e-6
                            assert abs(pelvis.transform.translation.z - 0.55) < 1e-6
                            upright = buffer.lookup_transform('base_footprint', 'mid360_upright', rclpy.time.Time())
                            assert abs(upright.transform.translation.z - 1.02618) < 1e-6
                            q = upright.transform.rotation
                            expected_w = math.cos(-0.00248080/2) * math.cos(0.66736485/2)
                            assert abs(abs(q.w) - expected_w) < 1e-6
                            print(f'PASS: {len(expected)} joints and {len(links)} link transforms')
                            return
                        if any(process.poll() is not None for process in processes):
                            break
                    log.seek(0)
                    raise AssertionError(f'Missing joints: {expected - received}; missing TF: {missing}\n{log.read()}')
    finally:
        for process in processes:
            process.terminate()
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
