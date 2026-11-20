"""Check neutral joint coverage and TF using a fake IPC peer, without SDK access.

Run with ROS sourced, an isolated ROS_DOMAIN_ID, and adapter/URDF paths as args.
"""
import os
import math
from pathlib import Path
import socket
import struct
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
    if os.environ.get('ROS_DOMAIN_ID') != '217' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Use isolated ROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1')
    adapter, urdf = sys.argv[1:]
    root = ET.parse(urdf).getroot()
    pelvis_origin = root.find("./joint[@name='base_footprint_to_base_link']/origin")
    pelvis_height = float(pelvis_origin.get('xyz').split()[2])
    expected = {j.get('name') for j in root.findall('joint') if j.get('type') != 'fixed'}
    links = {link.get('name') for link in root.findall('link')} - {'base_link'}
    processes = []
    connection = None
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
                    peer.settimeout(5)
                    connection, _ = peer.accept()
                    connection.setblocking(False)
                    sequence = 0
                    deadline = time.monotonic() + 20
                    missing = links
                    while time.monotonic() < deadline:
                        sequence += 1
                        packet = struct.pack('<BBHIQQQ3f4f3ff', 3, 1, 0, 0, sequence,
                                             time.time_ns(), time.monotonic_ns(),
                                             0, 0, 0.73, 1, 0, 0, 0, 0, 0, 0, 0)
                        connection.send(packet)
                        rclpy.spin_once(node, timeout_sec=0.1)
                        missing = {link for link in links if not buffer.can_transform('base_link', link, rclpy.time.Time())}
                        if not missing and received == expected and buffer.can_transform('odom', 'base_footprint', rclpy.time.Time()):
                            floor = buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
                            pelvis = buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
                            assert abs(floor.transform.translation.z) < 1e-6
                            assert abs(pelvis.transform.translation.z - pelvis_height) < 1e-6
                            upright = buffer.lookup_transform('base_footprint', 'mid360_upright', rclpy.time.Time())
                            q = upright.transform.rotation
                            assert all(math.isfinite(value) for value in (q.x, q.y, q.z, q.w))
                            assert abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) < 1e-6
                            print(f'PASS: {len(expected)} joints and {len(links)} link transforms')
                            connection.close()
                            return
                        if any(process.poll() is not None for process in processes):
                            break
                    log.seek(0)
                    raise AssertionError(f'Missing joints: {expected - received}; missing TF: {missing}\n{log.read()}')
    finally:
        if connection is not None:
            connection.close()
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
