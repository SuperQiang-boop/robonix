"""Isolated ROS smoke test; run in a network-isolated container, never on robot DDS.

Publishes synthetic inputs and sends commands only to /robonix/nomotion/cmd_vel.
Example container invocation is documented in README.md.
"""
import math
import os
import tempfile
import time
from pathlib import Path

# Refuse to run with the deployment's normal ROS domain or velocity topic.
if os.environ.get('ROS_DOMAIN_ID') != '191':
    raise RuntimeError('this test requires isolated ROS_DOMAIN_ID=191')
if os.environ.get('ROBONIX_VELOCITY_OUTPUT_TOPIC') != '/robonix/nomotion/cmd_vel':
    raise RuntimeError('this test requires the non-motion output topic')

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from nav2_wrapper.velocity_guard import VelocityGuardNode
from relocalization_rbnx.coordination import MapWriter, read_record, write_record
from relocalization_rbnx.navigation import mapping_navigation_ready


def run():
    """Exercise the actual ROS guard with synthetic sensors and a temporary map owner."""
    with tempfile.TemporaryDirectory() as temporary:
        directory = Path(temporary)
        os.environ.update(
            RELOCALIZATION_COORDINATION_DIR=str(directory),
            ROBONIX_NAV_MAX_LINEAR_SPEED_MPS='0.2',
            ROBONIX_NAV_DEFAULT_SPEED_PERCENTAGE='75',
            ROBONIX_NAV_GUARD_SPEED_LIMIT_TOPIC='/fixture/speed_limit',
            ROBONIX_RELOCALIZATION_MAX_ANGULAR_RPS='0.15',
            ROBONIX_NAV_GUARD_MAP_TOPIC='/fixture/map',
            ROBONIX_NAV_GUARD_ODOM_TOPIC='/fixture/odom',
            ROBONIX_NAV_GUARD_SCAN_TOPIC='/fixture/scan',
            ROBONIX_NAV_TRACE_DIR=str(directory / 'traces'),
            ROBONIX_NAV_GUARD_USE_SIM_TIME='false',
        )
        writer = MapWriter(directory, 'mapping')
        writer.mapping('fixture-map')
        rclpy.init()
        fixture = Node('mapping_guard_fixture')
        guard = VelocityGuardNode()
        executor = SingleThreadedExecutor()
        executor.add_node(fixture)
        executor.add_node(guard)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        map_pub = fixture.create_publisher(OccupancyGrid, '/fixture/map', qos)
        costmap_pub = fixture.create_publisher(OccupancyGrid, '/global_costmap/costmap', qos)
        odom_pub = fixture.create_publisher(Odometry, '/fixture/odom', qos_profile_sensor_data)
        scan_pub = fixture.create_publisher(LaserScan, '/fixture/scan', qos_profile_sensor_data)
        cmd_pub = fixture.create_publisher(Twist, '/cmd_vel_guard_input', 10)
        outputs = []
        output_sub = fixture.create_subscription(Twist, '/robonix/nomotion/cmd_vel',
            lambda msg: outputs.append((msg.linear.x, msg.angular.z)), 10)
        tf = TransformBroadcaster(fixture)
        static_tf = StaticTransformBroadcaster(fixture)
        sensor_tf = TransformStamped()
        sensor_tf.header.frame_id = 'base_link'
        sensor_tf.child_frame_id = 'fixture_lidar'
        sensor_tf.transform.rotation.w = 1.0
        static_tf.sendTransform(sensor_tf)
        occupancy = OccupancyGrid()
        occupancy.header.frame_id = 'map'
        occupancy.info.width = occupancy.info.height = 10
        occupancy.info.resolution = .05
        occupancy.info.origin.orientation.w = 1.0
        occupancy.data = [0] * 100
        map_pub.publish(occupancy)

        def tick(duration, scan=True, transform=True, command=True):
            """Publish selected synthetic inputs and spin both isolated nodes."""
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                stamp = fixture.get_clock().now().to_msg()
                odom = Odometry()
                odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_link'
                odom.pose.pose.orientation.w = 1.0
                odom_pub.publish(odom)
                costmap = OccupancyGrid()
                costmap.header.stamp, costmap.header.frame_id = stamp, 'map'
                costmap.info = occupancy.info
                costmap.data = occupancy.data
                costmap_pub.publish(costmap)
                if scan:
                    lidar = LaserScan()
                    lidar.header.stamp, lidar.header.frame_id = stamp, 'fixture_lidar'
                    lidar.angle_min, lidar.angle_increment = -math.pi, math.pi / 36
                    lidar.range_min, lidar.range_max = .1, 12.
                    lidar.ranges = [2.] * 72
                    scan_pub.publish(lidar)
                if transform:
                    pose = TransformStamped()
                    pose.header.stamp, pose.header.frame_id, pose.child_frame_id = stamp, 'map', 'base_link'
                    pose.transform.rotation.w = 1.
                    tf.sendTransform(pose)
                if command:
                    cmd = Twist()
                    cmd.linear.x, cmd.angular.z = .5, .3
                    cmd_pub.publish(cmd)
                for _ in range(8):
                    executor.spin_once(timeout_sec=.002)

        try:
            tick(1.)
            assert mapping_navigation_ready(directory), 'healthy mapping did not become ready'
            assert outputs and any(x > 0 for x, _ in outputs), 'healthy commands did not pass'
            assert all(abs(x) <= .150001 and abs(w) <= .150001 for x, w in outputs), 'speed bound violated'
            print('PASS healthy mapping forwards bounded commands without a saved map')
            tick(.8, scan=False)
            assert outputs[-1] == (0., 0.), 'lost scan did not stop output'
            assert not mapping_navigation_ready(directory), 'lost scan left admission ready'
            print('PASS missing scan blocks output and admission')
            tick(.3)
            assert mapping_navigation_ready(directory), 'restored scan did not recover'
            tick(.8, transform=False)
            assert outputs[-1] == (0., 0.), 'stale TF did not stop output'
            print('PASS stale TF blocks output')
            tick(.3)
            writer.invalidate()
            tick(.2)
            assert outputs[-1] == (0., 0.), 'map transition did not stop output'
            assert read_record(directory / 'navigation.json')['stopped'], 'stationary transition was not acknowledged'
            writer.mapping('fixture-map')
            tick(.2)
            assert not mapping_navigation_ready(directory), 'old occupancy survived map reload'
            map_pub.publish(occupancy)
            tick(.4)
            assert mapping_navigation_ready(directory), 'new occupancy did not recover mapping'
            print('PASS map reload revokes old readiness until occupancy arrives')
            identity = dict(map_id='fixture-map', generation=1, database_sha256='a'*64)
            writer.loaded(identity)
            tick(.2)
            assert outputs[-1] == (0., 0.), 'unverified localization forwarded output'
            state = read_record(directory / 'mapping.json')
            write_record(directory / 'status.json', dict(identity,
                provider_id='relocalization', active=True, busy=False, localization_valid=True,
                mapping_process_id=state['process_id'], load_id=state['load_id']))
            tick(.15)
            assert outputs[-1][0] > 0, 'verified localization did not forward output'
            tick(.5)
            assert outputs[-1] == (0., 0.), 'expired localization did not stop output'
            print('PASS localization verification and expiry remain enforced')
        finally:
            executor.shutdown()
            guard.destroy_node()
            fixture.destroy_node()
            rclpy.shutdown()
            writer.close()


if __name__ == '__main__':
    run()
