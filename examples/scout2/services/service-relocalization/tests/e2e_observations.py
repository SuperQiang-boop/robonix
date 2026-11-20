"""Isolated real ROS sensor transport; no Atlas, map writes or robot connection."""
from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path
import random
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from relocalization_rbnx.observations import CaptureOptions
from relocalization_rbnx.ros_observations import ObservationCollector


class RosObservationTests(unittest.TestCase):
    def setUp(self):
        """Create owned publishers and a separately spun collector in a private domain."""
        self.temp = tempfile.TemporaryDirectory(prefix='relocalization-observations-')
        self.addCleanup(self.temp.cleanup)
        self.environment = patch.dict(os.environ, ROS_DOMAIN_ID=str(random.randint(101, 200)),
            ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
            ROS_LOG_DIR=str(Path(self.temp.name)/'ros-logs'))
        self.environment.start()
        self.addCleanup(self.environment.stop)
        self.context = Context()
        rclpy.init(args=[], context=self.context)
        self.addCleanup(self.context.shutdown)
        self.input = rclpy.create_node('observation_fixture', context=self.context)
        self.addCleanup(self.input.destroy_node)
        self.node = rclpy.create_node('observation_collector', context=self.context)
        self.addCleanup(self.node.destroy_node)
        self.original_services = list(self.node.services)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.input)
        self.executor.add_node(self.node)
        self.addCleanup(self.executor.shutdown)
        self.cloud_pub = self.input.create_publisher(PointCloud2, '/fixture/cloud', qos_profile_sensor_data)
        self.odom_pub = self.input.create_publisher(Odometry, '/fixture/odom', qos_profile_sensor_data)
        self.tf_pub = StaticTransformBroadcaster(self.input)
        self.expected = np.eye(4)
        self.expected[0, 3] = 0.25
        self.options = CaptureOptions(scan_duration_ns=50_000_000, header_is_end=True)
        self.collector = self.new_collector()
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.token = SimpleNamespace(cancelled=False)
        self.addCleanup(self.close_capture)
        self.points = np.column_stack([np.linspace(1, 5, 40), np.zeros(40), np.ones(40)])
        self.sensor_frame = 'lidar'
        self.last_odom = None
        self.odom_timer = self.input.create_timer(0.02, self.publish_odom)
        self.cloud_timer = self.input.create_timer(0.12, self.publish_cloud)
        self.publish_tf()
        self.spin_until(lambda: self.cloud_pub.get_subscription_count() == 1 and
                        self.odom_pub.get_subscription_count() == 1)
        self.spin_for(0.65)

    def new_collector(self):
        return ObservationCollector(self.node, '/fixture/cloud', '/fixture/odom',
            base_frame='base_link', odom_frame='odom',
            expected_base_from_sensor=self.expected, options=self.options)

    def close_capture(self):
        """Wake and join owned workers before destroying executor, nodes or context."""
        if hasattr(self.token, 'cancel'):
            self.token.cancel()
        else:
            self.token.cancelled = True
        try:
            self.collector.close()
        finally:
            self.pool.shutdown(wait=True)

    def spin_until(self, predicate, timeout=4):
        """Bound every transport wait and keep executor work on the calling thread."""
        deadline = time.monotonic()+timeout
        while not predicate() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)
        self.assertTrue(predicate(), 'ROS fixture wait exceeded its deadline')

    def spin_for(self, seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)

    def publish_tf(self):
        """Transmit calibrated static geometry through the real /tf_static topic."""
        message = TransformStamped()
        message.header.stamp = self.input.get_clock().now().to_msg()
        message.header.frame_id, message.child_frame_id = 'base_link', self.sensor_frame
        message.transform.rotation.w = 1.0
        message.transform.translation.x = 0.25
        self.tf_pub.sendTransform(message)

    def publish_odom(self):
        """Emit dense stationary poses with true acquisition time and frame identity."""
        message = Odometry()
        message.header.stamp = self.input.get_clock().now().to_msg()
        message.header.frame_id, message.child_frame_id = 'odom', 'base_link'
        message.pose.pose.orientation.w = 1.0
        message.pose.pose.position.x = getattr(self, 'motion_x', 0.0)
        self.last_odom = message
        self.odom_pub.publish(message)

    def publish_cloud(self):
        """Emit float32 XYZ with an explicitly end-stamped synthetic scan window."""
        message = PointCloud2()
        message.header.stamp = self.input.get_clock().now().to_msg()
        message.header.frame_id = self.sensor_frame
        message.height, message.width = 1, len(self.points)
        message.point_step, message.row_step = 12, len(self.points)*12
        message.fields = [PointField(name=name, offset=index*4, datatype=PointField.FLOAT32, count=1)
                          for index, name in enumerate(('x', 'y', 'z'))]
        message.data = self.points.astype('<f4').tobytes()
        self.cloud_pub.publish(message)

    def start_capture(self, timeout=2):
        return self.pool.submit(self.collector.collect, timeout, self.token)

    def test_receives_two_independent_scans_and_releases_resources(self):
        """Verify middleware delivery, calibration and node reuse after closing twice."""
        future = self.start_capture()
        self.spin_until(future.done)
        fit, validation = future.result()
        self.assertLess(fit.end_stamp_ns, validation.start_stamp_ns)
        np.testing.assert_allclose(fit.points, self.points+[0.25, 0, 0])
        self.assertEqual(fit.sensor_origin, (0.25, 0, 0))
        self.assertFalse(fit.points.flags.writeable)
        self.collector.close()
        self.collector.close()
        self.assertEqual(list(self.node.subscriptions), [])
        self.assertEqual(list(self.node.services), self.original_services)
        self.collector = self.new_collector()
        self.assertEqual(len(list(self.node.subscriptions)), 4)

    def test_timeout_and_cancellation_stop_waiting(self):
        """Silence cannot return observations; cancellation promptly wakes capture."""
        self.cloud_timer.cancel()
        future = self.start_capture(0.15)
        self.spin_until(future.done)
        with self.assertRaisesRegex(TimeoutError, 'waiting for two'):
            future.result()
        future = self.start_capture()
        self.token.cancelled = True
        self.spin_until(future.done, timeout=0.5)
        with self.assertRaisesRegex(RuntimeError, 'cancelled'):
            future.result()

    def test_missing_historical_tf_waits_then_recovers(self):
        """A missing TF is retryable; historical calibration arriving later suffices."""
        self.sensor_frame = 'delayed_lidar'
        future = self.start_capture()
        self.spin_for(0.32)
        self.assertFalse(future.done())
        self.publish_tf()
        self.spin_until(future.done)
        self.assertEqual(len(future.result()), 2)

    def test_exact_tf_lookup_waits_for_slightly_late_odometry_transform(self):
        """A transform arriving just after its requested stamp remains queryable."""
        broadcaster = TransformBroadcaster(self.input)

        def publish_odom_tf():
            message = TransformStamped()
            message.header.stamp = self.input.get_clock().now().to_msg()
            message.header.frame_id, message.child_frame_id = 'odom', 'base_link'
            message.transform.rotation.w = 1.0
            broadcaster.sendTransform(message)

        publish_odom_tf()
        timer = self.input.create_timer(0.01, publish_odom_tf)
        target = self.node.get_clock().now().nanoseconds+30_000_000
        future = self.pool.submit(self.collector._lookup, 'odom', 'base_link', target)
        self.spin_until(future.done, timeout=0.5)
        self.assertEqual(future.result().header.frame_id, 'odom')
        self.input.destroy_timer(timer)

    def test_duplicate_odometry_latches_failure(self):
        """A real repeated message is surfaced to capture instead of being ignored."""
        self.odom_timer.cancel()
        self.spin_for(0.04)
        future = self.start_capture()
        self.odom_pub.publish(self.last_odom)
        self.spin_until(future.done)
        with self.assertRaisesRegex(RuntimeError, 'duplicated or moved backwards'):
            future.result()

    def test_expired_buffered_scans_are_replaced_after_tf_and_stream_recover(self):
        """A temporary TF wait/scan pause must reacquire, never accept old scans."""
        self.sensor_frame = 'recovering_lidar'
        future = self.start_capture(3)
        self.spin_for(0.34)
        self.assertFalse(future.done())
        self.cloud_timer.cancel()
        self.spin_for(0.65)
        self.assertFalse(future.done(), 'expired buffered scans must not abort the request')
        recovered = self.node.get_clock().now().nanoseconds
        self.publish_tf()
        self.cloud_timer.reset()
        self.spin_until(future.done)
        observations = future.result()
        self.assertGreater(observations[0].end_stamp_ns, recovered)
        self.assertLess(observations[0].end_stamp_ns, observations[1].start_stamp_ns)

    def test_expired_buffered_scans_timeout_without_a_replacement(self):
        """An expired buffer cannot produce output or extend the request deadline."""
        self.sensor_frame = 'missing_lidar'
        future = self.start_capture(1.2)
        self.spin_for(0.34)
        self.cloud_timer.cancel()
        self.spin_until(future.done)
        with self.assertRaisesRegex(TimeoutError, 'fresh scans'):
            future.result()

    def test_close_wakes_waiter(self):
        """Closing cannot strand a capture thread awaiting new ROS messages."""
        self.cloud_timer.cancel()
        future = self.start_capture()
        self.collector.close()
        self.spin_until(future.done, timeout=0.5)
        with self.assertRaisesRegex(RuntimeError, 'closed'):
            future.result()

    def test_clock_source_change_invalidates_capture(self):
        """Switching ROS time source must fail even between sensor callbacks."""
        from rclpy.parameter import Parameter
        self.cloud_timer.cancel()
        self.odom_timer.cancel()
        future = self.start_capture()
        self.node.set_parameters([Parameter('use_sim_time', value=True)])
        self.spin_until(future.done, timeout=0.5)
        with self.assertRaisesRegex(ValueError, 'ROS clock'):
            future.result()

    def test_malformed_cloud_fails_without_partial_observations(self):
        """Malformed layout sent through DDS cannot become matching input."""
        self.cloud_timer.cancel()
        self.points = self.points[:2]
        self.cloud_timer = self.input.create_timer(0.12, self.publish_cloud)
        future = self.start_capture()
        self.spin_until(future.done)
        with self.assertRaisesRegex(ValueError, 'PointCloud2 layout'):
            future.result()


if __name__ == '__main__':
    unittest.main(verbosity=2)
