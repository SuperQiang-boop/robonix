"""Expose a gravity-corrected cloud via its own TF; leave raw data untouched."""

import argparse
from collections import deque
import copy
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu, PointCloud2
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from .gravity_filter import GravityFilter, level_rotation


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def rotation(transform):
    q = transform.rotation
    return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()


class GravityAlignment(Node):
    """Publish original point bytes under a distinct, timestamped aligned frame.

    The new frame has the nominal sensor origin and a gravity-corrected
    orientation in target_frame. Raw sensor/robot TF edges remain untouched,
    and consumers retain the lidar viewpoint needed for range/ray tracing.
    """

    def __init__(self, config):
        """Validate configuration and subscribe without commanding the robot."""
        super().__init__("mid360_gravity_alignment")
        self.target = str(config.get("target_frame", "base_footprint"))
        self.output_frame = str(config.get("output_frame", "mid360_gravity_aligned"))
        input_topic = str(config.get("input_topic", "/scanner/cloud"))
        output_topic = str(config.get("output_topic", "/scanner/cloud_aligned"))
        self.scale = float(config.get("acceleration_scale", 9.80665))
        angles = [float(config.get("imu_roll_offset_deg", 0.0)),
                  float(config.get("imu_pitch_offset_deg", 0.0))]
        if input_topic == output_topic or self.target == self.output_frame:
            raise ValueError("aligned topic/frame must be distinct from their input/parent")
        if not all(math.isfinite(value) for value in [self.scale, *angles]) or self.scale <= 0:
            raise ValueError("acceleration_scale must be positive; calibration must be finite")
        self.calibration = Rotation.from_euler("xy", angles, degrees=True).as_matrix()
        self.filter = GravityFilter()
        self.pending = deque()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.publisher = self.create_publisher(PointCloud2, output_topic, qos_profile_sensor_data)
        self.create_subscription(Imu, config.get("imu_topic", "/livox/imu"),
                                 self.on_imu, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, input_topic, self.on_cloud, qos_profile_sensor_data)
        self.create_timer(0.02, self.flush)
        self.published = 0
        self.get_logger().info(f"{input_topic} -> {output_topic}; hold still for IMU initialization")

    def on_imu(self, message):
        """Transform both IMU vectors consistently before fusion in nominal base axes."""
        if message.header.frame_id == self.output_frame:
            raise ValueError("aligned frame cannot also be the raw IMU frame")
        try:
            tf = self.buffer.lookup_transform(self.target, message.header.frame_id,
                                             rclpy.time.Time.from_msg(message.header.stamp))
        except TransformException as exc:
            self.get_logger().warning(f"IMU TF unavailable: {exc}", throttle_duration_sec=5.0)
            return
        matrix = self.calibration @ rotation(tf.transform)
        a, w = message.linear_acceleration, message.angular_velocity
        self.filter.update(seconds(message.header.stamp),
                           matrix @ [a.x, a.y, a.z] * self.scale,
                           matrix @ [w.x, w.y, w.z])

    def on_cloud(self, message):
        """Queue clouds briefly to wait for IMU bracketing their exact timestamp."""
        if message.header.frame_id in (self.output_frame, self.target):
            raise ValueError("aligned frame must be distinct; input needs the sensor viewpoint")
        if len(self.pending) >= 5:
            self.pending.popleft()
            self.get_logger().warning("dropping cloud: alignment queue full", throttle_duration_sec=5.0)
        self.pending.append((time.monotonic(), message))

    def flush(self):
        """Publish matching TF/cloud pairs, or visibly drop data after 0.25 s.

        No uncorrected fallback is published when IMU, initialization or TF is
        unavailable. The cloud layout, every field and acquisition stamp survive.
        """
        while self.pending:
            arrived, message = self.pending[0]
            up = self.filter.at(seconds(message.header.stamp))
            try:
                if up is None:
                    raise ValueError("no initialized, timestamp-matched IMU")
                tf = self.buffer.lookup_transform(self.target, message.header.frame_id,
                                                 rclpy.time.Time.from_msg(message.header.stamp))
                corrected = level_rotation(up) @ rotation(tf.transform)
            except (TransformException, ValueError) as exc:
                if time.monotonic() - arrived < 0.25:
                    return
                self.pending.popleft()
                self.get_logger().warning(f"dropping cloud: {exc}", throttle_duration_sec=5.0)
                continue
            self.pending.popleft()
            aligned_tf = TransformStamped()
            aligned_tf.header.stamp = message.header.stamp
            aligned_tf.header.frame_id = self.target
            aligned_tf.child_frame_id = self.output_frame
            aligned_tf.transform.translation = copy.deepcopy(tf.transform.translation)
            q = Rotation.from_matrix(corrected).as_quat()
            (aligned_tf.transform.rotation.x, aligned_tf.transform.rotation.y,
             aligned_tf.transform.rotation.z, aligned_tf.transform.rotation.w) = map(float, q)
            self.broadcaster.sendTransform(aligned_tf)
            output = copy.deepcopy(message)
            output.header.frame_id = self.output_frame
            self.publisher.publish(output)
            self.published += 1
            if self.published == 1:
                self.get_logger().info("IMU initialized; publishing gravity-aligned clouds")


def main():
    """Run the optional child process; teardown all ROS resources on shutdown."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config-json", required=True)
    args = parser.parse_args()
    rclpy.init()
    node = None
    try:
        node = GravityAlignment(json.loads(args.config_json))
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
