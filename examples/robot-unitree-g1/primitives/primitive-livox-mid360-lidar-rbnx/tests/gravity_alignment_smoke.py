"""ROS process smoke test. Run only in an isolated, nonzero ROS_DOMAIN_ID."""

import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu, PointCloud2, PointField
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mid360_driver.gravity_filter import level_rotation


def make_transform(child, matrix, translation):
    """Make distinct sensor mount TFs to exercise IMU/cloud frame conversion."""
    transform = TransformStamped()
    transform.header.frame_id = "base_footprint"
    transform.child_frame_id = child
    transform.transform.translation.x = float(translation[0])
    transform.transform.translation.y = float(translation[1])
    transform.transform.translation.z = float(translation[2])
    q = Rotation.from_matrix(matrix).as_quat()
    (transform.transform.rotation.x, transform.transform.rotation.y,
     transform.transform.rotation.z, transform.transform.rotation.w) = map(float, q)
    return transform


def main():
    """Check startup, TF geometry, byte preservation, IMU loss and recovery."""
    if not 1 <= int(os.environ.get("ROS_DOMAIN_ID", "0")) <= 232:
        raise RuntimeError("set an isolated ROS_DOMAIN_ID, e.g. 217; never use the robot domain")
    rclpy.init()
    node = rclpy.create_node("gravity_alignment_smoke")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    broadcaster = StaticTransformBroadcaster(node)
    nominal = Rotation.from_euler("x", -0.00248079759).as_matrix()
    imu_mount = Rotation.from_euler("xyz", [2, 4, 30], degrees=True).as_matrix()
    origin = np.array([0, 0, 1.27618])
    broadcaster.sendTransform([make_transform("test_lidar", nominal, origin),
                               make_transform("test_imu", imu_mount, origin)])
    cloud_pub = node.create_publisher(PointCloud2, "/test/raw", qos_profile_sensor_data)
    imu_pub = node.create_publisher(Imu, "/test/imu", qos_profile_sensor_data)
    received = []
    subscription = node.create_subscription(PointCloud2, "/test/aligned", received.append,
                                             qos_profile_sensor_data)
    calibration = Rotation.from_euler("xy", [0.4265856886, 1.1663485205], degrees=True).as_matrix()
    up = Rotation.from_euler("xy", [1.5, -4], degrees=True).apply([0, 0, 1])
    acceleration = imu_mount.T @ calibration.T @ up
    expected = np.array([[-3, -1, 0], [-3, 1, 0], [3, -1, 0],
                         [3, 1, 0], [2, 0, 0.20], [0, 2, 0.30]])
    raw_xyz = (expected - origin) @ level_rotation(up) @ nominal
    # Organized, big-endian data with row padding and unrelated metadata bytes.
    cloud = PointCloud2()
    cloud.header.frame_id = "test_lidar"
    cloud.height, cloud.width, cloud.point_step, cloud.row_step = 2, 3, 24, 80
    cloud.is_bigendian, cloud.is_dense = True, True
    cloud.fields = [PointField(name=name, offset=index * 4, datatype=7, count=1)
                    for index, name in enumerate(("x", "y", "z", "intensity"))]
    cloud.fields += [PointField(name="timestamp", offset=16, datatype=8, count=1)]
    payload = bytearray((index % 251 for index in range(cloud.row_step * cloud.height)))
    dtype = np.dtype({"names": ["x", "y", "z"], "formats": [">f4"] * 3,
                      "offsets": [0, 4, 8], "itemsize": 24})
    view = np.ndarray((2, 3), dtype=dtype, buffer=payload, strides=(80, 24))
    for index, name in enumerate(("x", "y", "z")):
        view[name] = raw_xyz[:, index].reshape(2, 3)
    cloud.data = bytes(payload)
    config = {"input_topic": "/test/raw", "output_topic": "/test/aligned",
              "imu_topic": "/test/imu", "output_frame": "test_aligned",
              "imu_roll_offset_deg": 0.4265856886, "imu_pitch_offset_deg": 1.1663485205}
    package = Path(__file__).resolve().parents[1]
    sent = {}
    with tempfile.TemporaryFile(mode="w+") as logs:
        process = subprocess.Popen([sys.executable, "-m", "mid360_driver.gravity_alignment",
                                    "--config-json", json.dumps(config)], cwd=package,
                                   stdout=logs, stderr=subprocess.STDOUT)
        try:
            started, next_cloud = time.monotonic(), 0.0
            start_ros = node.get_clock().now().nanoseconds * 1e-9
            checked = 0
            while time.monotonic() - started < 11.0:
                elapsed = time.monotonic() - started
                stamp = node.get_clock().now().to_msg()
                # Lose IMU for one second, then require stationary reinitialization.
                if not 5.0 <= elapsed < 6.0:
                    imu = Imu()
                    imu.header.stamp, imu.header.frame_id = stamp, "test_imu"
                    (imu.linear_acceleration.x, imu.linear_acceleration.y,
                     imu.linear_acceleration.z) = map(float, acceleration)
                    imu_pub.publish(imu)
                if elapsed >= next_cloud:
                    cloud.header.stamp = stamp
                    sent[(stamp.sec, stamp.nanosec)] = copy.deepcopy(cloud)
                    cloud_pub.publish(cloud)
                    next_cloud = elapsed + 0.1
                rclpy.spin_once(node, timeout_sec=0.005)
                for output in received[checked:]:
                    source = sent[(output.header.stamp.sec, output.header.stamp.nanosec)]
                    self_stamp = output.header.stamp
                    # The input message must remain in its original frame.
                    assert source.header.frame_id == "test_lidar"
                    assert output.header.frame_id == "test_aligned"
                    assert bytes(output.data) == bytes(source.data)
                    assert output.fields == source.fields
                    for attr in ("height", "width", "point_step", "row_step", "is_bigendian", "is_dense"):
                        assert getattr(output, attr) == getattr(source, attr)
                    try:
                        tf = buffer.lookup_transform("base_footprint", "test_aligned",
                                                     rclpy.time.Time.from_msg(self_stamp))
                    except TransformException:
                        break  # The TF transport can arrive after its matching cloud.
                    q, t = tf.transform.rotation, tf.transform.translation
                    matrix = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
                    actual = raw_xyz @ matrix.T + [t.x, t.y, t.z]
                    np.testing.assert_allclose(actual, expected, atol=1e-7)
                    assert np.count_nonzero(actual[:, 2] >= 0.15) == 2
                    # No acquisition within the outage's middle can be published.
                    relative = self_stamp.sec + self_stamp.nanosec * 1e-9 - start_ros
                    assert not 5.2 < relative < 6.0
                    checked += 1
                if process.poll() is not None:
                    raise RuntimeError(f"alignment child exited: {process.returncode}")
            assert checked > 30, f"only {checked} output clouds validated"
            assert received[-1].header.stamp.sec + received[-1].header.stamp.nanosec * 1e-9 - start_ros > 9
            print(f"PASS: {checked} clouds; TF leveling, 20/30 cm obstacles, all bytes/fields, IMU loss/recovery")
        finally:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
            logs.seek(0)
            print(logs.read())
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
