#!/usr/bin/env python3
"""Measure residual sensor tilt from stationary IMU; never alter TF or hardware.

Run with ROS Humble sourced while G1 stands upright and stationary on level ground.
This estimates roll/pitch only; gravity cannot calibrate yaw or sensor height.
"""
import math
import time
import statistics
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def main():
    """Collect eight seconds of corrected IMU and reject motion/no-data samples."""
    rclpy.init()
    node = rclpy.create_node('g1_mid360_level_check')
    samples = []
    node.create_subscription(Imu, '/livox/imu', samples.append, qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if len(samples) < 100:
            raise RuntimeError(f'Only {len(samples)} IMU samples; check driver and DDS before calibration')
        if {s.header.frame_id for s in samples} != {'livox_frame'}:
            raise RuntimeError('Expected corrected IMU frame livox_frame')
        vectors = [(s.linear_acceleration.x, s.linear_acceleration.y,
                    s.linear_acceleration.z) for s in samples]
        mean = [statistics.mean(v[i] for v in vectors) for i in range(3)]
        norm = math.sqrt(sum(x*x for x in mean))
        spread = math.sqrt(sum(statistics.pvariance(v[i] for v in vectors) for i in range(3)))
        gyro = max(math.sqrt(sum(getattr(s.angular_velocity, axis)**2 for axis in 'xyz')) for s in samples)
        if not all(math.isfinite(x) for x in (*mean, norm, spread, gyro)) or norm < 0.5:
            raise RuntimeError('Invalid acceleration')
        if spread / norm > 0.03 or gyro > 0.05:
            raise RuntimeError('Robot is moving or vibrating; repeat while stationary')
        # R_y(pitch) R_x(roll) maps measured positive gravity to positive Z.
        roll = math.atan2(mean[1], mean[2])
        pitch = math.atan2(-mean[0], math.hypot(mean[1], mean[2]))
        print(f'{len(samples)} samples; acceleration={mean}; gyro_max={gyro:.5f} rad/s')
        print(f'Residual sensor-to-level rotation: roll={roll:.8f}, pitch={pitch:.8f} rad')
        print('Valid only if body is upright on level ground. Not applied automatically; yaw/height unobservable.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
