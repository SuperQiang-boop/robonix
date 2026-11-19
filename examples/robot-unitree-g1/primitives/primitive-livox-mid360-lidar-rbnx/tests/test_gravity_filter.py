"""Exercise physical tilt, transient acceleration and temporal failure handling."""

import sys
from pathlib import Path
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mid360_driver.gravity_filter import GravityFilter, level_rotation


class GravityFilterTests(unittest.TestCase):
    def initialized(self, up=(0, 0, 1), bias=(0, 0, 0)):
        """Initialize with a stationary sensor and return the last timestamp."""
        estimate = GravityFilter()
        for index in range(241):
            stamp = 10.0 + index * 0.005
            estimate.update(stamp, np.array(up) * 9.80665, bias)
        self.assertIsNotNone(estimate.at(stamp))
        return estimate, stamp

    def test_leveling_rotates_about_sensor_origin_and_keeps_obstacle_height(self):
        """A sloped apparent floor levels while a real 20 cm obstacle survives."""
        up = Rotation.from_euler("xy", [3, -4], degrees=True).apply([0, 0, 1])
        tilt = level_rotation(up)
        origin = np.array([0, 0, 1.27618])
        correct = np.array([[-3, 0, 0], [3, 0, 0], [-2, 0, 0.2]])
        raw = (correct - origin) @ tilt + origin
        aligned = (raw - origin) @ tilt.T + origin
        np.testing.assert_allclose(aligned, correct, atol=1e-12)
        np.testing.assert_allclose(tilt @ up, [0, 0, 1], atol=1e-12)

    def test_static_tilt_and_gyro_bias(self):
        """Stationary tilt and bias do not create orientation drift."""
        up = Rotation.from_euler("xy", [3, -4], degrees=True).apply([0, 0, 1])
        estimate, stamp = self.initialized(up, [0.01, -0.008, 0.003])
        np.testing.assert_allclose(estimate.at(stamp), up, atol=1e-12)
        np.testing.assert_allclose(estimate.bias, [0.01, -0.008, 0.003], atol=1e-12)

    def test_gyro_follows_rotation_when_acceleration_is_rejected(self):
        """Gyro still tracks physical pitch during a non-gravity acceleration burst."""
        estimate, stamp = self.initialized()
        for index in range(1, 101):
            estimate.update(stamp + index * 0.005, [0, 0, 20], [0, 0.2, 0])
        expected = Rotation.from_rotvec([0, -0.1, 0]).apply([0, 0, 1])
        np.testing.assert_allclose(estimate.at(stamp + 0.5), expected, atol=1e-12)

    def test_large_lateral_acceleration_does_not_become_tilt(self):
        """Even a plausible magnitude is rejected when its direction jumps."""
        estimate, stamp = self.initialized()
        for index in range(1, 101):
            estimate.update(stamp + index * 0.005, [2, 0, 9.80665], [0, 0, 0])
        np.testing.assert_allclose(estimate.at(stamp + 0.5), [0, 0, 1], atol=1e-12)

    def test_missing_and_stale_imu_never_supply_orientation(self):
        """No latest-value fallback or interpolation across a data gap is allowed."""
        self.assertIsNone(GravityFilter().at(10.0))
        estimate, stamp = self.initialized()
        self.assertIsNone(estimate.at(stamp + 0.001))
        self.assertIsNone(estimate.at(9.0))
        estimate.update(stamp + 0.2, [0, 0, 9.80665], [0, 0, 0])
        self.assertIsNone(estimate.at(stamp))
        self.assertIsNone(estimate.at(stamp + 0.2))

    def test_timestamp_interpolation_and_late_packet(self):
        """A late packet cannot rewind the filter; bracketing estimates interpolate."""
        estimate, stamp = self.initialized()
        estimate.update(stamp + 0.01, [0, 0, 20], [0, 1, 0])
        estimate.update(stamp + 0.005, [0, 0, 20], [10, 0, 0])
        expected = Rotation.from_rotvec([0, -0.005, 0]).apply([0, 0, 1])
        np.testing.assert_allclose(estimate.at(stamp + 0.005), expected, atol=1e-12)

    def test_initialization_rejects_motion_and_invalid_samples(self):
        """Moving, nonfinite, or incorrectly scaled IMU cannot initialize output."""
        for acceleration, gyro in (([0, 0, 1], [0, 0, 0]),
                                   ([0, 0, 9.80665], [0, 0, 0.2]),
                                   ([0, 0, float('nan')], [0, 0, 0])):
            estimate = GravityFilter()
            for index in range(300):
                estimate.update(index * 0.005, acceleration, gyro)
            self.assertIsNone(estimate.at(1.0))

    def test_clock_rewind_requires_reinitialization(self):
        estimate, stamp = self.initialized()
        estimate.update(1.0, [0, 0, 9.80665], [0, 0, 0])
        self.assertIsNone(estimate.at(stamp))


if __name__ == "__main__":
    unittest.main()
