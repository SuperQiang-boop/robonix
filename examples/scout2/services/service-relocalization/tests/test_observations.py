"""PointCloud2, historic TF and stationary odometry validation tests."""
from types import SimpleNamespace as NS
import unittest

import numpy as np

from relocalization_rbnx.observations import (CaptureOptions, ObservationUnavailable,
    check_stationary, cloud_xyz, prepare_pair, transform_matrix)


def stamp(value):
    return NS(sec=value // 1_000_000_000, nanosec=value % 1_000_000_000)


def cloud(points, stamp_ns, *, frame='lidar', big=False, row_padding=0):
    """Pack a padded PointCloud2 stand-in with XYZ float32 fields."""
    points = np.asarray(points, dtype='>f4' if big else '<f4')
    height, width = 1, len(points)
    step, row_step = 16, width * 16 + row_padding
    data = bytearray(row_step)
    for i, point in enumerate(points):
        for j, value in enumerate(point):
            data[i * step + j * 4:i * step + (j + 1) * 4] = np.asarray(value, dtype=points.dtype).tobytes()
    fields = [NS(name=name, offset=index * 4, datatype=7, count=1)
              for index, name in enumerate(('x', 'y', 'z'))]
    return NS(width=width, height=height, point_step=step, row_step=row_step,
        data=bytes(data), fields=fields, is_bigendian=big,
        header=NS(frame_id=frame, stamp=stamp(stamp_ns)))


def odom(stamp_ns, *, x=0, speed=0, frame='odom', child='base_link', yaw=0):
    """Build a finite planar Odometry stand-in with matching frame identity."""
    return NS(header=NS(frame_id=frame, stamp=stamp(stamp_ns)), child_frame_id=child,
        pose=NS(pose=NS(position=NS(x=x, y=0, z=0), orientation=NS(
            x=0, y=0, z=np.sin(yaw/2), w=np.cos(yaw/2)))),
        twist=NS(twist=NS(linear=NS(x=speed, y=0, z=0),
                           angular=NS(x=0, y=0, z=0))))


def tf(stamp_ns, *, x=0, frame='base_link', child='lidar'):
    """Build base_from_lidar TransformStamped stand-in."""
    return NS(header=NS(frame_id=frame, stamp=stamp(stamp_ns)), child_frame_id=child,
        transform=NS(translation=NS(x=x, y=0, z=0),
                     rotation=NS(x=0, y=0, z=0, w=1)))


class ObservationTests(unittest.TestCase):
    def setUp(self):
        """Keep fresh scans inside a densely bracketed stationary interval."""
        self.points = np.column_stack([np.linspace(1, 5, 40), np.zeros(40), np.ones(40)])
        self.start = 1_000_000_000
        self.duration = 100_000_000
        self.options = CaptureOptions(scan_duration_ns=self.duration, header_is_end=True)
        self.clouds = [cloud(self.points, self.start + self.duration, row_padding=4),
                       cloud(self.points, self.start + 400_000_000, big=True)]
        self.odometry = [odom(self.start + offset) for offset in
                         (-100_000_000, 0, 100_000_000, 200_000_000, 300_000_000, 400_000_000, 500_000_000)]
        self.lookup_calls = []

    def lookup(self, target, source, time_ns):
        self.lookup_calls.append((target, source, time_ns))
        return tf(time_ns)

    def prepare(self, clouds=None, odometry=None, **kwargs):
        """Run preparation with explicit calibration and a valid current clock."""
        args = dict(clouds=self.clouds, odometry=self.odometry,
            lookup_transform=self.lookup, base_frame='base_link', odom_frame='odom',
            expected_base_from_sensor=np.eye(4), now_ns=self.start+500_000_000,
            options=self.options)
        args.update(kwargs)
        if clouds is not None:
            args['clouds'] = clouds
        if odometry is not None:
            args['odometry'] = odometry
        return prepare_pair(**args)

    def test_decodes_little_big_endian_and_row_padding(self):
        for value in self.clouds:
            np.testing.assert_allclose(cloud_xyz(value), self.points)

    def test_decodes_organized_float64_and_nonzero_offsets(self):
        """Rows, fields and points can all have independent padding."""
        message = cloud(self.points, self.start)
        message.height, message.width = 2, 20
        message.point_step, message.row_step = 32, 20*32+13
        message.is_bigendian = True
        message.fields = [NS(name=name, offset=4+index*8, datatype=8, count=1)
                          for index, name in enumerate(('x', 'y', 'z'))]
        data = bytearray(message.row_step*message.height)
        for index, point in enumerate(self.points):
            offset = index//20*message.row_step + index%20*32 + 4
            data[offset:offset+24] = point.astype('>f8').tobytes()
        message.data = bytes(data)
        np.testing.assert_array_equal(cloud_xyz(message), self.points)

    def test_start_stamped_scan_and_calibrated_extrinsic(self):
        """Start timestamps define duration; output points use the map extrinsic."""
        expected = np.eye(4)
        expected[0, 3] = 0.25
        pair = [cloud(self.points, self.start), cloud(self.points, self.start+300_000_000)]
        result = self.prepare(clouds=pair, expected_base_from_sensor=expected,
            lookup_transform=lambda target, source, stamp: tf(stamp, x=0.25),
            options=CaptureOptions(scan_duration_ns=self.duration, header_is_end=False))
        np.testing.assert_allclose(result[0].points, self.points+[0.25, 0, 0])
        self.assertEqual(result[0].sensor_origin, (0.25, 0, 0))
        self.assertEqual(result[0].end_stamp_ns, self.start+self.duration)

    def test_rejects_speed_attitude_and_invalid_time(self):
        """Zero net displacement cannot hide speed, rotation or a malformed clock."""
        for replacement in (odom(self.start+200_000_000, speed=0.03),
                            odom(self.start+200_000_000, yaw=0.02)):
            messages = self.odometry.copy()
            messages[3] = replacement
            with self.assertRaisesRegex(ValueError, 'moved'):
                self.prepare(odometry=messages)
        for now in (0, -1, float('nan'), True):
            with self.assertRaisesRegex(ValueError, 'current ROS time'):
                self.prepare(now_ns=now)

    def test_rejects_invalid_layout_fields_and_nonfinite_returns(self):
        """Reject unsafe field access while removing nonfinite sensor returns."""
        invalid = cloud(self.points, self.start)
        invalid.fields[0].offset = invalid.point_step
        with self.assertRaisesRegex(ValueError, 'extends'):
            cloud_xyz(invalid)
        invalid = cloud(self.points, self.start)
        invalid.fields[1].offset = 0
        with self.assertRaisesRegex(ValueError, 'overlap'):
            cloud_xyz(invalid)
        values = cloud(np.vstack([self.points, [np.nan, 0, 0]]), self.start)
        self.assertEqual(len(cloud_xyz(values)), len(self.points))

    def test_builds_two_historically_transformed_stationary_observations(self):
        """Check both capture windows and all six historical TF queries."""
        fit, validation = self.prepare()
        self.assertLess(fit.end_stamp_ns, validation.start_stamp_ns)
        self.assertEqual(fit.points.shape, (40, 3))
        self.assertEqual(validation.end_stamp_ns, self.start+400_000_000)
        self.assertEqual(len(self.lookup_calls), 6)
        self.assertEqual([call[2] for call in self.lookup_calls[:3]],
                         [self.start, self.start+50_000_000, self.start+self.duration])
        self.assertFalse(fit.points.flags.writeable)

    def test_odometry_must_bracket_and_be_dense(self):
        """Missing brackets wait; motion and internal sampling gaps reject."""
        with self.assertRaises(ObservationUnavailable):
            check_stationary(self.odometry[2:], self.start, self.start+400_000_000,
                             'base_link', 'odom', self.options)
        broken = self.odometry.copy()
        broken[3] = odom(self.start+200_000_000, x=0.1)
        with self.assertRaisesRegex(ValueError, 'moved'):
            self.prepare(odometry=broken)
        sparse = self.odometry[:3] + self.odometry[4:]
        with self.assertRaisesRegex(ValueError, 'gap'):
            self.prepare(odometry=sparse)

    def test_rejects_odometry_reset_duplicate_and_frame_mismatch(self):
        """Reject time resets and transforms or odometry in unrelated frames."""
        reset = self.odometry.copy()
        reset[3] = odom(self.start+100_000_000)
        with self.assertRaisesRegex(ValueError, 'backwards'):
            self.prepare(odometry=reset)
        with self.assertRaisesRegex(ValueError, 'frame identity'):
            self.prepare(odometry=[odom(self.start-100_000_000, child='other'), *self.odometry[1:]])
        with self.assertRaisesRegex(ValueError, 'frame identity'):
            transform_matrix(tf(self.start, child='other'), 'base_link', 'lidar')

    def test_rejects_moving_sensor_tf_and_unverified_timing(self):
        """Do not infer timing or accept an extrinsic changing during a scan."""
        def moving(target, source, time_ns):
            return tf(time_ns, x=0.1 if time_ns == self.start+50_000_000 else 0)
        with self.assertRaisesRegex(ValueError, 'TF differs'):
            self.prepare(lookup_transform=moving)
        with self.assertRaisesRegex(ValueError, 'explicit scan duration'):
            self.prepare(options=CaptureOptions(scan_duration_ns=0, header_is_end=False))
        with self.assertRaisesRegex(ValueError, 'overlap'):
            self.prepare(clouds=[self.clouds[0], cloud(self.points,
                self.start+self.duration*2)])

    def test_rejects_stale_future_and_zero_scan_timestamps(self):
        """Acquisition time must be positive and inside the freshness bound."""
        for stamp_ns, now in ((0, self.start+1_000_000_000),
                              (self.start+1_000_000_000, self.start+700_000_000),
                              (self.start, self.start+2_000_000_000)):
            with self.assertRaises(ValueError):
                self.prepare(clouds=[cloud(self.points, stamp_ns), self.clouds[1]], now_ns=now)


if __name__ == '__main__':
    unittest.main(verbosity=2)
