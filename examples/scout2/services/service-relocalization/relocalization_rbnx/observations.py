"""Validate stationary ROS observations without starting ROS or the matcher."""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class Observation:
    """One capture window, points and sensor origin expressed in base coordinates."""

    points: np.ndarray
    sensor_origin: tuple[float, float, float]
    start_stamp_ns: int
    end_stamp_ns: int


class ObservationUnavailable(RuntimeError):
    """More odometry is needed to bracket a capture interval."""


@dataclass(frozen=True)
class CaptureOptions:
    """Internal limits; duration and stamp convention require driver verification."""

    scan_duration_ns: int
    header_is_end: bool
    max_age_s: float = 0.5
    max_odom_gap_s: float = 0.1
    stationary_window_s: float = 0.5
    max_displacement_m: float = 0.01
    max_rotation_rad: float = 0.01
    max_linear_speed_mps: float = 0.02
    max_angular_speed_rps: float = 0.02
    min_range_m: float = 0.5
    max_range_m: float = 50.0
    voxel_size_m: float = 0.05

    def validate(self):
        """Bound time, geometry and stationary thresholds before accepting samples."""
        if (type(self.scan_duration_ns) is not int or not 0 < self.scan_duration_ns <= 500_000_000 or
                type(self.header_is_end) is not bool):
            raise ValueError('explicit scan duration and timestamp convention are required')
        for name, value in vars(self).items():
            if name in ('scan_duration_ns', 'header_is_end'):
                continue
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'invalid capture option: {name}')
        if (self.max_age_s > 2 or self.max_odom_gap_s > 0.2 or
                not 0.2 <= self.stationary_window_s <= 2 or
                self.max_displacement_m > 0.05 or self.max_rotation_rad > 0.05 or
                self.max_linear_speed_mps > 0.1 or self.max_angular_speed_rps > 0.1 or
                not 0.01 <= self.voxel_size_m <= 0.2 or
                not self.min_range_m < self.max_range_m <= 100):
            raise ValueError('capture limits exceed supported stationary scan bounds')


def stamp_ns(stamp) -> int:
    """Reject zero, negative or malformed ROS time instead of using arrival time."""
    if (type(stamp.sec) is not int or type(stamp.nanosec) is not int or
            not 0 <= stamp.nanosec < 1_000_000_000 or stamp.sec < 0):
        raise ValueError('invalid ROS timestamp')
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    if not 0 < value < 2**63:
        raise ValueError('ROS acquisition timestamp must be positive')
    return value


def cloud_xyz(message) -> np.ndarray:
    """Decode bounded PointCloud2 XYZ, respecting endian, field and row padding.

    Float32/64 XYZ are supported; invalid returns are filtered after decoding.
    Output owns its memory, regardless of message data ownership or is_dense.
    """
    count = message.width * message.height
    if (not 3 <= count <= 1_000_000 or message.point_step <= 0 or
            message.row_step < message.width * message.point_step or
            len(message.data) != message.row_step * message.height or len(message.data) > 64_000_000):
        raise ValueError('invalid or oversized PointCloud2 layout')
    names = [field.name for field in message.fields]
    if len(set(names)) != len(names) or not {'x', 'y', 'z'}.issubset(names):
        raise ValueError('PointCloud2 must have unique x/y/z fields')
    fields = {field.name: field for field in message.fields}
    columns, ranges = [], []
    for name in ('x', 'y', 'z'):
        field = fields[name]
        if field.datatype not in (7, 8) or field.count != 1:
            raise ValueError('PointCloud2 XYZ must be scalar float32 or float64')
        size = 4 if field.datatype == 7 else 8
        if field.offset < 0 or field.offset + size > message.point_step:
            raise ValueError('PointCloud2 XYZ field extends beyond point stride')
        ranges.append(set(range(field.offset, field.offset + size)))
        dtype = np.dtype(('>' if message.is_bigendian else '<') + f'f{size}')
        values = np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
                            offset=field.offset, strides=(message.row_step, message.point_step))
        columns.append(values.reshape(-1).astype(float))
    if any(ranges[i] & ranges[j] for i in range(3) for j in range(i)):
        raise ValueError('PointCloud2 XYZ fields overlap')
    points = np.column_stack(columns)
    return points[np.isfinite(points).all(axis=1)]


def rotation(quaternion) -> np.ndarray:
    """Require a normalized finite xyzw quaternion and return its rotation."""
    q = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w], dtype=float)
    if not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > 1e-3:
        raise ValueError('invalid or nonunit orientation quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def transform_matrix(message, target: str, source: str) -> np.ndarray:
    """Validate the exact requested frame pair and finite rigid TF geometry."""
    if message.header.frame_id != target or message.child_frame_id != source:
        raise ValueError('TF frame identity differs from requested transform')
    transform = message.transform
    matrix = np.eye(4)
    matrix[:3, :3] = rotation(transform.rotation)
    matrix[:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]
    if not np.isfinite(matrix).all() or np.max(np.abs(matrix[:3, 3])) > 10:
        raise ValueError('invalid sensor extrinsic translation')
    return matrix


def _odom_geometry(message, base_frame, odom_frame):
    """Extract trusted frame identities, finite pose and complete 3D speed vectors."""
    if message.header.frame_id != odom_frame or message.child_frame_id != base_frame:
        raise ValueError('odometry frame identity mismatch')
    pose, twist = message.pose.pose, message.twist.twist
    position = np.array([pose.position.x, pose.position.y, pose.position.z])
    linear = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
    angular = np.array([twist.angular.x, twist.angular.y, twist.angular.z])
    if not all(np.isfinite(value).all() for value in (position, linear, angular)):
        raise ValueError('nonfinite odometry pose or speed')
    return position, rotation(pose.orientation), np.linalg.norm(linear), np.linalg.norm(angular)


def check_stationary(messages, start_ns, end_ns, base_frame, odom_frame, options):
    """Require dense bracketing odometry and low speed/total pose excursion.

    Every sample including the gap between scans is checked. A motion or reset
    returning to the starting pose cannot pass via endpoint-only comparison.
    Wheel odometry alone cannot detect slipping or externally carrying a robot.
    """
    stamps = [stamp_ns(message.header.stamp) for message in messages]
    if any(a >= b for a, b in zip(stamps, stamps[1:])):
        raise ValueError('odometry timestamps duplicate or move backwards')
    before = [i for i, stamp in enumerate(stamps) if stamp <= start_ns]
    after = [i for i, stamp in enumerate(stamps) if stamp >= end_ns]
    if not before or not after:
        raise ObservationUnavailable('odometry does not yet bracket the capture interval')
    first, last = before[-1], after[0]
    selected = messages[first:last+1]
    if any(b-a > options.max_odom_gap_s * 1e9 for a, b in zip(stamps[first:last], stamps[first+1:last+1])):
        raise ValueError('odometry has a gap during capture')
    reference_position, reference_rotation, _, _ = _odom_geometry(selected[0], base_frame, odom_frame)
    for message in selected:
        position, attitude, linear, angular = _odom_geometry(message, base_frame, odom_frame)
        angle = math.acos(float(np.clip((np.trace(reference_rotation.T @ attitude)-1)/2, -1, 1)))
        if (np.linalg.norm(position-reference_position) > options.max_displacement_m or
                angle > options.max_rotation_rad or linear > options.max_linear_speed_mps or
                angular > options.max_angular_speed_rps):
            raise ValueError('robot moved or odometry reset during stationary capture')


def prepare_pair(clouds, odometry, lookup_transform, *, base_frame: str, odom_frame: str,
                 expected_base_from_sensor: np.ndarray, now_ns: int, options: CaptureOptions):
    """Prepare two stationary observations using explicit driver timing and TF.

    lookup_transform(target, source, stamp_ns) must query historical time, never
    latest TF. The expected rigid transform must come from the selected map's
    calibrated sensor. No deskew is performed: only stationary capture passes.
    """
    options.validate()
    if type(now_ns) is not int or not 0 < now_ns < 2**63:
        raise ValueError('current ROS time must be a positive integer')
    if not base_frame or not odom_frame or base_frame == odom_frame or len(clouds) != 2:
        raise ValueError('two scans and distinct explicit base/odom frames are required')
    expected = np.array(expected_base_from_sensor, dtype=float, copy=True)
    if (expected.shape != (4, 4) or not np.isfinite(expected).all() or
            not np.allclose(expected[3], [0, 0, 0, 1], atol=1e-8) or
            not np.allclose(expected[:3, :3].T @ expected[:3, :3], np.eye(3), atol=1e-6) or
            not np.isclose(np.linalg.det(expected[:3, :3]), 1, atol=1e-6)):
        raise ValueError('expected sensor extrinsic must be a rigid matrix')
    windows = []
    if clouds[0].header.frame_id != clouds[1].header.frame_id or not clouds[0].header.frame_id:
        raise ValueError('scan sensor frame changed or is empty')
    for cloud in clouds:
        stamp = stamp_ns(cloud.header.stamp)
        start = stamp-options.scan_duration_ns if options.header_is_end else stamp
        end = start+options.scan_duration_ns
        if start <= 0 or end > now_ns or now_ns-end > options.max_age_s * 1e9:
            raise ValueError('scan is stale, future dated or outside valid ROS time')
        windows.append((start, end))
    if windows[1][0] <= windows[0][1] or windows[1][1]-windows[0][0] > 2e9:
        raise ValueError('scan capture windows overlap, repeat or are too far apart')
    guard_start = min(windows[0][0], windows[1][1]-int(options.stationary_window_s*1e9))
    check_stationary(odometry, guard_start, windows[1][1], base_frame, odom_frame, options)
    result = []
    source = clouds[0].header.frame_id
    for cloud, (start, end) in zip(clouds, windows):
        matrices = [transform_matrix(lookup_transform(base_frame, source, stamp), base_frame, source)
                    for stamp in (start, (start+end)//2, end)]
        if any(not np.allclose(matrix, expected, rtol=0, atol=1e-4) for matrix in matrices):
            raise ValueError('historical sensor TF differs from map calibration')
        points = cloud_xyz(cloud)
        ranges = np.linalg.norm(points, axis=1)
        points = points[(ranges >= options.min_range_m) & (ranges <= options.max_range_m)]
        points = points @ expected[:3, :3].T + expected[:3, 3]
        _, first = np.unique(np.floor(points/options.voxel_size_m), axis=0, return_index=True)
        points = np.ascontiguousarray(points[np.sort(first)])
        if not 20 <= len(points) <= 10000:
            raise ValueError('filtered scan must contain 20 to 10000 voxel points')
        points.flags.writeable = False
        result.append(Observation(points, tuple(expected[:3, 3]), start, end))
    return tuple(result)
