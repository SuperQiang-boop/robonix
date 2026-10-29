"""Read live lidar, odometry and TF diagnostics without motion or pose injection."""
from __future__ import annotations

import argparse
from collections import deque
import json
import math
from pathlib import Path
import threading
import time

import numpy as np

from .profile import read_profile
from .observations import (CaptureOptions, _odom_geometry, cloud_xyz, stamp_ns,
                           transform_matrix)


def bounds(values):
    """Report observed ranges; sample intervals are never treated as scan duration."""
    return None if not values else dict(min=min(values), median=float(np.median(values)), max=max(values))


def point_time_ranges(message):
    """Read scalar timing fields as raw numbers without assigning units or epoch.

    PointCloud2 layouts have already passed cloud_xyz validation. Timing units
    and the relationship to header.stamp still require driver documentation.
    """
    types = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
    result = {}
    for field in message.fields:
        if field.name.lower() not in ('t', 'time', 'timestamp', 'time_stamp', 'offset_time'):
            continue
        if field.count != 1 or field.datatype not in types:
            raise ValueError('unsupported scalar point timing field')
        dtype = np.dtype(('>' if message.is_bigendian else '<')+types[field.datatype])
        if field.offset < 0 or field.offset+dtype.itemsize > message.point_step:
            raise ValueError('point timing field exceeds point stride')
        values = np.ndarray((message.height, message.width), dtype=dtype,
            buffer=message.data, offset=field.offset, strides=(message.row_step, message.point_step))
        if not np.isfinite(values).all():
            raise ValueError('nonfinite point timing field')
        result[field.name] = dict(min=float(values.min()), max=float(values.max()),
                                 units='unverified')
    return result


class InputProbe:
    """Keep bounded diagnostics and callback errors without guessing missing data."""

    def __init__(self, base_frame='base_link', odom_frame='odom'):
        """Store bounded sample history; never retain complete point clouds."""
        self.base_frame, self.odom_frame = base_frame, odom_frame
        self.clouds, self.odom = deque(maxlen=12000), deque(maxlen=12000)
        self.errors, self.error_count = deque(maxlen=20), 0
        self.cloud_count = self.odom_count = 0
        self.last_cloud = None

    def receive(self, message, now_ns, *, cloud):
        """Record valid metadata or a visible error; malformed inputs cannot pass."""
        try:
            stamp = stamp_ns(message.header.stamp)
            samples = self.clouds if cloud else self.odom
            if samples and stamp <= samples[-1]['stamp_ns']:
                raise ValueError('sensor timestamp duplicated or moved backwards')
            item = dict(stamp_ns=stamp, age_s=(now_ns-stamp)/1e9)
            if cloud:
                points = cloud_xyz(message)
                item.update(frame=message.header.frame_id, points=len(points),
                    fields=[dict(name=f.name, offset=f.offset, datatype=f.datatype, count=f.count)
                            for f in message.fields], point_time_ranges=point_time_ranges(message))
                self.cloud_count += 1
                self.last_cloud = (message.header.frame_id, stamp)
            else:
                _, _, linear, angular = _odom_geometry(message, self.base_frame, self.odom_frame)
                item.update(linear_speed_mps=float(linear), angular_speed_rps=float(angular))
                self.odom_count += 1
            samples.append(item)
        except (ValueError, TypeError, AttributeError, OverflowError) as exc:
            self.error_count += 1
            self.errors.append(('lidar: ' if cloud else 'odom: ')+str(exc))

    def report(self):
        """Summarize observed timing, frame/schema changes, speeds and errors."""
        result = dict(errors=list(self.errors), error_count=self.error_count)
        for name, samples, count in (('lidar', self.clouds, self.cloud_count),
                                     ('odom', self.odom, self.odom_count)):
            result[name] = dict(samples=count, retained_samples=len(samples),
                header_interval_s=bounds([(b['stamp_ns']-a['stamp_ns'])/1e9
                                          for a, b in zip(samples, list(samples)[1:])]),
                header_age_s=bounds([s['age_s'] for s in samples]))
        result['lidar'].update(frames=sorted({s['frame'] for s in self.clouds}),
            point_count=bounds([s['points'] for s in self.clouds]),
            last_fields=self.clouds[-1]['fields'] if self.clouds else [],
            last_point_time_ranges=self.clouds[-1]['point_time_ranges'] if self.clouds else {},
            field_layouts=len({json.dumps(s['fields'], sort_keys=True) for s in self.clouds}))
        result['odom'].update(linear_speed_mps=bounds([s['linear_speed_mps'] for s in self.odom]),
            angular_speed_rps=bounds([s['angular_speed_rps'] for s in self.odom]))
        result['timing_note'] = 'Header intervals and raw point fields do not establish scan duration or header convention.'
        return result


def run(profile, output, *, seconds=10.0, lidar_topic='/velodyne_points', odom_topic='/odom',
        base_frame='base_link', odom_frame='odom'):
    """Collect metadata, historical extrinsics and, with valid timing, two scans.

    Use the real production collector for the final stationary-input check.
    Write a report even if no sensors arrive or timing is still unknown.
    No Atlas, pose-estimate RPC, map load or robot command is issued.
    """
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Odometry
    from tf2_ros import Buffer, TransformListener, TransformException
    from .ros_observations import ObservationCollector
    from _relocalization_bbs import CancellationToken

    if not math.isfinite(seconds) or not 1 <= seconds <= 60:
        raise ValueError('probe seconds must be in [1, 60]')
    if not all(isinstance(t, str) and t.startswith('/') for t in (lidar_topic, odom_topic)):
        raise ValueError('probe topics must be explicit absolute ROS names')
    if Path(output).exists():
        raise ValueError('probe output must be a new file')
    configured = read_profile(profile)
    expected = np.asarray(configured['base_from_sensor'], dtype=float).reshape(4, 4)
    if (not np.isfinite(expected).all() or not np.allclose(expected[3], [0, 0, 0, 1]) or
            not np.allclose(expected[:3, :3].T @ expected[:3, :3], np.eye(3), atol=1e-6) or
            not np.isclose(np.linalg.det(expected[:3, :3]), 1, atol=1e-6)):
        raise ValueError('probe calibration must be a finite rigid transform')
    probe, errors = InputProbe(base_frame, odom_frame), []
    context = Context()
    rclpy.init(args=[], context=context)
    node = executor = listener = collector = worker = None
    token = CancellationToken()
    report = {}
    try:
        node = rclpy.create_node('relocalization_input_probe', context=context,
                                 start_parameter_services=False)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        buffer = Buffer()
        listener = TransformListener(buffer, node, spin_thread=False)
        node.create_subscription(PointCloud2, lidar_topic,
            lambda m: probe.receive(m, node.get_clock().now().nanoseconds, cloud=True), qos_profile_sensor_data)
        node.create_subscription(Odometry, odom_topic,
            lambda m: probe.receive(m, node.get_clock().now().nanoseconds, cloud=False), qos_profile_sensor_data)
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=min(0.05, max(0, deadline-time.monotonic())))
        report = probe.report()
        report.update(lidar_topic=lidar_topic, odom_topic=odom_topic,
                      base_frame=base_frame, odom_frame=odom_frame,
                      timing=configured['capture'], stationary_capture_passed=False)
        if len(report['lidar']['frames']) > 1 or report['lidar']['field_layouts'] > 1:
            errors.append('lidar frame or point field layout changed during probe')
        if probe.last_cloud is None:
            errors.append('no valid lidar samples received')
        else:
            frame, stamp = probe.last_cloud
            try:
                actual = transform_matrix(buffer.lookup_transform(base_frame, frame,
                    Time(nanoseconds=stamp)), base_frame, frame)
                report['live_base_from_sensor'] = actual.reshape(-1).tolist()
                report['live_extrinsics_match'] = bool(np.allclose(actual, expected, atol=1e-4, rtol=0))
                if not report['live_extrinsics_match']:
                    errors.append('historical live TF differs from saved calibration')
            except (ValueError, TransformException) as exc:
                errors.append('historical sensor TF unavailable: '+str(exc))
        if not probe.odom:
            errors.append('no valid odometry samples received')
        try:
            options = CaptureOptions(**configured['capture'])
            options.validate()
            if errors or probe.error_count:
                raise ValueError('metadata/TF errors prevent stationary capture')
            collector = ObservationCollector(node, lidar_topic, odom_topic, base_frame=base_frame,
                odom_frame=odom_frame, expected_base_from_sensor=expected, options=options)
            worker = threading.Thread(target=executor.spin, daemon=True)
            worker.start()
            fit, validation = collector.collect(8.0, token)
            report.update(stationary_capture_passed=True, capture_points=[len(fit.points), len(validation.points)],
                capture_windows_ns=[[fit.start_stamp_ns, fit.end_stamp_ns],
                                    [validation.start_stamp_ns, validation.end_stamp_ns]])
        except (ValueError, RuntimeError, TypeError, TimeoutError) as exc:
            errors.append('stationary capture: '+str(exc))
        report['errors'] += errors
        report['error_count'] += len(errors)
    finally:
        token.cancel()
        if executor is not None:
            executor.shutdown(timeout_sec=2)
        if worker is not None:
            worker.join(3)
        if collector is not None:
            collector.close()
        if listener is not None:
            listener.unregister()
        if node is not None:
            node.destroy_node()
        context.shutdown()
    with Path(output).open('x') as stream:
        stream.write(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    return report


def main():
    """Return nonzero for missing/unverified inputs and keep the diagnostic file."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--lidar-topic', default='/velodyne_points')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--odom-frame', default='odom')
    args = parser.parse_args()
    try:
        report = run(args.profile, args.output, seconds=args.seconds, lidar_topic=args.lidar_topic,
            odom_topic=args.odom_topic, base_frame=args.base_frame, odom_frame=args.odom_frame)
        print(json.dumps(dict(report=str(args.output), stationary_capture_passed=report['stationary_capture_passed'],
                              errors=report['errors']), ensure_ascii=False))
        if not report['stationary_capture_passed'] or report['error_count']:
            raise SystemExit(1)
    except (OSError, ValueError, TypeError, RuntimeError) as exc:
        parser.exit(1, f'input probe failed: {exc}\n')


if __name__ == '__main__':
    main()
