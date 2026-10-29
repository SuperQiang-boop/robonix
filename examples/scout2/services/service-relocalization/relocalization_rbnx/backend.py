"""Production assembly of snapshot matching, mapping handoff and live checking."""
from __future__ import annotations

import json
import logging
import math
from pathlib import Path
import threading
import time

import numpy as np

from .computation import make_task_runner
from .coordination import MapReader
from .map_index import read_export
from .handoff import handoff_candidate
from .mapping_io import MappingIO
from .matching import MatchingOptions, OfflineMatcher, _validate_options
from .observations import CaptureOptions
from .profile import read_profile
from .ros_observations import ObservationCollector
from .runtime import VerifiedResult


def native_options(cls, values):
    """Reject misspelled native fields instead of creating Python attributes."""
    result = cls()
    if not isinstance(values, dict):
        raise ValueError('algorithm options must be an object')
    for name, value in values.items():
        if not hasattr(type(result), name) or not isinstance(value, (float, int)) or isinstance(value, bool):
            raise ValueError(f'invalid {cls.__name__} option: {name}')
        if not math.isfinite(value):
            raise ValueError(f'nonfinite {cls.__name__} option: {name}')
        setattr(result, name, value)
    return result


class Backend:
    """Own one matcher; create new observation resources for each request.

    Status reads never wait on ROS or run computation. A separate monitor
    scores fresh scans at mapping's fixed pose and latches failure/cancellation.
    The accepted request remains valid only while that evidence stays fresh.
    """

    def __init__(self, settings, node, endpoints):
        """Bind sensor calibration to the currently loaded map's checked index."""
        import _relocalization_bbs as native
        self.settings, self.node, self.endpoints = settings, node, endpoints
        profile = read_profile(settings.profile_file)
        self.profile = profile
        self.reader = MapReader(settings.coordination_dir, settings.mapping_provider_id)
        self._matcher_lock = threading.RLock()
        self.matcher = None
        self._matcher_path = None
        self._matcher_identity = None
        self.capture = CaptureOptions(**profile['capture'])
        self.capture.validate()
        self.registration = native_options(native.RegistrationOptions, profile['registration'])
        if not {'base_z', 'roll', 'pitch'} <= profile['registration'].keys():
            raise ValueError('explicit calibrated base_z, roll and pitch are required')
        self.search = native_options(native.SearchOptions, profile.get('search', {}))
        self.options = MatchingOptions(**profile.get('matching', {}))
        _validate_options(self.options)
        self.expected = np.asarray(profile['base_from_sensor'], dtype=float)
        if self.expected.size == 16:
            self.expected = self.expected.reshape(4, 4)
        if (self.expected.shape != (4, 4) or not np.isfinite(self.expected).all() or
                not np.allclose(self.expected[3], [0, 0, 0, 1], atol=1e-8) or
                not np.allclose(self.expected[:3, :3].T @ self.expected[:3, :3], np.eye(3), atol=1e-6) or
                not np.isclose(np.linalg.det(self.expected[:3, :3]), 1, atol=1e-6)):
            raise ValueError('base_from_sensor must be a finite rigid 4x4 transform')
        self._lock = threading.RLock()
        self._resources, self._monitor, self._token = [], None, None
        self._checked, self._checked_at, self._load = None, 0, None
        self._closed, self._error = False, ''

    def _matcher_for(self, identity):
        """Find or prepare the immutable index matching the loaded snapshot."""
        from .map_index import verify_index

        with self._matcher_lock:
            if self._matcher_identity == identity and self.matcher is not None:
                return self.matcher
            matcher, index_path = None, None
            parent = self.settings.index_dir / identity['map_id']
            if parent.is_dir():
                for candidate in sorted(parent.iterdir()):
                    manifest = candidate / 'index.json'
                    if (not candidate.is_dir() or not manifest.is_file() or
                            not candidate.resolve().is_relative_to(self.settings.index_dir)):
                        continue
                    try:
                        indexed = json.loads(manifest.read_text())['identity']
                        if any(indexed.get(key) != identity[key] for key in
                               ('map_id', 'generation', 'database_sha256')):
                            continue
                        verify_index(candidate, indexed)
                        matcher = OfflineMatcher(candidate, indexed)
                        index_path = candidate
                        break
                    except (OSError, ValueError, KeyError, TypeError):
                        continue
            reference = self.profile.get('index')
            if matcher is None and reference:
                try:
                    reference = Path(reference)
                    if not reference.is_absolute():
                        reference = self.settings.profile_file.parent / reference
                    reference = reference.resolve(strict=True)
                    if reference.is_relative_to(self.settings.index_dir):
                        indexed = json.loads((reference / 'index.json').read_text())['identity']
                        if all(indexed.get(key) == identity[key] for key in
                               ('map_id', 'generation', 'database_sha256')):
                            verify_index(reference, indexed)
                            matcher, index_path = OfflineMatcher(reference, indexed), reference
                except (OSError, ValueError, KeyError, TypeError):
                    pass
            if matcher is None:
                from .prepare import prepare

                package = Path(__file__).resolve().parents[1]
                exporters = (package / 'rbnx-build/backend/relocalization_export',
                             package / 'rbnx-build/backend-container/relocalization_export')
                exporter = next((path for path in exporters if path.is_file()), None)
                image = None
                if exporter is not None and exporter.parent.name == 'backend-container':
                    image_file = exporter.parent / 'image-id'
                    if image_file.is_file():
                        image = image_file.read_text().strip()
                if exporter is None:
                    raise RuntimeError('no matching map index; build the snapshot exporter before localization')
                if exporter.parent.name == 'backend-container' and not image:
                    raise RuntimeError('container snapshot exporter has no recorded image ID; rebuild the exporter')
                prepared = prepare(self.settings.maps_dir, identity['map_id'], self.settings.index_dir,
                                   exporter, image=image)
                indexed = json.loads((prepared / 'index.json').read_text())['identity']
                if any(indexed.get(key) != identity[key] for key in
                       ('map_id', 'generation', 'database_sha256')):
                    raise RuntimeError('prepared index does not match the loaded mapping snapshot')
                verify_index(prepared, indexed)
                matcher = OfflineMatcher(prepared, indexed)
                index_path = prepared
            report, _, _ = read_export(index_path)
            for item in report['nodes']:
                if not np.allclose(np.asarray(item['base_from_sensor']).reshape(3, 4),
                                   self.expected[:3], atol=1e-4, rtol=0):
                    raise ValueError('live calibration differs from saved scan extrinsics')
            self.matcher = matcher
            self._matcher_path = index_path
            self._matcher_identity = identity.copy()
            return matcher

    def prepare_map_index(self, identity):
        """Prepare the exact loaded map index before any localization request."""
        return self._matcher_for(identity)

    def has_map_index(self, identity):
        """Report whether the currently cached matcher belongs to this snapshot."""
        with self._matcher_lock:
            return self.matcher is not None and self._matcher_identity == identity

    def _authorize(self, load, token):
        """Reject heartbeat loss, cancellation, provider restart or any map reload."""
        if self._closed or token.cancelled:
            raise RuntimeError('relocalization backend closed or cancelled')
        actual = self.reader.current()
        for name in ('process_id', 'load_id', 'map_id', 'generation', 'database_sha256'):
            if actual[name] != load[name]:
                raise RuntimeError('mapping process or loaded snapshot changed')
        return {key: actual[key] for key in ('map_id', 'generation', 'database_sha256')}

    def current_map_identity(self, map_id):
        """Resolve a user map ID to the snapshot identity currently loaded by mapping."""
        if not isinstance(map_id, str) or not map_id.strip():
            raise ValueError('map_id must be a nonempty string')
        if self.settings.map_id and map_id != self.settings.map_id:
            raise ValueError('requested map_id differs from the configured map')
        identity = self.reader.current()
        if identity.get('map_id') != map_id:
            raise ValueError('requested map_id differs from the loaded mapping snapshot')
        generation, digest = identity.get('generation'), identity.get('database_sha256')
        if type(generation) is not int or not 0 <= generation < 2**64:
            raise RuntimeError('loaded mapping snapshot has an invalid generation')
        if (not isinstance(digest, str) or len(digest) != 64 or
                any(char not in '0123456789abcdef' for char in digest)):
            raise RuntimeError('loaded mapping snapshot has an invalid database digest')
        return dict(map_id=map_id, generation=generation, database_sha256=digest)

    def _pose(self, pose):
        """Enforce the fixed calibrated height/tilt model for every verification."""
        if (abs(pose['z']-self.registration.base_z) > 0.05 or
                abs(math.remainder(pose['roll']-self.registration.roll, 2*math.pi)) > 0.05 or
                abs(math.remainder(pose['pitch']-self.registration.pitch, 2*math.pi)) > 0.05):
            raise ValueError('mapping pose height/tilt differs from calibration')
        return [pose['x'], pose['y'], pose['yaw']]

    def run(self, map_id, generation, digest, timeout_s, token):
        """Run the real matcher and handoff under one map lease and deadline."""
        deadline = time.monotonic()+timeout_s
        self.stop_request()
        with self._lock:
            if self._closed:
                raise RuntimeError('relocalization backend is closed')
            self._token = token
            self._checked, self._error = None, ''
        try:
            collector = ObservationCollector(self.node, self.endpoints['lidar'], self.endpoints['odom'],
                base_frame=self.settings.base_frame, odom_frame=self.settings.odom_frame,
                expected_base_from_sensor=self.expected, options=self.capture)
            self._resources.append(collector)
            with self.reader.lease(deadline, token) as load:
                matcher = self._matcher_for({key: load[key] for key in
                    ('map_id', 'generation', 'database_sha256')})
                authorize = lambda remaining, cancelled: self._authorize(load, cancelled)
                runner = make_task_runner(collector, matcher, authorize,
                    search_options=self.search, registration_options=self.registration,
                    options=self.options)
                result = runner(map_id, generation, digest, deadline-time.monotonic(), token)
                mapping = MappingIO(self.node, self.endpoints['pose'], self.endpoints['estimate'],
                                    require_injection=True)
                self._resources.append(mapping)
                result = handoff_candidate(collector, matcher, mapping, result,
                    lambda: self._authorize(load, token), registration_options=self.registration,
                    options=self.options, timeout_s=deadline-time.monotonic(), cancelled=token)
                self._authorize(load, token)
                with self._lock:
                    self._load, self._checked, self._checked_at = load, result, time.monotonic()
            monitor = threading.Thread(target=self._track,
                args=(collector, mapping, load, token, matcher),
                name='relocalization-verification', daemon=True)
            self._monitor = monitor
            monitor.start()
            return VerifiedResult(self.current)
        except Exception:
            self.stop_request()
            raise

    def _track(self, collector, mapping, load, token, matcher):
        """Continuously check new scan geometry without blocking status publication."""
        from tf2_ros import TransformException
        try:
            while not token.cancelled and not self._closed:
                try:
                    with self.reader.lease(time.monotonic()+1.0, token):
                        self._authorize(load, token)
                        observation = collector.collect_tracking(0.8, token)
                        pose = mapping.read_pose(observation.end_stamp_ns, time.monotonic()+0.3, token,
                                                 allow_before=True)
                        pose = collector.pose_at(observation, pose)
                        result = matcher.evaluate(observation, self._pose(pose), self.registration,
                            self.options, 0.3, token)
                        collector.validate_tracking(observation)
                        self._authorize(load, token)
                        with self._lock:
                            self._checked, self._checked_at = result, time.monotonic()
                except TransformException as exc:
                    # Mapping and odom publishers can be a few tens of ms apart.
                    # Keep the last verified result while waiting for a bracketing
                    # TF sample; Runtime.current() still expires stale evidence.
                    logging.debug('relocalization live verification waiting for TF: %s', exc)
                    continue
        except Exception as exc:
            with self._lock:
                self._error = str(exc)
            token.cancel()
            logging.warning('relocalization live verification stopped: %s', exc)

    def current(self):
        """Read cached evidence without waiting; expiry permanently revokes receipt."""
        with self._lock:
            token, load = self._token, self._load
            if self._error or token is None or load is None or self._checked is None:
                raise RuntimeError(self._error or 'no verified mapping handoff')
            try:
                self._authorize(load, token)
                now_ros = self.node.get_clock().now().nanoseconds
                age_ros = now_ros-self._checked.observation_stamp_ns
                if (not 0 <= age_ros <= self.capture.max_age_s*1e9 or
                        time.monotonic()-self._checked_at > self.capture.max_age_s):
                    raise RuntimeError('live localization evidence expired')
                return self._checked
            except Exception:
                token.cancel()
                raise

    def stop_request(self):
        """Cancel and join monitoring before destroying resources or another task."""
        if self._token is not None:
            self._token.cancel()
        if self._monitor is not None:
            self._monitor.join(timeout=3)
            if self._monitor.is_alive():
                raise RuntimeError('live localization monitor did not stop')
            self._monitor = None
        remaining, errors = [], []
        for resource in reversed(self._resources):
            try:
                resource.close()
            except Exception as exc:
                remaining.append(resource)
                errors.append(str(exc))
        self._resources = list(reversed(remaining))
        if errors:
            raise RuntimeError('; '.join(errors))

    def close(self):
        self._closed = True
        self.stop_request()
