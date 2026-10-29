"""Inject a candidate through mapping and verify new stationary observations."""
from __future__ import annotations

import copy
import math
import time

from .computation import _remaining


def handoff_candidate(collector, matcher, mapping, candidate, authorize, *,
                      registration_options, options, timeout_s, cancelled):
    """Complete plan step 7 using the existing mapping interfaces.

    authorize() must reject a stale request and enforce the mapping transition
    interlock against this matcher's snapshot for the entire operation. Caller
    owns the resources and spins ROS separately. A successful RPC alone never
    counts as successful handoff: collect new scans after acknowledgement, read
    fresh mapping feedback, score without refinement, and recheck continuous
    stationary odometry through verification. Failure propagates. This returns
    geometry only; ongoing validation and navigation integration remain required.
    """
    if type(timeout_s) not in (int, float) or not math.isfinite(timeout_s) or not 0 < timeout_s <= 120:
        raise ValueError('handoff timeout_s must be in (0, 120]')
    if not callable(authorize):
        raise TypeError('current request and mapping interlock is required')
    deadline = time.monotonic()+timeout_s
    registration_options = copy.copy(registration_options)
    options = copy.deepcopy(options)
    candidate = copy.deepcopy(candidate)
    _remaining(deadline, cancelled)
    authorize()
    stationary_since = collector.validate_current(_remaining(deadline, cancelled), cancelled)
    acknowledged = mapping.inject(candidate, deadline, cancelled, authorize)
    last_disagreement = None
    while True:
        remaining = _remaining(deadline, cancelled)
        try:
            observations = collector.collect(remaining, cancelled)
        except TimeoutError as exc:
            if last_disagreement is None:
                raise
            raise ValueError(f'mapping pose did not converge to the global relocalization candidate: '
                             f'{last_disagreement}') from exc
        if observations[0].start_stamp_ns <= acknowledged:
            raise ValueError('handoff verification scan predates mapping acknowledgement')
        poses = []
        for observation in observations:
            _remaining(deadline, cancelled)
            authorize()
            pose = mapping.read_pose(observation.end_stamp_ns, deadline, cancelled)
            distance = math.hypot(pose['x']-candidate.x, pose['y']-candidate.y)
            heading = abs(math.remainder(pose['yaw']-candidate.yaw, 2*math.pi))
            tilt = max(abs(pose['z']-registration_options.base_z),
                abs(math.remainder(pose['roll']-registration_options.roll, 2*math.pi)),
                abs(math.remainder(pose['pitch']-registration_options.pitch, 2*math.pi)))
            if distance > options.same_position_m or heading > options.same_heading_rad or tilt > 0.05:
                last_disagreement = (f'position_error={distance:.3f} m, heading_error={heading:.3f} rad, '
                                     f'height_or_tilt_error={tilt:.3f}')
                break
            poses.append(pose)
        if last_disagreement is not None and len(poses) != len(observations):
            continue
        results = []
        for observation, pose in zip(observations, poses):
            authorize()
            results.append(matcher.evaluate(observation, [pose['x'], pose['y'], pose['yaw']],
                registration_options, options, _remaining(deadline, cancelled), cancelled))
        collector.validate_after_matching(observations, _remaining(deadline, cancelled),
            cancelled, stationary_since_ns=stationary_since)
        authorize()
        _remaining(deadline, cancelled)
        return results[-1]
