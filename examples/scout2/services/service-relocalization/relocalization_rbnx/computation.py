"""Internal ROS capture, global matching and optional verified mapping handoff."""
from __future__ import annotations

import copy
import math
import time


def _remaining(deadline, cancelled):
    """Use the same native cancellation token and whole-operation deadline."""
    from _relocalization_bbs import SearchCancelled, SearchTimeout
    if cancelled.cancelled:
        raise SearchCancelled('relocalization computation cancelled')
    remaining = deadline-time.monotonic()
    if remaining <= 0:
        raise SearchTimeout('relocalization computation deadline exceeded')
    return min(remaining, 120.0)


def compute_candidate(collector, matcher, *, search_options, registration_options,
                      options, timeout_s, cancelled):
    """Capture, match and revalidate under one deadline with no robot actions.

    Caller owns/spins the collector node, binds the matcher to the actually
    loaded snapshot and supplies verified scan timing, extrinsics and base
    height/tilt. Run off the ROS executor thread with a native CancellationToken.
    Return only a geometric candidate: handoff and its verification are separate
    required work before localization can become valid. All failures propagate.
    """
    if type(timeout_s) not in (float, int) or not math.isfinite(timeout_s) or not 0 < timeout_s <= 120:
        raise ValueError('computation timeout_s must be in (0, 120]')
    deadline = time.monotonic()+timeout_s
    _remaining(deadline, cancelled)
    search_options = copy.copy(search_options)
    registration_options = copy.copy(registration_options)
    options = copy.deepcopy(options)
    observations = collector.collect(_remaining(deadline, cancelled), cancelled)
    result = matcher.match(*observations, search_options, registration_options, options,
                           _remaining(deadline, cancelled), cancelled)
    collector.validate_after_matching(observations, _remaining(deadline, cancelled), cancelled)
    _remaining(deadline, cancelled)
    return result


def make_task_runner(collector, matcher, loaded_identity, *, search_options,
                     registration_options, options, mapping=None):
    """Bind a loaded matcher to the authoritative mapping snapshot reader.

    loaded_identity(remaining_s, cancelled) must read the actual mapping instance
    and return its map_id, generation and database_sha256. A directory's latest
    saved snapshot is not evidence of what mapping loaded. No such production
    reader is wired yet; callers must supply it explicitly. The callback and
    computation must honour the supplied deadline/token. When mapping is
    supplied, inject through pose_estimate and verify new scans/pose feedback.
    Production callers must also interlock mapping transitions and cancel on
    provider changes: identity reads alone cannot detect switching away/back.
    """
    identity = matcher.identity
    fields = ('map_id', 'generation', 'database_sha256')
    expected = {name: identity[name] for name in fields}
    search_options = copy.copy(search_options)
    registration_options = copy.copy(registration_options)
    options = copy.deepcopy(options)
    if not callable(loaded_identity):
        raise TypeError('authoritative loaded map identity reader is required')

    def run(map_id, generation, digest, timeout_s, cancelled):
        """Reject mismatched requests and a map switch while computation runs."""
        if type(timeout_s) not in (int, float) or not math.isfinite(timeout_s) or not 0 < timeout_s <= 120:
            raise ValueError('task timeout_s must be in (0, 120]')
        deadline = time.monotonic()+timeout_s
        requested = dict(map_id=map_id, generation=generation, database_sha256=digest)
        if requested != expected:
            raise ValueError('requested map identity differs from the loaded matcher snapshot')
        def authorize():
            """Recheck request cancellation and actual mapping identity at handoff."""
            current = copy.deepcopy(loaded_identity(_remaining(deadline, cancelled), cancelled))
            if current != expected:
                raise ValueError('actual loaded mapping snapshot changed during handoff')
            _remaining(deadline, cancelled)
        before = copy.deepcopy(loaded_identity(_remaining(deadline, cancelled), cancelled))
        if before != expected:
            raise ValueError('actual loaded mapping snapshot differs from the matcher')
        result = compute_candidate(collector, matcher, search_options=search_options,
            registration_options=registration_options, options=options,
            timeout_s=_remaining(deadline, cancelled), cancelled=cancelled)
        after = copy.deepcopy(loaded_identity(_remaining(deadline, cancelled), cancelled))
        if after != before or after != expected:
            raise ValueError('actual loaded mapping snapshot changed during computation')
        if mapping is not None:
            from .handoff import handoff_candidate
            result = handoff_candidate(collector, matcher, mapping, result, authorize,
                registration_options=registration_options, options=options,
                timeout_s=_remaining(deadline, cancelled), cancelled=cancelled)
        _remaining(deadline, cancelled)
        return result

    return run
