"""Persist UUID-scoped safety stops before cancellation and preserve user-cancel semantics."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import re


def _failure_path(directory: Path, goal_uuid: str) -> Path:
    """Accept only complete ROS action UUIDs, never arbitrary filesystem components."""
    if re.fullmatch(r"[0-9a-f]{32}", goal_uuid) is None:
        raise ValueError("invalid navigation action UUID")
    return Path(directory) / f"guard-{goal_uuid}.json"


def write_guard_failure(directory, goal_uuid, reason, robot_pose=None, goal_pose=None):
    """Atomically save the safety cause before the guard submits its ROS cancellation."""
    target = _failure_path(Path(directory), goal_uuid)
    record = {"goal_uuid": goal_uuid, "reason": str(reason)[:512]}
    if robot_pose is not None and goal_pose is not None:
        values = [float(value) for value in (*robot_pose, *goal_pose)]
        if len(values) == 6 and all(math.isfinite(value) for value in values):
            record["xy_error_m"] = math.hypot(values[0] - values[3], values[1] - values[4])
            record["yaw_error_rad"] = abs(math.atan2(
                math.sin(values[2] - values[5]), math.cos(values[2] - values[5])))
            record["final_pose"] = values[:3]
    temporary = target.with_suffix(".tmp")
    try:
        temporary.write_text(json.dumps(record, allow_nan=False), encoding="utf-8")
        os.replace(temporary, target)
    finally:
        temporary.unlink(missing_ok=True)


def read_guard_failure(directory, goal_uuid):
    """Read only the matching bounded record; stale, malformed or missing files are ignored."""
    try:
        target = _failure_path(Path(directory), goal_uuid)
        with target.open(encoding="utf-8") as stream:
            payload = stream.read(8193)
        if len(payload) > 8192:
            return None
        record = json.loads(payload)
        if not isinstance(record, dict) or record.get("goal_uuid") != goal_uuid:
            return None
        if not isinstance(record.get("reason"), str) or not record["reason"]:
            return None
        return record
    except (OSError, ValueError, TypeError):
        return None


def classify_guard_result(nav2_state, user_cancel_requested, failure):
    """Map a guard stop to FAILED without changing explicit or unrelated cancellations."""
    if not failure or user_cancel_requested:
        return nav2_state, ""
    detail = "safety_stop=" + failure["reason"]
    for key, unit in (("xy_error_m", "m"), ("yaw_error_rad", "rad")):
        value = failure.get(key)
        if isinstance(value, (int, float)) and math.isfinite(value):
            detail += f" {key}={value:.3f}{unit}"
    return "FAILED", detail
