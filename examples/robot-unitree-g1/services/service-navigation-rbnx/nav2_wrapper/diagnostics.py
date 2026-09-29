"""Bounded, transport-safe diagnostics for Nav2 Humble action results."""

from __future__ import annotations

import re


_ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
_SIGNALS = (
    "failed to make progress",
    "controller patience exceeded",
    "resulting plan has 0 poses",
    "received plan with zero length",
    "no valid trajectories",
    "failed to create plan",
    "collision ahead",
    "rotation shim controller",
    "extrapolation error",
    "transform timeout",
    "timed out waiting for transform",
    "spin failed",
)


def classify_nav2_line(line: str) -> str:
    """Return one concise actionable line, or empty for routine Nav2 output."""
    clean = _ANSI.sub("", line).strip()
    lowered = clean.lower()
    if not any(signal in lowered for signal in _SIGNALS):
        return ""
    return clean[-240:]


def format_result_detail(
    status: str,
    feedback: dict[str, object] | None,
    diagnostics: list[str],
) -> str:
    """Describe an action result without inventing unavailable Humble codes."""
    parts = [status.lower()]
    if feedback:
        parts.append(
            "distance_remaining={:.3f}m recoveries={} last_pose=({:.3f},{:.3f})".format(
                float(feedback.get("distance_remaining", 0.0)),
                int(feedback.get("recoveries", 0)),
                float(feedback.get("x", 0.0)),
                float(feedback.get("y", 0.0)),
            )
        )
    unique: list[str] = []
    for item in diagnostics:
        if item and item not in unique:
            unique.append(item)
    if unique:
        parts.append("nav2=" + " | ".join(unique[-3:]))
    return "; ".join(parts)


def summarize_blockage(
    data: "list[int]",
    width: int,
    height: int,
    resolution: float,
    threshold: int = 99,
    radius_m: float = 1.5,
) -> str:
    """One-line blockage report from a robot-centred rolling OccupancyGrid.

    The local costmap window is centred on the robot, so the grid centre
    stands in for the robot pose and no TF lookup is needed. Reports the
    nearest cell at or above `threshold` (occupancy 0-100 scale; 100 is
    lethal, inscribed-inflation publishes 99) plus how many such cells sit
    within `radius_m`. Empty string when the grid is unusable.
    """
    import math as _math

    if width <= 0 or height <= 0 or resolution <= 0 or len(data) < width * height:
        return ""
    cx, cy = (width - 1) / 2.0, (height - 1) / 2.0
    nearest = None  # (distance_m, bearing_deg)
    within = 0
    for iy in range(height):
        row = iy * width
        for ix in range(width):
            if data[row + ix] < threshold:
                continue
            d = _math.hypot(ix - cx, iy - cy) * resolution
            if d <= radius_m:
                within += 1
            if nearest is None or d < nearest[0]:
                bearing = _math.degrees(_math.atan2(iy - cy, ix - cx))
                nearest = (d, bearing)
    if nearest is None:
        return f"local costmap: no cells >= {threshold} in the window"
    return (
        f"local costmap: nearest blocked cell {nearest[0]:.2f}m at "
        f"{nearest[1]:.0f}deg, {within} blocked cells within {radius_m:.1f}m"
    )
