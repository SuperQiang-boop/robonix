#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Expose the G1 face camera's existing ROS 2 RGB topic as a primitive."""
from __future__ import annotations

import logging
import os

from robonix_api import Err, Ok, Primitive

logging.basicConfig(
    level=os.environ.get("FACE_CAMERA_LOG_LEVEL", "INFO"),
    format="[face_camera] %(message)s",
)
log = logging.getLogger("face_camera")

cap = Primitive(id="face_camera", namespace="robonix/primitive/camera")


@cap.on_init
def init(cfg: dict):
    """Wait for the robot-published RGB topic and declare its capability."""
    topic = str(cfg.get("rgb_topic", "/face/camera/color/image_raw")).strip()
    timeout_s = float(cfg.get("sentinel_timeout_s", 30.0))
    if not topic.startswith("/"):
        return Err("rgb_topic must be an absolute ROS 2 topic")
    if timeout_s <= 0:
        return Err("sentinel_timeout_s must be positive")
    if not cap.wait_for_topic(topic, "Image", timeout_s):
        return Err(f"no Image on {topic} within {timeout_s:.1f}s")
    cap.declare_ros2_topic(
        "robonix/primitive/camera/rgb",
        topic=topic,
        qos="best_effort",
        description="G1 face camera RGB image stream.",
    )
    log.info("initialized from existing topic %s", topic)
    return Ok()


if __name__ == "__main__":
    cap.run()
