"""Validate cold configuration without opening maps, ROS nodes or channels."""
from __future__ import annotations

import math
import os
import re
from dataclasses import dataclass, replace
from pathlib import Path


@dataclass(frozen=True)
class Settings:
    mapping_provider_id: str
    lidar_provider_id: str
    odom_provider_id: str
    maps_dir: Path
    index_dir: Path
    map_id: str | None = None
    base_frame: str = "base_link"
    odom_frame: str = "odom"
    use_sim_time: bool = False
    # A loaded localization map triggers requests unless explicitly disabled.
    auto_relocalize: bool = True
    timeout_s: float = 15.0
    status_period_s: float = 1.0
    profile_file: Path | None = None
    coordination_dir: Path | None = None

    @classmethod
    def parse(cls, config: dict, *, deployment_dir: Path | None = None) -> Settings:
        """Reject invalid fields and resolve paths from the deployment directory.

        Paths need not exist at INIT; map/index preparation is a later stage.
        Relative paths require an explicit deployment base to avoid cwd drift.
        """
        if not isinstance(config, dict):
            raise ValueError("configuration must be an object")
        unknown = set(config) - set(cls.__dataclass_fields__)
        if unknown:
            raise ValueError(f"unknown configuration keys: {sorted(unknown)}")
        try:
            value = cls(**config)
        except TypeError as exc:
            raise ValueError(str(exc)) from exc
        for name in ("mapping_provider_id", "lidar_provider_id",
                     "odom_provider_id", "base_frame", "odom_frame"):
            item = getattr(value, name)
            if not isinstance(item, str) or not item or item != item.strip():
                raise ValueError(f"{name} must be a nonempty string without surrounding whitespace")
        if value.map_id is not None and (not isinstance(value.map_id, str) or
                not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,127}", value.map_id)):
            raise ValueError("map_id must contain only letters, digits, underscores and hyphens")
        if value.base_frame == value.odom_frame:
            raise ValueError("base_frame and odom_frame must differ")
        if type(value.use_sim_time) is not bool:
            raise ValueError("use_sim_time must be boolean")
        if type(value.auto_relocalize) is not bool:
            raise ValueError("auto_relocalize must be boolean")
        for name, maximum in (("timeout_s", 120), ("status_period_s", 5)):
            item = getattr(value, name)
            if type(item) not in (int, float) or not math.isfinite(item) or not 0 < item <= maximum:
                raise ValueError(f"{name} must be a finite number in (0, {maximum}]")
        base = deployment_dir or os.environ.get("RBNX_INVOCATION_CWD")
        if deployment_dir is not None and not Path(deployment_dir).is_absolute():
            raise ValueError('deployment_dir must be absolute')
        if deployment_dir is None and base is not None and not Path(base).is_absolute():
            raise ValueError('RBNX_INVOCATION_CWD must be absolute')
        paths = {}
        if (value.profile_file is None) != (value.coordination_dir is None):
            raise ValueError('profile_file and coordination_dir must be configured together')
        for name in ("maps_dir", "index_dir", "profile_file", "coordination_dir"):
            item = getattr(value, name)
            if item is None and name in ('profile_file', 'coordination_dir'):
                continue
            if not isinstance(item, (str, Path)) or not str(item).strip():
                raise ValueError(f"{name} must be a nonempty path")
            path = Path(item)
            if not path.is_absolute():
                if base is None or not Path(base).is_absolute():
                    raise ValueError(f"relative {name} requires an absolute RBNX_INVOCATION_CWD")
                path = Path(base) / path
            paths[name] = path.resolve()
        if paths["maps_dir"] == paths["index_dir"] or paths["maps_dir"] in paths["index_dir"].parents:
            raise ValueError("index_dir must be outside the read-only maps_dir")
        return replace(value, **paths)
