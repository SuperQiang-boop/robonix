"""Read the sensor calibration profile without loading native matcher code."""
from __future__ import annotations

import json
from pathlib import Path


def read_profile(path):
    """Load sensor timing and calibration independently of map identity."""
    path = Path(path).resolve(strict=True)
    if path.stat().st_size > 256_000:
        raise ValueError('relocalization profile exceeds 256 kB')
    value = json.loads(path.read_text())
    required = {'capture', 'base_from_sensor', 'registration'}
    allowed = required | {'index', 'search', 'matching'}
    if not isinstance(value, dict) or not required <= value.keys() or value.keys()-allowed:
        raise ValueError('profile requires capture, base_from_sensor and registration')
    return value
