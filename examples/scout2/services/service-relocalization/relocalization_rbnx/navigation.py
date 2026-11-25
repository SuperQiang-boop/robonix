"""Private navigation checks shared by the wrapper and final velocity guard."""
from __future__ import annotations

from pathlib import Path

from .coordination import read_record


def mapping_active(directory):
    """Recognize live mapping; callers must also check navigation inputs."""
    try:
        mapping = read_record(Path(directory) / 'mapping.json', 0.6)
        return mapping.get('ready') is True and mapping.get('mode') == 'mapping'
    except (OSError, KeyError, ValueError, TypeError, RuntimeError):
        return False


def check(directory, provider_id='relocalization'):
    """Check mapping liveness or exact localization identity before motion.

    Mapping callers must additionally enforce live sensor/TF readiness. The
    wrapper requires the final guard's mapping_navigation_ready result.
    """
    try:
        directory = Path(directory)
        mapping = read_record(directory / 'mapping.json', 0.6)
        if mapping.get('ready') is not True:
            return False, 'mapping is not ready'
        if mapping.get('mode') == 'mapping':
            if not mapping.get('map_id'):
                return False, 'mapping has no active map id'
            if not mapping.get('process_id') or not mapping.get('load_id'):
                return False, 'mapping process identity is incomplete'
            return True, ''
        if mapping.get('mode') != 'localization':
            return False, 'mapping mode is unavailable'
        status = read_record(directory / 'status.json', 0.4)
        if (status.get('provider_id') != provider_id or not status.get('active') or
                not status.get('localization_valid') or status.get('busy')):
            return False, status.get('detail', 'relocalization is not verified')
        for field in ('map_id', 'generation', 'database_sha256'):
            if status.get(field) != mapping.get(field):
                return False, 'localization status belongs to another map snapshot'
        if status.get('mapping_process_id') != mapping.get('process_id') or status.get('load_id') != mapping.get('load_id'):
            return False, 'mapping was restarted or its database reloaded'
        return True, ''
    except (OSError, KeyError, ValueError, TypeError, RuntimeError) as exc:
        return False, f'localization unavailable: {exc}'


def mapping_navigation_ready(directory):
    """Require a fresh final-guard acknowledgement for this mapping load.

    The guard checks live odometry, scan and TF itself before publishing ready.
    Admission and dispatch also check Nav2 and its current costmap locally.
    """
    try:
        directory = Path(directory)
        mapping = read_record(directory / 'mapping.json', 0.6)
        navigation = read_record(directory / 'navigation.json', 0.3)
        return (mapping.get('ready') is True and mapping.get('mode') == 'mapping'
                and navigation.get('ready') is True
                and navigation.get('mapping_process_id') == mapping['process_id']
                and navigation.get('load_id') == mapping['load_id'])
    except (OSError, KeyError, ValueError, TypeError, RuntimeError):
        return False
