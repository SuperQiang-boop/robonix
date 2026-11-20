"""Private navigation checks shared by the wrapper and final velocity guard."""
from __future__ import annotations

from pathlib import Path

from .coordination import read_record


def mapping_active(directory):
    """Recognize fresh mapping only to leave manual mapping motion undisturbed.

    This never grants Nav2 admission; check() still requires localization.
    Transitions, stale records and missing mapping processes fail closed.
    """
    try:
        mapping = read_record(Path(directory) / 'mapping.json', 0.6)
        return mapping.get('ready') is True and mapping.get('mode') == 'mapping'
    except (OSError, KeyError, ValueError, TypeError, RuntimeError):
        return False


def check(directory, provider_id='relocalization'):
    """Check fresh validity and exact mapping process/load before accepting motion."""
    try:
        directory = Path(directory)
        mapping = read_record(directory / 'mapping.json', 0.6)
        status = read_record(directory / 'status.json', 0.4)
        if not mapping.get('ready') or mapping.get('mode') != 'localization':
            return False, 'mapping localization snapshot is unavailable'
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
