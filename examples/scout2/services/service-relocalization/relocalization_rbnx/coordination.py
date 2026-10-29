"""Private same-host map-load coordination; no additional capability or RPC.

The directory must be shared at the same absolute path by mapping, navigation
and relocalization containers. All participants use the host monotonic clock.
Map transitions revoke readiness before taking an exclusive flock; a task holds
a shared flock through matching and pose handoff. Atomic replacement prevents
partially read identity or status files, and boot IDs reject persisted leases.
"""
from __future__ import annotations

from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import threading
import time
import uuid


def boot_id():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def write_record(path, value):
    """Atomically publish a bounded private record with host-clock freshness."""
    path = Path(path)
    data = dict(value, boot_id=boot_id(), updated_ns=time.monotonic_ns())
    temporary = path.with_name(f'.{path.name}.{uuid.uuid4().hex}')
    try:
        temporary.write_text(json.dumps(data, allow_nan=False))
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def read_record(path, max_age_s=1.0):
    """Reject missing, stale, malformed or previous-boot coordination data."""
    path = Path(path)
    if path.stat().st_size > 65536:
        raise ValueError(f'oversized coordination record: {path.name}')
    value = json.loads(path.read_text())
    age = time.monotonic_ns()-value['updated_ns']
    if value['boot_id'] != boot_id() or not 0 <= age <= max_age_s*1e9:
        raise RuntimeError(f'stale coordination record: {path.name}')
    return value


class MapReader:
    """Bind every request to one mapping process and one successful load."""

    def __init__(self, directory, provider_id):
        self.directory, self.provider_id = Path(directory), provider_id

    def state(self):
        """Return a fresh mapping state for either mapping or localization mode."""
        value = read_record(self.directory / 'mapping.json')
        if value.get('provider_id') != self.provider_id:
            raise RuntimeError('mapping provider identity changed')
        if not value.get('ready') or value.get('mode') not in ('mapping', 'localization'):
            raise RuntimeError('mapping has no ready map state')
        return value

    def current(self):
        """Return only an actually loaded, localization-mode snapshot identity."""
        value = self.state()
        if value.get('mode') != 'localization':
            raise RuntimeError('mapping has no ready localization snapshot')
        return value

    @contextmanager
    def lease(self, deadline, cancelled):
        """Prevent database transitions throughout a task, with bounded waits."""
        with (self.directory / 'map.lock').open('a') as stream:
            while True:
                if cancelled.cancelled or time.monotonic() >= deadline:
                    raise TimeoutError('map lease cancelled or timed out')
                try:
                    fcntl.flock(stream, fcntl.LOCK_SH | fcntl.LOCK_NB)
                    break
                except BlockingIOError:
                    time.sleep(0.02)
            try:
                yield self.current()
            finally:
                fcntl.flock(stream, fcntl.LOCK_UN)


class MapWriter:
    """Mapping-owned readiness heartbeat and exclusive map-transition lock."""

    def __init__(self, directory, provider_id):
        """Prevent competing mapping writers and immediately revoke old readiness."""
        self.directory = Path(directory)
        if not self.directory.is_absolute():
            raise ValueError('coordination directory must be absolute')
        self.directory.mkdir(parents=True, exist_ok=True)
        self._owner = (self.directory / 'mapping.owner').open('a')
        try:
            fcntl.flock(self._owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except Exception:
            self._owner.close()
            raise
        self._lock, self._stop = threading.RLock(), threading.Event()
        self._state = dict(provider_id=provider_id, process_id=uuid.uuid4().hex,
                           load_id=uuid.uuid4().hex, ready=False, mode='')
        self._write()
        self._thread = threading.Thread(target=self._heartbeat,
            name='mapping-snapshot-heartbeat', daemon=True)
        self._thread.start()

    def _write(self):
        write_record(self.directory / 'mapping.json', self._state)

    def _heartbeat(self):
        """Publish liveness; a publication failure stops refreshing the record."""
        while not self._stop.wait(0.2):
            with self._lock:
                self._write()

    def invalidate(self):
        """Revoke old results even for a reload of the same database bytes."""
        with self._lock:
            self._state.update(ready=False, load_id=uuid.uuid4().hex)
            self._write()

    @contextmanager
    def transition(self, timeout_s=10):
        """Revoke first, wait for guarded navigation to stop, then exclude tasks.

        An existing navigation record requires a fresh stationary acknowledgement
        for this load_id. Failure leaves mapping unavailable and does not mutate
        its database. Bootstrap without any navigation process needs no ack.
        """
        self.invalidate()
        deadline = time.monotonic()+timeout_s
        if (self.directory / 'navigation.json').exists():
            while True:
                try:
                    ack = read_record(self.directory / 'navigation.json', 0.5)
                    if ack.get('stopped') and ack.get('load_id') == self._state['load_id']:
                        break
                except (OSError, ValueError, KeyError, RuntimeError):
                    pass
                if time.monotonic() >= deadline:
                    raise TimeoutError('navigation did not acknowledge a stationary stop')
                time.sleep(0.02)
        with (self.directory / 'map.lock').open('a') as stream:
            while True:
                try:
                    fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    break
                except BlockingIOError:
                    if time.monotonic() >= deadline:
                        raise TimeoutError('relocalization did not release the map lease')
                    time.sleep(0.02)
            try:
                yield
            finally:
                fcntl.flock(stream, fcntl.LOCK_UN)

    def loaded(self, identity):
        """Advertise a snapshot only after load_database and occupancy checks."""
        with self._lock:
            self._state.update(identity, ready=True, mode='localization')
            self._write()

    def mapping(self, map_id):
        """Advertise live mapping without claiming localization validity."""
        with self._lock:
            self._state = dict(provider_id=self._state['provider_id'],
                               process_id=self._state['process_id'])
            self._state.update(map_id=map_id, load_id=uuid.uuid4().hex,
                               ready=True, mode='mapping')
            self._write()

    def close(self):
        """Revoke readiness before stopping the heartbeat and releasing ownership."""
        self.invalidate()
        self._stop.set()
        self._thread.join(timeout=2)
        if self._thread.is_alive():
            raise RuntimeError('mapping identity heartbeat did not stop')
        self._owner.close()
