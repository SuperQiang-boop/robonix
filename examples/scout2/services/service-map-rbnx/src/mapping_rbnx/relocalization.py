"""Optional same-host integration with the independently deployed service."""
from __future__ import annotations

from contextlib import nullcontext
import hashlib
import logging
import os
from pathlib import Path
import sys
import threading

log = logging.getLogger(__name__)
writer = None
_startup = None
_stop = threading.Event()
snapshots = {}


def configure(provider_id, config=None):
    """Enable coordination for a deployment using either mapping or localization."""
    global writer
    config = config or {}
    directory = config.get('coordination_dir') or os.environ.get('RELOCALIZATION_COORDINATION_DIR')
    if not directory:
        return
    package_root = config.get('relocalization_package_root') or os.environ.get('RELOCALIZATION_PACKAGE_ROOT')
    if not package_root:
        raise ValueError('relocalization_package_root is required with coordination_dir')
    invocation_dir = os.environ.get('RBNX_INVOCATION_CWD')
    if not Path(directory).is_absolute():
        if not invocation_dir or not Path(invocation_dir).is_absolute():
            raise ValueError('relative coordination_dir requires RBNX_INVOCATION_CWD')
        directory = Path(invocation_dir) / directory
    if not Path(package_root).is_absolute():
        if not invocation_dir or not Path(invocation_dir).is_absolute():
            raise ValueError('relative relocalization_package_root requires RBNX_INVOCATION_CWD')
        package_root = Path(invocation_dir) / package_root
    root = Path(package_root).resolve(strict=True)
    sys.path.insert(0, str(root))
    from relocalization_rbnx.coordination import MapWriter
    if writer is not None:
        if writer.directory == Path(directory) and writer._state['provider_id'] == provider_id:
            return
        raise RuntimeError('deactivate mapping before changing relocalization configuration')
    writer = MapWriter(directory, provider_id)


def transition():
    return writer.transition() if writer is not None else nullcontext()


def fingerprint(database, map_id):
    """Hash a stable, sidecar-free saved checkpoint and its persisted generation."""
    path = Path(database)
    for suffix in ('-wal', '-shm', '-journal'):
        if Path(str(path)+suffix).exists():
            raise ValueError('save a stable checkpoint before localization')
    def version():
        """Compare content metadata without read-induced access-time changes."""
        stat = path.stat()
        return stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns
    stat = version()
    generation = int((path.parent / 'generation').read_text().strip())
    if not 0 <= generation < 2**64:
        raise ValueError('invalid saved map generation')
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    if version() != stat:
        raise ValueError('saved checkpoint changed while hashing')
    return dict(map_id=map_id, generation=generation, database_sha256=digest.hexdigest())


def loaded(runtime_db):
    """Publish the source identity captured while making this runtime copy."""
    if writer is not None:
        if _stop.is_set():
            raise RuntimeError('mapping shutdown interrupted localization load')
        from . import lifecycle
        identity = snapshots[runtime_db]
        state = lifecycle.current()
        if any(identity[k] != state[k] for k in ('map_id', 'generation')):
            raise ValueError('loaded lifecycle differs from the copied map identity')
        writer.loaded(identity)


def mapping(map_id):
    """Publish mapping readiness without requiring a saved localization map."""
    if writer is not None:
        writer.mapping(map_id)


def start(map_id, mode):
    """Confirm startup via the real database-load RPC without blocking Driver INIT."""
    global _startup
    if writer is None:
        return
    _stop.clear()
    if mode == 'mapping':
        log.info('mapping mode active; relocalization will wait for a localization map')
        return
    if mode != 'localization' or not map_id:
        raise ValueError('automatic relocalization requires a named localization map')

    def run():
        """Retry engine startup only; successful load publishes one new identity."""
        from . import map_ops
        from rtabmap_msgs.srv import LoadDatabase
        node = map_ops._get_node()
        if node is None:
            log.error('cannot confirm localization load: ROS node unavailable')
            return
        client = node.create_client(LoadDatabase, f'{map_ops.RTABMAP_NS}/load_database')
        try:
            for _ in range(120):
                if _stop.is_set():
                    return
                if client.wait_for_service(timeout_sec=1):
                    result = map_ops.load_map_impl(map_id)
                    if result['ok']:
                        log.info('localization snapshot ready: %s', result['detail'])
                    else:
                        log.error('localization startup load failed: %s', result['detail'])
                        if result['detail'] == 'navigation did not acknowledge a stationary stop':
                            # Nav2's guard can start after mapping/relocalization
                            # INIT. No database operation happened on this path.
                            continue
                    return
            log.error('localization engine startup confirmation timed out')
        finally:
            node.destroy_client(client)

    _startup = threading.Thread(target=run, name='mapping-localization-load', daemon=True)
    _startup.start()


def close():
    """Revoke readiness and terminate startup work before normal map shutdown."""
    global writer
    _stop.set()
    if writer is not None:
        writer.invalidate()
    if _startup is not None:
        _startup.join(timeout=5)
        if _startup.is_alive():
            raise RuntimeError('mapping startup load has not stopped')
    if writer is not None:
        writer.close()
        writer = None
