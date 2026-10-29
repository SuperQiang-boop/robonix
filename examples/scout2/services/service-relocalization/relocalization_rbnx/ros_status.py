"""Own the ROS context and periodic status publisher independently of algorithms."""
from __future__ import annotations

import logging
import threading
import time
from pathlib import Path

from .coordination import MapReader, write_record

_AUTO_READY_STABLE_S = 1.0
_AUTO_RETRY_DELAY_S = 1.0
_AUTO_RETRY_LIMIT = 3
_INDEX_RETRY_MAX_DELAY_S = 30.0


class StatusPublisher:
    """Publish status at a bounded rate; consumers must expire missing samples."""

    def __init__(self, runtime, settings, provider_id):
        self.runtime, self.settings = runtime, settings
        self.runtime_provider_id = provider_id
        self.topic = f"/robonix/relocalization/{provider_id}/status"
        self.context = self.node = self.executor = self.thread = None
        self.error = ""
        self._stop = threading.Event()
        self._supervisor = None
        self._coordination_owner = None
        self.backend = None

    def start(self):
        """Create only owned ROS resources; close() handles partial startup."""
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rclpy.signals import SignalHandlerOptions
        from relocalization.msg import Status

        self.context = Context()
        rclpy.init(args=[], context=self.context, signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node("relocalization_status", context=self.context,
            parameter_overrides=[Parameter("use_sim_time", value=self.settings.use_sim_time)])
        self.publisher = self.node.create_publisher(Status, self.topic,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.node.create_timer(self.settings.status_period_s, self.publish)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.publish()
        self.thread = threading.Thread(target=self.run, name="relocalization-status", daemon=True)
        self.thread.start()
        if self.settings.coordination_dir is not None:
            import fcntl
            directory = Path(self.settings.coordination_dir)
            directory.mkdir(parents=True, exist_ok=True)
            self._coordination_owner = (directory / 'relocalization.owner').open('a')
            fcntl.flock(self._coordination_owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self._supervisor = threading.Thread(target=self.supervise,
                name='relocalization-map-monitor', daemon=True)
            self._supervisor.start()

    def supervise(self):
        """Cancel on any load transition and automatically attempt each ready load.

        This runs on wall time independently of the ROS executor. It also writes
        the private status consumed by navigation's final velocity guard. A ready
        load must remain visible briefly before the first attempt, and a failed
        automatic attempt gets a small bounded retry budget so mapping and TF can
        finish settling without turning a bad map into an endless task loop. The
        index preparation retries with capped backoff independently of that
        request budget. Map identity comes only from mapping's heartbeat.
        """
        reader = MapReader(self.settings.coordination_dir, self.settings.mapping_provider_id)
        loads, previous = {}, None
        while not self._stop.is_set():
            try:
                mapping = reader.state()
                marker = (mapping['process_id'], mapping['load_id'])
                if previous is not None and marker != previous:
                    self.runtime._invalidate('mapping process or database load changed')
                previous = marker
                if mapping['mode'] == 'mapping':
                    self.runtime.set_detail('mapping mode active; waiting for a localization map')
                    status = self.runtime.status()
                    status['map_id'] = mapping.get('map_id', '')
                else:
                    now = time.monotonic()
                    state = loads.setdefault(marker, dict(first_seen=now, attempts=0,
                        request_id='', next_attempt=now+_AUTO_READY_STABLE_S, exhausted=False,
                        index_ready=False, index_retry_delay=_AUTO_RETRY_DELAY_S,
                        next_index_attempt=now))
                    status = self.runtime.status()
                    if (status['active'] and not state['index_ready'] and
                            now >= state['next_index_attempt']):
                        if self.backend is None:
                            self.runtime.set_detail('waiting for relocalization backend')
                        else:
                            self.runtime.set_detail(f"preparing index for loaded map {mapping['map_id']}")
                            try:
                                self.backend.prepare_map_index({key: mapping[key] for key in
                                    ('map_id', 'generation', 'database_sha256')})
                                state['index_ready'] = True
                                self.runtime.set_detail('')
                            except Exception as exc:
                                delay = state['index_retry_delay']
                                state['next_index_attempt'] = time.monotonic() + delay
                                state['index_retry_delay'] = min(delay * 2, _INDEX_RETRY_MAX_DELAY_S)
                                logging.error('map index preparation failed for %s: %s',
                                              mapping['map_id'], exc, exc_info=True)
                                self.runtime.set_detail(f'map index preparation failed: {exc}')
                    if self.settings.auto_relocalize and status['active'] and not status['busy']:
                        if status['localization_valid']:
                            state['exhausted'] = True
                        elif state['request_id'] and status['request_id'] == state['request_id']:
                            state['request_id'] = ''
                            state['next_attempt'] = now + _AUTO_RETRY_DELAY_S
                        if (state['index_ready'] and not state['exhausted'] and not state['request_id'] and
                                state['attempts'] < _AUTO_RETRY_LIMIT and now >= state['next_attempt']):
                            response = self.runtime.relocalize(mapping['map_id'], mapping['generation'],
                                mapping['database_sha256'], self.settings.timeout_s)
                            state['attempts'] += 1
                            if response['accepted']:
                                state['request_id'] = response['request_id']
                            else:
                                state['next_attempt'] = now + _AUTO_RETRY_DELAY_S
                                logging.warning('automatic relocalization rejected: %s', response['detail'])
                            if state['attempts'] >= _AUTO_RETRY_LIMIT and not state['request_id']:
                                state['exhausted'] = True
                    status = self.runtime.status()
                status.update(provider_id=self.runtime_provider_id,
                    mapping_process_id=mapping['process_id'], load_id=mapping['load_id'])
            except Exception as exc:
                self.runtime._invalidate(f'mapping unavailable: {exc}')
                status = dict(self.runtime.status(), provider_id=self.runtime_provider_id)
                status['localization_valid'] = False
            try:
                write_record(Path(self.settings.coordination_dir) / 'status.json', status)
            except Exception as exc:
                self.runtime._invalidate(f'localization status publication failed: {exc}')
                logging.error('localization coordination publication failed: %s', exc)
            self._stop.wait(0.1)

    def publish(self):
        """Attach ROS publication time; never substitute it for acquisition time."""
        from relocalization.msg import Status
        values = self.runtime.status(self.node.get_clock().now().nanoseconds)
        self.publisher.publish(Status(**values))

    def run(self):
        """Expose unexpected executor failure and invalidate runtime availability."""
        try:
            self.executor.spin()
        except Exception as exc:
            self.error = str(exc)
            self.runtime.deactivate()
            logging.exception("relocalization status publisher failed")

    def close(self):
        """Stop the executor before destroying its node and ROS context."""
        self._stop.set()
        if self._supervisor is not None:
            self._supervisor.join(timeout=3)
            if self._supervisor.is_alive():
                raise RuntimeError('mapping identity supervisor did not stop')
        self.runtime.deactivate()
        if self.executor is not None:
            if not self.executor.shutdown(timeout_sec=5):
                raise RuntimeError("relocalization ROS executor did not stop")
        if self.thread is not None:
            self.thread.join(timeout=5)
            if self.thread.is_alive():
                raise RuntimeError("relocalization status thread did not stop")
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self.context is not None and self.context.ok():
            self.context.shutdown()
        if self._coordination_owner is not None:
            self._coordination_owner.close()
            self._coordination_owner = None
