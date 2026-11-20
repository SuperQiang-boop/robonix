"""Atlas registration, lifecycle and RPCs for the relocalization service."""
from __future__ import annotations

import logging
import threading

from robonix_api import ATLAS, Err, Ok, Service
from robonix_api.atlas_types import Transport

from .runtime import Runtime
from .settings import Settings
from .ros_status import StatusPublisher
from .backend import Backend

service = Service(id="relocalization", namespace="robonix/service/relocalization")

# Service construction adds this package's generated protobuf directory.
import grpc  # noqa: E402
import relocalization_pb2 as pb  # noqa: E402

_lock = threading.RLock()
_settings = None
_runtime = Runtime()
_publisher = None
_channels = []
_status_topic = None
_backend = None


def dependencies(settings):
    """Bind each required capability to its configured provider via Atlas."""
    return (
        (settings.lidar_provider_id, "robonix/primitive/lidar/lidar3d", Transport.ROS2),
        (settings.odom_provider_id, "robonix/primitive/chassis/odom", Transport.ROS2),
        (settings.mapping_provider_id, "robonix/service/map/pose_estimate", Transport.GRPC),
        (settings.mapping_provider_id, "robonix/service/map/pose", Transport.ROS2),
        (settings.mapping_provider_id, "robonix/service/map/lifecycle", Transport.ROS2),
    )


@service.on_init
def init(config):
    """Validate cold configuration; no map or ROS resources are acquired here."""
    global _settings
    with _lock:
        if _publisher is not None or _channels:
            return Err("deactivate before initializing again")
        try:
            _settings = Settings.parse(config)
            return Ok()
        except (ValueError, TypeError) as exc:
            return Err(str(exc))


@service.on_activate
def activate():
    """Connect Atlas endpoints and assemble real matching, handoff and monitoring."""
    global _publisher, _status_topic, _backend
    with _lock:
        if _settings is None:
            return Err("service has not been initialized")
        if _publisher is not None or _channels:
            return Err("deactivate before activating again")
        try:
            for provider_id, contract_id, transport in dependencies(_settings):
                capability = ATLAS.find_unique_capability(contract_id=contract_id,
                    transport=transport, provider_id=provider_id)
                _channels.append(service.connect_capability(capability, contract_id, transport))
            _publisher = StatusPublisher(_runtime, _settings, service.id)
            # Atlas declarations last for the provider registration, including
            # inactive periods. Reuse the authoritative topic on reactivation.
            if _status_topic is None:
                _status_topic = service.declare_ros2_topic(
                    "robonix/service/relocalization/status",
                    _publisher.topic, qos="reliable")
            _publisher.topic = _status_topic
            _publisher.start()
            if _settings.profile_file is None or _settings.coordination_dir is None:
                raise RuntimeError('profile_file and coordination_dir are required for a usable backend')
            endpoints = {
                'lidar': next(ch.endpoint for ch in _channels if ch.contract_id.endswith('/lidar3d')),
                'odom': next(ch.endpoint for ch in _channels if ch.contract_id.endswith('/odom')),
                'pose': next(ch.endpoint for ch in _channels if ch.contract_id.endswith('/map/pose')),
                'estimate': next(ch.endpoint for ch in _channels if ch.contract_id.endswith('/pose_estimate')),
            }
            _backend = Backend(_settings, _publisher.node, endpoints)
            _publisher.backend = _backend
            _runtime.activate(_settings, _backend.run)
            return Ok()
        except Exception as exc:
            logging.exception("relocalization activation failed")
            cleanup = deactivate()
            return Err(f"{exc}; cleanup: {cleanup}")


@service.on_deactivate
def deactivate():
    """Invalidate availability and release all owned resources, retaining failures."""
    global _publisher, _backend
    with _lock:
        try:
            _runtime.deactivate()
        except Exception as exc:
            # Keep ROS and channels alive while a worker may still use them.
            return Err(f"worker cleanup: {exc}")
        errors = []
        if _backend is not None:
            try:
                _backend.close()
                _backend = None
            except Exception as exc:
                return Err(f'backend cleanup: {exc}')
        if _publisher is not None:
            try:
                _publisher.close()
                _publisher = None
            except Exception as exc:
                errors.append(f"status cleanup: {exc}")
        remaining = []
        for channel in reversed(_channels):
            try:
                channel.close()
            except Exception as exc:
                remaining.append(channel)
                errors.append(f"channel cleanup: {exc}")
        _channels[:] = list(reversed(remaining))
        return Err("; ".join(errors)) if errors else Ok()


@service.on_shutdown
def shutdown():
    return deactivate()


@service.grpc("robonix/service/relocalization/relocalize")
def relocalize(request, context):
    """Reject invalid or inactive calls, and never accept work without a backend."""
    try:
        result = _runtime.relocalize(request.map_id, request.generation,
                                    request.database_sha256, request.timeout_s)
        return pb.Relocalize_Response(**result)
    except ValueError as exc:
        context.abort(grpc.StatusCode.INVALID_ARGUMENT, str(exc))
    except RuntimeError as exc:
        context.abort(grpc.StatusCode.FAILED_PRECONDITION, str(exc))


@service.grpc("robonix/service/relocalization/relocalize_by_map")
def relocalize_by_map(request, context):
    """Resolve the loaded map snapshot and schedule the standard async request."""
    try:
        with _lock:
            if _backend is None:
                raise RuntimeError('relocalization service backend is not ready')
            identity = _backend.current_map_identity(request.map_id)
            if not _backend.has_map_index(identity):
                raise RuntimeError('map index is still being prepared; retry after status is ready')
            result = _runtime.relocalize(**identity, timeout_s=0)
        return pb.RelocalizeByMap_Response(**result)
    except ValueError as exc:
        context.abort(grpc.StatusCode.INVALID_ARGUMENT, str(exc))
    except RuntimeError as exc:
        context.abort(grpc.StatusCode.FAILED_PRECONDITION, str(exc))


@service.grpc("robonix/service/relocalization/get_status")
def get_status(_request, _context):
    """Allow availability inspection even before activation and after failures."""
    return pb.GetStatus_Response(status=pb.Status(**_runtime.status()))


@service.grpc("robonix/service/relocalization/cancel")
def cancel(request, context):
    """Cancel only work matching the accepted request identity."""
    try:
        return pb.Cancel_Response(**_runtime.cancel(request.request_id))
    except ValueError as exc:
        context.abort(grpc.StatusCode.INVALID_ARGUMENT, str(exc))


if __name__ == "__main__":
    service.run()
