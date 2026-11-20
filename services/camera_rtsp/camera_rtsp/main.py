"""Atlas registration and Driver lifecycle for the camera RTSP service."""
import logging
import threading

from robonix_api import ATLAS, Err, Ok, Service
from robonix_api.atlas_types import Transport
import camera_rtsp_pb2 as camera_pb

from .runtime import Pipeline, Settings

service = Service(id="camera_rtsp", namespace="robonix/service/camera_rtsp")
RGB = "robonix/primitive/camera/rgb"
_settings = None
_pipeline = None
_active = False
_lock = threading.RLock()


def discover(settings):
    return ATLAS.find_unique_capability(contract_id=RGB, transport=Transport.ROS2,
                                        provider_id=settings.camera_provider_id)


@service.on_init
def init(config):
    """Validate cold settings and require an unambiguous ROS 2 RGB capability."""
    global _settings
    try:
        candidate = Settings.parse(config)
        discover(candidate)
        _settings = candidate
        return Ok()
    except Exception as exc:
        return Err(str(exc))


@service.on_activate
def activate():
    """Acquire resources and become available only after an RTSP frame decodes."""
    global _pipeline, _active
    with _lock:
        if _settings is None:
            return Err("service has not been initialized")
        if _pipeline is not None:
            return Err("deactivate before activating again")
        try:
            channel = service.connect_capability(discover(_settings), RGB, Transport.ROS2)
            _pipeline = Pipeline(_settings, channel)
            _pipeline.start()
            _active = True
            return Ok()
        except Exception as exc:
            logging.exception("camera RTSP activation failed")
            cleanup = deactivate()
            return Err(f"{exc}; cleanup: {cleanup}")


@service.on_deactivate
def deactivate():
    """Release even partially acquired resources; preserve failed cleanup for retry."""
    global _pipeline, _active
    with _lock:
        _active = False
        try:
            if _pipeline is not None:
                _pipeline.close()
                _pipeline = None
            return Ok()
        except Exception as exc:
            return Err(f"RTSP cleanup failed: {exc}")


@service.on_shutdown
def shutdown():
    return deactivate()


@service.grpc("robonix/service/camera_rtsp/get_stream")
def get_stream(_request, context):
    """Reject inactive calls; report asynchronous input or encoder failures explicitly."""
    import grpc
    with _lock:
        if not _active or _pipeline is None:
            context.abort(grpc.StatusCode.FAILED_PRECONDITION, "camera RTSP service is not active")
        pipeline = _pipeline
        available = pipeline.ready and pipeline.healthy()
        return camera_pb.GetStream_Response(available=available,
            url=_settings.rtsp_url if available else "",
            camera_provider_id=_settings.camera_provider_id,
            error="" if available else (pipeline.error or "RTSP pipeline is unavailable"))


if __name__ == "__main__":
    service.run()
