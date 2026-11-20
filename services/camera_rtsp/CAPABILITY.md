# Camera RTSP capabilities

Provider kind: service; id: `camera_rtsp`; namespace: `robonix/service/camera_rtsp`.

| Contract | Transport | Shape |
|---|---|---|
| `robonix/service/camera_rtsp/driver` | gRPC | Existing lifecycle Driver.srv |
| `robonix/service/camera_rtsp/get_stream` | gRPC | Empty request; available, url, camera_provider_id, error |

The ROS 2 input is the existing `robonix/primitive/camera/rgb` topic_out contract
(sensor_msgs/Image), selected by configured provider id using Atlas. The Atlas
channel is closed on deactivation and rediscovered on activation. RTSP is an
external media endpoint returned by GetStream, not an Atlas transport.

INIT validates configuration and discovery. ACTIVATE owns ROS context/node/thread,
MediaMTX (unless external), FFmpeg and a temporary server configuration. Success
requires decoding a frame from the published URL. DEACTIVATE and SHUTDOWN release
these resources, including partial activation. Inactive GetStream calls fail with
gRPC FAILED_PRECONDITION; failures while active return available=false, empty URL
and an error. Reactivation requires DEACTIVATE first. No extra lifecycle states.
