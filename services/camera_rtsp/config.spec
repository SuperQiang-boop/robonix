# Configuration passed to Driver INIT (JSON object). Unknown keys are rejected.
# camera_provider_id: required nonempty string; unique ROS2 RGB provider in Atlas.
# rtsp_url: required rtsp://host:port/path, without credentials/query/fragment.
# manage_server: boolean, default true. Own a MediaMTX server on loopback.
#   false: publish to an existing external RTSP server instead.
# fps: integer 1..60, default 15. Maximum output frame rate; input frames are dropped.
# startup_timeout: finite seconds (0,120], default 25. Includes decoded-frame probe.
# frame_timeout: finite seconds (0,120], default 5. Missing input/blocked writer limit.
# Dimensions and encoding are inferred from the first Image; changes require reactivation.
