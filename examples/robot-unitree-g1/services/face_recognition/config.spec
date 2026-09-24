# Configuration passed to Driver INIT (JSON object). Unknown keys are rejected.
# external_root: optional directory containing the VSIS service binary and assets.
#   Default: the bundled services/face_recognition/vsis_video_service directory.
# binary_path: optional executable path, default <external_root>/bin/vsis_video_service.
# startup_timeout: finite seconds (1,300], default 45. Waits for the external ready log marker.
# speak_host: loopback address for the compatibility HTTP proxy, default 127.0.0.1.
# speak_port: TCP port consumed by the external binary, default 10086.
# speech_provider_id: optional provider id used to select one speech service capability.
# ready_marker: optional UTF-8 log marker, default "特征库就绪".
