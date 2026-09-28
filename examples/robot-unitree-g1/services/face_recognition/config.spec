# Configuration passed to Driver INIT (JSON object). Unknown keys are rejected.
# external_root: optional directory containing the VSIS service binary and assets.
#   Default: the bundled services/face_recognition/vsis_video_service directory.
# binary_path: optional executable path, default <external_root>/bin/vsis_video_service.
# startup_timeout: finite seconds (1,300], default 45. Waits for the external ready marker.
# ready_marker: optional child-output marker, default "FACE_READY". The VSIS
# child emits this protocol line independently of the configured log sinks.
# log/video/face/audio: VSIS configuration sections passed through the Driver.
# The bridge writes them into an ephemeral cfg.yaml for the child; the bundled
# config/cfg.yaml is only used when VSIS is run without Robonix.
# log: mode 0..2 (default 2), level 0..5 (default 1).
# video: source (default "", RTSP/HTTP(S) URL or V4L2 device),
#   reconnect_interval_ms positive (default 3000), width/height nonnegative
#   (defaults 1280/720; ignored for network streams).
# face: search_threshold [0.45,0.5] (default 0.45), greet_cooldown_sec >= 0
#   (default 30), max_faces >= 1 (default 5), confirm_hits >= 1 (default 1),
#   absent_sec >= 1 (default 3), intimacy_upgrade_encounters exactly four
#   strictly increasing positive integers for levels 2..5 (default [5,15,30,60]).
# audio: host nonempty (default 127.0.0.1), port 1..65535 (default 10086).
# The face encounter event capability is exposed at
# robonix/service/face_recognition/events/next.
