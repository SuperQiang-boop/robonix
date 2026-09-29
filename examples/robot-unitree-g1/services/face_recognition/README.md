# Face recognition service

This package bridges the external VSIS C++ face recognition executable into Robonix. It preserves the external camera and InspireFace implementation while adding Atlas registration, Driver lifecycle, a readiness gate, process cleanup, and confirmed face-encounter events.

Build with `bash scripts/build.sh`. Start through `rbnx boot` after adding the service to the deployment manifest. The package includes `vsis_video_service/` with the executable, runtime library, configuration, InspireFace assets, feature database, mapping, and photos. Omit `external_root` to use this bundled project, or set it to another directory containing the same layout. Greeting speech is owned by `g1_face_greeting`.

The service requires a working external camera/feature database. It becomes active only after the VSIS `FACE_READY` protocol line is observed. `deactivate` stops only the child process and bridge-owned resources. Speech is handled by `g1_face_greeting`, not by this service. `people/resolve` returns an ID for an exact, unique registered name; missing and ambiguous names fail. `events/next` includes `observed_at`, the time the encounter entered the bridge queue, so later-created message tasks cannot be triggered by buffered earlier events.

Video sources accept RTSP, HTTP, and HTTPS stream URLs through OpenCV's FFmpeg backend, or V4L2 device numbers/paths such as `0` and `/dev/video0`. For example, set `video.source` in the face recognition service's `config` in `robonix_manifest.yaml` to `http://camera-host:8080/video.mjpg`. The HTTP(S) endpoint must provide a continuous video stream in a format supported by the installed OpenCV/FFmpeg build (for example, MJPEG); a normal web page or a single still-image URL is not necessarily a video stream. HTTPS additionally depends on FFmpeg TLS and certificate support on the target system. Network capture open/read operations use five-second timeouts before the existing reconnect loop retries the source.

The enrollment command supports `face_enroll add <name> <photo> [intimacy] [hello]`.
Legacy records without intimacy fields load as level 1. The service counts one
encounter after the configured confirmation hits and absence interval, persists
the count atomically, and emits it through
`robonix/service/face_recognition/events/next`.

Managed VSIS configuration (`log`, `video`, `face`, `audio`) is validated by
the bridge and passed to the subprocess in a temporary `cfg.yaml`; it does
not rewrite the bundled `vsis_video_service/config/cfg.yaml`. That file
remains a fallback only when launching the C++ executable directly. Set
`face.intimacy_upgrade_encounters: [5, 15, 30, 60]` in the manifest to
choose the cumulative encounter thresholds for levels 2–5, respectively.
Thresholds must be positive and strictly increasing. Confirmed encounters
are stored in `vsis_video_service/person/mapping.json` and survive restarts;
changing thresholds recalculates the automatic level from those counts on
the next start. The effective level remains the higher of the registered
base level and the automatic level. `audio` is forwarded for compatibility
but greeting speech is owned by `g1_face_greeting`.

Rebuild the VSIS binary after updating its source. From this package directory:

```bash
cmake -S vsis_video_service -B rbnx-build/vsis -DCMAKE_BUILD_TYPE=Release
cmake --build rbnx-build/vsis --parallel 2
```

Then restart the face recognition service through the deployment lifecycle.
