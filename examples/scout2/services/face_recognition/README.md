# Face recognition service

This package bridges the external VSIS C++ face recognition executable into Robonix. It preserves the external camera and InspireFace implementation while adding Atlas registration, Driver lifecycle, a readiness gate, process cleanup, and speech capability discovery.

Build with `bash scripts/build.sh`. Start through `rbnx boot` after adding the service to the deployment manifest. The package includes `vsis_video_service/` with the executable, runtime library, configuration, InspireFace assets, feature database, mapping, and photos. Omit `external_root` to use this bundled project, or set it to another directory containing the same layout. The external configuration's `audio.host`/`audio.port` must equal the bridge's `speak_host`/`speak_port`.

The service requires an active `robonix/service/speech/speak` MCP capability and a working external camera/feature database. It becomes active only after the VSIS ready log marker is observed. `deactivate` stops only the child process and bridge-owned resources.

Speech timeout fix: the bridge waits at most 60 seconds for the speech MCP call,
and the bundled C++ caller waits 65 seconds for the HTTP response. Rebuild the
VSIS binary after updating its source; replacing only the Python bridge leaves
the old three-second timeout in place. From this package directory:

```bash
cmake -S vsis_video_service -B rbnx-build/vsis -DCMAKE_BUILD_TYPE=Release
cmake --build rbnx-build/vsis --parallel 2
```

Then restart the face recognition service through the deployment lifecycle.
A timeout leaves playback outcome unknown, so the bridge does not retry speech
automatically. Disconnected HTTP callers are logged without sending a second
response. HTTP 200 from MCP alone does not prove playback succeeded.
