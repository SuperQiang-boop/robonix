# Face recognition service

This package is a Robonix service bridge for the external bundled `vsis_video_service/` C++ executable. The external program keeps ownership of camera capture, InspireFace inference, the face feature database, `person/mapping.json`, and its recognition/greeting policy.

The bridge registers `robonix/service/face_recognition/driver`, validates the external executable and the speech dependency during `CMD_INIT`, discovers `robonix/service/speech/speak` through Atlas during activation, starts a loopback `/speak` compatibility proxy, and then starts the VSIS executable. It reports `ACTIVE` only after the configured ready marker (`特征库就绪` by default) appears in the child output.

The VSIS configuration remains in `<external_root>/config/cfg.yaml`. Its `audio.host` and `audio.port` must match `speak_host` and `speak_port` in the deployment config. A successful `/speak` request is translated to the standard speech MCP capability; no fixed speech service address is used by the bridge.

The bridge owns one child process, one loopback HTTP proxy, one Atlas speech channel, and one output-drain thread. Deactivation and shutdown release each resource and are safe to repeat. The external project must provide its executable, InspireFace runtime library, model pack, feature database, mapping file, photos, and a valid camera source before activation.
