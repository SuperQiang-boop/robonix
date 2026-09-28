# Face recognition service

This package is a Robonix service bridge for the external bundled `vsis_video_service/` C++ executable. The external program keeps ownership of camera capture, InspireFace inference, the face feature database, and `person/mapping.json`; greeting behavior is owned by `g1_face_greeting`.

The bridge registers `robonix/service/face_recognition/driver` and `robonix/service/face_recognition/events/next`, validates the external executable during `CMD_INIT`, starts the VSIS executable, and reports `ACTIVE` only after the configured ready marker (`FACE_READY` by default) appears in the child output. The event capability returns confirmed registered-face encounters from a bounded queue.

The VSIS configuration remains in `<external_root>/config/cfg.yaml`. Its audio settings are retained for compatibility but are not used by the event-only bridge. Greeting speech is owned by `g1_face_greeting`.

The bridge owns one child process, one output-drain thread, and the bounded event queue. Deactivation and shutdown release each resource and are safe to repeat. The external project must provide its executable, InspireFace runtime library, model pack, feature database, mapping file, photos, and a valid camera source before activation. Speech is intentionally not owned by this service; `g1_face_greeting` is the single greeting behavior owner.
