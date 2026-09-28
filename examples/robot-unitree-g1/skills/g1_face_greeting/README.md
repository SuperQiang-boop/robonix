# G1 Face Greeting

This skill consumes confirmed encounter events from `face_recognition`. It is the only owner of greeting speech and intimacy-selected arm actions.

The current implementation is stationary-mode orchestration only. Keep `navigation_enabled: false` and `turn_to_face_enabled: false` until navigation pause/resume and camera geometry are verified. In this mode the skill never calls navigation and never rotates the robot; it only speaks the registered `hello` text and runs the configured arm action. **It does not read navigation or motion state**, so if the face service emits an encounter while G1 is moving, this skill can still speak and move its arms. Do not activate it during navigation or other robot motion until an independent safety interlock is added and verified.

Intimacy is supplied by the face service. The default action mapping is:

- levels 1-2: `wave_under_head`
- level 3: `wave_above_head`
- level 4: `make_heart_with_right_hand`
- level 5: `make_heart_with_both_hands`

Build with `bash scripts/build.sh`, which creates a package venv containing FastMCP's HTTP client support, then add this package to the deployment manifest after `face_recognition`, speech, and `g1_arm_actions`. The build intentionally pins MCP to major version 1 and FastMCP to major version 3 because the service and skill use the MCP v1 server API. The virtualenv must be rebuilt if it still resolves incompatible pre-existing packages.
