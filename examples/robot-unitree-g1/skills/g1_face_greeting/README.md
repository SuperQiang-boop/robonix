# G1 Face Greeting

This skill consumes confirmed encounter events from `face_recognition`. It is the only owner of greeting speech and intimacy-selected arm actions.

The current implementation is stationary-mode orchestration only. Keep `navigation_enabled: false` and `turn_to_face_enabled: false` until navigation pause/resume and camera geometry are verified. In this mode the skill never calls navigation and never rotates the robot; it only speaks the registered `hello` text and runs the configured arm action. **It does not read navigation or motion state**, so if the face service emits an encounter while G1 is moving, this skill can still speak and move its arms. Do not activate it during navigation or other robot motion until an independent safety interlock is added and verified.

Intimacy is supplied by the face service. The default action mapping is:

- levels 1-2: `wave_under_head`
- level 3: `wave_above_head`
- level 4: `make_heart_with_right_hand`
- level 5: `make_heart_with_both_hands`

Build with `bash scripts/build.sh`, which creates a package venv containing FastMCP's HTTP client support, then add this package to the deployment manifest after `face_recognition`, speech, and `g1_arm_actions`. The build intentionally pins MCP to major version 1 and FastMCP to major version 3 because the service and skill use the MCP v1 server API. The virtualenv must be rebuilt if it still resolves incompatible pre-existing packages.

Pilot can call `create_message` with an exact registered `person_name`, verbatim `message`, and `sender` (empty string if unspecified). This ordinary-message capability only resolves the person through `face_recognition/people/resolve` and never calls Scene. It then stores a one-time task bound to the registered person ID. `list_messages`, `message_status`, and `cancel_message` query or cancel tasks by ID. Multiple pending messages to the same person are spoken together during the next observed encounter, combined with the normal greeting in one speech request while the arm action starts in parallel. Greeting cooldown does not suppress a pending message. A prior buffered face event cannot deliver a message created later.

Creation is idempotent for an identical pending target, sender, message, and tracked object: repeated Pilot/tool calls return the original `task_id` instead of creating another task. Whitespace and terminal punctuation differences are ignored for this deduplication check. A task that is already `delivered` is not deduplicated, so a later explicit request can intentionally send the same words again.

For the minimal object handoff flow, call the separate `create_object_message` capability with `object_label`, for example `cup`. Only this capability calls `robonix/system/scene/list_objects`, requires exactly one current object with that label, and stores its stable Scene `object_id` in the task. It does not infer relations such as "on the stool" and does not track by label after creation. `message_status` calls Scene again only for tasks created through `create_object_message` and reports `object_status`: `present` when that ID is still listed, `removed` when Scene no longer lists it, and `unknown` when Scene cannot be queried. Under the current agreed assumption, `removed` means the object was taken away. A missing/temporarily unavailable Scene response is never treated as removal.

Example Pilot request:

```text
我把杯子放在机器人面前了。下次见到小B时，告诉他小C让他把杯子拿走。
```

Pilot should call `create_object_message` with `person_name: 小B`, the spoken `message`, `sender: 小C`, and `object_label: cup`. After the encounter, a follow-up query for the task status refreshes the cup state before answering whether it remains in front of the robot. For an ordinary message, call `create_message` and do not query Scene first.

Set `message_store_path` in the deployment manifest to an absolute persistent SQLite path (the example uses `${ROBONIX_DEPLOY_DIR}/data/face_messages.sqlite3`, outside `rbnx-build`). Keep the skill activated after task creation so it continues consuming face events; a restart requires activation again. Task states are `pending`, `delivering`, `delivered`, `uncertain`, and `cancelled`. `delivered` means only that the speech service returned success; speech errors or restart during delivery become `uncertain` and are **not** automatically repeated. A task already being delivered cannot be cancelled. No unsolicited Pilot notification is sent; users can query task status. Task creation is limited to 160 characters per message and 180 combined pending message/sender characters per recipient, to keep one speech request bounded. The current mode still cannot detect robot motion, so do not activate it while navigating.
