# robonix-planner

`robonix-planner` is the VLM-driven reasoning and planning service for Robonix. It receives `SystemPlanner` tasks, builds the model prompt from the current Src capability catalog, asks the VLM for an RTDL plan, expands that RTDL into a `Plan`, and sends the plan to `robonix-executor`.

## Build

From the repo root:

```sh
cargo build -p robonix-planner
```

The crate is part of the top-level Cargo workspace and is built / installed by
`make build` and `make install` respectively.

## Run

Manual launch:

```sh
robonix-planner \
  --vlm-upstream https://api.openai.com/v1 \
  --vlm-api-key sk-... \
  --vlm-model gpt-5.5
```

Common configuration:

- `--src` / `ROBONIX_SRC_ENDPOINT`: Src endpoint. Defaults to `127.0.0.1:50051`.
- `--listen` / `ROBONIX_PLANNER_LISTEN`: Planner gRPC listen address. Defaults to `127.0.0.1:50071`.
- `--id` / `ROBONIX_PLANNER_PROVIDER_ID`: Planner provider id registered with Src. Defaults to `planner`.
- `--vlm-upstream` / `ROBONIX_VLM_UPSTREAM`: OpenAI-compatible API base URL.
- `--vlm-api-key` / `ROBONIX_VLM_API_KEY`: VLM API key.
- `--vlm-model` / `ROBONIX_VLM_MODEL`: VLM model name.
- `--vlm-format` / `ROBONIX_VLM_FORMAT`: API dialect. Only `openai` is currently supported.
- `--config` / `ROBONIX_CONFIG_PATH`: optional YAML config file.
- `--log`: env_logger filter. Falls back to `RUST_LOG`, then `robonix_planner=info`.
- `ROBONIX_PLANNER_SOUL`: optional path to a SOUL markdown file. If unset, Planner tries `~/.robonix/SOUL.md`.
- `ROBONIX_PLANNER_MAX_TOOL_ROUNDS`: maximum RTDL execution rounds per turn. Defaults to `64`.

## Prompt assets

The VLM-facing RTDL envelope rules (grammar, example, constraints) live in `rtdl_protocol.md` at the crate root and are embedded at compile time via `include_str!`. Planner sends that frozen document on the first planning round and a bounded grammar/admission reminder on later rounds. Edit the document to change the authoritative RTDL instructions without touching `planner.rs`.

Planner's standing system prompt is built in `src/planner.rs`; it includes the runtime operating principles, including the rule that failed required capability calls stop autonomous physical task progress until the user confirms the next step.

For World-resolved navigation, Planner computes the planar quaternion from the
successful `goal_near`/`goal_room` result in code before publishing or dispatching
the Plan. An LLM's quaternion is replaced only when the map-frame XY matches a
World goal dispatched in the same user interaction (within 0.1 mm). Independent
positions, other frames and direct navigation clients are unchanged. Results
are matched by call id, provider and contract; a user steer clears both pending
resolutions and cached poses. Matching resolutions older than 120 seconds must
be requested again. Resolution and dependent navigation in the same tree are
rejected. This does not grant arrival success or bypass Nav2 collision checks.
Offline regression: `cargo test -p robonix-planner navigation_goal`.

Src discovery still runs before every planning round. When its LLM-visible provider, contract, description, and schema fields are unchanged, Planner reuses the rendered capability catalog. The startup-cached Robotdesc YAML is serialized as compact JSON while the authoritative URDF remains unchanged.

Every request writes a `[planner/prompt]` JSON log with text bytes and four-byte token estimates for the standing context, RTDL protocol, capability catalog, task, in-flight trees, Executor state, live embodiment, environment, history, and correction. Planner supplies a random per-turn `prompt_cache_key`, with stable sections ordered before live state, so supporting providers can reuse the longest unchanged prefix without receiving a session identifier. Compatible providers also report exact prompt/completion tokens and cached prompt tokens through streaming usage; those totals are logged separately under the same prefix. The fake VLM's reported usage is explicitly a deterministic four-byte estimate, not a production tokenizer result. A provider that rejects a named optional cache/usage field with HTTP 400 or 422 is retried once without those fields; unrelated client errors are returned unchanged.

## RTDL Planning Flow

Planner no longer sends OpenAI `tools` / function schemas as the primary planning path. Instead, it writes the RTDL grammar and available capability list into the prompt. The model must return a single JSON object:

```json
{
  "content": "I will inspect the current scene.",
  "rtdl": {
    "op": "sequence",
    "children": [
      {
        "op": "do",
        "cap": "camera_snapshot",
        "args": {}
      }
    ]
  }
}
```

MVP RTDL supports only:

- `sequence`: ordered children.
- `parallel`: concurrent children. Executor waits for every child and does not cancel sibling branches when one fails.
- `do`: one capability call, where `cap` is the unique `capability_name` shown in the prompt and `args` is a JSON object.

Planner buffers the full assistant JSON before showing user-visible text, validates the RTDL, expands it into an arena-style `Plan { nodes, root_index }`, sends that `Plan` to Gateway, and dispatches it to Executor. Executor interprets `sequence`, `parallel`, and `do` nodes directly.

Parallel example:

```json
{
  "content": "I will inspect both signals.",
  "rtdl": {
    "op": "parallel",
    "children": [
      { "op": "do", "cap": "camera_snapshot", "args": {} },
      { "op": "do", "cap": "battery_status", "args": {} }
    ]
  }
}
```

An empty sequence means the model is done for the turn:

```json
{ "content": "Done.", "rtdl": { "op": "sequence", "children": [] } }
```

### Temporary requests and greeting state

A root `cancel_plan` can opt into `resume_after: true`. Planner retains the interrupted user goal and dispatched step/results in its session state, independently of model history, waits for cancellation, and restores only unfinished intent after successful temporary work and explicit `task_update.verified: true`. `done` with false or omitted verification does not trigger return, and structured capability errors inhibit it even when transport succeeds. A replacement discards the targeted task while preserving unrelated retained parents; a global stop clears automatic return; failure or uncertain cancellation blocks it. Records are in memory: process restart never automatically restarts physical work. Nonrestartable actions still require capability-specific decisions.

`ROBONIX_GREETING_STATE_PATH` optionally reads a local atomic JSON snapshot produced by the G1 greeting skill. Planner refreshes it before planning and dispatch and watches transitions every 500 ms; it waits to dispatch new business trees while greeting/conversation inhibits movement. Cancellation remains available. Stop invalidates late voice requests for the matching session and encounter. Greeting voice observes the existing status stream without taking it away from the frontend; restoring the original goal emits its in-progress task state. Without this variable, greeting coordination is disabled. The snapshot is local deployment coordination, not a new capability contract. Run `cargo test -p robonix-planner task_control` and the fake-VLM interruption smoke test before deployment.
