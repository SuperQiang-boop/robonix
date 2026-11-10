# robonix-gateway

Gateway is the user-input gateway in front of Planner. It exposes one text/API
submission path and one push-to-talk voice path, normalises identity metadata,
applies optional access policy, then forwards accepted tasks to Planner.

Gateway does not implement ASR, TTS, microphone capture, speaker output, or
voiceprint itself. Those are separate providers discovered through Src.

## Architecture

```
User
  │
  ├─ Text / API task
  │     │
  │     ▼
  │   robonix/system/gateway/submit
  │     │ normalize context_json.user_id
  │     │ optional access check
  │     ▼
  │   Planner SubmitTask
  │     │
  │     ▼
  │   PlannerEvent stream
  │
  └─ Voice session (Ctrl+V in rbnx chat today)
        │
        ▼
      robonix/system/gateway/voice
        │
        ├─ robonix/primitive/audio/mic_stream
        ├─ robonix/service/speech/asr_stream
        ├─ robonix/service/voiceprint/identify
        ├─ voice access gate before Planner/TTS/action
        ├─ build planner::Task with text + context_json.user_id/access
        ├─ Planner SubmitTask
        ├─ optional robonix/service/speech/speak
        ├─ speech/tts + speaker playback when speak is absent
        │
        ▼
      VoiceEvent stream
```

## Contracts

- `robonix/system/gateway/submit`: accepts a `planner::Task` and streams
  `PlannerEvent` responses back to the caller.
- `robonix/system/gateway/voice`: starts a voice session and streams
  `VoiceEvent` state, ASR, voiceprint, Planner, and TTS progress.

Both contracts are registered by the `gateway` provider id.

Voice replies and prompts prefer `robonix/service/speech/speak` over MCP,
using the selected TTS provider and speaker target. This supports G1's built-in
text playback without exporting PCM. When that capability is absent, Gateway
uses `speech/tts` followed by the speaker stream. Playback failures are reported
without retrying another path; native success confirms acceptance, not audible
completion. ASR and microphone selection are independent of reply playback.

## Identity and access

Planner `Task` does not have a top-level `user_id` field. Gateway stores the
normalised identity in `Task.context_json.user_id`:

- text/API path: defaults to `local:<os_user>` when absent,
- voice path: uses `voice:<speaker_id>` from voiceprint when available.

When `ROBONIX_GATEWAY_ACCESS_ENABLED=1`:

- text/API tasks must carry, or be normalised to, a user id listed in
  `ROBONIX_GATEWAY_ALLOWED_USERS`;
- voice turns may capture audio and run voiceprint first, but cannot enter
  Planner, TTS, or any robot action unless voiceprint identifies an enrolled
  speaker above `ROBONIX_GATEWAY_VOICE_THRESHOLD` and `voice:<speaker_id>` is
  allowed;
- a client-provided user hint is audit metadata only for the voice path and
  cannot bypass voiceprint.

Accepted tasks also get `context_json.access` metadata so downstream logs can
distinguish allow-list and voiceprint grants.

## Mock mode

Mock mode skips real microphone and ASR and uses preset text. It is useful for
checking Gateway/Planner plumbing, but it is not a voice identity test.

```bash
ROBONIX_GATEWAY_VOICE_MOCK=1 \
ROBONIX_GATEWAY_VOICE_MOCK_TEXT="hello" \
robonix-gateway
```

## Environment Variables

| Variable | Default | Description |
|------|--------|------|
| `ROBONIX_SRC` | `127.0.0.1:50051` | Src address |
| `ROBONIX_PLANNER_ENDPOINT` | `127.0.0.1:50071` | Planner address |
| `ROBONIX_GATEWAY_PORT` | `50081` | Gateway listen port |
| `ROBONIX_GATEWAY_VOICE_MOCK` | (unset) | Set to `1` to skip real mic+ASR and use preset text |
| `ROBONIX_GATEWAY_VOICE_MOCK_TEXT` | `Hello, please introduce yourself.` | Text used in mock mode |
| `ROBONIX_GATEWAY_SOURCE` | (unset) | Set to `text` to enable the stdin text loop (headless) |
| `ROBONIX_GATEWAY_ACCESS_ENABLED` | `0` | Set to `1` to require access before Planner/TTS/action |
| `ROBONIX_GATEWAY_ALLOWED_USERS` | (empty) | Comma/space separated ids such as `local:alice,voice:alice` |
| `ROBONIX_GATEWAY_VOICE_THRESHOLD` | `0.25` | Minimum voiceprint confidence for a voice user to pass |
| `ROBONIX_GATEWAY_VOICE_SAVE_DIR` | (unset) | Directory for captured voice-session audio dumps |

## TUI Shortcuts

| Key | Function |
|------|------|
| `Enter` | Send text message |
| `Ctrl+V` | Start voice conversation (default 5-second recording) |
| `Esc` | Interrupt the current turn (abort_turn) |
| `Ctrl+C` | Quit |
| `PageUp/PageDown` | Scroll history |

## Voice Path VoiceEvent Types

| kind | Name | Description |
|------|------|------|
| 0 | SESSION_STARTED | Session started |
| 1 | RECORDING_STARTED | Recording / streaming capture started |
| 2 | RECORDING_DONE | Recording finished |
| 3 | ASR_PARTIAL | ASR intermediate result |
| 4 | ASR_FINAL | ASR final result |
| 5 | USER_IDENTIFIED | Voiceprint recognition result |
| 6 | PLANNER | Wrapped PlannerEvent |
| 7 | TTS_STARTED | TTS started |
| 8 | TTS_DONE | TTS finished |
| 9 | SESSION_DONE | Session ended normally |
| 10 | ERROR | Error |

Voice capture checks that its consumer is still connected before submitting recognized speech to Planner. Greeting requests reuse the original task session and existing voice authorization; a face match alone does not authorize a speaker. This prevents a closed greeting capture from silently submitting a later command.
