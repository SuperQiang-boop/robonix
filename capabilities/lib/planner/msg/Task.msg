# TaskSource: TEXT=0 AUDIO=1 API=2
#
# TODO(post-gateway-port): drop `source` + `audio_data` — modality
# normalisation belongs in gateway (mic → ASR → text → planner). planner
# should never see PCM bytes. Keeping them here only because gateway's
# `dev` branch still writes raw audio into Task; remove once gateway
# moves off this path.
#
# TODO(post-gateway-port): add `string user_id` — who issued the task
# (free-form: user@host, uuid, …). `session_id` only identifies the
# conversation thread, not the user. Multi-user-on-one-robot has no
# discriminator today.
#
# TODO(post-gateway-port): replace `context_json` with typed control
# bools. Current consumers parse JSON for two flags only:
#   {"session_end": true}  → trigger compact_memory, no VLM turn
#   {"abort_turn":  true}  → cancel any in-flight turn for session_id
# Lift those into `bool session_end` + `bool abort_turn` fields; keep
# context_json as the genuine-extras escape hatch (image refs, location,
# …). Gateway currently writes to context_json directly, so deferred
# until gateway ports.
string task_id
string session_id
uint32 source
string text
uint8[] audio_data
string context_json
uint64 timestamp_ms
