# G1 Audio Primitive

`g1_audio` implements the shared `robonix/primitive/audio/speaker` gRPC
contract. It adapts the stream to the Unitree SDK2 G1 `AudioClient` and plays
it through the robot's built-in speaker. The existing speech service remains
responsible for TTS synthesis and calls the same speaker contract as before.

## Input format

The shared `AudioChunk` contract does not carry an `AudioConfig`, so this
provider accepts the format currently emitted by Robonix speech: raw 16 kHz,
mono, signed 16-bit little-endian PCM (`pcm_s16le`). The Unitree SDK2 example
requires exactly this sample rate and channel count. The primitive rejects an
empty stream, odd byte counts, and streams larger than `max_audio_bytes`.
Playback is serialized per provider. A successful gRPC response means all SDK
play requests returned success; SDK2 does not expose a hardware-confirmed
playback-complete event.

## Configuration

- `network_interface`: interface connected to the G1 network; defaults to
  `enp7s0` and should match the other G1 SDK2 providers.
- `max_audio_bytes`: per-stream memory bound; defaults to 32 MiB.
- `startup_timeout_s`: SDK client readiness timeout; defaults to 15 seconds.

Driver activation initializes SDK2 and probes the audio service via
`GetVolume`. Deactivation and shutdown close the IPC channel and reap the
helper. Initialize `third_party/unitree_sdk2` before building. G1 hardware
connectivity and installed audio firmware are required at activation and for
real playback; host-side builds and mock tests do not replace that device
acceptance test.
