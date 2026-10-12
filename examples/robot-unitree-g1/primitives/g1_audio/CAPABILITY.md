# G1 Audio Primitive

`g1_audio` implements the shared `robonix/primitive/audio/mic` and
`robonix/primitive/audio/speaker` gRPC contracts. Speaker playback uses the
Unitree SDK2 G1 `AudioClient`; microphone capture enables the G1 voice service
and receives its PCM multicast stream. The existing speech service remains
responsible for TTS synthesis and calls the same speaker contract as before.

## Microphone

The mic handler is a server stream. It enables voice API 1008 with `mode=1`,
joins `mic_multicast_group:mic_multicast_port`, and yields each received UDP
payload as an `audio/AudioChunk`. A single mic stream is allowed at a time.
Disconnecting the client or shutting down the primitive closes the socket and
requests voice API 1008 `mode=2`.

The multicast transport is the G1 voice-service protocol used by the reference
driver in `tmp/g1-audio-driver`; it is not a PulseAudio or FIFO dependency.

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
- `mic_multicast_group` / `mic_multicast_port`: G1 microphone multicast endpoint;
  default to `239.168.123.161:5555`.
- `mic_bind_address`: optional local IPv4 address used for multicast membership;
  it does not change the UDP bind address. If empty, the IPv4 address of
  `network_interface` is selected explicitly.
- `mic_sample_rate`, `mic_channels`, `mic_bits_per_sample`: microphone PCM
  metadata; defaults to 16000/1/16.
- `mic_max_packet_bytes`: upper bound for one UDP packet; defaults to 65535.
- `mic_socket_timeout_s`: receive timeout used to observe client cancellation;
  defaults to 0.5 seconds.

Driver activation initializes SDK2 and probes the audio service via
`GetVolume`. Deactivation and shutdown close the IPC channel, stop any active
microphone stream, and reap the helpers. Microphone voice control uses the same
C++ `third_party/unitree_sdk2` tree as speaker playback; no Unitree SDK2 Python
package is required. G1 hardware connectivity, voice service availability,
multicast delivery, and installed audio firmware are required for device
acceptance; host-side builds and mock tests do not replace that test.
