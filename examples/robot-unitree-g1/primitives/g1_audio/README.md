# G1 Audio

This package provides the G1 built-in speaker as a Robonix audio primitive.
The `speech` service continues to synthesize speech; only its speaker provider
changes from the browser relay to `g1_audio`.

## Build and start

Initialize the SDK submodule if it is missing:

```bash
# Run from examples/robot-unitree-g1
git submodule update --init third_party/unitree_sdk2
rbnx build -f robonix_manifest.yaml
rbnx boot -f robonix_manifest.yaml
```

The build needs CMake, a C++17 compiler, and Unitree SDK2 libraries for the
host architecture. `G1_NETWORK_INTERFACE` must name the network interface
connected to the G1 (the deployment default is `enp7s0`). The primitive becomes
ready only after its SDK2 audio client can reach the robot audio service.

Check registration after boot:

```bash
rbnx caps -v | grep -E 'g1_audio|speech/speak'
```

## Speak a test phrase

Call the `robonix/service/speech/speak` MCP capability from a Robonix-connected
MCP client (for example, the project's Pilot/client) with:

```json
{"target":"g1_audio","text":"你好，这是 G1 语音播报测试。"}
```

`target` selects the speaker provider ID; it can be omitted because the
deployment config sets `g1_audio` as the default. A successful response has
`ok: true`. Confirm audibly that the G1's built-in speaker plays the phrase;
the response indicates SDK playback requests succeeded, not that hardware
playback completion was independently acknowledged.

For the quickest smoke test, first make sure the robot is powered on, the G1
network interface is reachable, and the configured speech/TTS backend is
available. Then call the phrase above once. If `g1_audio` is absent in
`rbnx caps -v`, inspect the primitive's build/start logs. If speech reports no
speaker provider or SDK activation fails, check the provider ID, selected
network interface, robot connectivity, and G1 audio service/firmware.

Stop the deployment with `Ctrl-C` in the terminal running `rbnx boot`.

## Speech integration

The deployment manifest selects `g1_audio` as
`speech.default_speaker_provider_id`. Applications should call
`robonix/service/speech/speak` with `text` and optionally `target`; the speech
service performs synthesis and routes the PCM stream to the selected speaker.
They do not need to call the SDK primitive directly.
The existing `audio_client_bridge` remains available for browser/client audio
but is no longer the default TTS output.

The provider accepts 16 kHz mono `pcm_s16le`, matching the current speech
service and the G1 SDK2 streaming example. See `CAPABILITY.md` for constraints,
response semantics, and device acceptance requirements.
