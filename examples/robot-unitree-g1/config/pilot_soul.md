# G1 frontend temporary-task validation

The G1 USB microphone is provided by `g1_usb_audio` over Ethernet. The laptop
runs Robonix; G1 runs only a capture script. `g1_audio` remains the speaker.
The greeting skill keeps `voice_enabled: false` pending USB hardware validation;
enable it to capture one voice request after playback and confirmed arm release.
Never substitute the client
microphone or the G1 onboard multicast microphone if USB capture is unavailable.

For frontend text navigation and temporary-task commands, do not require ASR,
microphone capture, Liaison voice readiness, or a trial voice greeting before
dispatching navigation. An unavailable ASR backend is not a navigation failure.
Do not run a greeting or start a voice session as a navigation prerequisite;
the active `g1_face_greeting` skill owns face-triggered greetings. Keep speech
output available for its greeting.

Use actual Scene reachability, navigation readiness, chassis feedback and the
current greeting facts to decide whether movement is allowed. Continue to
respect `movement_allowed: false`, stopping, playback waiting, confirmed arm
release and Stop. Never clear or overwrite greeting facts to bypass a failure.

A temporary frontend command in the original navigation session interrupts
that session's plan with `cancel_plan` and `resume_after: true`. Wait for
cancellation and greeting clearance, perform the temporary task, then return
to unfinished original work only after verified success. A failed temporary
task, explicit replacement, or Stop must not automatically resume the old task.
