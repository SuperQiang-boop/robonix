---
description: Guarded Unitree G1 preset upper-arm actions backed by SDK2 LocoClient.
---

# Unitree G1 arm action primitive

This package exposes one guarded arm action RPC:

- `robonix/primitive/arm/action`

## Supported Actions

### Standard Actions (23)
- `release_arm` - Release arm
- `turn_back_wave` - Turn back and wave
- `blow_kiss_with_both_hands` - Blow kiss with both hands
- `blow_kiss_with_left_hand` - Blow kiss with left hand
- `blow_kiss_with_right_hand` - Blow kiss with right hand
- `both_hands_up` - Both hands up
- `clamp` - Clamp action
- `high_five` - High five
- `hug` - Hug
- `make_heart_with_both_hands` - Make heart with both hands
- `make_heart_with_right_hand` - Make heart with right hand
- `refuse` - Refuse gesture
- `right_hand_up` - Right hand up
- `ultraman_ray` - Ultraman ray
- `wave_under_head` - Wave under head
- `wave_above_head` - Wave above head
- `shake_hand` - Shake hand
- `box_left_hand_win` - Box left hand win
- `box_right_hand_win` - Box right hand win
- `box_both_hand_win` - Box both hand win
- `right_hand_on_heart` - Right hand on heart
- `both_hands_up_deviate_right` - Both hands up deviate right
- `forward_push` - Forward push

### Custom Actions (4)
- `Waist_Drum_Dance` - Waist drum dance (9.5s)
- `Spin_discs` - Spin discs (6.9s)
- `Scratch_head` - Scratch head (8.1s)
- `Throw_money` - Throw money (8.1s)

The implementation calls the official Unitree SDK2 `g1::LocoClient` preset
actions. It does not expose arbitrary `SetTaskId`, low-level joint commands,
Cartesian control, hand or finger control, or a continuous arm controller.

## Safety boundary

Actions are disabled by default. A deployment must set `allow_actions=true`
and provide the exact acknowledgement string
`G1_ARM_ACTIONS_APPROVED`. The Python provider checks the gate before spawning
the SDK2 helper, and the SDK2 helper independently checks the acknowledgement
before initializing Unitree SDK2.

The helper initializes SDK2 for one requested action, calls the matching
LocoClient method, and exits. A non-zero SDK2 result or process failure is
reported as `accepted=false`.

These actions move the robot upper body and are not validated merely because
the package builds.
