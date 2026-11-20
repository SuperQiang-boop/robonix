# G1 Arm Action Primitive

Guarded Robonix primitive for Unitree G1 preset upper-arm actions backed by
SDK2 `g1::G1ArmActionClient`.

`release_arm` requires SDK acceptance plus new feedback on
`rt/arm/action/state` received after the release request starts: `id=99` and
`holding=false`. The deployed G1 firmware also reports no active preset as
`id=0`, `holding=false`, `name=""` (observed on this robot). This form requires
at least three consecutive samples spanning 0.2 seconds, no inter-sample gap
over 0.3 seconds, and the latest sample less than 0.3 seconds old when confirmed.
Other action IDs, holding states, missing/nonempty idle names and stale samples
do not confirm release. The successful result reports the actual observed ID.
Feedback arriving while the SDK call is still in progress is
retained; samples received before the request are ignored. SDK failure still
returns failure even if a release state was received. A later holding, unrelated
action, or invalid state prevents confirmation. The helper waits up to 2 seconds
after the RPC reply for confirmation and reports the number of received/invalid
samples, latest valid state and parsing error when it times out. This separates
missing feedback from a controller that reports an unexpected state. Greeting cannot
resume navigation using acceptance alone. This confirms the controller has
released its preset; it does not measure the physical joint position. Other
presets retain their acceptance response. Calls are serialized in the primitive
so release cannot overtake an earlier helper that is still running after a
caller times out. Rebuild the helper and reload the primitive after updating.

## Capabilities

- `robonix/primitive/arm/driver`: provider lifecycle.
- `robonix/primitive/arm/action`: run one allowlisted preset action.

### Supported Actions

#### Standard Actions (23)
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

#### Custom Actions (4)
- `Waist_Drum_Dance` - Waist drum dance (9.5s)
- `Spin_discs` - Spin discs (6.9s)
- `Scratch_head` - Scratch head (8.1s)
- `Throw_money` - Throw money (8.1s)

## Build

```bash
export UNITREE_SDK2_DIR=/path/to/unitree_sdk2
bash scripts/build.sh
```

Offline regression (C++ compiler and SDK2 checkout required; no robot connection):

```bash
python3 -m unittest discover -s tests -v
```

## Runtime

Actions are disabled unless both gates are set:

- `allow_actions=true` in the Robonix manifest, or
  `G1_ARM_ACTIONS_ENABLED=true`.
- `action_ack=G1_ARM_ACTIONS_APPROVED` in the manifest, or
  `G1_ARM_ACTION_ACK=G1_ARM_ACTIONS_APPROVED`.

The provider also needs `network_interface`, normally wired from
`G1_NETWORK_INTERFACE`.
