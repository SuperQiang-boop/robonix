# G1 Arm Action Primitive

Guarded Robonix primitive for Unitree G1 preset upper-arm actions backed by
SDK2 `g1::LocoClient`.

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

## Runtime

Actions are disabled unless both gates are set:

- `allow_actions=true` in the Robonix manifest, or
  `G1_ARM_ACTIONS_ENABLED=true`.
- `action_ack=G1_ARM_ACTIONS_APPROVED` in the manifest, or
  `G1_ARM_ACTION_ACK=G1_ARM_ACTIONS_APPROVED`.

The provider also needs `network_interface`, normally wired from
`G1_NETWORK_INTERFACE`.
