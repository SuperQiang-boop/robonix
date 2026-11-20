---
description: User-invocable G1 upper-arm preset action skill.
---

# G1 arm actions skill

This skill exposes one MCP tool:

- `robonix/skill/g1_arm_actions/run`

It accepts a single allowlisted action name and calls the G1 arm primitive
capability `robonix/primitive/arm/action` over gRPC. It does not talk to SDK2
directly and does not expose arbitrary task IDs or low-level joint commands.

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
