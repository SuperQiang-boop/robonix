# g1_face_greeting skill instance configuration.
#
# navigation_enabled and turn_to_face_enabled are reserved integration switches.
# They must remain false until navigation pause/resume and camera geometry are
# verified on the target robot. With both false, the skill only speaks and runs
# the intimacy-selected arm action; it never calls navigation or rotates.
face_provider_id:
  type: string
  default: face_recognition
scene_provider_id:
  type: string
  default: scene
speech_provider_id:
  type: string
  default: speech
arm_provider_id:
  type: string
  default: g1_arm_actions
navigation_enabled:
  type: boolean
  default: false
turn_to_face_enabled:
  type: boolean
  default: false
poll_wait_ms:
  type: integer
  default: 1000
  range: [100, 5000]
cooldown_sec:
  type: number
  default: 30
  range: [0, 3600]
message_store_path:
  type: string
  description: Absolute SQLite file path in persistent deployment storage; do not use rbnx-build.
action_level_1:
  type: string
  default: wave_under_head
action_level_2:
  type: string
  default: wave_under_head
action_level_3:
  type: string
  default: wave_above_head
action_level_4:
  type: string
  default: make_heart_with_right_hand
action_level_5:
  type: string
  default: make_heart_with_both_hands
# Actions are intentionally restricted to the stationary greeting allowlist:
# wave_under_head, wave_above_head, make_heart_with_right_hand,
# make_heart_with_both_hands.
