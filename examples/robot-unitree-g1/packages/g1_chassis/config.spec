# Human-readable deployment config reference, not a separately parsed schema.
required:
  network_interface:
    type: string
    fallback_env: G1_NETWORK_INTERFACE
    description: Explicit Ethernet interface connected to G1, required even without motion.

optional:
  odom_source:
    type: string
    default: sdk_pose
    allowed: [sdk_pose]
  state_topic:
    type: string
    default: rt/odommodestate
    description: Nonempty rt/ DDS topic using the SDK SportModeState layout, not a ROS topic name.
  odom_publish_rate_hz:
    type: float
    default: 50.0
    range: [10.0, 100.0]
  state_timeout_s:
    type: float
    default: 0.15
    range: [0.05, 0.30]
    description: Maximum local reception age. Expired state stops TF and command refresh.
  allow_motion:
    type: boolean
    default: false
    fallback_env: G1_ALLOW_MOTION
    description: Explicit motion gate; true supplies the daemon physical-motion acknowledgement.
  publish_odom_tf:
    type: boolean
    default: true
  odom_topic:
    type: string
    default: /odom
  twist_in_topic:
    type: string
    default: /cmd_vel
  joint_state_topic:
    type: string
    default: /joint_states
    description: Neutral visualization placeholders, not measured joint feedback.
  odom_frame:
    type: string
    default: odom
  base_frame:
    type: string
    default: base_footprint
  max_linear_x_mps:
    type: float
    default: 0.35
    range: [0.0, 1.0]
  max_linear_y_mps:
    type: float
    default: 0.20
    range: [0.0, 0.5]
  max_angular_z_rps:
    type: float
    default: 0.60
    range: [0.0, 2.0]

constraints:
  - Robot source stamps are zero on the tested firmware; timestamps represent local DDS reception.
  - Pose differences determine child-frame planar twist; raw SDK velocity frame is not assumed.
  - Pose jumps greater than 0.30 m or 0.50 rad latch a fault until adapter restart.
  - SDK daemon watchdog is fixed at 300 ms; adapter command refresh expires after 250 ms.
  - Stand/swing height, IMU/velocity frame, acceleration and command_timeout_s fields are not implemented by this provider.
