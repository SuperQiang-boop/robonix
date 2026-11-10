# Robot Description primitive

This primitive publishes the robot body model managed by Robotdesc through ROS 2
`robot_state_publisher`. Robotdesc's `robonix/system/robotdesc/get_urdf` gRPC API is the
only URDF input. The package has no URDF path, model file, or per-deployment
body configuration of its own.

It publishes fixed URDF joints on `/tf_static` and uses standard
`/joint_states` input for movable joints. Mapping, navigation, scene, and other
ROS 2 consumers use the resulting TF tree directly.

## Runtime modes

- `package_manifest.yaml`: Docker ROS 2 runtime.
- `package_manifest.native.yaml`: host ROS 2 runtime.

Both modes default to `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and use the deployment's
existing `ROBONIX_ZENOH_ROUTER`, `ROBONIX_ZENOH_MODE`, and
`ROBONIX_ZENOH_LISTEN` settings. Robotdesc supplies `ROBONIX_ROBOTDESC_ENDPOINT` to every
primitive it launches; operators do not configure this package separately.
