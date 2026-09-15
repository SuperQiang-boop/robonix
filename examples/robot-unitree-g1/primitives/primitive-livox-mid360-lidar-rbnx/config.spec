# G1 data correction: edit src/livox_ros_driver2/config/MID360_config.json
# lidar_configs[0].extrinsic_parameter uses DEGREES; roll=180 rotates both
# points and IMU. This is distinct from the optional TF-only extrinsics below.
# G1 deployment overrides frame_id=mid360_upright and uses URDF output frames.
# Runtime config accepted by the MID-360 lidar primitive.
#
# This file documents the mapping passed as the package's `config:` value in a
# robot deployment manifest. It is not loaded by the provider. Values below are
# runtime defaults unless a field is marked as required or optional.

config:
  # string, required unless LIVOX_LIDAR_IP is set.
  # IPv4 address configured on the physical MID-360. The host must have a
  # reachable interface on the same network.
  lidar_ip: 192.168.1.161

  # string, default: auto.
  # Host IPv4 address given to the Livox SDK. `auto` selects the source address
  # of the route to lidar_ip. Set an explicit address only when route-based
  # detection chooses the wrong NIC.
  host_ip: auto

  # integer, default: 0.
  # Livox SDK transfer format. 0 publishes sensor_msgs/PointCloud2 with the
  # Livox PointXYZRTLT fields. Format 2 uses pcl::PointCloud and is rejected by
  # livox_ros_driver2 on ROS2.
  xfer_format: 0

  # float (Hz), default: 10.0.
  # Point-cloud publication frequency requested from the Livox driver.
  publish_freq: 10.0

  # string, default: livox_frame.
  # ROS frame_id written into PointCloud2 messages. It must match the lidar
  # link used by the robot URDF when the URDF publishes the mount transform.
  frame_id: livox_frame

  # string, default: base_link.
  # Parent frame used only when the optional extrinsics block is present.
  parent_frame: base_link

  # mapping, optional; no runtime default.
  # Static pose of frame_id in parent_frame. Translation is in metres and
  # rotation is roll/pitch/yaw in radians. Omit the whole block when the robot
  # description provider already publishes parent_frame -> frame_id. The same
  # TF edge must never have two publishers.
  # extrinsics:
  #   x: 0.0
  #   y: 0.0
  #   z: 0.0
  #   roll: 0.0
  #   pitch: 0.0
  #   yaw: 0.0

  # string, default: /scanner/cloud.
  # Absolute ROS topic monitored for readiness and exposed by the lidar
  # capability. Change it only together with the bundled driver output.
  lidar_topic: /scanner/cloud

  # float (seconds), default: 30.0.
  # Maximum wait for a PointCloud2 sample during each startup attempt.
  sentinel_timeout_s: 30.0

  # integer, default: 3.
  # Number of Livox driver startup attempts before the provider reports ERROR.
  livox_retries: 3
