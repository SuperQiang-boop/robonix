# Runtime config accepted by the RealSense D435i camera primitive.
#
# This file documents the mapping passed as the package's `config:` value in a
# robot deployment manifest. It is not loaded by the provider. Values below are
# runtime defaults.

config:
  # string, default: camera_435i.
  # ROS node/camera namespace. Default topic names are derived from this value
  # unless they are overridden explicitly below.
  camera_name: camera_435i

  # string WIDTHxHEIGHTxFPS, default: 640x480x30.
  # Color stream profile. It must be supported by the attached camera and the
  # available USB link bandwidth.
  rgb_profile: 640x480x30

  # string WIDTHxHEIGHTxFPS, default: 848x480x30.
  # Depth stream profile. It must be supported by the attached camera. When
  # align_depth is true, depth is geometrically aligned to the color stream.
  depth_profile: 848x480x30

  # boolean, default: true.
  # Enable the camera gyro and accelerometer streams in realsense2_camera.
  enable_imu: true

  # boolean, default: true.
  # Publish depth registered into the color optical frame.
  align_depth: true

  # boolean, default: true.
  # Ask the RealSense ROS driver to synchronize enabled image streams.
  enable_sync: true

  # boolean, default: true.
  # Enable the librealsense spatial depth post-processing filter.
  spatial_filter: true

  # boolean, default: true.
  # Enable the librealsense temporal depth post-processing filter.
  temporal_filter: true

  # boolean, default: false.
  # Enable hole filling. This can invent depth at missing pixels, so leave it
  # disabled unless the robot-specific mapping configuration validates it.
  hole_filling_filter: false

  # string, default: /<camera_name>/color/image_raw.
  # Absolute color Image topic monitored for readiness and exposed through the
  # RGB capability.
  rgb_topic: /<camera_name>/color/image_raw

  # string, default: /<camera_name>/aligned_depth_to_color/image_raw.
  # Absolute aligned-depth Image topic exposed by the depth capability. When
  # align_depth is false, set this to the actual unaligned depth topic.
  depth_topic: /<camera_name>/aligned_depth_to_color/image_raw

  # string, default: /<camera_name>/color/camera_info.
  # Absolute CameraInfo topic paired with rgb_topic and aligned depth.
  intrinsics_topic: /<camera_name>/color/camera_info

  # string, default: /camera/extrinsics.
  # Latched Atlas extrinsics topic. The provider fills it from the TF transform
  # below after the camera and robot TF publishers are available.
  extrinsics_topic: /camera/extrinsics

  # string, default: base_link.
  # TF parent frame for the robot-to-camera installation transform.
  extrinsics_base_frame: base_link

  # string, default: camera_color_optical_frame.
  # TF child frame for the RGB optical frame. Set this to the actual frame
  # published by the robot's camera driver when it uses another frame name.
  extrinsics_camera_frame: camera_color_optical_frame

  # float (seconds), default: 30.0.
  # Maximum startup wait for the first RGB Image message.
  sentinel_timeout_s: 30.0
