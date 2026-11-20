# Runtime config accepted by the G1 face camera RGB bridge.

config:
  # Absolute ROS 2 sensor_msgs/Image topic already published by the G1 stack.
  rgb_topic: /face/camera/color/image_raw

  # Maximum wait for the first image during Driver(CMD_INIT).
  sentinel_timeout_s: 30.0
