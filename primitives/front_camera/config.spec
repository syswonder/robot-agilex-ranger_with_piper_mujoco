config:
  rgb_topic:
    type: string
    default: /front/rgb
    description: Absolute ROS 2 RGB Image topic.
    example: /front/rgb
    failure: CMD_INIT fails when invalid or when no RGB frame arrives before the timeout.
  depth_topic:
    type: string
    default: /front/depth
    description: Absolute aligned float-depth Image topic; an empty value disables depth.
    example: /front/depth
    failure: CMD_INIT fails when a non-empty value is not an absolute topic.
  info_topic:
    type: string
    default: /front/camera_info
    description: Absolute CameraInfo topic matching the RGB and depth geometry.
    example: /front/camera_info
    failure: CMD_INIT fails when the value is not an absolute non-root topic.
  extrinsics_topic:
    type: string
    default: ""
    description: Absolute latched camera mount transform topic; empty disables this capability.
    example: /front/extrinsics
    failure: CMD_INIT fails when a non-empty value is not an absolute topic.
  sentinel_timeout_s:
    type: float
    unit: seconds
    default: 30.0
    range: greater than 0
    description: Maximum wait for the first RGB frame during CMD_INIT.
    example: 90
    failure: CMD_INIT fails on a non-numeric, non-positive, or expired value.
