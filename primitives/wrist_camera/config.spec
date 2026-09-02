config:
  rgb_topic:
    type: string
    default: /wrist/rgb
    description: Absolute ROS 2 RGB Image topic mounted on the Piper wrist.
    example: /wrist/rgb
    failure: CMD_INIT fails when invalid or when no RGB frame arrives before the timeout.
  depth_topic:
    type: string
    default: /wrist/depth
    description: Absolute wrist float-depth Image topic; an empty value disables depth.
    example: /wrist/depth
    failure: CMD_INIT fails when a non-empty value is not an absolute topic.
  info_topic:
    type: string
    default: /wrist/camera_info
    description: Absolute CameraInfo topic matching the wrist streams.
    example: /wrist/camera_info
    failure: CMD_INIT fails when the value is not an absolute non-root topic.
  extrinsics_topic:
    type: string
    default: ""
    description: Optional absolute latched extrinsics topic.
    example: /wrist/extrinsics
    failure: CMD_INIT fails when a non-empty value is not an absolute topic.
  advertise_streams:
    type: bool
    default: false
    description: Advertise continuous RGB-D contracts to global consumers such as Scene.
    example: false
    failure: CMD_INIT fails when the value is not a boolean.
  sentinel_timeout_s:
    type: float
    unit: seconds
    default: 30.0
    range: greater than 0
    description: Maximum wait for the first RGB frame during CMD_INIT.
    example: 90
    failure: CMD_INIT fails on a non-numeric, non-positive, or expired value.
