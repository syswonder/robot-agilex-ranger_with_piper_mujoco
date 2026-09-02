config:
  joint_states_topic:
    type: string
    default: /arm/joint_states_single
    description: Absolute feedback topic containing six Piper joints and gripper opening.
    example: /arm/joint_states_single
    failure: CMD_INIT fails when invalid or when no feedback arrives before the timeout.
  joint_command_topic:
    type: string
    default: /arm/joint_command
    description: Absolute named JointState command topic for joints and gripper.
    example: /arm/joint_command
    failure: CMD_INIT fails when the value is not an absolute non-root topic.
  end_pose_topic:
    type: string
    default: /arm/end_pose
    description: Absolute measured Piper TCP pose topic in arm_base_link.
    example: /arm/end_pose
    failure: CMD_INIT fails when the value is not an absolute non-root topic.
  pos_command_topic:
    type: string
    default: /arm/pos_command
    description: Absolute Cartesian Piper TCP command topic in arm_base_link.
    example: /arm/pos_command
    failure: CMD_INIT fails when the value is not an absolute non-root topic.
  sentinel_timeout_s:
    type: float
    unit: seconds
    default: 30.0
    range: greater than 0
    description: Maximum wait for the first joint feedback during CMD_INIT.
    example: 90
    failure: CMD_INIT fails on a non-numeric, non-positive, or expired value.
