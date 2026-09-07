"""Piper arm capability provider backed by the active simulator runtime."""
from robonix_api import Primitive, Ok, Err

from .runtime import package_root, provider_id, timeout_value, topic_value

provider = Primitive(
    id=provider_id("piper_ctl"), namespace="robonix/primitive/arm",
    pkg_root=package_root())


@provider.on_init
def initialize(config):
    """Declare measured and command topics after confirming joint feedback."""
    config = config or {}
    try:
        joints = topic_value(config, "joint_states_topic", "/arm/joint_states_single")
        joint_command = topic_value(config, "joint_command_topic", "/arm/joint_command")
        end_pose = topic_value(config, "end_pose_topic", "/arm/end_pose")
        pos_command = topic_value(config, "pos_command_topic", "/arm/pos_command")
        sentinel_timeout_s = timeout_value(config)
    except ValueError as error:
        return Err(str(error))
    if not provider.wait_for_topic(joints, "JointState", sentinel_timeout_s):
        return Err(f"no Piper JointState received on {joints}; start the simulator runtime first")
    provider.declare_ros2_topic("robonix/primitive/arm/joint_states", joints, qos="reliable")
    provider.declare_ros2_topic("robonix/primitive/arm/joint_command", joint_command, qos="reliable")
    provider.declare_ros2_topic("robonix/primitive/arm/end_pose", end_pose, qos="reliable")
    provider.declare_ros2_topic("robonix/primitive/arm/pos_command", pos_command, qos="reliable")
    return Ok()


@provider.on_shutdown
def shutdown():
    """Complete lifecycle shutdown; the simulator watchdog holds the last pose."""
    return Ok()


if __name__ == "__main__":
    provider.run()
