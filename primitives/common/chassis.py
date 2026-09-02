"""Ranger chassis capability provider backed by simulated ROS topics."""
from __future__ import annotations

import json
import math
import os
import time

from robonix_api import Primitive, Ok, Err

from .runtime import package_root, provider_id, topic_value

provider = Primitive(
    id=provider_id("ranger_chassis"), namespace="robonix/primitive/chassis",
    pkg_root=package_root())
cmd_vel_pub = None

import chassis_pb2  # noqa: E402
import std_msgs_pb2  # noqa: E402


@provider.grpc("robonix/primitive/chassis/move")
def move(request):
    """Execute a bounded velocity or distance burst and always publish a stop."""
    if cmd_vel_pub is None:
        return chassis_pb2.ExecuteMoveCommand_Response(
            status=std_msgs_pb2.String(data=json.dumps({"error": "driver not initialized"})))
    from geometry_msgs.msg import Twist
    command = request.command
    linear_speed = float(os.environ.get("SIM_LINEAR_SPEED", "0.25"))
    angular_speed = float(os.environ.get("SIM_ANGULAR_SPEED", "0.55"))
    forward_m = float(getattr(command, "forward_m", 0.0))
    rotate_deg = float(getattr(command, "rotate_deg", 0.0))
    velocity = Twist()
    if forward_m:
        velocity.linear.x = math.copysign(linear_speed, forward_m)
        duration = abs(forward_m) / linear_speed
    elif rotate_deg:
        velocity.angular.z = math.copysign(angular_speed, rotate_deg)
        duration = abs(math.radians(rotate_deg)) / angular_speed
    else:
        velocity.linear.x = float(command.linear_x)
        velocity.linear.y = float(command.linear_y)
        velocity.angular.z = float(command.angular_z)
        duration = max(0.05, float(command.duration_sec or 1.0))
    for _ in range(max(1, int(duration * 10))):
        cmd_vel_pub.publish(velocity)
        time.sleep(0.1)
    cmd_vel_pub.publish(Twist())
    return chassis_pb2.ExecuteMoveCommand_Response(
        status=std_msgs_pb2.String(data=json.dumps({"status": "done", "duration_sec": duration})))


@provider.on_init
def initialize(config):
    """Bind capability metadata to the bridge-owned chassis topics."""
    global cmd_vel_pub
    from geometry_msgs.msg import Twist
    try:
        odom_topic = topic_value(config, "odom_topic", "/odom")
        command_topic = topic_value(config, "command_topic", "/cmd_vel")
    except ValueError as error:
        return Err(str(error))
    cmd_vel_pub = provider.create_publisher(
        "robonix/primitive/chassis/twist_in", topic=command_topic,
        msg_type=Twist, qos="reliable", declare=False)
    provider.declare_ros2_topic("robonix/primitive/chassis/twist_in", command_topic, qos="reliable")
    provider.declare_ros2_topic("robonix/primitive/chassis/odom", odom_topic, qos="reliable")
    return Ok()


@provider.on_shutdown
def shutdown():
    """Stop the mobile base before unregistering its provider."""
    if cmd_vel_pub is not None:
        from geometry_msgs.msg import Twist
        cmd_vel_pub.publish(Twist())
    return Ok()


if __name__ == "__main__":
    provider.run()
