#!/usr/bin/env python3
"""Runtime acceptance checks for the browser MuJoCo/ROS bridge."""
from __future__ import annotations

import argparse
import json
import math
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu, JointState, LaserScan, PointCloud2
from std_msgs.msg import Empty, String


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)
LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class AcceptanceNode(Node):
    def __init__(self) -> None:
        super().__init__("mujoco_robonix_acceptance")
        self.messages = {}
        self.create_subscription(LaserScan, "/scan", lambda m: self._set("scan", m), SENSOR_QOS)
        self.create_subscription(PointCloud2, "/mid360/points", lambda m: self._set("cloud", m), SENSOR_QOS)
        self.create_subscription(Imu, "/mid360/imu", lambda m: self._set("imu", m), SENSOR_QOS)
        self.create_subscription(Image, "/front/rgb", lambda m: self._set("front_rgb", m), SENSOR_QOS)
        self.create_subscription(Image, "/front/depth", lambda m: self._set("front_depth", m), SENSOR_QOS)
        self.create_subscription(Image, "/wrist/rgb", lambda m: self._set("wrist_rgb", m), SENSOR_QOS)
        self.create_subscription(Image, "/wrist/depth", lambda m: self._set("wrist_depth", m), SENSOR_QOS)
        self.create_subscription(Odometry, "/odom", lambda m: self._set("odom", m), 20)
        self.create_subscription(JointState, "/arm/joint_states_single", lambda m: self._set("joints", m), 20)
        self.create_subscription(String, "/sim/task_objects", lambda m: self._set("objects", m), 10)
        self.create_subscription(String, "/sim/pick_status", lambda m: self._set("pick", m), LATCHED_QOS)
        self.create_subscription(OccupancyGrid, "/map", lambda m: self._set("map", m), LATCHED_QOS)
        self.create_subscription(
            PointCloud2, "/rtabmap/cloud_map", lambda m: self._set("map_cloud", m), LATCHED_QOS)
        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 20)
        self.arm_command = self.create_publisher(JointState, "/arm/joint_command", 10)
        self.pick_command = self.create_publisher(String, "/sim/pick_command", 10)
        self.reset_command = self.create_publisher(Empty, "/sim/reset", 10)
        self.navigation = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def _set(self, name, message) -> None:
        self.messages[name] = message

    def spin_until(self, predicate, timeout: float, description: str) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return
        raise AssertionError(f"timeout waiting for {description}")

    def reset(self) -> None:
        self.reset_command.publish(Empty())
        start = time.monotonic()
        while time.monotonic() - start < 2.0:
            rclpy.spin_once(self, timeout_sec=0.05)


def image_stats(message: Image) -> dict:
    if message.encoding == "32FC1":
        values = np.frombuffer(message.data, dtype=np.dtype("<f4"))
        valid = values[np.isfinite(values) & (values > 0.0)]
        assert valid.size > 100, f"{message.header.frame_id} depth has no valid returns"
        return {"valid": int(valid.size), "min": float(valid.min()), "max": float(valid.max())}
    values = np.frombuffer(message.data, dtype=np.uint8)
    assert values.size > 100 and float(values.std()) > 2.0, f"{message.header.frame_id} RGB is blank"
    return {"mean": float(values.mean()), "std": float(values.std())}


def object_positions(message: String) -> dict[str, list[float]]:
    return {item["name"]: item["position"] for item in json.loads(message.data)}


def drive_to_position(node: AcceptanceNode, target: tuple[float, float], timeout: float = 25.0) -> float:
    """Use the holonomic base to reach the environment's manipulation pose."""
    deadline = time.monotonic() + timeout
    distance = math.inf
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        odom = node.messages.get("odom")
        if odom is None:
            continue
        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation
        dx, dy = target[0] - position.x, target[1] - position.y
        distance = math.hypot(dx, dy)
        if distance < 0.035:
            break
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        command = Twist()
        command.linear.x = max(-0.22, min(0.22, 0.8 * (math.cos(yaw) * dx + math.sin(yaw) * dy)))
        command.linear.y = max(-0.22, min(0.22, 0.8 * (-math.sin(yaw) * dx + math.cos(yaw) * dy)))
        node.cmd_vel.publish(command)
    node.cmd_vel.publish(Twist())
    assert distance < 0.06, f"failed to reach manipulation pose; remaining distance {distance:.3f}m"
    return distance


def navigate_to_position(
    node: AcceptanceNode, target: tuple[float, float], yaw: float = 0.0, timeout: float = 90.0,
) -> dict:
    """Reach the manipulation pose through the Nav2 action used by Robonix."""
    node.spin_until(lambda: "map" in node.messages, 20.0, "SLAM map before navigation")
    assert node.navigation.wait_for_server(timeout_sec=10.0), "NavigateToPose action is unavailable"
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.pose.position.x, goal.pose.pose.position.y = target
    goal.pose.pose.orientation.z = math.sin(yaw * 0.5)
    goal.pose.pose.orientation.w = math.cos(yaw * 0.5)
    send_future = node.navigation.send_goal_async(goal)
    node.spin_until(send_future.done, 10.0, "Nav2 goal acceptance")
    handle = send_future.result()
    assert handle is not None and handle.accepted, "Nav2 rejected manipulation goal"
    result_future = handle.get_result_async()
    node.spin_until(result_future.done, timeout, "Nav2 manipulation goal")
    result = result_future.result()
    assert result.status == GoalStatus.STATUS_SUCCEEDED, f"Nav2 failed with status {result.status}"
    pose = node.messages["odom"].pose.pose
    actual_yaw = math.atan2(
        2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y),
        1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z),
    )
    position_error = math.hypot(pose.position.x - target[0], pose.position.y - target[1])
    yaw_error = abs(math.atan2(math.sin(actual_yaw - yaw), math.cos(actual_yaw - yaw)))
    assert position_error <= 0.06, f"Nav2 stopped {position_error:.3f}m from manipulation pose"
    assert yaw_error <= 0.07, f"Nav2 stopped {yaw_error:.3f}rad from manipulation yaw"
    return {
        "target": [*target, yaw],
        "actual": [pose.position.x, pose.position.y, actual_yaw],
        "position_error_m": position_error,
        "yaw_error_rad": yaw_error,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--motion", action="store_true", help="drive forward and verify odometry")
    parser.add_argument("--pick", action="store_true", help="pick and release an environment task object")
    parser.add_argument("--pick-object", default="task_water_glass")
    parser.add_argument("--pick-hold-s", type=float, default=5.0)
    parser.add_argument("--pick-via-nav2", action="store_true")
    parser.add_argument("--pick-base-position", nargs=2, type=float, default=(3.92, 3.25), metavar=("X", "Y"))
    parser.add_argument(
        "--pick-waypoint", nargs=2, type=float, action="append", default=[], metavar=("X", "Y"),
        help="collision-free intermediate base point; may be repeated",
    )
    parser.add_argument("--require-map", action="store_true", help="require a non-empty RTAB-Map grid")
    parser.add_argument("--require-task-objects", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = AcceptanceNode()
    report = {}
    try:
        required = {"scan", "cloud", "imu", "front_rgb", "front_depth", "wrist_rgb", "wrist_depth", "odom", "joints"}
        node.spin_until(lambda: required <= node.messages.keys(), 30.0, "sensor suite")

        scan = node.messages["scan"]
        finite_ranges = [value for value in scan.ranges if math.isfinite(value)]
        assert len(scan.ranges) >= 360 and len(finite_ranges) >= 10
        cloud = node.messages["cloud"]
        assert cloud.width >= 100 and cloud.point_step == 12
        joints = node.messages["joints"]
        assert len(joints.position) >= 7
        report["sensors"] = {
            "scan_samples": len(scan.ranges),
            "scan_returns": len(finite_ranges),
            "cloud_points": cloud.width,
            "front_rgb": image_stats(node.messages["front_rgb"]),
            "front_depth": image_stats(node.messages["front_depth"]),
            "wrist_rgb": image_stats(node.messages["wrist_rgb"]),
            "wrist_depth": image_stats(node.messages["wrist_depth"]),
            "arm_joints": len(joints.position),
        }

        if args.require_map:
            node.spin_until(
                lambda: {"map", "map_cloud"} <= node.messages.keys(),
                30.0,
                "occupancy grid and fused map cloud",
            )
            grid = node.messages["map"]
            map_cloud = node.messages["map_cloud"]
            values = np.asarray(grid.data, dtype=np.int16)
            occupied = int(np.count_nonzero(values >= 50))
            free = int(np.count_nonzero(values == 0))
            assert grid.info.width > 0 and occupied > 0 and free > 0
            assert map_cloud.width > 100 and map_cloud.point_step >= 12
            report["map"] = {
                "width": grid.info.width,
                "height": grid.info.height,
                "occupied": occupied,
                "free": free,
                "cloud_points": map_cloud.width,
            }

        if args.require_task_objects or args.pick:
            node.spin_until(lambda: "objects" in node.messages and len(object_positions(node.messages["objects"])) >= 3, 15.0, "task objects")
            report["task_objects"] = sorted(object_positions(node.messages["objects"]))

        if args.motion:
            node.reset()
            start_msg = node.messages["odom"]
            start = np.array([start_msg.pose.pose.position.x, start_msg.pose.pose.position.y])
            command = Twist()
            command.linear.x = 0.18
            deadline = time.monotonic() + 2.8
            while time.monotonic() < deadline:
                node.cmd_vel.publish(command)
                rclpy.spin_once(node, timeout_sec=0.08)
            node.cmd_vel.publish(Twist())
            node.spin_until(lambda: time.monotonic() >= deadline + 0.5, 1.0, "base stop")
            end_msg = node.messages["odom"]
            end = np.array([end_msg.pose.pose.position.x, end_msg.pose.pose.position.y])
            distance = float(np.linalg.norm(end - start))
            assert distance > 0.06, f"base moved only {distance:.3f}m"
            report["motion"] = {"distance_m": distance, "start": start.tolist(), "end": end.tolist()}

        if args.pick:
            node.reset()
            node.spin_until(lambda: "objects" in node.messages, 5.0, "reset object state")
            if args.pick_via_nav2:
                report["manipulation_navigation"] = navigate_to_position(
                    node, tuple(args.pick_base_position))
            else:
                for waypoint in args.pick_waypoint:
                    drive_to_position(node, tuple(waypoint))
                drive_to_position(node, tuple(args.pick_base_position))
            node.spin_until(lambda: args.pick_object in object_positions(node.messages["objects"]), 5.0, "pick object")
            before = object_positions(node.messages["objects"])[args.pick_object][2]
            previous_sequence = json.loads(node.messages.get("pick", String(data='{"sequence":-1}')).data).get("sequence", -1)
            node.pick_command.publish(String(data=args.pick_object))

            def pick_finished() -> bool:
                if "pick" not in node.messages:
                    return False
                status = json.loads(node.messages["pick"].data)
                return status.get("sequence", -1) > previous_sequence and status.get("status") in ("succeeded", "failed")

            node.spin_until(pick_finished, 45.0, "pick result")
            status = json.loads(node.messages["pick"].data)
            assert status["status"] == "succeeded", status
            lifted = object_positions(node.messages["objects"])[args.pick_object][2]
            assert lifted - before > 0.08, f"object lift was only {lifted - before:.3f}m"
            hold_deadline = time.monotonic() + args.pick_hold_s
            held_heights = []
            while time.monotonic() < hold_deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                held_heights.append(object_positions(node.messages["objects"])[args.pick_object][2])
            hold_drift = max(held_heights) - min(held_heights)
            assert min(held_heights) - before > 0.08, "object fell during post-success hold verification"
            assert hold_drift < 0.02, f"object drifted {hold_drift:.3f}m while held"
            previous_sequence = status["sequence"]
            release = JointState()
            release.name = ["gripper"]
            release.position = [0.07]
            node.arm_command.publish(release)

            def placement_finished() -> bool:
                placement = json.loads(node.messages["pick"].data)
                return placement.get("sequence", -1) > previous_sequence and placement.get("status") in ("succeeded", "failed")

            node.spin_until(placement_finished, 45.0, "safe placement result")
            placement = json.loads(node.messages["pick"].data)
            assert placement["status"] == "succeeded", placement
            final_z = object_positions(node.messages["objects"])[args.pick_object][2]
            assert abs(final_z - before) < 0.04, f"placed object height error was {final_z - before:.3f}m"
            joint_positions = dict(zip(node.messages["joints"].name, node.messages["joints"].position))
            stow = [0.0, 0.82, -2.2, 0.0, 1.1, 0.0]
            stow_error = max(abs(joint_positions[f"joint{i + 1}"] - target) for i, target in enumerate(stow))
            assert stow_error < 0.06, f"arm did not return to stow pose; max error {stow_error:.3f}rad"
            report["pick"] = {
                "status": status["status"], "hold_s": args.pick_hold_s,
                "hold_drift_m": hold_drift, "z_before": before,
                "z_lifted": lifted, "z_placed": final_z,
                "placement_status": placement["status"], "stow_error_rad": stow_error,
            }

        print(json.dumps({"ok": True, **report}, ensure_ascii=False, indent=2))
    finally:
        node.cmd_vel.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
