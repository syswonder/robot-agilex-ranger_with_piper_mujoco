#!/usr/bin/env python3
"""Bridge a selectable MuJoCo runtime to the ROS 2 graph used by Robonix."""
from __future__ import annotations

import asyncio
import base64
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import Pose, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan, PointCloud2, PointField
from std_msgs.msg import Empty, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import websockets


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


def decode_array(encoded: str | None, dtype: Any) -> np.ndarray:
    """Decode a base64 payload without making an unnecessary second copy."""
    if not encoded:
        return np.empty(0, dtype=dtype)
    return np.frombuffer(base64.b64decode(encoded), dtype=dtype)


def stamp(seconds: float) -> RosTime:
    """Convert a floating-point simulation timestamp to a ROS timestamp."""
    value = max(0.0, float(seconds))
    sec = int(value)
    return RosTime(sec=sec, nanosec=int((value - sec) * 1_000_000_000))


def set_quaternion(target: Any, quaternion_wxyz: list[float]) -> None:
    """Copy a MuJoCo wxyz quaternion into a ROS xyzw message field."""
    if len(quaternion_wxyz) != 4:
        quaternion_wxyz = [1.0, 0.0, 0.0, 0.0]
    target.w, target.x, target.y, target.z = map(float, quaternion_wxyz)


def camera_info(width: int, height: int, fovy_deg: float, frame_id: str, at: RosTime) -> CameraInfo:
    """Build a pinhole CameraInfo matching the browser projection matrix."""
    fy = height / (2.0 * math.tan(math.radians(fovy_deg) * 0.5))
    fx = fy
    cx = (width - 1) * 0.5
    cy = (height - 1) * 0.5
    msg = CameraInfo()
    msg.header.stamp = at
    msg.header.frame_id = frame_id
    msg.width, msg.height = width, height
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0] * 5
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg


class BridgeState:
    """Thread-safe connection diagnostics shared with the health server."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.connected = False
        self.backend = ""
        self.environment = ""
        self.last_frame_wall_time = 0.0
        self.frames: dict[str, int] = {}

    def record(self, kind: str) -> None:
        """Record one received simulator frame."""
        with self.lock:
            self.last_frame_wall_time = time.time()
            self.frames[kind] = self.frames.get(kind, 0) + 1

    def snapshot(self) -> dict[str, Any]:
        """Return JSON-compatible health information."""
        with self.lock:
            age = time.time() - self.last_frame_wall_time if self.last_frame_wall_time else None
            return {
                "ok": self.connected and age is not None and age < 3.0,
                "runtimeConnected": self.connected,
                "browserConnected": self.connected,
                "backend": self.backend,
                "environment": self.environment,
                "lastFrameAgeSec": age,
                "frames": dict(self.frames),
            }


class HealthHandler(BaseHTTPRequestHandler):
    """Serve bridge status without adding another Python web dependency."""

    state: BridgeState

    def do_GET(self) -> None:  # noqa: N802
        payload = json.dumps(self.state.snapshot()).encode()
        self.send_response(200 if self.path in ("/", "/health") else 404)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, _format: str, *_args: Any) -> None:
        return


class MujocoRosBridge(Node):
    """Publish simulator state as standard ROS messages and forward commands."""

    def __init__(self, shared_state: BridgeState) -> None:
        super().__init__("mujoco_robonix_bridge")
        self.shared_state = shared_state
        self.loop: asyncio.AbstractEventLoop | None = None
        self.websocket: Any = None
        self.sequence = 0
        self.last_sim_time = 0.0

        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 20)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", SENSOR_QOS)
        self.cloud_pub = self.create_publisher(PointCloud2, "/mid360/points", SENSOR_QOS)
        self.map_cloud_pub = self.create_publisher(
            PointCloud2, "/rtabmap/cloud_map", LATCHED_QOS)
        self.imu_pub = self.create_publisher(Imu, "/mid360/imu", SENSOR_QOS)
        self.joints_pub = self.create_publisher(JointState, "/arm/joint_states_single", 20)
        self.standard_joints_pub = self.create_publisher(JointState, "/joint_states", 20)
        self.end_pose_pub = self.create_publisher(Pose, "/arm/end_pose", 20)
        self.object_state_pub = self.create_publisher(String, "/sim/task_objects", 10)
        self.pick_status_pub = self.create_publisher(String, "/sim/pick_status", LATCHED_QOS)
        self.camera_publishers: dict[str, dict[str, Any]] = {
            "front_rgbd": {
                "rgb": self.create_publisher(Image, "/front/rgb", SENSOR_QOS),
                "depth": self.create_publisher(Image, "/front/depth", SENSOR_QOS),
                "info": self.create_publisher(CameraInfo, "/front/camera_info", LATCHED_QOS),
            },
            "wrist_rgb": {
                "rgb": self.create_publisher(Image, "/wrist/rgb", SENSOR_QOS),
                "depth": self.create_publisher(Image, "/wrist/depth", SENSOR_QOS),
                "info": self.create_publisher(CameraInfo, "/wrist/camera_info", LATCHED_QOS),
            },
        }
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.front_extrinsics_pub = self.create_publisher(
            TransformStamped, "/front/extrinsics", LATCHED_QOS)
        self.create_subscription(Twist, "/cmd_vel", self._on_twist, 20)
        self.create_subscription(JointState, "/arm/joint_command", self._on_arm_command, 20)
        self.create_subscription(Pose, "/arm/pos_command", self._on_pose_command, 10)
        self.create_subscription(String, "/sim/pick_command", self._on_pick_command, 10)
        self.create_subscription(Empty, "/sim/reset", self._on_reset, 10)
        self.create_subscription(
            PointCloud2, "/cloud_map", self._on_map_cloud, LATCHED_QOS)
        self._publish_static_transforms()

    def _publish_static_transforms(self) -> None:
        """Publish measured simulator mounts in the base_link frame."""
        transforms = [
            ("base_link", "mid360_link", (0.18, 0.0, 0.425), (0.0, 0.0, 0.0, 1.0)),
            ("base_link", "front_camera_optical_frame", (0.302, 0.0, 0.300), (0.5, -0.5, 0.5, -0.5)),
            ("base_link", "arm_base_link", (-0.20, 0.0, 0.025), (0.0, 0.0, 1.0, 0.0)),
        ]
        messages = []
        for parent, child, xyz, xyzw in transforms:
            message = TransformStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id, message.child_frame_id = parent, child
            message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = xyz
            message.transform.rotation.x, message.transform.rotation.y = xyzw[0], xyzw[1]
            message.transform.rotation.z, message.transform.rotation.w = xyzw[2], xyzw[3]
            messages.append(message)
        self.static_tf.sendTransform(messages)
        self.front_extrinsics_pub.publish(messages[1])

    def send(self, payload: dict[str, Any]) -> None:
        """Schedule a command on the asyncio thread from an rclpy callback."""
        if self.loop is None or self.websocket is None:
            return
        self.sequence += 1
        payload["sequence"] = self.sequence
        asyncio.run_coroutine_threadsafe(self.websocket.send(json.dumps(payload)), self.loop)

    def _on_twist(self, msg: Twist) -> None:
        self.send({
            "type": "cmd_vel", "linearX": msg.linear.x, "linearY": msg.linear.y,
            "angularZ": msg.angular.z,
        })

    def _on_arm_command(self, msg: JointState) -> None:
        self.send({"type": "arm_joint_command", "names": list(msg.name), "positions": list(msg.position)})

    def _on_pose_command(self, msg: Pose) -> None:
        self.send({
            "type": "arm_pose_command",
            "position": [msg.position.x, msg.position.y, msg.position.z],
            "quaternion": [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z],
        })

    def _on_pick_command(self, msg: String) -> None:
        name = msg.data.strip()
        if name:
            self.pick_status_pub.publish(String(data=json.dumps({"status": "accepted", "object": name})))
            self.send({"type": "pick_object", "name": name})

    def _on_reset(self, _msg: Empty) -> None:
        self.send({"type": "reset"})

    def _on_map_cloud(self, msg: PointCloud2) -> None:
        """Bridge RTAB-Map's actual output to its advertised Robonix topic."""
        self.map_cloud_pub.publish(msg)

    def receive(self, payload: dict[str, Any]) -> None:
        """Route one simulator protocol frame to its ROS publisher."""
        kind = str(payload.get("type", ""))
        self.shared_state.record(kind)
        if kind == "state":
            self._publish_state(payload)
        elif kind == "scan":
            self._publish_scan(payload)
        elif kind == "pointcloud":
            self._publish_cloud(payload)
        elif kind == "camera":
            self._publish_camera(payload)
        elif kind == "pick_status":
            self.pick_status_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))

    def _publish_state(self, payload: dict[str, Any]) -> None:
        sim_time = float(payload.get("simulationTime", 0.0))
        self.last_sim_time = sim_time
        at = stamp(sim_time)
        self.clock_pub.publish(Clock(clock=at))
        robot = payload.get("robot") or {}
        base = robot.get("base") or {}
        position = base.get("position") or [0.0, 0.0, 0.0]
        quaternion = base.get("quaternion") or [1.0, 0.0, 0.0, 0.0]
        linear = base.get("linearVelocity") or [0.0] * 3
        angular = base.get("angularVelocity") or [0.0] * 3

        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = at, "odom", "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = map(float, position)
        set_quaternion(odom.pose.pose.orientation, quaternion)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = map(float, linear)
        odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = map(float, angular)
        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp, transform.header.frame_id, transform.child_frame_id = at, "odom", "base_link"
        transform.transform.translation.x, transform.transform.translation.y = float(position[0]), float(position[1])
        transform.transform.translation.z = float(position[2])
        set_quaternion(transform.transform.rotation, quaternion)
        self.tf.sendTransform(transform)

        arm = robot.get("arm") or {}
        joints = JointState()
        joints.header.stamp, joints.header.frame_id = at, "arm_base_link"
        joints.name = list(arm.get("names") or [])
        joints.position = [float(value) for value in arm.get("positions") or []]
        joints.velocity = [float(value) for value in arm.get("velocities") or []]
        self.joints_pub.publish(joints)
        self.standard_joints_pub.publish(joints)
        end = arm.get("endPose")
        if end:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, end["position"])
            set_quaternion(pose.orientation, end["quaternion"])
            self.end_pose_pub.publish(pose)
        self.object_state_pub.publish(String(data=json.dumps(robot.get("objects") or [], separators=(",", ":"))))

        imu_data = payload.get("imu")
        if imu_data:
            imu = Imu()
            imu.header.stamp, imu.header.frame_id = at, "mid360_link"
            gyro = imu_data.get("gyroscope") or [0.0] * 3
            accel = imu_data.get("accelerometer") or [0.0] * 3
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = map(float, gyro)
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = map(float, accel)
            imu.orientation_covariance[0] = -1.0
            self.imu_pub.publish(imu)

    def _publish_scan(self, payload: dict[str, Any]) -> None:
        ranges = decode_array(payload.get("rangesF32"), np.dtype("<f4"))
        msg = LaserScan()
        msg.header.stamp, msg.header.frame_id = stamp(payload.get("timestamp", self.last_sim_time)), "mid360_link"
        msg.angle_min = float(payload.get("angleMin", -math.pi))
        msg.angle_max = float(payload.get("angleMax", math.pi))
        msg.angle_increment = float(payload.get("angleIncrement", 2 * math.pi / max(1, ranges.size)))
        msg.scan_time = 0.1
        msg.time_increment = msg.scan_time / max(1, ranges.size)
        msg.range_min, msg.range_max = float(payload.get("rangeMin", 0.1)), float(payload.get("rangeMax", 30.0))
        msg.ranges = ranges.astype(np.float32, copy=False).tolist()
        self.scan_pub.publish(msg)

    def _publish_cloud(self, payload: dict[str, Any]) -> None:
        points = decode_array(payload.get("xyzF32"), np.dtype("<f4")).reshape((-1, 3))
        msg = PointCloud2()
        msg.header.stamp, msg.header.frame_id = stamp(payload.get("timestamp", self.last_sim_time)), "mid360_link"
        msg.height, msg.width = 1, int(points.shape[0])
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian, msg.point_step = False, 12
        msg.row_step, msg.is_dense = msg.width * msg.point_step, False
        msg.data = points.astype(np.dtype("<f4"), copy=False).tobytes()
        self.cloud_pub.publish(msg)

    def _publish_camera(self, payload: dict[str, Any]) -> None:
        camera_id = str(payload.get("id", ""))
        publishers = self.camera_publishers.get(camera_id)
        if publishers is None:
            return
        width, height = int(payload["width"]), int(payload["height"])
        frame_id = "front_camera_optical_frame" if camera_id == "front_rgbd" else "wrist_camera_optical_frame"
        at = stamp(payload.get("timestamp", self.last_sim_time))
        rgba = decode_array(payload.get("rgbaU8"), np.uint8).reshape((height, width, 4))
        rgb = Image()
        rgb.header.stamp, rgb.header.frame_id = at, frame_id
        rgb.height, rgb.width, rgb.encoding, rgb.is_bigendian, rgb.step = height, width, "rgb8", 0, width * 3
        rgb.data = np.ascontiguousarray(rgba[:, :, :3]).tobytes()
        publishers["rgb"].publish(rgb)
        publishers["info"].publish(camera_info(width, height, float(payload.get("fovyDeg", 60.0)), frame_id, at))

        depth_payload = payload.get("depth")
        if depth_payload and "depth" in publishers:
            depth_width, depth_height = int(depth_payload["width"]), int(depth_payload["height"])
            depth_values = decode_array(depth_payload.get("dataF32"), np.dtype("<f4")).reshape((depth_height, depth_width))
            if (depth_width, depth_height) != (width, height):
                scale_y, scale_x = height // depth_height, width // depth_width
                depth_values = np.repeat(np.repeat(depth_values, scale_y, axis=0), scale_x, axis=1)[:height, :width]
            depth = Image()
            depth.header.stamp, depth.header.frame_id = at, frame_id
            depth.height, depth.width, depth.encoding, depth.is_bigendian, depth.step = height, width, "32FC1", 0, width * 4
            depth.data = depth_values.astype(np.dtype("<f4"), copy=False).tobytes()
            publishers["depth"].publish(depth)


async def websocket_main(node: MujocoRosBridge, host: str, port: int) -> None:
    """Accept one active simulator runtime and replace stale sessions deterministically."""
    node.loop = asyncio.get_running_loop()

    async def handle(websocket: Any) -> None:
        if node.websocket is not None and node.websocket is not websocket:
            await node.websocket.close(4001, "replaced by a newer simulator runtime")
        node.websocket = websocket
        node.shared_state.connected = True
        try:
            async for raw in websocket:
                try:
                    payload = json.loads(raw)
                except (json.JSONDecodeError, TypeError):
                    continue
                if payload.get("type") == "hello":
                    node.shared_state.environment = str(payload.get("environment", ""))
                    node.shared_state.backend = str(payload.get("backend") or "web")
                    node.get_logger().info(
                        f"runtime connected: backend={node.shared_state.backend} "
                        f"environment={node.shared_state.environment} "
                        f"mode={payload.get('visualMode')}"
                    )
                else:
                    node.receive(payload)
        finally:
            if node.websocket is websocket:
                node.websocket = None
                node.shared_state.connected = False
                node.get_logger().warning("simulator runtime disconnected; motion watchdog will stop the robot")

    async with websockets.serve(handle, host, port, max_size=32 * 1024 * 1024, ping_interval=10, ping_timeout=10):
        await asyncio.Future()


def main() -> None:
    """Start ROS, health HTTP, and WebSocket runtimes with clean shutdown."""
    rclpy.init()
    state = BridgeState()
    node = MujocoRosBridge(state)

    def spin_ros() -> None:
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    ros_thread = threading.Thread(target=spin_ros, daemon=True)
    ros_thread.start()
    HealthHandler.state = state
    health = ThreadingHTTPServer((os.environ.get("BRIDGE_BIND", "0.0.0.0"), int(os.environ.get("BRIDGE_HEALTH_PORT", "8766"))), HealthHandler)
    threading.Thread(target=health.serve_forever, daemon=True).start()
    try:
        asyncio.run(websocket_main(node, os.environ.get("BRIDGE_BIND", "0.0.0.0"), int(os.environ.get("BRIDGE_WS_PORT", "8765"))))
    except KeyboardInterrupt:
        pass
    finally:
        health.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=2)


if __name__ == "__main__":
    main()
