"""RGB or RGB-D camera capability provider with MCP snapshots."""
from __future__ import annotations

import os
import threading
import time
from io import BytesIO

import numpy as np
from PIL import Image as PillowImage
from robonix_api import Primitive, Ok, Err

from .runtime import package_root, provider_id, timeout_value, topic_value

provider = Primitive(
    id=provider_id("front_camera"), namespace="robonix/primitive/camera",
    pkg_root=package_root())
lock = threading.Lock()
latest_rgb = None
latest_depth = None

import builtin_interfaces_mcp  # noqa: E402
import std_msgs_mcp  # noqa: E402
from sensor_msgs_mcp import Image  # noqa: E402
from std_msgs_mcp import Empty  # noqa: E402


def image_to_jpeg(message, depth=False):
    """Convert bridge RGB/float-depth images to compact MCP JPEG payloads."""
    if depth:
        raw = np.frombuffer(message.data, dtype=np.float32).reshape(message.height, message.width)
        valid = np.isfinite(raw)
        if valid.any():
            near, far = np.percentile(raw[valid], [2, 98])
            array = np.where(valid, np.clip((raw - near) / max(far - near, 1e-6), 0, 1) * 255, 0).astype(np.uint8)
        else:
            array = np.zeros((message.height, message.width), dtype=np.uint8)
    else:
        array = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.width, 3)
    buffer = BytesIO()
    PillowImage.fromarray(array).save(buffer, format="JPEG", quality=88)
    return buffer.getvalue(), message.width, message.height, message.header.frame_id


def receive_rgb(message):
    """Cache a compressed RGB frame for snapshot calls."""
    global latest_rgb
    with lock:
        latest_rgb = image_to_jpeg(message)


def receive_depth(message):
    """Cache a normalized depth frame for depth snapshot calls."""
    global latest_depth
    with lock:
        latest_depth = image_to_jpeg(message, depth=True)


def as_mcp(cached) -> Image:
    """Wrap cached JPEG bytes as the camera contract's Image message."""
    if cached is None:
        raise RuntimeError("camera has not produced a frame")
    data, width, height, frame = cached
    now = time.time()
    header = std_msgs_mcp.Header(
        stamp=builtin_interfaces_mcp.Time(sec=int(now), nanosec=int(now % 1 * 1e9)), frame_id=frame)
    return Image(header=header, height=height, width=width, encoding="jpeg", is_bigendian=0, step=len(data), data=data)


@provider.mcp("robonix/primitive/camera/snapshot")
def snapshot(_request: Empty) -> Image:
    """Return the most recent RGB image as JPEG."""
    with lock:
        return as_mcp(latest_rgb)


@provider.mcp("robonix/primitive/camera/depth_snapshot")
def depth_snapshot(_request: Empty) -> Image:
    """Return the most recent normalized depth image as JPEG."""
    with lock:
        return as_mcp(latest_depth)


@provider.on_init
def initialize(config):
    """Bind camera contracts to the selected bridge topic set."""
    config = config or {}
    try:
        rgb_topic = topic_value(config, "rgb_topic", "/front/rgb")
        depth_topic = topic_value(config, "depth_topic", "", allow_empty=True)
        info_topic = topic_value(config, "info_topic", "/front/camera_info")
        extrinsics_topic = topic_value(config, "extrinsics_topic", "", allow_empty=True)
        sentinel_timeout_s = timeout_value(config)
    except ValueError as error:
        return Err(str(error))
    advertise_streams = config.get("advertise_streams", True)
    if not isinstance(advertise_streams, bool):
        return Err("advertise_streams must be a boolean")
    provider.create_subscription(
        "robonix/primitive/camera/rgb", topic=rgb_topic, msg_type="Image",
        callback=receive_rgb, qos="best_effort", declare=False)
    if not provider.wait_for_topic(rgb_topic, "Image", sentinel_timeout_s):
        return Err(f"no RGB image received on {rgb_topic}; start the simulator runtime first")
    if depth_topic:
        provider.create_subscription(
            "robonix/primitive/camera/depth", topic=depth_topic, msg_type="Image",
            callback=receive_depth, qos="best_effort", declare=False)
    # Scene auto-discovers one provider per canonical stream contract. A wrist
    # camera can still expose MCP snapshots and feed a manipulation skill while
    # staying out of that global perception route.
    if advertise_streams:
        provider.declare_ros2_topic("robonix/primitive/camera/rgb", rgb_topic, qos="best_effort")
        provider.declare_ros2_topic("robonix/primitive/camera/intrinsics", info_topic, qos="reliable")
        if extrinsics_topic:
            provider.declare_ros2_topic(
                "robonix/primitive/camera/extrinsics", extrinsics_topic,
                qos="transient_local")
        if depth_topic:
            provider.declare_ros2_topic("robonix/primitive/camera/depth", depth_topic, qos="best_effort")
    return Ok()


@provider.on_shutdown
def shutdown():
    """Complete lifecycle shutdown."""
    return Ok()


if __name__ == "__main__":
    provider.run()
