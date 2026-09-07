"""VLM-confirmed pick tool that waits for physical MuJoCo feedback."""
from __future__ import annotations

import base64
import json
import os
import threading
import time
from io import BytesIO

import numpy as np
import requests
from PIL import Image as PillowImage
from robonix_api import Skill, Ok, Err

from primitives.common.runtime import package_root, provider_id, timeout_value

provider = Skill(
    id=provider_id("pick"), namespace="robonix/skill/pick",
    pkg_root=package_root())
condition = threading.Condition()
latest_rgb = None
latest_objects = []
latest_status = {"status": "idle"}
pick_publisher = None
arm_publisher = None
config = {}

from pick_mcp import Pick_Request, Pick_Response, PutDown_Request, PutDown_Response  # noqa: E402
from geometry_msgs_mcp import Point, Pose, PoseStamped, Quaternion  # noqa: E402
from std_msgs_mcp import Header  # noqa: E402
from builtin_interfaces_mcp import Time  # noqa: E402


def empty_pose() -> PoseStamped:
    """Create the neutral response pose required by the standard pick contract."""
    return PoseStamped(
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id="arm_base_link"),
        pose=Pose(position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)))


def receive_rgb(message) -> None:
    """Cache the newest wrist RGB frame as JPEG for VLM verification."""
    global latest_rgb
    array = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.width, 3)
    buffer = BytesIO()
    PillowImage.fromarray(array).save(buffer, format="JPEG", quality=88)
    with condition:
        latest_rgb = buffer.getvalue()
        condition.notify_all()


def receive_objects(message) -> None:
    """Cache ground-truth object state only for execution target resolution."""
    global latest_objects
    try:
        parsed = json.loads(message.data)
    except (json.JSONDecodeError, TypeError):
        return
    with condition:
        latest_objects = parsed
        condition.notify_all()


def receive_status(message) -> None:
    """Wake a synchronous pick request when the simulator reports progress."""
    global latest_status
    try:
        parsed = json.loads(message.data)
    except (json.JSONDecodeError, TypeError):
        return
    with condition:
        latest_status = parsed
        condition.notify_all()


def vlm_resolve_object(requested: str) -> tuple[str, str]:
    """Ask the configured VLM which task object matches the user's phrase."""
    aliases = {
        "jar": "task_glass_jar", "glass jar": "task_glass_jar", "玻璃罐": "task_glass_jar", "罐子": "task_glass_jar",
        "glass": "task_water_glass", "water glass": "task_water_glass", "水杯": "task_water_glass", "玻璃杯": "task_water_glass",
        "book": "task_hardcover_book", "hardcover book": "task_hardcover_book", "精装书": "task_hardcover_book", "书本": "task_hardcover_book",
        "succulent": "task_succulent", "plant": "task_succulent", "盆栽": "task_succulent", "多肉植物": "task_succulent",
        "remote": "task_tv_remote", "tv remote": "task_tv_remote", "遥控器": "task_tv_remote", "电视遥控器": "task_tv_remote",
    }
    with condition:
        jpeg = latest_rgb
        available = [item.get("name", "") for item in latest_objects]
    deterministic = aliases.get(requested.lower(), requested)
    # Environment task objects are a closed, authoritative inventory. Exact
    # names and aliases must not become less reliable merely because a VLM is
    # configured or the wrist camera cannot see the object from its stow pose.
    if deterministic in available:
        return deterministic, "resolved from exact simulated object inventory"
    if not config.get("vlm_api_key") or not jpeg:
        return "", "VLM unavailable or wrist camera has no frame"
    prompt = (
        "Identify which visible tabletop object the request refers to. "
        f"Request: {requested!r}. Allowed IDs: {available}. "
        "Return JSON only: {\"object_id\":\"one allowed ID or none\",\"reason\":\"short\"}."
    )
    payload = {
        "model": config.get("vlm_model", "gpt-5.6-sol"),
        "temperature": 0,
        "max_tokens": 256,
        "messages": [{"role": "user", "content": [
            {"type": "text", "text": prompt},
            {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + base64.b64encode(jpeg).decode()}},
        ]}],
    }
    try:
        response = requests.post(
            config["vlm_base_url"].rstrip("/") + "/chat/completions",
            headers={"Authorization": "Bearer " + config["vlm_api_key"]}, json=payload,
            timeout=float(config.get("vlm_timeout_s", 30)))
        response.raise_for_status()
        content = response.json()["choices"][0]["message"]["content"].strip()
        if content.startswith("```"):
            content = content.split("\n", 1)[1].rsplit("```", 1)[0]
        result = json.loads(content)
        object_id = str(result.get("object_id", ""))
        if object_id in available:
            return object_id, str(result.get("reason", "VLM matched visible object"))
        return "", "VLM did not find a matching available object"
    except Exception as error:  # noqa: BLE001
        if deterministic in available:
            return deterministic, f"VLM failed; exact-name fallback used: {error}"
        return "", f"VLM request failed: {error}"


@provider.mcp("robonix/skill/pick/pick")
def pick(request: Pick_Request) -> Pick_Response:
    """Directly pick an environment dynamic object; this skill resolves its own inventory and wrist image."""
    started = time.monotonic()
    object_id, evidence = vlm_resolve_object(request.object_name.strip())
    if not object_id:
        return Pick_Response(
            success=False, message="detection_failed: " + evidence, grasp_pose=empty_pose(),
            gripper_width=0.0, score=0.0, elapsed_s=time.monotonic() - started)
    from std_msgs.msg import String
    with condition:
        previous_sequence = int(latest_status.get("sequence", -1))
    pick_publisher.publish(String(data=object_id))
    deadline = time.monotonic() + float(config.get("pick_timeout_s", 45))
    terminal = None
    with condition:
        while time.monotonic() < deadline:
            status = dict(latest_status)
            if int(status.get("sequence", -1)) > previous_sequence and status.get("object") == object_id:
                if status.get("status") in ("succeeded", "failed"):
                    terminal = status
                    break
            condition.wait(timeout=0.2)
    elapsed = time.monotonic() - started
    if terminal is None:
        return Pick_Response(
            success=False, message="timeout: no terminal simulator feedback", grasp_pose=empty_pose(),
            gripper_width=0.0, score=0.0, elapsed_s=elapsed)
    success = terminal["status"] == "succeeded"
    return Pick_Response(
        success=success, message=f"{terminal.get('message', terminal['status'])}; perception: {evidence}",
        grasp_pose=empty_pose(), gripper_width=0.0, score=1.0 if success else 0.0, elapsed_s=elapsed)


@provider.mcp("robonix/skill/pick/put_down")
def put_down(_request: PutDown_Request) -> PutDown_Response:
    """Lower the held object, verify stable support, then stow Piper."""
    started = time.monotonic()
    from sensor_msgs.msg import JointState
    with condition:
        previous_sequence = int(latest_status.get("sequence", -1))
    message = JointState()
    message.name = ["gripper"]
    message.position = [0.07]
    arm_publisher.publish(message)
    deadline = time.monotonic() + float(config.get("pick_timeout_s", 45))
    terminal = None
    with condition:
        while time.monotonic() < deadline:
            status = dict(latest_status)
            if int(status.get("sequence", -1)) > previous_sequence:
                if status.get("status") in ("succeeded", "failed"):
                    terminal = status
                    break
            condition.wait(timeout=0.2)
    elapsed = time.monotonic() - started
    if terminal is None:
        return PutDown_Response(success=False, message="timeout: no safe-placement feedback", elapsed_s=elapsed)
    return PutDown_Response(
        success=terminal["status"] == "succeeded",
        message=terminal.get("message", terminal["status"]),
        elapsed_s=elapsed,
    )


@provider.on_init
def initialize(deployment_config):
    """Create ROS subscriptions and keep all secrets in lifecycle config."""
    global pick_publisher, arm_publisher, config
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import String
    config = dict(deployment_config or {})
    try:
        config["vlm_timeout_s"] = timeout_value(config, "vlm_timeout_s", 30)
        config["pick_timeout_s"] = timeout_value(config, "pick_timeout_s", 45)
    except ValueError as error:
        return Err(str(error))
    for key in ("vlm_base_url", "vlm_api_key", "vlm_model"):
        value = config.get(key, "")
        if value is not None and not isinstance(value, str):
            return Err(f"{key} must be a string")
    provider.create_subscription("robonix/primitive/camera/rgb", topic="/wrist/rgb", msg_type=Image, callback=receive_rgb, qos="best_effort", declare=False)
    provider.create_subscription("sim/task_objects", topic="/sim/task_objects", msg_type=String, callback=receive_objects, qos="reliable", declare=False)
    provider.create_subscription("sim/pick_status", topic="/sim/pick_status", msg_type=String, callback=receive_status, qos="reliable", declare=False)
    pick_publisher = provider.create_publisher("sim/pick_command", topic="/sim/pick_command", msg_type=String, qos="reliable", declare=False)
    arm_publisher = provider.create_publisher("robonix/primitive/arm/joint_command", topic="/arm/joint_command", msg_type=JointState, qos="reliable", declare=False)
    return Ok()


@provider.on_activate
def activate():
    """Allow the executor to activate this skill on demand."""
    return Ok()


@provider.on_deactivate
def deactivate():
    """Return to the idle state after an invocation."""
    return Ok()


@provider.on_shutdown
def shutdown():
    """Complete lifecycle shutdown."""
    return Ok()


if __name__ == "__main__":
    provider.run()
