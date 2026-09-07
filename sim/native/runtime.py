#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import base64
import json
import os
import signal
import sys
import time
from pathlib import Path

import mujoco
import numpy as np
import websockets

from sim.native.controller import RangerPiperController
from sim.native.scene_builder import NativeSceneBuilder
from sim.native.sensors import NativeSensorSuite


def encoded(array: np.ndarray) -> str:
    return base64.b64encode(np.ascontiguousarray(array).tobytes()).decode("ascii")


def configure_viewer_camera(model: mujoco.MjModel, data: mujoco.MjData, camera) -> None:
    """Initialize a robot-centered free camera that retains user pan controls."""
    base_body = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "ranger_base_link")
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.trackbodyid = -1
    camera.lookat[:] = data.xpos[base_body]
    camera.distance = 3.2
    camera.azimuth = 135.0
    camera.elevation = -25.0


class NativeRuntime:
    def __init__(self, root: Path, environment: str, robot: str, viewer_enabled: bool) -> None:
        self.builder = NativeSceneBuilder(root)
        self.scene_path, self.environment, self.robot = self.builder.build(environment, robot)
        self.model = mujoco.MjModel.from_xml_path(str(self.scene_path))
        self.data = mujoco.MjData(self.model)
        self.controller = RangerPiperController(self.model, self.data)
        self.sensors = NativeSensorSuite(self.model, self.data, self.robot["sensors"])
        self.viewer_enabled = viewer_enabled
        self.viewer = None
        self.running = True
        self.last_pick_sequence = -1
        self.camera_index = 0
        self.next_state = 0.0
        self.next_lidar = 0.0
        self.next_camera = 0.0

    def stop(self) -> None:
        self.running = False

    async def run(self, websocket_url: str) -> None:
        if self.viewer_enabled:
            import mujoco.viewer
            self.viewer = mujoco.viewer.launch_passive(
                self.model, self.data, show_left_ui=False, show_right_ui=False)
            with self.viewer.lock():
                self.viewer.opt.geomgroup[3] = 0
                self.viewer.opt.geomgroup[4] = 0
                configure_viewer_camera(self.model, self.data, self.viewer.cam)
        try:
            async with websockets.connect(websocket_url, max_size=32 * 1024 * 1024) as websocket:
                await websocket.send(json.dumps({
                    "type": "hello", "protocolVersion": 1, "backend": "native",
                    "environment": self.environment["id"], "robot": self.robot["id"],
                    "visualMode": self.environment["visualMode"],
                }))
                receiver = asyncio.create_task(self._receive(websocket))
                try:
                    await self._simulate(websocket)
                finally:
                    receiver.cancel()
                    await asyncio.gather(receiver, return_exceptions=True)
        finally:
            self.sensors.close()
            if self.viewer is not None:
                self.viewer.close()
            self.builder.cleanup(self.scene_path)

    async def _receive(self, websocket) -> None:
        async for raw in websocket:
            try:
                message = json.loads(raw)
            except (json.JSONDecodeError, TypeError):
                continue
            if message.get("type") == "ping":
                await websocket.send(json.dumps({
                    "type": "pong", "sequence": message.get("sequence"),
                    "simulationTime": float(self.data.time),
                }))
                continue
            accepted = self.controller.command(message)
            await websocket.send(json.dumps({
                "type": "command_ack", "sequence": message.get("sequence"),
                "accepted": bool(accepted),
            }))

    async def _simulate(self, websocket) -> None:
        start_wall = time.monotonic()
        start_sim = float(self.data.time)
        next_viewer_sync = start_wall
        while self.running and (self.viewer is None or self.viewer.is_running()):
            now = time.monotonic()
            target_sim = start_sim + now - start_wall
            while self.data.time < target_sim and self.running:
                self.controller.step()
                mujoco.mj_step(self.model, self.data)
            await self._publish_due(websocket)
            if self.viewer is not None and now >= next_viewer_sync:
                self.viewer.sync()
                next_viewer_sync = now + 1.0 / 60.0
            await asyncio.sleep(0.001)

    async def _publish_due(self, websocket) -> None:
        sim_time = float(self.data.time)
        if sim_time >= self.next_state:
            await websocket.send(json.dumps({
                "type": "state", "environment": self.environment["id"],
                "simulationTime": sim_time, "robot": self.controller.state(),
                "imu": self.sensors.imu(),
            }, separators=(",", ":")))
            status = self.controller.pick_status
            if status["sequence"] != self.last_pick_sequence:
                self.last_pick_sequence = status["sequence"]
                await websocket.send(json.dumps({
                    "type": "pick_status", **status, "simulationTime": sim_time,
                }, separators=(",", ":")))
            self.next_state = sim_time + 0.02
        if sim_time >= self.next_lidar:
            scan, cloud = self.sensors.lidar()
            await websocket.send(json.dumps({
                "type": "scan", "timestamp": scan["timestamp"], "frame": "mid360_link",
                "angleMin": scan["angleMin"], "angleMax": scan["angleMax"],
                "angleIncrement": scan["angleIncrement"], "rangeMin": scan["rangeMin"],
                "rangeMax": scan["rangeMax"], "rangesF32": encoded(scan["ranges"]),
            }, separators=(",", ":")))
            await websocket.send(json.dumps({
                "type": "pointcloud", "timestamp": cloud["timestamp"], "frame": "mid360_link",
                "pointCount": int(cloud["points"].shape[0]), "xyzF32": encoded(cloud["points"]),
            }, separators=(",", ":")))
            self.next_lidar = sim_time + 1.0 / float(self.robot["sensors"]["lidar"]["updateHz"])
        if sim_time >= self.next_camera:
            camera_ids = list(self.sensors.cameras)
            camera = self.sensors.camera(camera_ids[self.camera_index % len(camera_ids)])
            self.camera_index += 1
            await websocket.send(json.dumps({
                "type": "camera", "timestamp": camera["timestamp"], "id": camera["id"],
                "frame": "front_camera_optical_frame" if camera["id"] == "front_rgbd" else "wrist_camera_optical_frame",
                "width": camera["width"], "height": camera["height"],
                "rgbaU8": encoded(camera["rgb"]), "fovyDeg": camera["fovyDeg"],
                "depth": {"width": camera["depth"]["width"], "height": camera["depth"]["height"],
                          "dataF32": encoded(camera["depth"]["data"])},
            }, separators=(",", ":")))
            self.next_camera = sim_time + 1.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Native MuJoCo runtime for Robonix")
    parser.add_argument("--environment", default=os.environ.get("SIM_ENVIRONMENT", "scenesmith_house_187"))
    parser.add_argument("--robot", default=os.environ.get("SIM_ROBOT", "ranger_mini_v3_piper"))
    parser.add_argument("--websocket", default=os.environ.get("BRIDGE_WS_URL", "ws://127.0.0.1:8765"))
    parser.add_argument("--headless", action="store_true", default=os.environ.get("SIM_HEADLESS") == "1")
    return parser.parse_args()


async def async_main() -> int:
    args = parse_args()
    try:
        runtime = NativeRuntime(Path(__file__).resolve().parents[2], args.environment, args.robot, not args.headless)
    except Exception as error:
        print(f"[native] startup failed: {error}", file=sys.stderr)
        return 2
    loop = asyncio.get_running_loop()
    for name in ("SIGINT", "SIGTERM"):
        signal_name = getattr(signal, name, None)
        if signal_name is not None:
            loop.add_signal_handler(signal_name, runtime.stop)
    try:
        await runtime.run(args.websocket)
    except Exception as error:
        print(f"[native] runtime failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(async_main()))
