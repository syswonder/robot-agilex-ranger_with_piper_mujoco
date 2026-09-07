#!/usr/bin/env python3
"""Fast model, sensor, GPU-rendering, and SPZ guard checks for native MuJoCo."""
from __future__ import annotations

import json
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from sim.native.controller import RangerPiperController
from sim.native.runtime import configure_viewer_camera
from sim.native.scene_builder import NativeSceneBuilder
from sim.native.sensors import NativeSensorSuite


def main() -> None:
    builder = NativeSceneBuilder(ROOT)
    rejected_spz = []
    for environment_id in ("kitchen", "meeting_room"):
        try:
            builder.build(environment_id, "ranger_mini_v3_piper")
        except ValueError as error:
            assert "does not support SPZ" in str(error), error
            rejected_spz.append(environment_id)
        else:
            raise AssertionError(f"native scene builder accepted SPZ environment {environment_id}")

    scene_path, environment, robot = builder.build(
        "scenesmith_house_187", "ranger_mini_v3_piper")
    sensors = None
    try:
        model = mujoco.MjModel.from_xml_path(str(scene_path))
        data = mujoco.MjData(model)
        controller = RangerPiperController(model, data)
        sensors = NativeSensorSuite(model, data, robot["sensors"])
        for _ in range(20):
            controller.step()
            mujoco.mj_step(model, data)

        viewer_camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(viewer_camera)
        configure_viewer_camera(model, data, viewer_camera)
        option = mujoco.MjvOption()
        perturb = mujoco.MjvPerturb()
        scene = mujoco.MjvScene(model, maxgeom=1000)
        mujoco.mjv_updateScene(
            model, data, option, perturb, viewer_camera,
            mujoco.mjtCatBit.mjCAT_ALL, scene)
        lookat_before = viewer_camera.lookat.copy()
        mujoco.mjv_moveCamera(
            model, mujoco.mjtMouse.mjMOUSE_MOVE_H, 0.15, 0.0,
            scene, viewer_camera)
        pan_delta = float(np.linalg.norm(viewer_camera.lookat - lookat_before))
        assert viewer_camera.type == mujoco.mjtCamera.mjCAMERA_FREE
        assert pan_delta > 1e-6, "native viewer camera did not accept pan input"

        scan, cloud = sensors.lidar()
        imu = sensors.imu()
        cameras = {}
        for camera_id in sensors.cameras:
            frame = sensors.camera(camera_id)
            rgb = frame["rgb"][:, :, :3]
            depth = frame["depth"]["data"]
            valid_depth = depth[np.isfinite(depth) & (depth > 0)]
            assert rgb.size > 100 and float(rgb.std()) > 2.0, f"{camera_id} RGB is blank"
            assert valid_depth.size > 100, f"{camera_id} depth has no valid returns"
            cameras[camera_id] = {
                "rgb_std": float(rgb.std()),
                "valid_depth": int(valid_depth.size),
            }

        finite_scan = int(np.count_nonzero(np.isfinite(scan["ranges"])))
        assert scan["ranges"].size >= 360 and finite_scan >= 10
        assert cloud["points"].shape[0] >= 100
        assert len(imu["gyroscope"]) == 3 and len(imu["accelerometer"]) == 3
        print(json.dumps({
            "ok": True,
            "environment": environment["id"],
            "model": {"nq": model.nq, "nv": model.nv, "nu": model.nu},
            "scan_returns": finite_scan,
            "cloud_points": int(cloud["points"].shape[0]),
            "cameras": cameras,
            "viewer_pan_delta": pan_delta,
            "rejected_spz": rejected_spz,
        }, indent=2))
    finally:
        if sensors is not None:
            sensors.close()
        builder.cleanup(scene_path)


if __name__ == "__main__":
    main()
