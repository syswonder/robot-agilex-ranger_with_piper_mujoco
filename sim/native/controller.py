from __future__ import annotations

import math
import time
from dataclasses import dataclass

import mujoco
import numpy as np


WHEELS = ("fl", "fr", "rl", "rr")
WHEELBASE = 0.5
TRACK = 0.38
WHEEL_RADIUS = 0.09
PIPER_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
PIPER_STOW = np.array([0.0, 0.82, -2.2, 0.0, 1.1, 0.0])
PIPER_GRASP_SEED = np.array([0.0, 1.57, -1.3485, 0.0, 0.0, 0.0])
PIPER_LIMITS = np.array([
    [-2.618, 2.618], [0.0, 3.14], [-2.697, 0.0],
    [-1.832, 1.832], [-1.22, 1.22], [-3.14, 3.14],
])
COMMAND_WATCHDOG_S = 0.35
MAX_LINEAR_SPEED = 0.6
MAX_YAW_RATE = 1.2
IK_EPSILON = 1e-4
IK_DAMPING = 2e-3
GRASP_HOLD_VERIFY_S = 3.0
PLACE_SETTLE_S = 1.5


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _matrix_quaternion(matrix: np.ndarray) -> list[float]:
    quaternion = np.empty(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quaternion, np.asarray(matrix, dtype=np.float64).reshape(9))
    return quaternion.tolist()


@dataclass
class GraspAttachment:
    object_name: str
    joint_id: int
    site_id: int
    support_z: float
    world_offset: np.ndarray


class RangerPiperController:
    """Native implementation of the Ranger/Piper controller contract."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self.model = model
        self.data = data
        self.actuators = self._names(mujoco.mjtObj.mjOBJ_ACTUATOR, model.nu)
        self.joints = self._names(mujoco.mjtObj.mjOBJ_JOINT, model.njnt)
        self.bodies = self._names(mujoco.mjtObj.mjOBJ_BODY, model.nbody)
        self.sites = self._names(mujoco.mjtObj.mjOBJ_SITE, model.nsite)
        self.targets = np.r_[PIPER_STOW, 0.0].astype(np.float64)
        self.initial_qpos = data.qpos.copy()
        self.initial_qvel = data.qvel.copy()
        self.last_twist_time = -math.inf
        self.twist = np.zeros(3)
        self.estopped = False
        self.pick_task: dict | None = None
        self.grasp_attachment: GraspAttachment | None = None
        self.pick_status = {"sequence": 0, "status": "idle", "object": "", "message": ""}
        self.reset()

    def _names(self, object_type, count: int) -> dict[str, int]:
        return {
            mujoco.mj_id2name(self.model, object_type, index): index
            for index in range(count)
            if mujoco.mj_id2name(self.model, object_type, index)
        }

    def reset(self) -> None:
        self.data.qpos[:] = self.initial_qpos
        self.data.qvel[:] = self.initial_qvel
        self.data.ctrl[:] = 0
        self.targets[:] = np.r_[PIPER_STOW, 0.0]
        self.last_twist_time = -math.inf
        self.twist[:] = 0
        self.estopped = False
        self.pick_task = None
        self.grasp_attachment = None
        for index, name in enumerate((*PIPER_JOINTS, "joint7")):
            joint = self.joints[f"piper_{name}"]
            self.data.qpos[self.model.jnt_qposadr[joint]] = self.targets[index]
            self.data.qvel[self.model.jnt_dofadr[joint]] = 0
        self._apply_arm_targets()
        mujoco.mj_forward(self.model, self.data)
        self._set_pick_status("idle", "", "Arm reset")

    def command(self, message: dict) -> bool:
        kind = message.get("type")
        if kind == "cmd_vel":
            self.twist[:] = [
                _clamp(float(message.get("linearX", 0.0)), -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED),
                _clamp(float(message.get("linearY", 0.0)), -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED),
                _clamp(float(message.get("angularZ", 0.0)), -MAX_YAW_RATE, MAX_YAW_RATE),
            ]
            self.last_twist_time = time.monotonic()
            self.estopped = False
            return True
        if kind == "arm_joint_command":
            return self._set_arm_joint_command(message)
        if kind == "arm_pose_command":
            return self._set_arm_pose_command(message)
        if kind == "pick_object":
            return self._start_pick(str(message.get("name", "")))
        if kind == "emergency_stop":
            self.estopped = True
            self.last_twist_time = -math.inf
            if self.pick_task:
                self._set_pick_status("failed", self.pick_task["object_name"], "Emergency stop")
            self.pick_task = None
            self.grasp_attachment = None
            return True
        if kind == "reset":
            self.reset()
            return True
        return False

    def step(self) -> None:
        command = self._swerve(*self.twist) if (
            not self.estopped and time.monotonic() - self.last_twist_time <= COMMAND_WATCHDOG_S
        ) else self._swerve(0.0, 0.0, 0.0)
        for wheel in WHEELS:
            steer_target, wheel_rate = command[wheel]
            self.data.ctrl[self.actuators[f"{wheel}_steering"]] = steer_target
            steering_joint = self.joints[f"{wheel}_steering_joint"]
            current = self.data.qpos[self.model.jnt_qposadr[steering_joint]]
            error = abs(_normalize_angle(steer_target - current))
            alignment = _clamp((0.35 - error) / 0.2, 0.0, 1.0)
            self.data.ctrl[self.actuators[f"{wheel}_wheel_motor"]] = wheel_rate * alignment
        if self.grasp_attachment:
            self._update_attachment()
        if self.pick_task and not self.estopped:
            self._step_pick()
        self._apply_arm_targets()

    @staticmethod
    def _swerve(linear_x: float, linear_y: float, angular_z: float) -> dict[str, tuple[float, float]]:
        positions = {
            "fl": (WHEELBASE * 0.5, TRACK * 0.5),
            "fr": (WHEELBASE * 0.5, -TRACK * 0.5),
            "rl": (-WHEELBASE * 0.5, TRACK * 0.5),
            "rr": (-WHEELBASE * 0.5, -TRACK * 0.5),
        }
        result: dict[str, tuple[float, float]] = {}
        maximum = 0.0
        for wheel, (x, y) in positions.items():
            velocity_x = linear_x - angular_z * y
            velocity_y = linear_y + angular_z * x
            speed = math.hypot(velocity_x, velocity_y)
            angle = _normalize_angle(math.atan2(velocity_y, velocity_x)) if speed > 1e-6 else 0.0
            if angle > math.pi * 0.5:
                angle -= math.pi
                speed = -speed
            elif angle < -math.pi * 0.5:
                angle += math.pi
                speed = -speed
            rate = speed / WHEEL_RADIUS
            result[wheel] = (angle, rate)
            maximum = max(maximum, abs(rate))
        scale = 12.0 / maximum if maximum > 12.0 else 1.0
        return {wheel: (angle, rate * scale) for wheel, (angle, rate) in result.items()}

    def _apply_arm_targets(self) -> None:
        for index, name in enumerate(PIPER_JOINTS):
            self.data.ctrl[self.actuators[f"piper_{name}"]] = self.targets[index]
        self.data.ctrl[self.actuators["piper_gripper"]] = self.targets[6]

    def _set_arm_joint_command(self, message: dict) -> bool:
        names = message.get("names") or []
        positions = message.get("positions") or []
        if self.grasp_attachment and any(
            name in ("gripper", "piper_joint7") and float(positions[index]) > 0.05
            for index, name in enumerate(names[:len(positions)])
        ):
            return self._start_put_down()
        self.pick_task = None
        for name, raw_value in zip(names, positions):
            value = float(raw_value)
            plain = str(name).removeprefix("piper_")
            if plain in PIPER_JOINTS:
                index = PIPER_JOINTS.index(plain)
                self.targets[index] = _clamp(value, *PIPER_LIMITS[index])
            elif name in ("gripper", "piper_joint7"):
                self.targets[6] = _clamp(value * 0.5, 0.0, 0.035)
        return True

    def _set_arm_pose_command(self, message: dict) -> bool:
        position = np.asarray(message.get("position") or [], dtype=np.float64)
        if position.shape != (3,) or not np.isfinite(position).all():
            return False
        self.pick_task = None
        return self._solve_ik(self._arm_local_to_world(position))

    def _start_pick(self, raw_name: str) -> bool:
        aliases = {
            "glass": "task_water_glass", "water glass": "task_water_glass",
            "水杯": "task_water_glass", "玻璃杯": "task_water_glass",
        }
        requested = raw_name.strip().lower()
        object_name = aliases.get(requested, requested)
        body_id = self.bodies.get(object_name)
        if body_id is None:
            self._set_pick_status("failed", object_name, "Object is not available in this environment")
            return False
        position = self.data.xpos[body_id].copy()
        self.targets[6] = 0.035
        self.pick_task = {
            "object_name": object_name, "body_id": body_id, "phase": "approach",
            "phase_start": float(self.data.time), "initial_z": float(position[2]), "retries": 0,
        }
        self._set_pick_status("running", object_name, "Approaching object")
        accepted = self._solve_ik(position + np.array([0.0, 0.0, 0.18]), [PIPER_GRASP_SEED])
        if not accepted:
            self.pick_task = None
            self._set_pick_status("failed", object_name, "Object is outside the Piper workspace")
        return accepted

    def _start_put_down(self) -> bool:
        attachment = self.grasp_attachment
        if not attachment or self.pick_task:
            return False
        body_id = self.bodies.get(attachment.object_name)
        if body_id is None:
            return False
        tcp = self.data.site_xpos[attachment.site_id].copy()
        object_position = self.data.xpos[body_id].copy()
        target = tcp.copy()
        target[2] += attachment.support_z - object_position[2]
        self.pick_task = {
            "mode": "place", "object_name": attachment.object_name, "body_id": body_id,
            "phase": "place-descend", "phase_start": float(self.data.time),
            "support_z": attachment.support_z,
        }
        self._set_pick_status("running", attachment.object_name, "Lowering object to its support surface")
        if not self._solve_ik(target):
            self.pick_task = None
            return False
        return True

    def _solve_ik(self, target: np.ndarray, fallback_seeds: list[np.ndarray] | None = None) -> bool:
        site_id = self.sites["piper_tcp_site"]
        joint_ids = [self.joints[f"piper_{name}"] for name in PIPER_JOINTS]
        addresses = [self.model.jnt_qposadr[joint] for joint in joint_ids]
        saved = self.data.qpos[addresses].copy()
        seeds = [self.targets[:6].copy(), *(fallback_seeds or [])]
        best_candidate = None
        best_error = math.inf
        for seed in seeds:
            candidate = np.asarray(seed, dtype=np.float64).copy()
            error_norm = math.inf
            for _ in range(56):
                self.data.qpos[addresses] = candidate
                mujoco.mj_forward(self.model, self.data)
                current = self.data.site_xpos[site_id].copy()
                error = target - current
                error_norm = float(np.linalg.norm(error))
                if error_norm < 0.004:
                    break
                jacobian = np.zeros((3, 6))
                for joint in range(6):
                    self.data.qpos[addresses[joint]] = candidate[joint] + IK_EPSILON
                    mujoco.mj_forward(self.model, self.data)
                    jacobian[:, joint] = (self.data.site_xpos[site_id] - current) / IK_EPSILON
                    self.data.qpos[addresses[joint]] = candidate[joint]
                weighted = np.linalg.solve(jacobian @ jacobian.T + IK_DAMPING * np.eye(3), error)
                delta = jacobian.T @ weighted
                candidate += np.clip(delta, -0.18, 0.18)
                candidate = np.clip(candidate, PIPER_LIMITS[:, 0], PIPER_LIMITS[:, 1])
            if error_norm < best_error:
                best_error = error_norm
                best_candidate = candidate.copy()
        self.data.qpos[addresses] = saved
        mujoco.mj_forward(self.model, self.data)
        if best_candidate is None or best_error > 0.055:
            return False
        self.targets[:6] = best_candidate
        return True

    def _arm_at_target(self, tolerance: float = 0.035) -> bool:
        for index, name in enumerate(PIPER_JOINTS):
            joint = self.joints[f"piper_{name}"]
            if abs(self.data.qpos[self.model.jnt_qposadr[joint]] - self.targets[index]) > tolerance:
                return False
        return True

    def _attach(self, task: dict) -> bool:
        site_id = self.sites["piper_tcp_site"]
        joint_id = self.joints.get(f"{task['object_name']}_freejoint")
        if joint_id is None:
            return False
        tcp = self.data.site_xpos[site_id].copy()
        position = self.data.xpos[task["body_id"]].copy()
        if np.linalg.norm(tcp - position) > 0.13:
            return False
        self.grasp_attachment = GraspAttachment(
            task["object_name"], joint_id, site_id, task["initial_z"], position - tcp)
        self._update_attachment()
        return True

    def _update_attachment(self) -> None:
        attachment = self.grasp_attachment
        if not attachment:
            return
        qpos_address = self.model.jnt_qposadr[attachment.joint_id]
        dof_address = self.model.jnt_dofadr[attachment.joint_id]
        self.data.qpos[qpos_address:qpos_address + 3] = (
            self.data.site_xpos[attachment.site_id] + attachment.world_offset)
        self.data.qvel[dof_address:dof_address + 6] = 0

    def _step_pick(self) -> None:
        task = self.pick_task
        if not task:
            return
        if task.get("mode") == "place":
            self._step_place(task)
            return
        now = float(self.data.time)
        position = self.data.xpos[task["body_id"]].copy()
        if task["phase"] == "approach" and self._arm_at_target():
            task.update(phase="descend", phase_start=now)
            self._set_pick_status("running", task["object_name"], "Descending to grasp")
            if not self._solve_ik(position + np.array([0.0, 0.0, 0.075])):
                self._set_pick_status("failed", task["object_name"], "Object is outside the Piper workspace")
                self.pick_task = None
        elif task["phase"] == "descend" and self._arm_at_target():
            task.update(phase="close", phase_start=now)
            self.targets[6] = 0.0
            self._set_pick_status("running", task["object_name"], "Closing gripper")
        elif task["phase"] == "close" and now - task["phase_start"] > 1.2:
            if not self._attach(task):
                if task["retries"] < 1:
                    task.update(phase="approach", phase_start=now, retries=task["retries"] + 1)
                    self.targets[6] = 0.035
                    self._set_pick_status("running", task["object_name"], "Contact missed; retrying grasp")
                    if not self._solve_ik(position + np.array([0.0, 0.0, 0.18]), [PIPER_GRASP_SEED]):
                        self.pick_task = None
                else:
                    self._set_pick_status("failed", task["object_name"], "Gripper closed without valid contact")
                    self.pick_task = None
                return
            task.update(phase="lift", phase_start=now)
            self._set_pick_status("running", task["object_name"], "Lifting object")
            self._solve_ik(position + np.array([0.0, 0.0, 0.22]))
        elif task["phase"] == "lift" and self._arm_at_target():
            if position[2] <= task["initial_z"] + 0.06 or not self.grasp_attachment:
                self._set_pick_status("failed", task["object_name"], "Object was not securely lifted")
                self.pick_task = None
                return
            task.update(phase="hold", phase_start=now, hold_reference=position.copy(), max_hold_drift=0.0)
            self._set_pick_status("running", task["object_name"], "Verifying sustained grasp stability")
        elif task["phase"] == "hold":
            drift = float(np.linalg.norm(position - task["hold_reference"]))
            task["max_hold_drift"] = max(task["max_hold_drift"], drift)
            if not self.grasp_attachment or position[2] <= task["initial_z"] + 0.06 or task["max_hold_drift"] > 0.025:
                self._set_pick_status("failed", task["object_name"], "Object did not remain stable")
                self.pick_task = None
            elif now - task["phase_start"] >= GRASP_HOLD_VERIFY_S:
                self._set_pick_status("succeeded", task["object_name"], "Object remained securely held")
                self.pick_task = None
        elif now - task["phase_start"] > 12.0:
            self._set_pick_status("failed", task["object_name"], f"Timed out during {task['phase']}")
            self.pick_task = None

    def _step_place(self, task: dict) -> None:
        now = float(self.data.time)
        position = self.data.xpos[task["body_id"]].copy()
        if task["phase"] == "place-descend" and self._arm_at_target():
            self.targets[6] = 0.035
            self.grasp_attachment = None
            task.update(phase="place-settle", phase_start=now)
            self._set_pick_status("running", task["object_name"], "Object released; verifying stability")
        elif task["phase"] == "place-settle" and now - task["phase_start"] >= PLACE_SETTLE_S:
            joint = self.joints[f"{task['object_name']}_freejoint"]
            vertical_speed = abs(self.data.qvel[self.model.jnt_dofadr[joint] + 2])
            stable = abs(position[2] - task["support_z"]) <= 0.04 and vertical_speed <= 0.08
            if not stable:
                if now - task["phase_start"] <= 4.0:
                    return
                self._set_pick_status("failed", task["object_name"], "Object did not settle safely")
                self._stow()
                self.pick_task = None
                return
            tcp = self.data.site_xpos[self.sites["piper_tcp_site"]].copy()
            task.update(phase="place-retreat", phase_start=now)
            self._set_pick_status("running", task["object_name"], "Object is stable; retreating gripper")
            if not self._solve_ik(tcp + np.array([0.0, 0.0, 0.16])):
                self._stow()
                task.update(phase="place-stow", phase_start=now)
        elif task["phase"] == "place-retreat" and self._arm_at_target():
            self._stow()
            task.update(phase="place-stow", phase_start=now)
        elif task["phase"] == "place-stow" and self._arm_at_target():
            self._set_pick_status("succeeded", task["object_name"], "Object placed and arm stowed")
            self.pick_task = None
        elif now - task["phase_start"] > 12.0:
            self._set_pick_status("failed", task["object_name"], f"Timed out during {task['phase']}")
            self.pick_task = None

    def _stow(self) -> None:
        self.targets[:6] = PIPER_STOW
        self.targets[6] = 0.0

    def _set_pick_status(self, status: str, object_name: str, message: str) -> None:
        self.pick_status = {
            "sequence": self.pick_status["sequence"] + 1,
            "status": status, "object": object_name, "message": message,
        }

    def _arm_local_to_world(self, position: np.ndarray) -> np.ndarray:
        body = self.bodies["piper_base_link"]
        return self.data.xpos[body] + self.data.xmat[body].reshape(3, 3) @ position

    def _world_to_arm_local(self, position: np.ndarray) -> np.ndarray:
        body = self.bodies["piper_base_link"]
        matrix = self.data.xmat[body].reshape(3, 3)
        return matrix.T @ (position - self.data.xpos[body])

    def state(self) -> dict:
        base_joint = self.joints["ranger_base_freejoint"]
        qpos = self.model.jnt_qposadr[base_joint]
        dof = self.model.jnt_dofadr[base_joint]
        arm_ids = [self.joints[f"piper_{name}"] for name in PIPER_JOINTS]
        gripper = self.joints["piper_joint7"]
        tcp = self.sites["piper_tcp_site"]
        base_matrix = self.data.xmat[self.bodies["piper_base_link"]].reshape(3, 3)
        tcp_local_matrix = base_matrix.T @ self.data.site_xmat[tcp].reshape(3, 3)
        objects = [
            {"name": name, "position": self.data.xpos[body].tolist(), "quaternion": self.data.xquat[body].tolist()}
            for name, body in self.bodies.items()
            if name.startswith("task_") and name != "task_station"
        ]
        return {
            "timestamp": float(self.data.time), "externalControl": True, "estopped": self.estopped,
            "base": {
                "position": self.data.qpos[qpos:qpos + 3].tolist(),
                "quaternion": self.data.qpos[qpos + 3:qpos + 7].tolist(),
                "linearVelocity": self.data.qvel[dof:dof + 3].tolist(),
                "angularVelocity": self.data.qvel[dof + 3:dof + 6].tolist(),
            },
            "arm": {
                "names": [*PIPER_JOINTS, "gripper"],
                "positions": [*[float(self.data.qpos[self.model.jnt_qposadr[joint]]) for joint in arm_ids],
                              2.0 * float(self.data.qpos[self.model.jnt_qposadr[gripper]])],
                "velocities": [*[float(self.data.qvel[self.model.jnt_dofadr[joint]]) for joint in arm_ids],
                               2.0 * float(self.data.qvel[self.model.jnt_dofadr[gripper]])],
                "targets": [*self.targets[:6].tolist(), 2.0 * float(self.targets[6])],
                "endPose": {
                    "position": self._world_to_arm_local(self.data.site_xpos[tcp]).tolist(),
                    "quaternion": _matrix_quaternion(tcp_local_matrix),
                },
            },
            "objects": objects,
        }
