import { BaseController } from '../../../src/utils/controllers/BaseController.js';

const WHEELS = ['fl', 'fr', 'rl', 'rr'];
const WHEELBASE = 0.5;
const TRACK = 0.38;
const WHEEL_RADIUS = 0.09;
const DRIVE_SPEED = 0.6;
const CRAB_ANGLE = 0.65;
const ACKERMANN_ANGLE = 0.55;
const SPIN_RATE = 1.2;

const WHEEL_ACTUATORS = Object.fromEntries(WHEELS.map((wheel) => [wheel, `${wheel}_wheel_motor`]));
const STEERING_ACTUATORS = Object.fromEntries(WHEELS.map((wheel) => [wheel, `${wheel}_steering`]));
const STEERING_JOINTS = Object.fromEntries(WHEELS.map((wheel) => [wheel, `${wheel}_steering_joint`]));
const PIPER_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'];
const PIPER_ACTUATORS = PIPER_JOINTS.map((name) => `piper_${name}`);
// Stowed inside the Ranger footprint instead of extending 0.57 m behind the
// rear-mounted arm base. The old Menagerie home remains a useful IK seed.
const PIPER_STOW = [0, 0.82, -2.2, 0, 1.1, 0];
const PIPER_GRASP_SEED = [0, 1.57, -1.3485, 0, 0, 0];
const PIPER_LIMITS = [
  [-2.618, 2.618], [0, 3.14], [-2.697, 0],
  [-1.832, 1.832], [-1.22, 1.22], [-3.14, 3.14]
];
const PIPER_KEYS = [
  ['Digit1', 'KeyY'], ['Digit2', 'KeyU'], ['Digit3', 'KeyI'],
  ['Digit4', 'KeyO'], ['Digit5', 'KeyP'], ['Digit6', 'BracketLeft']
];
const PIPER_GRIPPER = 'piper_gripper';
const PIPER_GRIPPER_JOINT = 'piper_joint7';
const PIPER_JOINT_RATE = 0.8;
const PIPER_GRIPPER_RATE = 0.05;
const COMMAND_WATCHDOG_MS = 350;
const MAX_LINEAR_SPEED = 0.6;
const MAX_YAW_RATE = 1.2;
const IK_EPSILON = 1e-4;
const IK_DAMPING = 2e-3;
const GRASP_HOLD_VERIFY_S = 3.0;
const PLACE_SETTLE_S = 1.5;

function clamp(value, minimum, maximum) {
  return Math.max(minimum, Math.min(maximum, value));
}

function decodeNames(model, count, addresses) {
  const decoder = new TextDecoder('utf-8');
  const result = new Map();
  for (let index = 0; index < count; index++) {
    const address = addresses[index];
    let end = address;
    while (end < model.names.length && model.names[end] !== 0) end++;
    result.set(decoder.decode(model.names.subarray(address, end)), index);
  }
  return result;
}

function emptyCommand(mode = 'stop') {
  return {
    mode,
    wheelVelocities: Object.fromEntries(WHEELS.map((wheel) => [wheel, 0])),
    steeringAngles: Object.fromEntries(WHEELS.map((wheel) => [wheel, 0]))
  };
}

function ackermannCommand(linearVelocity, virtualSteeringAngle) {
  const command = emptyCommand('ackermann');
  if (Math.abs(virtualSteeringAngle) < 1e-6) {
    for (const wheel of WHEELS) command.wheelVelocities[wheel] = linearVelocity / WHEEL_RADIUS;
    return command;
  }
  const turnRadius = (WHEELBASE * 0.5) / Math.tan(virtualSteeringAngle);
  const yawRate = linearVelocity / turnRadius;
  const positions = {
    fl: [WHEELBASE * 0.5, TRACK * 0.5],
    fr: [WHEELBASE * 0.5, -TRACK * 0.5],
    rl: [-WHEELBASE * 0.5, TRACK * 0.5],
    rr: [-WHEELBASE * 0.5, -TRACK * 0.5]
  };
  for (const wheel of WHEELS) {
    const [x, y] = positions[wheel];
    const localRadius = turnRadius - y;
    const steeringAngle = Math.atan(x / localRadius);
    command.steeringAngles[wheel] = steeringAngle;
    command.wheelVelocities[wheel] = yawRate * localRadius /
      (Math.cos(steeringAngle) * WHEEL_RADIUS);
  }
  return command;
}

function crabCommand(linearVelocity, steeringAngle) {
  const command = emptyCommand('crab');
  for (const wheel of WHEELS) {
    command.steeringAngles[wheel] = steeringAngle;
    command.wheelVelocities[wheel] = linearVelocity / WHEEL_RADIUS;
  }
  return command;
}

function spinCommand(direction) {
  const command = emptyCommand('spin');
  const angle = Math.atan2(WHEELBASE, TRACK);
  const wheelRate = SPIN_RATE * Math.hypot(WHEELBASE * 0.5, TRACK * 0.5) / WHEEL_RADIUS;
  Object.assign(command.steeringAngles, { fl: -angle, fr: angle, rl: angle, rr: -angle });
  Object.assign(command.wheelVelocities, {
    fl: -wheelRate * direction,
    fr: wheelRate * direction,
    rl: -wheelRate * direction,
    rr: wheelRate * direction
  });
  return command;
}

function normalizeAngle(angle) {
  let value = angle;
  while (value > Math.PI) value -= 2 * Math.PI;
  while (value < -Math.PI) value += 2 * Math.PI;
  return value;
}

/** Convert a holonomic Twist into independent steer and drive targets. */
export function swerveCommand(linearX, linearY, angularZ) {
  const command = emptyCommand('twist');
  const positions = {
    fl: [WHEELBASE * 0.5, TRACK * 0.5],
    fr: [WHEELBASE * 0.5, -TRACK * 0.5],
    rl: [-WHEELBASE * 0.5, TRACK * 0.5],
    rr: [-WHEELBASE * 0.5, -TRACK * 0.5]
  };
  let maximumWheelRate = 0;
  for (const wheel of WHEELS) {
    const [x, y] = positions[wheel];
    const velocityX = linearX - angularZ * y;
    const velocityY = linearY + angularZ * x;
    let speed = Math.hypot(velocityX, velocityY);
    let steeringAngle = speed > 1e-6 ? normalizeAngle(Math.atan2(velocityY, velocityX)) : 0;
    // Ranger steering joints cannot rotate through pi. Reverse the wheel instead
    // and keep the equivalent steering target inside the mechanical range.
    if (steeringAngle > Math.PI * 0.5) {
      steeringAngle -= Math.PI;
      speed = -speed;
    } else if (steeringAngle < -Math.PI * 0.5) {
      steeringAngle += Math.PI;
      speed = -speed;
    }
    command.steeringAngles[wheel] = steeringAngle;
    command.wheelVelocities[wheel] = speed / WHEEL_RADIUS;
    maximumWheelRate = Math.max(maximumWheelRate, command.wheelVelocities[wheel]);
  }
  const scale = maximumWheelRate > 12 ? 12 / maximumWheelRate : 1;
  for (const wheel of WHEELS) command.wheelVelocities[wheel] *= scale;
  return command;
}

function matrixToQuaternion(matrix, offset) {
  const m00 = matrix[offset];
  const m01 = matrix[offset + 1];
  const m02 = matrix[offset + 2];
  const m10 = matrix[offset + 3];
  const m11 = matrix[offset + 4];
  const m12 = matrix[offset + 5];
  const m20 = matrix[offset + 6];
  const m21 = matrix[offset + 7];
  const m22 = matrix[offset + 8];
  const trace = m00 + m11 + m22;
  let w; let x; let y; let z;
  if (trace > 0) {
    const s = Math.sqrt(trace + 1) * 2;
    w = 0.25 * s; x = (m21 - m12) / s; y = (m02 - m20) / s; z = (m10 - m01) / s;
  } else if (m00 > m11 && m00 > m22) {
    const s = Math.sqrt(1 + m00 - m11 - m22) * 2;
    w = (m21 - m12) / s; x = 0.25 * s; y = (m01 + m10) / s; z = (m02 + m20) / s;
  } else if (m11 > m22) {
    const s = Math.sqrt(1 + m11 - m00 - m22) * 2;
    w = (m02 - m20) / s; x = (m01 + m10) / s; y = 0.25 * s; z = (m12 + m21) / s;
  } else {
    const s = Math.sqrt(1 + m22 - m00 - m11) * 2;
    w = (m10 - m01) / s; x = (m02 + m20) / s; y = (m12 + m21) / s; z = 0.25 * s;
  }
  return [w, x, y, z];
}

function solveLinear3(matrix, vector) {
  const augmented = [
    [matrix[0], matrix[1], matrix[2], vector[0]],
    [matrix[3], matrix[4], matrix[5], vector[1]],
    [matrix[6], matrix[7], matrix[8], vector[2]]
  ];
  for (let column = 0; column < 3; column++) {
    let pivot = column;
    for (let row = column + 1; row < 3; row++) {
      if (Math.abs(augmented[row][column]) > Math.abs(augmented[pivot][column])) pivot = row;
    }
    [augmented[column], augmented[pivot]] = [augmented[pivot], augmented[column]];
    const divisor = augmented[column][column];
    if (Math.abs(divisor) < 1e-12) return [0, 0, 0];
    for (let item = column; item < 4; item++) augmented[column][item] /= divisor;
    for (let row = 0; row < 3; row++) {
      if (row === column) continue;
      const factor = augmented[row][column];
      for (let item = column; item < 4; item++) augmented[row][item] -= factor * augmented[column][item];
    }
  }
  return augmented.map((row) => row[3]);
}

export function computeRangerPiperChassisCommand(keyStates) {
  if (keyStates.Space || keyStates.KeyX) return emptyCommand();
  const spinDirection = Number(Boolean(keyStates.KeyJ)) - Number(Boolean(keyStates.KeyL));
  if (spinDirection !== 0) return spinCommand(spinDirection);
  if (keyStates.KeyQ) return crabCommand(DRIVE_SPEED, CRAB_ANGLE);
  if (keyStates.KeyE) return crabCommand(DRIVE_SPEED, -CRAB_ANGLE);
  if (keyStates.KeyZ) return crabCommand(-DRIVE_SPEED, -CRAB_ANGLE);
  if (keyStates.KeyC) return crabCommand(-DRIVE_SPEED, CRAB_ANGLE);
  const driveDirection = Number(Boolean(keyStates.KeyW)) - Number(Boolean(keyStates.KeyS));
  const steerDirection = Number(Boolean(keyStates.KeyA)) - Number(Boolean(keyStates.KeyD));
  return ackermannCommand(DRIVE_SPEED * driveDirection, ACKERMANN_ANGLE * steerDirection);
}

export class RangerPiperController extends BaseController {
  constructor() {
    super();
    this.actuators = null;
    this.joints = null;
    this.targets = Float64Array.from([...PIPER_STOW, 0]);
    this.mujoco = null;
    this.resetHeld = false;
    this.bodies = null;
    this.sites = null;
    this.externalControl = false;
    this.estopped = false;
    this.lastTwistTimeMs = -Infinity;
    this.twist = { linearX: 0, linearY: 0, angularZ: 0 };
    this.pickTask = null;
    this.graspAttachment = null;
    this.pickStatus = { sequence: 0, status: 'idle', object: '', message: '' };
    this.initialQpos = null;
    this.initialQvel = null;
  }

  async initialize(model, data, mujoco) {
    const actuators = decodeNames(model, model.nu, model.name_actuatoradr);
    const joints = decodeNames(model, model.njnt, model.name_jntadr);
    const bodies = decodeNames(model, model.nbody, model.name_bodyadr);
    const sites = decodeNames(model, model.nsite, model.name_siteadr);
    const requiredActuators = [
      ...Object.values(WHEEL_ACTUATORS), ...Object.values(STEERING_ACTUATORS),
      ...PIPER_ACTUATORS, PIPER_GRIPPER
    ];
    const requiredJoints = [...PIPER_JOINTS.map((name) => `piper_${name}`), PIPER_GRIPPER_JOINT];
    for (const name of requiredActuators) {
      if (!actuators.has(name)) throw new Error(`Ranger Piper actuator not found: ${name}`);
    }
    for (const name of requiredJoints) {
      if (!joints.has(name)) throw new Error(`Ranger Piper joint not found: ${name}`);
    }
    this.actuators = actuators;
    this.joints = joints;
    this.bodies = bodies;
    this.sites = sites;
    this.mujoco = mujoco;
    this.initialQpos = Float64Array.from(data.qpos);
    this.initialQvel = Float64Array.from(data.qvel);
    this.reset(model, data);
    this.initialized = true;
  }

  reset(model, data) {
    if (!this.actuators || !this.joints) return;
    if (this.initialQpos?.length === data.qpos.length) data.qpos.set(this.initialQpos);
    if (this.initialQvel?.length === data.qvel.length) data.qvel.set(this.initialQvel);
    data.ctrl.fill(0);
    for (const name of [...Object.values(WHEEL_ACTUATORS), ...Object.values(STEERING_ACTUATORS)]) {
      data.ctrl[this.actuators.get(name)] = 0;
    }
    this.targets.set([...PIPER_STOW, 0]);
    this.estopped = false;
    this.lastTwistTimeMs = -Infinity;
    Object.assign(this.twist, { linearX: 0, linearY: 0, angularZ: 0 });
    this.pickTask = null;
    this.graspAttachment = null;
    this._setPickStatus('idle', '', 'Arm reset');
    const jointNames = [...PIPER_JOINTS.map((name) => `piper_${name}`), PIPER_GRIPPER_JOINT];
    for (let index = 0; index < jointNames.length; index++) {
      const jointId = this.joints.get(jointNames[index]);
      data.qpos[model.jnt_qposadr[jointId]] = this.targets[index];
      data.qvel[model.jnt_dofadr[jointId]] = 0;
    }
    this._applyPiperTargets(data);
    if (this.mujoco) this.mujoco.mj_forward(model, data);
  }

  _applyPiperTargets(data) {
    for (let index = 0; index < PIPER_ACTUATORS.length; index++) {
      data.ctrl[this.actuators.get(PIPER_ACTUATORS[index])] = this.targets[index];
    }
    data.ctrl[this.actuators.get(PIPER_GRIPPER)] = this.targets[6];
  }

  async step(keyStates, model, data, mujoco) {
    if (!this.initialized) await this.initialize(model, data, mujoco);
    if (keyStates.KeyX) {
      if (!this.resetHeld) this.reset(model, data);
      this.resetHeld = true;
    } else {
      this.resetHeld = false;
    }

    let command;
    if (this.externalControl) {
      const fresh = performance.now() - this.lastTwistTimeMs <= COMMAND_WATCHDOG_MS;
      command = fresh && !this.estopped
        ? swerveCommand(this.twist.linearX, this.twist.linearY, this.twist.angularZ)
        : emptyCommand();
    } else {
      command = computeRangerPiperChassisCommand(keyStates);
    }
    for (const wheel of WHEELS) {
      data.ctrl[this.actuators.get(STEERING_ACTUATORS[wheel])] = command.steeringAngles[wheel];
      const jointId = this.joints.get(STEERING_JOINTS[wheel]);
      const currentAngle = data.qpos[model.jnt_qposadr[jointId]];
      const steeringError = Math.abs(normalizeAngle(command.steeringAngles[wheel] - currentAngle));
      // Do not drive sideways while the steering actuator is still rotating
      // toward a substantially different target. This prevents a direction
      // change from sweeping the chassis into nearby furniture.
      const alignmentScale = clamp((0.35 - steeringError) / 0.2, 0, 1);
      data.ctrl[this.actuators.get(WHEEL_ACTUATORS[wheel])] =
        command.wheelVelocities[wheel] * alignmentScale;
    }

    if (!this.externalControl) {
      const dt = Number(model.opt.timestep) || 0.002;
      for (let index = 0; index < PIPER_KEYS.length; index++) {
        const [increase, decrease] = PIPER_KEYS[index];
        const direction = Number(Boolean(keyStates[increase])) - Number(Boolean(keyStates[decrease]));
        this.targets[index] = clamp(
          this.targets[index] + direction * PIPER_JOINT_RATE * dt,
          PIPER_LIMITS[index][0], PIPER_LIMITS[index][1]
        );
      }
      const gripperDirection = Number(Boolean(keyStates.KeyV)) - Number(Boolean(keyStates.KeyB));
      this.targets[6] = clamp(this.targets[6] + gripperDirection * PIPER_GRIPPER_RATE * dt, 0, 0.035);
    }
    if (this.graspAttachment) this._updateGraspAttachment(model, data);
    if (this.pickTask && !this.estopped) this._stepPickTask(model, data);
    this._applyPiperTargets(data);
  }

  setExternalControlEnabled(enabled) {
    this.externalControl = Boolean(enabled);
    if (!this.externalControl) this.lastTwistTimeMs = -Infinity;
    return this.externalControl;
  }

  setTwist(command = {}) {
    this.twist.linearX = clamp(Number(command.linearX) || 0, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
    this.twist.linearY = clamp(Number(command.linearY) || 0, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
    this.twist.angularZ = clamp(Number(command.angularZ) || 0, -MAX_YAW_RATE, MAX_YAW_RATE);
    this.lastTwistTimeMs = performance.now();
    this.estopped = false;
    return true;
  }

  setArmJointCommand(command = {}, model, data) {
    const names = Array.isArray(command.names) ? command.names : [];
    const positions = Array.isArray(command.positions) ? command.positions : [];
    const requestsRelease = names.some((name, index) =>
      (name === 'gripper' || name === PIPER_GRIPPER_JOINT) && Number(positions[index]) > 0.05
    );
    if (requestsRelease && this.graspAttachment) {
      return this._startPutDown(model, data);
    }
    this.pickTask = null;
    for (let index = 0; index < Math.min(names.length, positions.length); index++) {
      const rawName = String(names[index]);
      const value = Number(positions[index]);
      if (!Number.isFinite(value)) continue;
      const jointIndex = PIPER_JOINTS.findIndex((name) => rawName === name || rawName === `piper_${name}`);
      if (jointIndex >= 0) {
        this.targets[jointIndex] = clamp(value, PIPER_LIMITS[jointIndex][0], PIPER_LIMITS[jointIndex][1]);
      } else if (rawName === 'gripper' || rawName === PIPER_GRIPPER_JOINT) {
        this.targets[6] = clamp(value * 0.5, 0, 0.035);
      }
    }
    return true;
  }

  setArmPoseCommand(command = {}, model, data) {
    const requested = Array.isArray(command.position) ? command.position.map(Number) : [];
    if (requested.length !== 3 || !requested.every(Number.isFinite)) return false;
    const position = command.frame === 'arm_base_link'
      ? this._armLocalToWorld(requested, data)
      : requested;
    this.pickTask = null;
    return this._solvePositionIk(position, model, data);
  }

  startPickObject(rawName, model, data) {
    const requested = String(rawName ?? '').trim().toLowerCase();
    const aliases = {
      jar: 'task_glass_jar', 'glass jar': 'task_glass_jar', '玻璃罐': 'task_glass_jar', '罐子': 'task_glass_jar',
      book: 'task_hardcover_book', 'hardcover book': 'task_hardcover_book', '精装书': 'task_hardcover_book', '书本': 'task_hardcover_book',
      succulent: 'task_succulent', plant: 'task_succulent', '盆栽': 'task_succulent', '多肉植物': 'task_succulent',
      glass: 'task_water_glass', 'water glass': 'task_water_glass', '水杯': 'task_water_glass', '玻璃杯': 'task_water_glass',
      remote: 'task_tv_remote', 'tv remote': 'task_tv_remote', '遥控器': 'task_tv_remote', '电视遥控器': 'task_tv_remote'
    };
    const objectName = aliases[requested] ?? requested;
    const bodyId = this.bodies?.get(objectName);
    if (bodyId === undefined) {
      this._setPickStatus('failed', objectName, `Object is not available in this environment: ${objectName}`);
      return false;
    }
    const position = Array.from(data.xpos.subarray(bodyId * 3, bodyId * 3 + 3), Number);
    this.targets[6] = 0.035;
    this.pickTask = {
      objectName, bodyId, phase: 'approach', phaseStart: Number(data.time),
      target: position, initialZ: position[2], retries: 0
    };
    this._setPickStatus('running', objectName, 'Approaching object');
    const accepted = this._solvePositionIk(
      [position[0], position[1], position[2] + 0.18], model, data,
      [PIPER_GRASP_SEED]
    );
    if (!accepted) {
      this.pickTask = null;
      this._setPickStatus('failed', objectName, 'Object is outside the Piper workspace');
    }
    return accepted;
  }

  _stowArm() {
    for (let index = 0; index < PIPER_STOW.length; index++) this.targets[index] = PIPER_STOW[index];
    this.targets[6] = 0;
  }

  _startPutDown(model, data) {
    if (!model || !data || !this.graspAttachment || this.pickTask) return false;
    const attachment = this.graspAttachment;
    const bodyId = this.bodies?.get(attachment.objectName);
    if (bodyId === undefined) return false;
    const tcp = Array.from(data.site_xpos.subarray(attachment.siteId * 3, attachment.siteId * 3 + 3), Number);
    const object = Array.from(data.xpos.subarray(bodyId * 3, bodyId * 3 + 3), Number);
    const targetTcp = [tcp[0], tcp[1], tcp[2] + attachment.supportZ - object[2]];
    this.pickTask = {
      mode: 'place', objectName: attachment.objectName, bodyId,
      phase: 'place-descend', phaseStart: Number(data.time), supportZ: attachment.supportZ
    };
    this._setPickStatus('running', attachment.objectName, 'Lowering object to its support surface');
    if (!this._solvePositionIk(targetTcp, model, data)) {
      this.pickTask = null;
      this._setPickStatus('failed', attachment.objectName, 'Unable to plan a safe placement descent');
      return false;
    }
    return true;
  }

  _setPickStatus(status, objectName, message) {
    this.pickStatus = {
      sequence: (this.pickStatus?.sequence ?? 0) + 1,
      status, object: objectName, message
    };
  }

  getPickStatus() {
    return { ...this.pickStatus };
  }

  _armLocalToWorld(position, data) {
    const bodyId = this.bodies.get('piper_base_link');
    if (bodyId === undefined) return position;
    const matrixOffset = bodyId * 9;
    const originOffset = bodyId * 3;
    const matrix = data.xmat;
    return [
      data.xpos[originOffset] + matrix[matrixOffset] * position[0] + matrix[matrixOffset + 1] * position[1] + matrix[matrixOffset + 2] * position[2],
      data.xpos[originOffset + 1] + matrix[matrixOffset + 3] * position[0] + matrix[matrixOffset + 4] * position[1] + matrix[matrixOffset + 5] * position[2],
      data.xpos[originOffset + 2] + matrix[matrixOffset + 6] * position[0] + matrix[matrixOffset + 7] * position[1] + matrix[matrixOffset + 8] * position[2]
    ];
  }

  _worldToArmLocal(position, data) {
    const bodyId = this.bodies.get('piper_base_link');
    if (bodyId === undefined) return position;
    const matrixOffset = bodyId * 9;
    const originOffset = bodyId * 3;
    const delta = [
      position[0] - data.xpos[originOffset],
      position[1] - data.xpos[originOffset + 1],
      position[2] - data.xpos[originOffset + 2]
    ];
    const matrix = data.xmat;
    return [
      matrix[matrixOffset] * delta[0] + matrix[matrixOffset + 3] * delta[1] + matrix[matrixOffset + 6] * delta[2],
      matrix[matrixOffset + 1] * delta[0] + matrix[matrixOffset + 4] * delta[1] + matrix[matrixOffset + 7] * delta[2],
      matrix[matrixOffset + 2] * delta[0] + matrix[matrixOffset + 5] * delta[1] + matrix[matrixOffset + 8] * delta[2]
    ];
  }

  _worldMatrixToArmQuaternion(worldMatrix, worldOffset, data) {
    const bodyId = this.bodies.get('piper_base_link');
    if (bodyId === undefined) return matrixToQuaternion(worldMatrix, worldOffset);
    const baseOffset = bodyId * 9;
    const local = new Float64Array(9);
    for (let row = 0; row < 3; row++) {
      for (let column = 0; column < 3; column++) {
        for (let axis = 0; axis < 3; axis++) {
          local[row * 3 + column] += data.xmat[baseOffset + axis * 3 + row] *
            worldMatrix[worldOffset + axis * 3 + column];
        }
      }
    }
    return matrixToQuaternion(local, 0);
  }

  _solvePositionIk(target, model, data, fallbackSeeds = []) {
    const siteId = this.sites.get('piper_tcp_site');
    if (siteId === undefined) return false;
    const jointIds = PIPER_JOINTS.map((name) => this.joints.get(`piper_${name}`));
    const addresses = jointIds.map((id) => model.jnt_qposadr[id]);
    const saved = addresses.map((address) => data.qpos[address]);
    const readSite = () => Array.from(data.site_xpos.subarray(siteId * 3, siteId * 3 + 3), Number);
    const seeds = [Array.from(this.targets.slice(0, 6)), ...fallbackSeeds];
    let bestCandidate = null;
    let bestError = Infinity;
    for (const seed of seeds) {
      const candidate = Array.from(seed);
      let errorNorm = Infinity;
      for (let iteration = 0; iteration < 56; iteration++) {
        for (let joint = 0; joint < 6; joint++) data.qpos[addresses[joint]] = candidate[joint];
        this.mujoco.mj_forward(model, data);
        const current = readSite();
        const error = target.map((value, index) => value - current[index]);
        errorNorm = Math.hypot(...error);
        if (errorNorm < 0.004) break;
        const jacobian = Array.from({ length: 3 }, () => new Array(6).fill(0));
        for (let joint = 0; joint < 6; joint++) {
          data.qpos[addresses[joint]] = candidate[joint] + IK_EPSILON;
          this.mujoco.mj_forward(model, data);
          const shifted = readSite();
          for (let axis = 0; axis < 3; axis++) jacobian[axis][joint] = (shifted[axis] - current[axis]) / IK_EPSILON;
          data.qpos[addresses[joint]] = candidate[joint];
        }
        const normal = new Array(9).fill(0);
        for (let row = 0; row < 3; row++) {
          for (let column = 0; column < 3; column++) {
            for (let joint = 0; joint < 6; joint++) normal[row * 3 + column] += jacobian[row][joint] * jacobian[column][joint];
            if (row === column) normal[row * 3 + column] += IK_DAMPING;
          }
        }
        const weightedError = solveLinear3(normal, error);
        for (let joint = 0; joint < 6; joint++) {
          let delta = 0;
          for (let axis = 0; axis < 3; axis++) delta += jacobian[axis][joint] * weightedError[axis];
          candidate[joint] = clamp(candidate[joint] + clamp(delta, -0.18, 0.18), PIPER_LIMITS[joint][0], PIPER_LIMITS[joint][1]);
        }
      }
      if (errorNorm < bestError) {
        bestError = errorNorm;
        bestCandidate = candidate;
      }
    }
    for (let joint = 0; joint < 6; joint++) data.qpos[addresses[joint]] = saved[joint];
    this.mujoco.mj_forward(model, data);
    if (bestError > 0.055 || !bestCandidate) return false;
    for (let joint = 0; joint < 6; joint++) this.targets[joint] = bestCandidate[joint];
    return true;
  }

  _armAtTarget(model, data, tolerance = 0.035) {
    for (let index = 0; index < 6; index++) {
      const jointId = this.joints.get(`piper_${PIPER_JOINTS[index]}`);
      if (Math.abs(data.qpos[model.jnt_qposadr[jointId]] - this.targets[index]) > tolerance) return false;
    }
    return true;
  }

  _attachGraspedObject(task, model, data) {
    const siteId = this.sites.get('piper_tcp_site');
    const jointId = this.joints.get(`${task.objectName}_freejoint`);
    if (siteId === undefined || jointId === undefined) return false;
    const tcp = Array.from(data.site_xpos.subarray(siteId * 3, siteId * 3 + 3), Number);
    const object = Array.from(data.xpos.subarray(task.bodyId * 3, task.bodyId * 3 + 3), Number);
    if (Math.hypot(tcp[0] - object[0], tcp[1] - object[1], tcp[2] - object[2]) > 0.13) return false;
    this.graspAttachment = {
      objectName: task.objectName,
      jointId,
      siteId,
      supportZ: task.initialZ,
      worldOffset: object.map((value, index) => value - tcp[index])
    };
    this._updateGraspAttachment(model, data);
    return true;
  }

  _updateGraspAttachment(model, data) {
    const attachment = this.graspAttachment;
    if (!attachment) return;
    const qposAddress = model.jnt_qposadr[attachment.jointId];
    const dofAddress = model.jnt_dofadr[attachment.jointId];
    const tcpOffset = attachment.siteId * 3;
    for (let axis = 0; axis < 3; axis++) {
      data.qpos[qposAddress + axis] = data.site_xpos[tcpOffset + axis] + attachment.worldOffset[axis];
    }
    for (let axis = 0; axis < 6; axis++) data.qvel[dofAddress + axis] = 0;
  }

  _stepPickTask(model, data) {
    const task = this.pickTask;
    if (task.mode === 'place') {
      this._stepPlaceTask(task, model, data);
      return;
    }
    const now = Number(data.time);
    const objectPosition = Array.from(data.xpos.subarray(task.bodyId * 3, task.bodyId * 3 + 3), Number);
    if (task.phase === 'approach' && this._armAtTarget(model, data)) {
      task.phase = 'descend'; task.phaseStart = now;
      this._setPickStatus('running', task.objectName, 'Descending to grasp');
      if (!this._solvePositionIk([objectPosition[0], objectPosition[1], objectPosition[2] + 0.075], model, data)) {
        this._setPickStatus('failed', task.objectName, 'Object is outside the Piper workspace');
        this.pickTask = null;
      }
    } else if (task.phase === 'descend' && this._armAtTarget(model, data)) {
      task.phase = 'close'; task.phaseStart = now; this.targets[6] = 0;
      this._setPickStatus('running', task.objectName, 'Closing gripper');
    } else if (task.phase === 'close' && now - task.phaseStart > 1.2) {
      if (!this._attachGraspedObject(task, model, data)) {
        if (task.retries < 1) {
          task.retries += 1;
          task.phase = 'approach';
          task.phaseStart = now;
          this.targets[6] = 0.035;
          this._setPickStatus('running', task.objectName, 'Contact missed; retrying grasp');
          const retryAccepted = this._solvePositionIk(
            [objectPosition[0], objectPosition[1], objectPosition[2] + 0.18],
            model, data, [PIPER_GRASP_SEED]
          );
          if (!retryAccepted) {
            this._setPickStatus('failed', task.objectName, 'Unable to plan a safe grasp retry');
            this.pickTask = null;
          }
        } else {
          this._setPickStatus('failed', task.objectName, 'Gripper closed without a valid object contact after retry');
          this.pickTask = null;
        }
        return;
      }
      task.phase = 'lift'; task.phaseStart = now;
      this._setPickStatus('running', task.objectName, 'Lifting object');
      this._solvePositionIk([objectPosition[0], objectPosition[1], objectPosition[2] + 0.22], model, data);
    } else if (task.phase === 'lift' && this._armAtTarget(model, data)) {
      const lifted = objectPosition[2] > task.initialZ + 0.06;
      if (!lifted || this.graspAttachment?.objectName !== task.objectName) {
        this._setPickStatus('failed', task.objectName, 'Gripper closed but object was not securely lifted');
        this.pickTask = null;
        return;
      }
      task.phase = 'hold';
      task.phaseStart = now;
      task.holdReference = objectPosition;
      task.maxHoldDrift = 0;
      this._setPickStatus('running', task.objectName, 'Verifying sustained grasp stability');
    } else if (task.phase === 'hold') {
      const attached = this.graspAttachment?.objectName === task.objectName;
      const drift = Math.hypot(...objectPosition.map((value, index) => value - task.holdReference[index]));
      task.maxHoldDrift = Math.max(task.maxHoldDrift, drift);
      if (!attached || objectPosition[2] <= task.initialZ + 0.06 || task.maxHoldDrift > 0.025) {
        this._setPickStatus('failed', task.objectName, 'Object did not remain stable during grasp verification');
        this.pickTask = null;
      } else if (now - task.phaseStart >= GRASP_HOLD_VERIFY_S) {
        this._setPickStatus(
          'succeeded', task.objectName,
          `Object remained securely held for ${GRASP_HOLD_VERIFY_S.toFixed(1)} seconds`
        );
        this.pickTask = null;
      }
    } else if (now - task.phaseStart > 12) {
      this._setPickStatus('failed', task.objectName, `Timed out during ${task.phase}`);
      this.pickTask = null;
    }
  }

  _stepPlaceTask(task, model, data) {
    const now = Number(data.time);
    const objectPosition = Array.from(data.xpos.subarray(task.bodyId * 3, task.bodyId * 3 + 3), Number);
    if (task.phase === 'place-descend' && this._armAtTarget(model, data)) {
      this.targets[6] = 0.035;
      this.graspAttachment = null;
      task.phase = 'place-settle';
      task.phaseStart = now;
      this._setPickStatus('running', task.objectName, 'Object released on support surface; verifying stability');
    } else if (task.phase === 'place-settle' && now - task.phaseStart >= PLACE_SETTLE_S) {
      const jointId = this.joints.get(`${task.objectName}_freejoint`);
      const verticalSpeed = jointId === undefined ? Infinity : Math.abs(data.qvel[model.jnt_dofadr[jointId] + 2]);
      const stable = Math.abs(objectPosition[2] - task.supportZ) <= 0.04 && verticalSpeed <= 0.08;
      if (!stable) {
        if (now - task.phaseStart <= 4.0) return;
        this._setPickStatus('failed', task.objectName, 'Object did not settle safely after release');
        this._stowArm();
        this.pickTask = null;
        return;
      }
      const siteId = this.sites.get('piper_tcp_site');
      const tcp = Array.from(data.site_xpos.subarray(siteId * 3, siteId * 3 + 3), Number);
      task.phase = 'place-retreat';
      task.phaseStart = now;
      this._setPickStatus('running', task.objectName, 'Object is stable; retreating gripper');
      if (!this._solvePositionIk([tcp[0], tcp[1], tcp[2] + 0.16], model, data)) {
        this._stowArm();
        task.phase = 'place-stow';
        task.phaseStart = now;
      }
    } else if (task.phase === 'place-retreat' && this._armAtTarget(model, data)) {
      this._stowArm();
      task.phase = 'place-stow';
      task.phaseStart = now;
      this._setPickStatus('running', task.objectName, 'Returning arm to transport-safe pose');
    } else if (task.phase === 'place-stow' && this._armAtTarget(model, data)) {
      this._setPickStatus('succeeded', task.objectName, 'Object placed stably and arm returned to stow pose');
      this.pickTask = null;
    } else if (now - task.phaseStart > 12) {
      this._setPickStatus('failed', task.objectName, `Timed out during ${task.phase}`);
      this.pickTask = null;
    }
  }

  emergencyStop() {
    this.estopped = true;
    this.lastTwistTimeMs = -Infinity;
    if (this.pickTask) this._setPickStatus('failed', this.pickTask.objectName, 'Emergency stop');
    this.pickTask = null;
    this.graspAttachment = null;
    return true;
  }

  getRobotState(model, data) {
    if (!this.initialized || !model || !data) return null;
    const baseJointId = this.joints.get('ranger_base_freejoint');
    const baseQposAddress = model.jnt_qposadr[baseJointId];
    const baseDofAddress = model.jnt_dofadr[baseJointId];
    const armNames = PIPER_JOINTS.map((name) => `piper_${name}`);
    const armPositions = armNames.map((name) => data.qpos[model.jnt_qposadr[this.joints.get(name)]]);
    const armVelocities = armNames.map((name) => data.qvel[model.jnt_dofadr[this.joints.get(name)]]);
    const gripperJointId = this.joints.get(PIPER_GRIPPER_JOINT);
    const tcpSiteId = this.sites.get('piper_tcp_site');
    const tcpOffset = tcpSiteId * 3;
    const objectStates = [];
    for (const [name, bodyId] of this.bodies.entries()) {
      if (!name.startsWith('task_') || name === 'task_station') continue;
      objectStates.push({
        name,
        position: Array.from(data.xpos.subarray(bodyId * 3, bodyId * 3 + 3), Number),
        quaternion: Array.from(data.xquat.subarray(bodyId * 4, bodyId * 4 + 4), Number)
      });
    }
    return {
      timestamp: Number(data.time),
      externalControl: this.externalControl,
      estopped: this.estopped,
      base: {
        position: Array.from(data.qpos.subarray(baseQposAddress, baseQposAddress + 3), Number),
        quaternion: Array.from(data.qpos.subarray(baseQposAddress + 3, baseQposAddress + 7), Number),
        linearVelocity: Array.from(data.qvel.subarray(baseDofAddress, baseDofAddress + 3), Number),
        angularVelocity: Array.from(data.qvel.subarray(baseDofAddress + 3, baseDofAddress + 6), Number)
      },
      arm: {
        names: [...PIPER_JOINTS, 'gripper'],
        positions: [...armPositions, 2 * data.qpos[model.jnt_qposadr[gripperJointId]]],
        velocities: [...armVelocities, 2 * data.qvel[model.jnt_dofadr[gripperJointId]]],
        targets: [...this.targets.slice(0, 6), 2 * this.targets[6]],
        endPose: tcpSiteId === undefined ? null : {
          position: this._worldToArmLocal(
            Array.from(data.site_xpos.subarray(tcpOffset, tcpOffset + 3), Number), data),
          quaternion: this._worldMatrixToArmQuaternion(data.site_xmat, tcpSiteId * 9, data)
        }
      },
      objects: objectStates
    };
  }

  getControlKeys() {
    return [
      'KeyW', 'KeyS', 'KeyA', 'KeyD', 'KeyQ', 'KeyE', 'KeyZ', 'KeyC',
      'KeyJ', 'KeyL', 'Space', 'KeyX',
      ...PIPER_KEYS.flat(), 'KeyV', 'KeyB'
    ];
  }

  getDescription() {
    return [
      'Base: W/S drive | A/D steer',
      'Crab: Q/E/Z/C | Spin: J/L',
      'Arm 1-3: 1/Y 2/U 3/I',
      'Arm 4-6: 4/O 5/P 6/[',
      'Grip: V/B | Stop: Space | Reset: X'
    ].join('\n');
  }
}
