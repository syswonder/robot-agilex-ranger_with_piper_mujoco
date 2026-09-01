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
const PIPER_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'];
const PIPER_ACTUATORS = PIPER_JOINTS.map((name) => `piper_${name}`);
const PIPER_HOME = [0, 1.57, -1.3485, 0, 0, 0];
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
    this.targets = Float64Array.from([...PIPER_HOME, 0]);
    this.mujoco = null;
    this.resetHeld = false;
  }

  async initialize(model, data, mujoco) {
    const actuators = decodeNames(model, model.nu, model.name_actuatoradr);
    const joints = decodeNames(model, model.njnt, model.name_jntadr);
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
    this.mujoco = mujoco;
    this.reset(model, data);
    this.initialized = true;
  }

  reset(model, data) {
    if (!this.actuators || !this.joints) return;
    for (const name of [...Object.values(WHEEL_ACTUATORS), ...Object.values(STEERING_ACTUATORS)]) {
      data.ctrl[this.actuators.get(name)] = 0;
    }
    this.targets.set([...PIPER_HOME, 0]);
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

    const command = computeRangerPiperChassisCommand(keyStates);
    for (const wheel of WHEELS) {
      data.ctrl[this.actuators.get(WHEEL_ACTUATORS[wheel])] = command.wheelVelocities[wheel];
      data.ctrl[this.actuators.get(STEERING_ACTUATORS[wheel])] = command.steeringAngles[wheel];
    }

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
    this._applyPiperTargets(data);
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
