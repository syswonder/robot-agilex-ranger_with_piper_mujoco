---
description: Control the simulated six-axis Piper arm and parallel gripper.
---

# Piper arm

Joint commands use `sensor_msgs/JointState`: names `joint1` through `joint6`
are radians and `gripper` is total jaw opening in metres from 0.0 to 0.07.
Cartesian poses use `arm_base_link`. The active simulator controller enforces joint
limits and reports measured state through `joint_states` and `end_pose`.

The primitive exposes low-level commands; the `pick` skill owns the tested
closed-loop grasp sequence. Static room furniture is not manipulable.
