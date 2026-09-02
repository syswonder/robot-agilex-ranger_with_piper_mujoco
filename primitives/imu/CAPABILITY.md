---
description: Stream the simulated MID-360 inertial measurement.
---

# MID-360 IMU

The stream contains MuJoCo-derived angular velocity and linear acceleration in
`mid360_link`. The provider is read-only and becomes ACTIVE only after a valid
sample proves that the browser simulation is running.
