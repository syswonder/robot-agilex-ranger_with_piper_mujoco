---
description: Control the simulated Ranger base and expose its local odometry.
---

# Ranger chassis

`move` executes one bounded motion burst and publishes a zero velocity when it
finishes. `twist_in` is the continuous velocity input used by Nav2. Commands are
expressed in `base_link`; odometry is reported in `odom`.

The simulator and bridge must be ready before this provider is activated. On
shutdown it sends a zero Twist. `move` is intended for short direct motions;
use the navigation service for collision-aware travel through a room.
