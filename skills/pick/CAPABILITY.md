---
description: Directly resolve and physically pick a named MuJoCo environment object with Piper.
---

# Pick and put down

After the base reaches the manipulation pose, call `pick` directly with the
user's object description. Do not call Scene `list_objects` or the front camera
first: environment task objects are intentionally kept in the Pick inventory,
and this skill already reads that inventory plus the rear Piper wrist image.
It restricts selection to the simulator's dynamic objects, confirms ambiguous
names with the configured VLM, starts the browser grasp state machine, and waits
for contact, lift, and three seconds of sustained-hold verification. The operation can take up to
`pick_timeout_s`.

In the default `scenesmith_house_187`, use the navigation pose
`(x=3.92, y=3.25, yaw=0)` before picking `水杯` / `task_water_glass`,
`遥控器` / `task_tv_remote`, or `盆栽` / `task_succulent`. In the legacy
`scenesmith_room_030`, use `(x=3.91, y=0.83, yaw=0)` for its task objects.

`put_down` lowers the object to its original support height, opens the gripper,
verifies that the object has settled, retreats, and stows the arm. The skill supports only task
objects registered by the selected environment; SceneSmith furniture and SPZ
visual content are static. Keep the simulator running and do not reset or switch
environments during a grasp.
