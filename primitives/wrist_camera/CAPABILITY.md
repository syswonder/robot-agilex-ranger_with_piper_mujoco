---
description: Expose RGB-D perception from the simulated Piper wrist camera.
---

# Wrist RGB-D camera

The optical frame is rigidly attached to `piper_link6` and moves with the arm.
The pick skill consumes its RGB frames for target confirmation. Snapshot depth
is visualization-only normalized JPEG; use the continuous ROS depth stream for
metric values.

The current package does not export a separate extrinsics capability because
the moving wrist transform is supplied through the simulation TF tree.

The deployment sets `advertise_streams=false`: wrist frames remain available
through this provider's snapshot tools and the Pick skill's private ROS input,
but they do not compete with the front camera for Scene's canonical RGB-D
stream contracts.
