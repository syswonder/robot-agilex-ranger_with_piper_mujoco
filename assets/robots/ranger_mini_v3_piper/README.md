# Ranger Mini V3 + Piper

This package combines the existing Ranger Mini V3 mobile base with the
MuJoCo Menagerie AgileX Piper model. All runtime MJCF and mesh assets are
contained in this directory.

## Deployment layout

The package follows the photographed Robonix Ranger layout: the Piper is on the
rear centerline, while an open four-post equipment rack occupies the front
deck. Transforms use the Ranger base frame where +X is forward and Y=0 is the
longitudinal centerline:

```xml
<body name="piper_mount" pos="-0.20 0 0.025" quat="0 0 0 1">
<body name="sensor_rack" pos="0.18 0 0.025">
```

Replace this `pos` (and add `quat` if required) after measuring the physical
mount. `piper_mount_frame` is provided as a visible calibration site.
The 180-degree yaw points the arm away from the chassis toward the rear.

The Robonix URDF explicitly marks its sensor geometry as CAD placeholders. This
model retains the published sensor frame locations for compatibility:

| Device | Frame position in `ranger_base_link` | Simulation |
| --- | --- | --- |
| Livox MID-360 | `[0.18, 0, 0.425]` | 360 x 59 degree, 1024-ray batched LiDAR at 10 Hz |
| MID-360 ICM40609 | Co-located with LiDAR | Native MuJoCo gyro and accelerometer |
| Front Orbbec Gemini 336L | `[0.28, 0, 0.30]` | 320x240 RGB and 160x120 collision depth |
| Piper wrist Orbbec DC1 | Attached below `piper_link6` | 320x180 RGB |

The front enclosure, compute boxes, camera bodies and sensor shells are visual
approximations of the deployment photo. Navigation frames are explicit MJCF
sites/cameras so measured extrinsics can later replace the placeholders without
changing the runtime API.

## Sensor runtime

This browser application cannot directly use the Python/CUDA backends from
MuJoCo-LiDAR or mjlab. Its self-contained equivalent uses MuJoCo WASM's native
`mj_multiRay` batch call for MID-360 and depth rays, native MJCF sensors for the
IMU, and the existing Three.js renderer for RGB. Open `Simulation > Sensors` to
show LiDAR points or capture either camera.

Select a camera and enable `Live RGBA view` to switch the main canvas to that
camera in real time. This reuses the existing render pass, including 3DGS,
instead of rendering the Gaussian scene a second time. Collision depth remains
available through the capture button so 19,200 depth rays do not run every frame.

The default 1024 rays per scan intentionally downsample the physical sensor's
point rate to keep the single-threaded browser simulation responsive. Change
`sensors.lidar.raysPerScan` and `updateHz` in `robot.json` when a different
accuracy/performance tradeoff is needed.

The same data is exposed through `window.mujocoSensors`:

```javascript
window.mujocoSensors.list();
window.mujocoSensors.scanLidar();
window.mujocoSensors.latestLidar();
window.mujocoSensors.readImu();
window.mujocoSensors.captureCamera('front_rgbd');
window.mujocoSensors.previewCamera('wrist_rgb');
window.mujocoSensors.startLiveCamera('front_rgbd');
window.mujocoSensors.stopLiveCamera();
window.mujocoSensors.showLidar(true);
```

LiDAR ranges and points use MuJoCo axes and SI units. Sensor rays target the
generated environment collision layer, avoiding reflections from the robot's
own visual meshes. RGB frames contain an RGBA byte array; depth frames contain
meters with `Infinity` for no return plus a per-pixel `geomIds` array.
Environment and sensor-visible obstacle collisions use geom group 3; the
robot's own collision shapes use group 4 so arm motion cannot cause self returns.

## Controls

- Ranger: `W/S` drive, `A/D` steer, `Q/E/Z/C` crab, `J/L` spin, `Space` stop.
- Piper joints 1-6: `1/Y`, `2/U`, `3/I`, `4/O`, `5/P`, `6/[`.
- Gripper: `V` open, `B` close.
- `X`: stop the base and restore the Piper home pose.

## Upstream sources

- MuJoCo Menagerie `agilex_piper`, commit
  `da76818e269b82289eba39808e2fb91d679d6994` (MIT). The copied license is in
  `third_party/agilex_piper-MIT.txt`.
- Robonix Ranger deployment, commit
  `336067f195ab978456bf0bdc1b747e4f696931c2`, reviewed for the deployment
  structure, photograph, URDF sensor frames, and hardware configuration.
- `soulde/Piper_mujoco`, commit
  `1efd678cb60656b37cdd5d352f022bb52bd2dab0`, reviewed for its controller and
  IK examples; no runtime files are copied from it.
- MuJoCo-LiDAR was reviewed for its native `mj_multiRay` CPU approach; no
  runtime files are copied from it.
- mjlab RGB-D documentation was reviewed for RGB/depth sensor semantics; its
  MuJoCo Warp implementation is not copied into this browser runtime.
