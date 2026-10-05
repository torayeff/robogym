# SO-101 dual arm (MuJoCo)

Two SO-101 arms with wrist cameras and an overhead camera, on a table with a prism and a storage tray.

```bash
uv run envs/so101_dual_arm/view.py
```

Opens the MuJoCo viewer in the folded rest pose. Drive the 12 position actuators (`{left,right}_{shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper}`, radians) with the Control sliders. Cameras: `overview`, `top_camera`, `left_wrist_camera`, `right_wrist_camera`. The prism and tray have free joints named `prism` and `storage`.

- `camera.py`: renders the robot cameras through their lens.
- `scene.xml`: floor, lights, table, prism, tray, rest keyframe.
- `robot/so101_bimanual.xml`: mounting rig and overhead camera; attaches `so101.xml` twice, prefixed `left_` and `right_`.
- `robot/so101.xml`: one arm with its wrist camera.
- `robot/assets/`: the 21 STL meshes.

## Cameras

`top_camera` and the wrist cameras are InnoMaker U20CAM-1080P-S1 modules at 640 × 480. Their wide-angle lens has strong barrel distortion, which MuJoCo's pinhole cameras cannot draw, so the viewer shows them undistorted, over the same 59.625° vertical field of view. `camera.py` renders them through the lens:

```python
from camera import LensRenderer  # from a script in envs/so101_dual_arm/

with LensRenderer(model) as renderer:
    image = renderer.render(data, "left_wrist_camera")  # (480, 640, 3) uint8
```

The lens values are nominal, from the [user manual](https://github.com/INNO-MAKER/U20CAM-1080P-S1/blob/a08d78bfc7fb6b1230bad0af28192d58b557e077/U20CAM-1080P-S1%20UserManual-v1.1.pdf) (v1.1) rather than a calibration. Its 106° horizontal field of view across the 1920 × 1080 sensor is more than a 2.8 mm pinhole lens covers on a 1/3″ sensor (about 86°), but what an equidistant lens covers: OpenCV's fisheye model with k1–k4 = 0. Taking 640 × 480 to be the 4:3 centre crop, scaled by 480/1080, gives 79.5° × 59.625°. The manual's figures do not all fit one lens: it also gives 103° horizontal, its 130° diagonal is 122° in this model, and its −17% TV distortion has no stated convention. The true vertical field of view could be anywhere from about 54° to 66°. A calibration replaces `U20CAM` in `camera.py` with its K and D, and the cameras' `fovy` with its vertical field of view.

## Sources

The arm is The Robot Studio's open-source [SO-101](https://github.com/TheRobotStudio/SO-ARM100).

`robot/so101.xml` is adapted from [MuJoCo Menagerie `robotstudio_so101`](https://github.com/google-deepmind/mujoco_menagerie/tree/822c2d8f877dd166c5b7d3c9f7e3c3b6589473b7/robotstudio_so101) at revision `822c2d8`, which Menagerie derived from The Robot Studio's [SO101 MJCF](https://github.com/TheRobotStudio/SO-ARM100/blob/608122e9ac330a753735f2e18aee73338e9ac407/Simulation/SO101/so101_new_calib.xml). Unchanged from Menagerie: body frames; joint limits, damping, friction loss and armature; the position actuator gains (kp 998.22, kv 2.731, ±2.94 N·m); the collision geometry from the upper arm to the gripper; and every link inertia except the gripper's. Changes:

- Printed parts are white instead of yellow.
- The fixed jaw is drawn with the SO-ARM100 wrist camera mount, replacing Menagerie's fixed-jaw and camera-mount meshes. It holds a 10 g U20CAM camera (see [Cameras](#cameras)) with box collision envelopes.
- The gripper link is 0.0939 kg (Menagerie: 0.087 kg), with its own inertia, to suit this mount.
- The `wrist_roll` control range is ±2.74385 rad, matching its joint range (Menagerie: −2.74385 to 2.84121, past the joint limit).
- The base and shoulder have collision boxes fitted to their meshes. Menagerie has none on the base, and its two shoulder boxes leave up to 14 mm of the bracket and servo uncovered. The base is welded to the world, which MuJoCo's parent-child filter skips, so a `<contact><exclude>` keeps the base and shoulder boxes apart.
- Gripper contacts use `solref="0.006 1"` and `solimp="0.95 0.99 0.001"` (Menagerie: `0.01 1`, the stiffest its 5 ms timestep allows, and the default `solimp`). The gripper geoms have contact priority, so these values apply to every grasp: at full grip the fingertips sink about 1 mm into the prism instead of 2.5 mm.
- Menagerie's `<option>`, `<visual>`, comments, and unused defaults and meshes are dropped; `scene.xml` sets the simulation and visual options.

| Meshes in `robot/assets/` | Source | Changes |
| --- | --- | --- |
| `arm_base`, `cam_mount_bottom`, `cam_mount_middle`, `cam_mount_top` | SO-ARM100 [overhead camera mount](https://github.com/TheRobotStudio/SO-ARM100/tree/7629d2ad9853d10fb903093a33ef6114099d97e5/Optional/Overhead_Cam_Mount_32x32_UVC_Module/stl), revision `7629d2a` | none |
| `wrist_cam_mount_32x32_uvc_module_so101` | SO-ARM100 [wrist camera mount](https://github.com/TheRobotStudio/SO-ARM100/blob/7629d2ad9853d10fb903093a33ef6114099d97e5/Optional/Wrist_Cam_Mount_32x32_UVC_Module/stl/Wrist_Cam_Mount_32x32_UVC_Module_SO101.stl), revision `7629d2a` | converted from ASCII to binary STL; 1,915 duplicate triangles removed |
| `u20cam_1080p` | Modeled for this project from the dimensions and photos of the [InnoMaker U20CAM-1080P](https://github.com/INNO-MAKER/U20CAM-1080P-S1) | — |
| The other 15 | [Menagerie `robotstudio_so101/assets`](https://github.com/google-deepmind/mujoco_menagerie/tree/822c2d8f877dd166c5b7d3c9f7e3c3b6589473b7/robotstudio_so101/assets), revision `822c2d8` | none |

Menagerie's `robotstudio_so101` and SO-ARM100 are both licensed under Apache-2.0; [robot/LICENSE](robot/LICENSE) is a copy of that license.

The rig (`robot/so101_bimanual.xml`: part placement, collision boxes, overhead camera) and `scene.xml` are authored for this project. The table approximates an IKEA LINNMON/ADILS (100 × 60 cm top, 74 cm high). The prism and tray are box primitives with uniform-density inertias. They sit at x = 0.23 m, about as close to the arms as helps: `wrist_flex` (±95°) keeps a straight-down gripper's fingertips below about 0.83 m, 9 cm above the table, wherever the target is, and from x = 0.23 m that still clears the prism's 0.81 m top.
