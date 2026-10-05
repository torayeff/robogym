# SO-101 dual arm (MuJoCo)

Two SO-101 arms with wrist cameras and an overhead camera, on a table with a prism and a storage tray.

```bash
uv run so101_dual_arm/sim.py
```

Opens the MuJoCo viewer in the folded rest pose. Drive the 12 position actuators (`{left,right}_{shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper}`, radians) with the Control sliders. Cameras: `overview`, `top_camera`, `left_wrist_camera`, `right_wrist_camera`.

- `scene.xml`: floor, lights, table, prism, tray, rest keyframe.
- `robot/so101_bimanual.xml`: mounting rig and overhead camera; attaches `so101.xml` twice, prefixed `left_` and `right_`.
- `robot/so101.xml`: one arm with its wrist camera.
- `robot/assets/`: the 21 STL meshes.

## Sources

The arm is The Robot Studio's open-source [SO-101](https://github.com/TheRobotStudio/SO-ARM100).

`robot/so101.xml` is adapted from [MuJoCo Menagerie `robotstudio_so101`](https://github.com/google-deepmind/mujoco_menagerie/tree/822c2d8f877dd166c5b7d3c9f7e3c3b6589473b7/robotstudio_so101) at revision `822c2d8`, which Menagerie derived from The Robot Studio's [SO101 MJCF](https://github.com/TheRobotStudio/SO-ARM100/blob/608122e9ac330a753735f2e18aee73338e9ac407/Simulation/SO101/so101_new_calib.xml). Unchanged from Menagerie: body frames; joint limits, damping, friction loss and armature; the position actuators (kp 998.22, kv 2.731, ±2.94 N·m); arm and gripper collision geometry; and every link inertia except the gripper's. Changes:

- Printed parts are white instead of yellow.
- The fixed jaw is drawn with the SO-ARM100 wrist camera mount, replacing Menagerie's fixed-jaw and camera-mount meshes. It holds a 10 g U20CAM camera (640 × 480, 73.48° vertical field of view) with box collision envelopes.
- The gripper link is 0.0939 kg (Menagerie: 0.087 kg), with its own inertia, to suit this mount.
- Menagerie's `<option>`, `<visual>`, comments, and unused defaults and meshes are dropped; `scene.xml` sets the simulation options.

| Meshes in `robot/assets/` | Source | Changes |
| --- | --- | --- |
| `arm_base`, `cam_mount_bottom`, `cam_mount_middle`, `cam_mount_top` | SO-ARM100 [overhead camera mount](https://github.com/TheRobotStudio/SO-ARM100/tree/7629d2ad9853d10fb903093a33ef6114099d97e5/Optional/Overhead_Cam_Mount_32x32_UVC_Module/stl), revision `7629d2a` | none |
| `wrist_cam_mount_32x32_uvc_module_so101` | SO-ARM100 [wrist camera mount](https://github.com/TheRobotStudio/SO-ARM100/blob/7629d2ad9853d10fb903093a33ef6114099d97e5/Optional/Wrist_Cam_Mount_32x32_UVC_Module/stl/Wrist_Cam_Mount_32x32_UVC_Module_SO101.stl), revision `7629d2a` | 1,915 duplicate triangles removed |
| `u20cam_1080p` | Modeled for this project from the dimensions and photos of the [InnoMaker U20CAM-1080P](https://github.com/INNO-MAKER/U20CAM-1080P-S1) | — |
| The other 15 | [Menagerie `robotstudio_so101/assets`](https://github.com/google-deepmind/mujoco_menagerie/tree/822c2d8f877dd166c5b7d3c9f7e3c3b6589473b7/robotstudio_so101/assets), revision `822c2d8` | none |

Menagerie's `robotstudio_so101` and SO-ARM100 are both licensed under Apache-2.0; [robot/LICENSE](robot/LICENSE) is a copy of that license.

The rig (`robot/so101_bimanual.xml`: part placement, collision boxes, overhead camera) and `scene.xml` are authored for this project. The table approximates an IKEA LINNMON/ADILS (100 × 60 cm top, 74 cm high). The prism and tray are box primitives with uniform-density inertias.
