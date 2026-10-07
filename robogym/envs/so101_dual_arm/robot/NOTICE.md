# Sources

The arm is The Robot Studio's open-source [SO-101](https://github.com/TheRobotStudio/SO-ARM100). `so101.xml` is adapted from [MuJoCo Menagerie `robotstudio_so101`](https://github.com/google-deepmind/mujoco_menagerie/tree/822c2d8f877dd166c5b7d3c9f7e3c3b6589473b7/robotstudio_so101) at revision `822c2d8`, which Menagerie derived from The Robot Studio's [SO101 MJCF](https://github.com/TheRobotStudio/SO-ARM100/blob/608122e9ac330a753735f2e18aee73338e9ac407/Simulation/SO101/so101_new_calib.xml). Both are licensed under Apache-2.0; [LICENSE](LICENSE) is a copy of that license. Changes from Menagerie:

- Printed parts are white instead of yellow.
- The fixed jaw is drawn with the SO-ARM100 wrist camera mount, replacing Menagerie's fixed-jaw and camera-mount meshes. It holds a 10 g U20CAM camera (`u20cam.xml`) with box collision envelopes.
- The gripper link is 0.0939 kg (Menagerie: 0.087 kg), with its own inertia, to suit this mount.
- The `wrist_roll` control range is ±2.74385 rad, matching its joint range (Menagerie: −2.74385 to 2.84121, past the joint limit).
- The base and shoulder have collision boxes fitted to their meshes, kept from colliding with each other by a `<contact><exclude>`.
- Gripper contacts use `solref="0.006 1"` and `solimp="0.95 0.99 0.001"` (Menagerie: `0.01 1` and the default `solimp`).
- Menagerie's `<option>`, `<visual>`, comments, and unused defaults and meshes are dropped.

`so101_dual_arm.xml` (the rig) and `u20cam.xml` are authored for this project.

| Meshes in `assets/` | Source | Changes |
| --- | --- | --- |
| `arm_base`, `cam_mount_bottom`, `cam_mount_middle`, `cam_mount_top` | SO-ARM100 [overhead camera mount](https://github.com/TheRobotStudio/SO-ARM100/tree/7629d2ad9853d10fb903093a33ef6114099d97e5/Optional/Overhead_Cam_Mount_32x32_UVC_Module/stl), revision `7629d2a` | none |
| `wrist_cam_mount_32x32_uvc_module_so101` | SO-ARM100 [wrist camera mount](https://github.com/TheRobotStudio/SO-ARM100/blob/7629d2ad9853d10fb903093a33ef6114099d97e5/Optional/Wrist_Cam_Mount_32x32_UVC_Module/stl/Wrist_Cam_Mount_32x32_UVC_Module_SO101.stl), revision `7629d2a` | converted from ASCII to binary STL; duplicate triangles removed |
| `u20cam_1080p` | Modeled for this project from the dimensions and photos of the [InnoMaker U20CAM-1080P](https://github.com/INNO-MAKER/U20CAM-1080P-S1) | — |
| The rest | [Menagerie `robotstudio_so101/assets`](https://github.com/google-deepmind/mujoco_menagerie/tree/822c2d8f877dd166c5b7d3c9f7e3c3b6589473b7/robotstudio_so101/assets), revision `822c2d8` | none |
