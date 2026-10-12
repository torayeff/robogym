# so101_dual_arm

Two [SO-101](https://github.com/TheRobotStudio/SO-ARM100) robot arms stand side by side at the back of a table, with a red block and a red tray in front of them. Each object starts where only the arm beside it can reach, so getting the block into the tray takes both arms. The scene is simulated in MuJoCo, and the agent sees it through a camera on each wrist and one overhead.

## Observations and actions

Each arm has six servos: `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll` and `gripper`, named with a `left_` or `right_` prefix.

An observation contains the simulation time in seconds, the 12 joint angles in radians, and a 640 × 480 JPEG image, base64-encoded, from each of the three cameras: `top_camera`, `left_wrist_camera` and `right_wrist_camera`. It doesn't include the positions of the block and the tray; the agent has to find them in the images.

An action sets target angles, in radians, for any of the servos, and the servos then move the joints toward them. Servos left out of an action keep their previous targets:

```json
{"joints": {"left_shoulder_lift": -0.75, "left_gripper": 1.2}}
```

`GET /spec` lists each servo's range and which way it turns, and the sizes of the block and the tray.

## Resets

A reset folds both arms, with the grippers just above the table, and places the block beside one arm and the tray beside the other, choosing at random which goes where. Each object is turned to a random angle, and the block either stands on end or lies on its side. Three constants in `physics.py` bound where each object's centre can land:

| Constant | Bound | Purpose |
| --- | --- | --- |
| `NEAR` | Maximum distance from its own arm's shoulder-pan axis | That arm can pick up the block with its gripper tilted no more than 25° from vertical, and reach over the tray with its gripper tilted no more than 40° |
| `FAR` | Minimum distance from the other arm's shoulder-pan axis | The other arm can't grasp the block, even near one end of a lying block, or get its gripper over the tray; at full stretch, its fingertips may just touch the object |
| `FRONT` | Minimum forward position, x | The whole object stays in view of the top camera |

These reach limits come from a search over each arm's joint angles that allowed poses sinking at most 1 mm into the table, the rig or the arm itself. At the height of a block, an arm's grasp point (the `gripperframe` site) reaches at most 0.436 m from its shoulder-pan axis, and above a tray at most 0.441 m. With an object at the `FAR` bound, the other arm's grasp point comes no closer than 19 mm to the graspable part of a lying block that points at it, and 14 mm to the space over a tray turned corner-on.

## Cameras

All three cameras are modeled on the InnoMaker U20CAM-1080P-S1 module. Its wide-angle lens bends straight lines noticeably (barrel distortion), but MuJoCo can only render through an ideal pinhole. So the environment renders a wider pinhole image and warps it through a model of the lens, in `camera.py`. MuJoCo's viewer shows the cameras without the distortion.

The lens model is based on the camera's [user manual](https://github.com/INNO-MAKER/U20CAM-1080P-S1/blob/a08d78bfc7fb6b1230bad0af28192d58b557e077/U20CAM-1080P-S1%20UserManual-v1.1.pdf) (v1.1), not on a calibration. The manual gives a 2.8 mm lens on a 1/3″, 1920 × 1080 sensor with a 106° horizontal field of view. A pinhole lens of that focal length would cover only about 86°, while an equidistant fisheye lens covers 106°, so `camera.py` uses OpenCV's fisheye model with all distortion coefficients at zero. Taking the camera's 640 × 480 mode to be the 4:3 centre of the sensor, scaled down, gives a field of view of 79.5° × 59.6°.

The manual isn't fully consistent: it also gives 103° horizontal, a 130° diagonal (this model has 122°), and a −17% TV distortion without saying which convention it uses. The real vertical field of view could be anywhere from about 54° to 66°. To match a real camera, calibrate it and replace `U20CAM` in `camera.py` with the calibrated K and D.

## Viewing the scene in MuJoCo

To explore the scene without the server, open it in MuJoCo's viewer:

```bash
uv run python -m robogym.envs.so101_dual_arm
```

The viewer shows the scene as a reset leaves it. Reload (Ctrl+L) resets it, and the sliders in the Control panel move the servos.

## Files

- `env.py`: the environment the server serves, with its spec, its action checks, and the rendering of observations and video.
- `physics.py`: the simulation and the random placement on reset.
- `camera.py`: the lens model.
- `__main__.py`: the MuJoCo viewer.
- `scene.xml`: the table, the block, the tray, the lights and the folded rest pose.
- `robot/so101_dual_arm.xml`: the mounting rig and the overhead camera, with both arms attached.
- `robot/so101.xml`: one arm with its wrist camera.
- `robot/u20cam.xml`: the camera module.
- `robot/assets/`: the 21 STL meshes.
- `Dockerfile`, `sandbox.toml`: the image and resources for a Modal sandbox.

`physics.py` builds two models from `scene.xml` in memory, leaving the mesh files unchanged. The simulation process gets a model without the meshes that are only drawn, 7 MB in size. The renderer gets the full scene, in which the two arms and three cameras share one copy of each mesh: 65 MB instead of the 125 MB MuJoCo would load by default.

The Modal sandbox has a T4 GPU, since rendering on the sandbox's CPUs is about ten times slower and takes CPU time from the simulation. A sandbox with a GPU can't give the simulation real-time priority, but with the rendering on the GPU, the simulation still runs on time.

## Sources and license

The arm is The Robot Studio's open-source [SO-101](https://github.com/TheRobotStudio/SO-ARM100). `robot/so101.xml` is adapted from [MuJoCo Menagerie's `robotstudio_so101`](https://github.com/google-deepmind/mujoco_menagerie/tree/822c2d8f877dd166c5b7d3c9f7e3c3b6589473b7/robotstudio_so101) at revision `822c2d8`, which Menagerie derived from The Robot Studio's [SO101 MJCF](https://github.com/TheRobotStudio/SO-ARM100/blob/608122e9ac330a753735f2e18aee73338e9ac407/Simulation/SO101/so101_new_calib.xml). Both are licensed under Apache-2.0, and [robot/LICENSE](robot/LICENSE) is a copy of that license.

Compared with Menagerie, the body frames, the joint limits, damping, friction loss and armature, the actuator gains and force range, the collision geometry from the upper arm to the gripper, and every link inertia except the gripper's are unchanged. The changes are:

- Printed parts are white instead of yellow.
- The fixed jaw is drawn with the SO-ARM100 wrist camera mount, replacing Menagerie's fixed-jaw and camera-mount meshes. It holds a 10 g U20CAM camera (`robot/u20cam.xml`) with box collision envelopes.
- The gripper link is 0.0939 kg (Menagerie: 0.087 kg), with its own inertia, to suit this mount.
- The `wrist_roll` control range is ±2.74385 rad, matching its joint range (Menagerie: −2.74385 to 2.84121, past the joint limit).
- The base and shoulder have collision boxes fitted to their meshes. Menagerie has none on the base, and its two shoulder boxes leave up to 14 mm of the bracket and servo uncovered. The base is welded to the world, which MuJoCo's parent-child filter skips, so a `<contact><exclude>` keeps the base and shoulder boxes apart.
- Gripper contacts use `solref="0.006 1"` and `solimp="0.95 0.99 0.001"` (Menagerie: `0.01 1`, the stiffest its 5 ms timestep allows, and the default `solimp`). The gripper geoms have contact priority, so these values apply to every grasp: at full grip the fingertips sink about 1 mm into the block instead of 2.5 mm.
- Menagerie's `<option>`, `<visual>`, comments, and unused defaults and meshes are dropped; `scene.xml` sets the simulation and visual options.

`robot/so101_dual_arm.xml` (the rig: part placement, collision boxes and the overhead camera), `robot/u20cam.xml` and `scene.xml` were written for this project. The block and the tray are boxes of uniform density.

| Meshes in `robot/assets/` | Source | Changes |
| --- | --- | --- |
| Printed parts: `base_so101_v2`, `base_motor_holder_so101_v1`, `waveshare_mounting_plate_so101_v2`, `motor_holder_so101_base_v1`, `rotation_pitch_so101_v1`, `upper_arm_so101_v1`, `under_arm_so101_v1`, `motor_holder_so101_wrist_v1`, `wrist_roll_pitch_so101_v2`, `moving_jaw_so101_v1`<br>Servos: `sts3215_03a_v1`, `sts3215_03a_no_horn_v1`<br>Gripper collision meshes: `wrist_roll_follower_so101_gripper_part0_v1`, `moving_jaw_so101_gripper_part0_v1`, `moving_jaw_so101_gripper_part1_v1` | [Menagerie `robotstudio_so101/assets`](https://github.com/google-deepmind/mujoco_menagerie/tree/822c2d8f877dd166c5b7d3c9f7e3c3b6589473b7/robotstudio_so101/assets), revision `822c2d8` | none |
| `arm_base`, `cam_mount_bottom`, `cam_mount_middle`, `cam_mount_top` | SO-ARM100 [overhead camera mount](https://github.com/TheRobotStudio/SO-ARM100/tree/7629d2ad9853d10fb903093a33ef6114099d97e5/Optional/Overhead_Cam_Mount_32x32_UVC_Module/stl), revision `7629d2a` | none |
| `wrist_cam_mount_32x32_uvc_module_so101` | SO-ARM100 [wrist camera mount](https://github.com/TheRobotStudio/SO-ARM100/blob/7629d2ad9853d10fb903093a33ef6114099d97e5/Optional/Wrist_Cam_Mount_32x32_UVC_Module/stl/Wrist_Cam_Mount_32x32_UVC_Module_SO101.stl), revision `7629d2a` | converted from ASCII to binary STL; 1,915 duplicate triangles removed |
| `u20cam_1080p` | Modeled for this project from the dimensions and photos of the [InnoMaker U20CAM-1080P](https://github.com/INNO-MAKER/U20CAM-1080P-S1) | — |
