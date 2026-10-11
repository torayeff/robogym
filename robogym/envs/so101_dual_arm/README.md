# SO-101 dual arm (MuJoCo)

Two SO-101 arms with wrist cameras and an overhead camera, on a table with a block and a tray. Each reset puts one object beside each arm, where the other arm can't pick it up ([Layouts](#layouts)).

```bash
uv run python -m robogym.envs.so101_dual_arm
```

Opens the scene in the MuJoCo viewer as a reset leaves it: arms folded, the block and the tray in random places. Reload (Ctrl+L) resets it. Drive the 12 position actuators (`{left,right}_{shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper}`, radians) with the Control sliders. Cameras: `overview`, `top_camera`, `left_wrist_camera`, `right_wrist_camera`. The block and the tray have free joints named `block` and `tray`.

- `physics.py`: what the physics process steps: scene.xml without its visual-only parts. `So101DualArmPhysics(seed=...)` repeats the same layouts.
- `env.py`: the environment as the server serves it: its spec, and the drawing of observations and the view from physics states.
- `camera.py`: the robot cameras' wide-angle lens.
- `Dockerfile`, `sandbox.toml`: the environment's Modal sandbox: its image, and the vCPUs, memory and GPU it runs on.
- `scene.xml`: floor, lights, table, block, tray, rest keyframe.
- `robot/so101_dual_arm.xml`: the mounting rig and the overhead camera; attaches `so101.xml` twice, prefixed `left_` and `right_`.
- `robot/so101.xml`: one arm with its wrist camera.
- `robot/u20cam.xml`: the camera module, used for all three cameras.
- `robot/assets/`: the 21 STL meshes.

## Layouts

Every reset puts the block beside one arm and the tray beside the other, which way round at random, each turned to a random angle; the block stands on end or lies on a side, equally often. `NEAR`, `FAR` and `FRONT` in `physics.py` bound where each object's centre is drawn, uniformly:

- within `NEAR` of its own arm's shoulder-pan axis, so that arm can pick the block up with its gripper within 25° of vertical, and get over the tray within 40°;
- at least `FAR` from the other arm's. That arm's grasp point (the `gripperframe` site) gets no more than 0.436 m from its pan axis at block heights and 0.441 m above a tray, so it can't take hold of the block, not even near one end of a lying one, or get over the inside of the tray. Its fingertips may just touch an object at full stretch;
- at x ≥ `FRONT`, which keeps all of it in the top camera's view.

The reaches come from searching the model's joint angles, counting poses that sink no more than 1 mm into the table, the rig or the arm itself. A search over the other arm's joint angles got its grasp point no closer than 19 mm to the graspable part of a lying block pointing at it, or 14 mm to over a tray turned corner-on.

## Cameras

`top_camera` and the wrist cameras are InnoMaker U20CAM-1080P-S1 modules at 640 × 480. Their wide-angle lens has strong barrel distortion, which MuJoCo's pinhole cameras can't draw. So `env.py` draws a wider pinhole image and `camera.py` maps it through the lens, while the MuJoCo viewer shows the cameras undistorted, through a pinhole with the lens's vertical field of view.

The lens values are nominal, from the [user manual](https://github.com/INNO-MAKER/U20CAM-1080P-S1/blob/a08d78bfc7fb6b1230bad0af28192d58b557e077/U20CAM-1080P-S1%20UserManual-v1.1.pdf) (v1.1) rather than a calibration. Its 106° horizontal field of view across the 1920 × 1080 sensor is more than a 2.8 mm pinhole lens covers on a 1/3″ sensor (about 86°), but what an equidistant lens covers: OpenCV's fisheye model with k1–k4 = 0. Taking 640 × 480 to be the 4:3 centre crop, scaled by 480/1080, gives 79.5° × 59.625°. The manual's figures don't all fit one lens: it also gives 103° horizontal, its 130° diagonal is 122° in this model, and its −17% TV distortion has no stated convention. The true vertical field of view could be anywhere from about 54° to 66°. A calibration replaces `U20CAM` in `camera.py` with its K and D.

## Sources

The arm is The Robot Studio's SO-101, adapted from MuJoCo Menagerie; the rig, the camera module and the scene are authored for this project. [robot/NOTICE.md](robot/NOTICE.md) gives each file's source, revision and changes, and [robot/LICENSE](robot/LICENSE) their license.
