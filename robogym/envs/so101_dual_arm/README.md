# SO-101 dual arm (MuJoCo)

Two SO-101 arms with wrist cameras and an overhead camera, on a table with a prism and a storage tray. Each reset puts one object beside each arm, out of the other arm's reach, at a random place and angle; `NEAR`, `FAR` and `FRONT` in `env.py` set where.

```bash
uv run python -m robogym.envs.so101_dual_arm
```

Opens the environment in the MuJoCo viewer as a reset leaves it, with the arms folded. Reload (Ctrl+L) resets it; the Control sliders drive the 12 servos.

- `env.py`: the scene as a robogym environment: goal angles for the 12 servos in; joint angles and the three robot cameras' images out. `So101DualArm(seed=...)` repeats the same layouts.
- `camera.py`: renders the robot cameras through their wide-angle lens.
- `scene.xml`: floor, lights, table, prism, tray, rest keyframe.
- `robot/`: the rig with its overhead camera (`so101_dual_arm.xml`), which attaches the arm (`so101.xml`) twice; the camera module (`u20cam.xml`); the meshes; and where they all come from ([NOTICE.md](robot/NOTICE.md)).

## Cameras

`top_camera` and the wrist cameras are InnoMaker U20CAM-1080P-S1 modules at 640 × 480. MuJoCo's pinhole cameras can't draw their wide-angle lens, so the MuJoCo viewer shows them undistorted, and `camera.py` renders them through the lens for observations. The lens values are nominal, from the [user manual](https://github.com/INNO-MAKER/U20CAM-1080P-S1/blob/a08d78bfc7fb6b1230bad0af28192d58b557e077/U20CAM-1080P-S1%20UserManual-v1.1.pdf) rather than a calibration. A calibration replaces `U20CAM` in `camera.py` with its K and D, and the `fovy` in `robot/u20cam.xml` with its vertical field of view.
