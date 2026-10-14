# so101_rubiks_cube

Two [SO-101](https://github.com/TheRobotStudio/SO-ARM100) robot arms stand side by side at the back of a table, with a standard 3×3×3 Rubik's cube (5.7 × 5.7 × 5.7 cm, 68.5 g) between them. Both arms can reach the cube to hold it, reorient it, and turn its six faces. The scene is simulated in MuJoCo, and the agent sees it through a camera on each wrist and one overhead.

## Observations and actions

Each arm has six servos: `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll` and `gripper`, named with a `left_` or `right_` prefix.

An observation contains the simulation time in seconds, the 12 joint angles in radians, and a 640 × 480 JPEG image, base64-encoded, from each of the three cameras: `top_camera`, `left_wrist_camera` and `right_wrist_camera`. It doesn't include the position or face angles of the Rubik's cube; the agent has to inspect the cube in the camera images.

An action sets target angles, in radians, for any of the servos, and the servos then move the joints toward them. Servos left out of an action keep their previous targets:

```json
{"joints": {"left_shoulder_lift": -0.75, "left_gripper": 1.2}}
```

`GET /spec` lists each servo's range and which way it turns, and the size of the Rubik's cube.

## Resets

A reset folds both arms, with the grippers just above the table, places the Rubik's cube at a random position and yaw between the arms, and scrambles its faces with 20 random non-cancelling quarter or half turns (`SCRAMBLE_MOVES` in `physics.py`). Two constants in `physics.py` bound where the cube's centre can land:

| Constant | Bound | Purpose |
| --- | --- | --- |
| `NEAR` | Maximum distance from both arms' shoulder-pan axes (0.33 m) | Either arm can grasp the cube or turn its faces with its gripper tilted no more than 25° from vertical |
| `FRONT` | Minimum forward position, x (0.20 m) | The cube stays clear of the folded arms and in view of the top camera |

## Viewing the scene in MuJoCo

To explore the scene without the server, open it in MuJoCo's viewer:

```bash
uv run python -m robogym.envs.so101_rubiks_cube
```

The viewer shows the scene as a reset leaves it. Reload (Ctrl+L) resets it, and the sliders in the Control panel move the servos.

## Files

- `env.py`: the environment the server serves, with its spec, its action checks, and the rendering of observations and video.
- `physics.py`: the simulation, the random placement and face scrambling on reset.
- `camera.py`: the lens model.
- `__main__.py`: the MuJoCo viewer.
- `scene.xml`: the table, the lights, the attached robot and Rubik's cube, and the folded rest pose.
- `cube/cube_3x3x3.xml`: the 26-cubelet articulated 3×3×3 Rubik's cube model.
- `cube/assets/sticker.png`: the UV sticker atlas texture.
- `Dockerfile`, `sandbox.toml`: the image and resources for a Modal sandbox.

`physics.py` builds two models from `scene.xml` in memory. The simulation process gets a model without the visual-only robot meshes and with all 26 identical cubelet collision meshes deduplicated into one shared mesh. The renderer gets the full scene with per-cubelet UV texture coordinates for the six sticker colors.

## Sources and license

The robot arm and mounting rig come from [so101_dual_arm](../so101_dual_arm/README.md).

`cube/cube_3x3x3.xml` and `cube/assets/sticker.png` are adapted from Kevin Zakka's [mujoco_cube](https://github.com/kevinzakka/mujoco_cube) (also included in Google DeepMind's MuJoCo model library under `model/cube`), licensed under Apache-2.0 ([cube/LICENSE](cube/LICENSE)). Compared with `mujoco_cube`, the cube root has a free joint and explicit core inertia, the face-turning actuators and standalone scene elements (`<option>`, `<visual>`, `<light>`, skybox) are removed so the cube is manipulated purely by contact with the robot arms, and the tabletop (`priority="1"`) and grippers (`priority="1"`) provide contact friction against the cube while cubelet-to-cubelet contacts remain frictionless (`condim="1"`).
