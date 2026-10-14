"""Open the SO-101 dual-arm Rubik's cube scene in MuJoCo's viewer; reload (Ctrl+L) resets it.

uv run python -m robogym.envs.so101_rubiks_cube
"""

import mujoco
import mujoco.viewer

from .camera import U20CAM
from .env import CAMERAS
from .physics import STATE, So101RubiksCubePhysics, compile_scene

physics = So101RubiksCubePhysics()
model = compile_scene(visual=True)
for camera in CAMERAS:
    model.cam_fovy[model.camera(camera).id] = U20CAM.fovy


def reset() -> tuple[mujoco.MjModel, mujoco.MjData]:
    physics.reset()
    data = mujoco.MjData(model)
    mujoco.mj_setState(model, data, physics.state(), STATE)
    return model, data


mujoco.viewer.launch(loader=reset)
