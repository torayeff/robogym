"""Open the SO-101 dual-arm environment in the MuJoCo viewer, as a reset leaves it.

    uv run python -m robogym.envs.so101_dual_arm

Reload (Ctrl+L) resets the environment, which puts the prism and tray in new places.
"""

import mujoco.viewer

from .env import So101DualArm

env = So101DualArm()


def reset() -> tuple[mujoco.MjModel, mujoco.MjData]:
    env.reset()
    return env.model, env.data


mujoco.viewer.launch(loader=reset)
