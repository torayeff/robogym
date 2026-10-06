"""Open the SO-101 dual-arm environment in the MuJoCo viewer, as a reset leaves it.

Reload (Ctrl+L) resets the environment, which puts the prism and tray in new places.

    uv run envs/so101_dual_arm/view.py
"""

import sys
from pathlib import Path

import mujoco.viewer

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # the repository root
from envs.so101_dual_arm.env import So101DualArm

env = So101DualArm()


def reset() -> tuple[mujoco.MjModel, mujoco.MjData]:
    env.reset()
    return env.model, env.data


mujoco.viewer.launch(loader=reset)
