"""The environments robogym serves, by name, each imported only when it's served."""

import importlib

from robogym.env import Env

ENVS = {
    "so101_dual_arm": "robogym.envs.so101_dual_arm.env:So101DualArm",
    "so101_rubiks_cube": "robogym.envs.so101_rubiks_cube.env:So101RubiksCube",
}


def load(name: str) -> type[Env]:
    module, _, cls = ENVS[name].partition(":")
    return getattr(importlib.import_module(module), cls)
