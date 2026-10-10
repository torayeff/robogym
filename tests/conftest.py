import pytest

from robogym.envs.so101_dual_arm.env import Renderer
from robogym.envs.so101_dual_arm.physics import So101DualArmPhysics, compile_scene


@pytest.fixture
def physics():
    return So101DualArmPhysics(seed=0)


@pytest.fixture(scope="session")
def renderer():
    renderer = Renderer(compile_scene(visual=True))
    yield renderer
    renderer.close()
