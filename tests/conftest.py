import pytest

from robogym.envs.so101_dual_arm.env import So101DualArm


@pytest.fixture
def env():
    with So101DualArm(seed=0) as env:
        yield env
