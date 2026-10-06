"""Tests for the SO-101 dual-arm environment, run without the server."""

import base64
import json

import cv2
import jsonschema
import mujoco
import numpy as np
import pytest

from envs.so101_dual_arm.env import So101DualArm


@pytest.fixture
def env():
    with So101DualArm() as env:
        yield env


def state(env: So101DualArm) -> np.ndarray:
    """Everything that decides how the simulation goes on.

    Time, positions, velocities, servo goals and the solver's warm start.
    """
    kind = mujoco.mjtState.mjSTATE_INTEGRATION
    out = np.empty(mujoco.mj_stateSize(env.model, kind))
    mujoco.mj_getState(env.model, env.data, out, kind)
    return out


def test_spec_has_json_schemas_for_every_servo_and_camera(env):
    spec = json.loads(json.dumps(env.spec()))  # as a client receives it
    jsonschema.Draft202012Validator.check_schema(spec["observation"])
    jsonschema.Draft202012Validator.check_schema(spec["action"])
    goals = spec["action"]["properties"]["joints"]["properties"]
    assert list(goals) == [env.model.actuator(i).name for i in range(env.model.nu)]
    ranges = [[goal["minimum"], goal["maximum"]] for goal in goals.values()]
    assert ranges == env.model.actuator_ctrlrange.tolist()
    images = spec["observation"]["properties"]["images"]["properties"]
    assert list(images) == ["top_camera", "left_wrist_camera", "right_wrist_camera"]


def test_observation_matches_spec(env):
    observation = json.loads(json.dumps(env.observe()))
    jsonschema.validate(observation, env.spec()["observation"])
    for jpeg in observation["images"].values():
        data = np.frombuffer(base64.b64decode(jpeg), np.uint8)
        assert cv2.imdecode(data, cv2.IMREAD_COLOR).shape == (480, 640, 3)


def test_act_moves_a_joint_to_its_goal_and_the_others_hold(env):
    start = env.observe()["joints"]
    goal = start["left_shoulder_lift"] + 1.0
    env.act({"joints": {"left_shoulder_lift": goal}})
    env.step(2.0)
    joints = env.observe()["joints"]
    assert joints["left_shoulder_lift"] == pytest.approx(goal, abs=0.01)
    held = start["right_shoulder_lift"]
    assert joints["right_shoulder_lift"] == pytest.approx(held, abs=0.01)


@pytest.mark.parametrize(
    ("action", "error"),
    [
        ({"gripper": 1.0}, "An action is"),
        (
            {"joints": {"left_shoulder_pan": 0.5, "left_elbow": 0.0}},
            "Unknown joint 'left_elbow'",
        ),
        ({"joints": {"left_gripper": 2.0}}, "from -0.17453 to 1.74533"),
        ({"joints": {"left_gripper": "open"}}, "must be a number"),
        ({"joints": {"left_gripper": float("nan")}}, "must be a number"),
    ],
)
def test_act_rejects_a_bad_action_and_changes_nothing(env, action, error):
    goals = env.data.ctrl.copy()
    with pytest.raises(ValueError, match=error):
        env.act(action)
    assert np.array_equal(env.data.ctrl, goals)


def test_step_advances_in_whole_physics_steps(env):
    env.step(1.0)
    assert env.time == pytest.approx(1.0)
    env.step(0.4 * env.model.opt.timestep)
    assert env.time == pytest.approx(1.0)


def test_reset_restores_the_whole_simulation(env):
    start = state(env)
    env.act({"joints": {"left_shoulder_lift": 0.0, "right_gripper": 1.7}})
    env.data.joint("prism").qpos[2] += 0.05  # lift the prism 5 cm so it falls
    env.step(2.0)
    env.reset()
    assert np.array_equal(state(env), start)


def test_observing_does_not_change_the_simulation(env):
    with So101DualArm() as unobserved:
        for sim in (env, unobserved):
            sim.act({"joints": {"left_shoulder_lift": 0.0}})
        for _ in range(10):
            env.observe()
            env.step(0.1)
            unobserved.step(0.1)
        assert np.array_equal(state(env), state(unobserved))
