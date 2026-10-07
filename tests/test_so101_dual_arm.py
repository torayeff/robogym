"""Tests for the SO-101 dual-arm environment, run without the server."""

import base64
import concurrent.futures
import json

import cv2
import jsonschema
import mujoco
import numpy as np
import pytest

from robogym.envs.so101_dual_arm.env import FAR, FRONT, NEAR, So101DualArm


def state(env: So101DualArm) -> np.ndarray:
    """MuJoCo's full integration state: everything the next mj_step depends on."""
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


def test_reset_folds_the_arms_and_draws_new_object_poses(env):
    start = {name: env.data.joint(name).qpos.copy() for name in ("prism", "storage")}
    env.act({"joints": {"left_shoulder_lift": 0.0, "right_gripper": 1.7}})
    env.data.joint("prism").qpos[2] += 0.05  # lift the prism 5 cm so it falls
    env.step(2.0)
    env.reset()
    rest = env.model.key("rest")
    assert env.time == 0.0
    assert np.array_equal(env.data.qpos[:12], rest.qpos[:12])
    assert np.array_equal(env.data.ctrl, rest.ctrl)
    assert not env.data.qvel.any()
    for name, pose in start.items():
        assert not np.array_equal(env.data.joint(name).qpos, pose)


def test_a_seed_gives_the_same_layouts():
    with So101DualArm(seed=0) as env, So101DualArm(seed=0) as same:
        assert np.array_equal(state(same), state(env))
        env.reset()
        same.reset()
        assert np.array_equal(state(same), state(env))


def centres(env: So101DualArm) -> dict:
    """The centres of the prism and of the tray's floor, in world coordinates."""
    mujoco.mj_forward(env.model, env.data)
    return {
        "prism": env.data.geom("prism").xpos.copy(),
        "storage": env.data.body("storage").xpos.copy(),
    }


def pan_axis(env: So101DualArm, arm: str) -> np.ndarray:
    return env.data.joint(f"{arm}_shoulder_pan").xanchor[:2]


def test_each_object_starts_at_rest_where_only_the_arm_beside_it_reaches(env):
    heights = set()
    for _ in range(20):
        env.reset()
        start = centres(env)
        assert np.sign(start["prism"][1]) == -np.sign(start["storage"][1])
        for name, (x, y, _) in start.items():
            own, other = ("left", "right") if y > 0 else ("right", "left")
            assert np.linalg.norm([x, y] - pan_axis(env, own)) <= NEAR[name]
            assert np.linalg.norm([x, y] - pan_axis(env, other)) >= FAR[name]
            assert x >= FRONT[name]
        heights.add(round(start["prism"][2], 3))
        env.step(0.5)
        for name, centre in centres(env).items():
            assert centre == pytest.approx(start[name], abs=1e-3)
    assert heights == {0.775, 0.75}  # the prism stood and lay


def test_observing_does_not_change_the_simulation():
    with So101DualArm(seed=0) as env, So101DualArm(seed=0) as unobserved:
        for sim in (env, unobserved):
            sim.act({"joints": {"left_shoulder_lift": 0.0}})
        for _ in range(10):
            env.observe()
            env.positions()
            env.step(0.1)
            unobserved.step(0.1)
        assert np.array_equal(state(env), state(unobserved))


def test_the_view_renderer_draws_positions_on_a_thread_of_its_own(env):
    renderer = env.view_renderer(320, 240)
    rest = env.positions()
    env.act({"joints": {"left_shoulder_lift": 0.0}})
    env.step(1.0)
    with concurrent.futures.ThreadPoolExecutor(max_workers=1) as thread:
        images = list(thread.map(renderer.render, [rest, env.positions()]))
        thread.submit(renderer.close).result()
    assert images[0].shape == (240, 320, 3) and images[0].dtype == np.uint8
    assert not np.array_equal(images[0], images[1])
