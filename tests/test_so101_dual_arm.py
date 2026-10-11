"""Tests for the SO-101 dual-arm scene's physics and drawing, without the server."""

import base64

import cv2
import mujoco
import numpy as np
import pytest

from robogym.env import VIEW_HEIGHT, VIEW_WIDTH
from robogym.envs.so101_dual_arm.physics import (
    FAR,
    FRONT,
    NEAR,
    STATE,
    So101DualArmPhysics,
    compile_scene,
)


def run(physics: So101DualArmPhysics, seconds: float) -> None:
    physics.step(round(seconds / physics.timestep))


def angle(physics: So101DualArmPhysics, joint: str) -> float:
    return physics.data.joint(joint).qpos.item()


def test_an_observation_is_of_the_state_it_was_made_from(physics, renderer):
    then, start = physics.state(), angle(physics, "left_shoulder_lift")
    physics.act({"joints": {"left_shoulder_lift": 0.0}})
    run(physics, 1.0)
    past = renderer.observe(then).result()
    now = renderer.observe(physics.state()).result()
    assert past["time"] == 0.0 and now["time"] == physics.time
    assert past["joints"]["left_shoulder_lift"] == start
    assert now["joints"]["left_shoulder_lift"] == angle(physics, "left_shoulder_lift")
    assert past["images"]["top_camera"] != now["images"]["top_camera"]
    assert renderer.observe(then).result() == past
    for jpeg in past["images"].values():
        image = np.frombuffer(base64.b64decode(jpeg), np.uint8)
        assert cv2.imdecode(image, cv2.IMREAD_COLOR).shape == (480, 640, 3)


def test_the_view_shows_the_state(physics, renderer):
    rest = renderer.view(physics.state()).result()
    physics.act({"joints": {"left_shoulder_lift": 0.0}})
    run(physics, 1.0)
    moved = renderer.view(physics.state()).result()
    assert rest.shape == (VIEW_HEIGHT, VIEW_WIDTH, 3) and rest.dtype == np.uint8
    assert not np.array_equal(rest, moved)


def test_act_moves_a_joint_to_its_goal_and_the_others_hold(physics):
    held = angle(physics, "right_shoulder_lift")
    goal = angle(physics, "left_shoulder_lift") + 1.0
    physics.act({"joints": {"left_shoulder_lift": goal}})
    run(physics, 2.0)
    assert angle(physics, "left_shoulder_lift") == pytest.approx(goal, abs=0.01)
    assert angle(physics, "right_shoulder_lift") == pytest.approx(held, abs=0.01)


def test_reset_folds_the_arms_and_draws_new_object_poses(physics):
    start = {name: physics.data.joint(name).qpos.copy() for name in ("block", "tray")}
    physics.act({"joints": {"left_shoulder_lift": 0.0, "right_gripper": 1.7}})
    physics.data.joint("block").qpos[2] += 0.05
    run(physics, 2.0)
    physics.reset()
    rest = physics.model.key("rest")
    assert physics.time == 0.0
    assert np.array_equal(physics.data.qpos[:12], rest.qpos[:12])
    assert np.array_equal(physics.data.ctrl, rest.ctrl)
    assert not physics.data.qvel.any()
    for name, pose in start.items():
        assert not np.array_equal(physics.data.joint(name).qpos, pose)


def test_the_physics_steps_exactly_as_the_whole_scene_would(physics):
    model = compile_scene(visual=True)
    whole = mujoco.MjData(model)
    mujoco.mj_setState(model, whole, physics.state(), STATE)
    goals = {"left_shoulder_lift": 0.0, "right_elbow_flex": 0.5, "left_gripper": 1.5}
    physics.act({"joints": goals})
    for joint, goal in goals.items():
        whole.actuator(joint).ctrl = goal
    run(physics, 3.0)
    mujoco.mj_step(model, whole, nstep=round(3.0 / physics.timestep))
    state = np.empty_like(physics.state())
    mujoco.mj_getState(model, whole, state, STATE)
    assert np.array_equal(state, physics.state())
    assert physics.model.nmesh < model.nmesh


def centres(physics: So101DualArmPhysics) -> dict:
    """The centres of the block and of the tray's floor, in world coordinates."""
    mujoco.mj_forward(physics.model, physics.data)
    return {
        "block": physics.data.geom("block").xpos.copy(),
        "tray": physics.data.body("tray").xpos.copy(),
    }


def pan_axis(physics: So101DualArmPhysics, arm: str) -> np.ndarray:
    return physics.data.joint(f"{arm}_shoulder_pan").xanchor[:2]


def test_each_object_starts_at_rest_where_only_the_arm_beside_it_reaches(physics):
    block_heights = set()
    for _ in range(20):
        physics.reset()
        start = centres(physics)
        assert np.sign(start["block"][1]) == -np.sign(start["tray"][1])
        for name, (x, y, _) in start.items():
            own, other = ("left", "right") if y > 0 else ("right", "left")
            assert np.linalg.norm([x, y] - pan_axis(physics, own)) <= NEAR[name]
            assert np.linalg.norm([x, y] - pan_axis(physics, other)) >= FAR[name]
            assert x >= FRONT[name]
        block_heights.add(round(start["block"][2], 3))
        run(physics, 0.5)
        for name, centre in centres(physics).items():
            assert centre == pytest.approx(start[name], abs=1e-3)
    standing, lying = 0.775, 0.75
    assert block_heights == {standing, lying}


def test_a_diverging_step_raises(physics, monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    physics.data.qvel[0] = np.nan
    with pytest.raises(FloatingPointError, match="diverged"):
        physics.step(1)
