"""Tests for the SO-101 dual-arm Rubik's cube scene's physics and drawing."""

import base64

import cv2
import mujoco
import numpy as np
import pytest

from robogym.env import VIEW_HEIGHT, VIEW_WIDTH
from robogym.envs.so101_rubiks_cube.env import Renderer
from robogym.envs.so101_rubiks_cube.physics import (
    FRONT,
    NEAR,
    STATE,
    So101RubiksCubePhysics,
    compile_scene,
)


@pytest.fixture
def cube_physics():
    return So101RubiksCubePhysics(seed=0)


@pytest.fixture(scope="module")
def cube_renderer():
    renderer = Renderer(compile_scene(visual=True))
    yield renderer
    renderer.close()


def run(physics: So101RubiksCubePhysics, seconds: float) -> None:
    physics.step(round(seconds / physics.timestep))


def angle(physics: So101RubiksCubePhysics, joint: str) -> float:
    return physics.data.joint(joint).qpos.item()


def test_an_observation_is_of_the_state_it_was_made_from(cube_physics, cube_renderer):
    then, start = cube_physics.state(), angle(cube_physics, "left_shoulder_lift")
    cube_physics.act({"joints": {"left_shoulder_lift": 0.0}})
    run(cube_physics, 1.0)
    past = cube_renderer.observe(then).result()
    now = cube_renderer.observe(cube_physics.state()).result()
    assert past["time"] == 0.0 and now["time"] == cube_physics.time
    assert past["joints"]["left_shoulder_lift"] == start
    assert now["joints"]["left_shoulder_lift"] == angle(
        cube_physics, "left_shoulder_lift"
    )
    assert past["images"]["top_camera"] != now["images"]["top_camera"]
    assert cube_renderer.observe(then).result() == past
    for jpeg in past["images"].values():
        image = np.frombuffer(base64.b64decode(jpeg), np.uint8)
        assert cv2.imdecode(image, cv2.IMREAD_COLOR).shape == (480, 640, 3)


def test_the_view_shows_the_state(cube_physics, cube_renderer):
    rest = cube_renderer.view(cube_physics.state()).result()
    cube_physics.act({"joints": {"left_shoulder_lift": 0.0}})
    run(cube_physics, 1.0)
    moved = cube_renderer.view(cube_physics.state()).result()
    assert rest.shape == (VIEW_HEIGHT, VIEW_WIDTH, 3) and rest.dtype == np.uint8
    assert not np.array_equal(rest, moved)


def test_act_moves_a_joint_to_its_goal_and_the_others_hold(cube_physics):
    held = angle(cube_physics, "right_shoulder_lift")
    goal = angle(cube_physics, "left_shoulder_lift") + 1.0
    cube_physics.act({"joints": {"left_shoulder_lift": goal}})
    run(cube_physics, 2.0)
    assert angle(cube_physics, "left_shoulder_lift") == pytest.approx(goal, abs=0.01)
    assert angle(cube_physics, "right_shoulder_lift") == pytest.approx(held, abs=0.01)


def test_reset_folds_the_arms_and_scrambles_the_cube(cube_physics):
    start_cube = cube_physics.data.qpos[12:].copy()
    cube_physics.act({"joints": {"left_shoulder_lift": 0.0, "right_gripper": 1.7}})
    cube_physics.data.joint("cube").qpos[2] += 0.05
    run(cube_physics, 2.0)
    cube_physics.reset()
    rest = cube_physics.model.key("rest")
    assert cube_physics.time == 0.0
    assert np.array_equal(cube_physics.data.qpos[:12], rest.qpos[:12])
    assert np.array_equal(cube_physics.data.ctrl, rest.ctrl)
    assert not cube_physics.data.qvel.any()
    assert not np.array_equal(cube_physics.data.qpos[12:], start_cube)
    assert not np.array_equal(cube_physics.data.qpos[19:], rest.qpos[19:])


def test_the_physics_steps_exactly_as_the_whole_scene_would(cube_physics):
    model = compile_scene(visual=True)
    whole = mujoco.MjData(model)
    mujoco.mj_setState(model, whole, cube_physics.state(), STATE)
    goals = {"left_shoulder_lift": 0.0, "right_elbow_flex": 0.5, "left_gripper": 1.5}
    cube_physics.act({"joints": goals})
    for joint, goal in goals.items():
        whole.actuator(joint).ctrl = goal
    run(cube_physics, 3.0)
    mujoco.mj_step(model, whole, nstep=round(3.0 / cube_physics.timestep))
    state = np.empty_like(cube_physics.state())
    mujoco.mj_getState(model, whole, state, STATE)
    assert np.array_equal(state, cube_physics.state())
    assert cube_physics.model.nmesh < model.nmesh


def pan_axis(physics: So101RubiksCubePhysics, arm: str) -> np.ndarray:
    return physics.data.joint(f"{arm}_shoulder_pan").xanchor[:2]


def test_the_cube_starts_at_rest_where_both_arms_reach_with_valid_cubelet_grid(
    cube_physics,
):
    for _ in range(20):
        cube_physics.reset()
        mujoco.mj_forward(cube_physics.model, cube_physics.data)
        start = cube_physics.data.body("cube").xpos.copy()
        x, y, z = start
        for arm in ("left", "right"):
            assert np.linalg.norm([x, y] - pan_axis(cube_physics, arm)) <= NEAR
        assert x >= FRONT
        assert z == pytest.approx(0.7685, abs=1e-4)

        # All 26 cubelets occupy the 26 distinct 3x3x3 grid cells in the cube frame
        rot = cube_physics.data.body("cube").xmat.reshape(3, 3)
        cells = set()
        for j_id in range(cube_physics.model.joint("pX").id, cube_physics.model.njnt):
            body_id = cube_physics.model.jnt_bodyid[j_id]
            geom_id = cube_physics.model.body_geomadr[body_id]
            rel = rot.T @ (cube_physics.data.geom_xpos[geom_id] - start)
            cells.add(tuple(np.round(rel / 0.019).astype(int)))
        assert len(cells) == 26

        run(cube_physics, 0.5)
        mujoco.mj_forward(cube_physics.model, cube_physics.data)
        after = cube_physics.data.body("cube").xpos.copy()
        assert after == pytest.approx(start, abs=1e-3)


def test_a_diverging_step_raises(cube_physics, monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    cube_physics.data.qvel[0] = np.nan
    with pytest.raises(FloatingPointError, match="diverged"):
        cube_physics.step(1)
