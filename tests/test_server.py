"""Tests for the HTTP server, running the SO-101 dual-arm environment in real time."""

import time

import jsonschema
import pytest
from fastapi.testclient import TestClient

from envs.so101_dual_arm.env import So101DualArm
from robogym.server import create_app


@pytest.fixture
def client():
    with TestClient(create_app(So101DualArm)) as client:
        yield client


def observe(client: TestClient) -> dict:
    response = client.get("/observation")
    assert response.status_code == 200
    return response.json()


def move_left_shoulder(client: TestClient, by: float) -> float:
    """Set the left shoulder's goal `by` radians from where it is; returns the goal."""
    goal = observe(client)["joints"]["left_shoulder_lift"] + by
    response = client.post("/action", json={"joints": {"left_shoulder_lift": goal}})
    assert response.status_code == 204
    return goal


def test_observation_matches_spec(client):
    spec = client.get("/spec").json()
    assert spec["name"] == "so101_dual_arm"
    jsonschema.validate(observe(client), spec["observation"])


def test_the_simulation_waits_for_start(client):
    move_left_shoulder(client, by=1.0)
    time.sleep(0.2)
    assert observe(client)["time"] == 0.0


def test_simulation_time_keeps_pace_with_the_wall_clock(client):
    assert client.post("/start").status_code == 204
    start = time.monotonic()
    first = observe(client)["time"]
    time.sleep(0.5)
    second = observe(client)["time"]
    assert second - first == pytest.approx(time.monotonic() - start, abs=0.1)


def test_an_action_moves_the_arm_while_the_simulation_runs(client):
    client.post("/start")
    goal = move_left_shoulder(client, by=1.0)
    time.sleep(1.5)
    joints = observe(client)["joints"]
    assert joints["left_shoulder_lift"] == pytest.approx(goal, abs=0.01)


def test_a_bad_action_is_rejected_with_the_reason(client):
    response = client.post("/action", json={"joints": {"left_elbow": 0.0}})
    assert response.status_code == 422
    assert "Unknown joint 'left_elbow'" in response.json()["detail"]


def test_pause_freezes_the_simulation_and_start_goes_on_from_there(client):
    client.post("/start")
    time.sleep(0.2)
    assert client.post("/pause").status_code == 204
    frozen = observe(client)
    goal = move_left_shoulder(client, by=1.0)
    time.sleep(0.3)
    still = observe(client)
    assert (still["time"], still["joints"]) == (frozen["time"], frozen["joints"])
    client.post("/start")
    assert frozen["time"] <= observe(client)["time"] < frozen["time"] + 0.1
    time.sleep(1.5)
    joints = observe(client)["joints"]
    assert joints["left_shoulder_lift"] == pytest.approx(goal, abs=0.01)


def test_reset_puts_everything_back_and_waits_for_start(client):
    start = observe(client)
    client.post("/start")
    move_left_shoulder(client, by=1.0)
    time.sleep(1.0)
    assert client.post("/reset").status_code == 204
    time.sleep(0.2)
    observation = observe(client)
    assert observation["time"] == 0.0
    assert observation["joints"] == pytest.approx(start["joints"], abs=1e-3)
