"""Tests for the HTTP server, serving the SO-101 dual-arm environment in real time."""

import functools
import multiprocessing
import os
import signal
import socket
import subprocess
import sys
import time

import httpx2
import pytest
from fastapi.testclient import TestClient

from robogym.envs.so101_dual_arm.env import So101DualArm
from robogym.envs.so101_dual_arm.physics import So101DualArmPhysics, compile_scene
from robogym.server import create_app


class StepFailsOnce(So101DualArmPhysics):
    """Its first step raises; a monkeypatch wouldn't reach the physics process."""

    failed = False

    def step(self, steps: int) -> None:
        if not self.failed:
            self.failed = True
            raise RuntimeError("step failed")
        super().step(steps)


@pytest.fixture
def client():
    with TestClient(create_app(So101DualArm)) as client:
        yield client


def status(client: TestClient) -> dict:
    return client.get("/status").json()


def left_shoulder(client: TestClient) -> float:
    return client.get("/observation").json()["joints"]["left_shoulder_lift"]


def test_the_spec_has_every_servo_and_camera(client):
    spec = client.get("/spec").json()
    model = compile_scene(visual=True)
    goals = spec["action"]["properties"]["joints"]["properties"]
    assert list(goals) == [model.actuator(i).name for i in range(model.nu)]
    ranges = [[goal["minimum"], goal["maximum"]] for goal in goals.values()]
    assert ranges == model.actuator_ctrlrange.tolist()
    images = spec["observation"]["properties"]["images"]["properties"]
    assert list(images) == ["top_camera", "left_wrist_camera", "right_wrist_camera"]


def test_an_action_moves_the_arm_while_the_simulation_runs(client):
    client.post("/resume")
    goal = left_shoulder(client) + 1.0
    response = client.post("/action", json={"joints": {"left_shoulder_lift": goal}})
    assert response.status_code == 204
    time.sleep(1.5)
    assert left_shoulder(client) == pytest.approx(goal, abs=0.01)


def test_a_bad_action_is_rejected_with_the_reason(client):
    response = client.post("/action", json={"joints": {"left_elbow": 0.0}})
    assert response.status_code == 422
    assert "Unknown joint 'left_elbow'" in response.json()["detail"]


def test_a_failed_step_resets_and_pauses_the_simulation():
    with TestClient(
        create_app(functools.partial(So101DualArm, StepFailsOnce))
    ) as client:
        client.post("/resume")
        time.sleep(0.1)
        after = status(client)
        assert (after["running"], after["time"], after["resets"]) == (False, 0.0, 1)
        client.post("/resume")
        time.sleep(0.2)
        assert status(client)["time"] > 0


def test_late_steps_are_reported(client):
    client.post("/resume")
    (physics,) = multiprocessing.active_children()
    os.kill(physics.pid, signal.SIGSTOP)
    time.sleep(0.05)
    os.kill(physics.pid, signal.SIGCONT)
    time.sleep(0.1)
    after = status(client)
    assert after["late_steps"] >= 1 and after["max_lateness"] >= 0.04


def test_the_server_stops_if_the_simulation_dies(monkeypatch, caplog):
    signals = []
    monkeypatch.setattr(signal, "raise_signal", signals.append)
    with TestClient(create_app(So101DualArm)):
        (physics,) = multiprocessing.active_children()
        physics.kill()
        time.sleep(0.5)
    assert signals == [signal.SIGTERM]
    assert "The simulation died" in caplog.text


def test_the_viewer_page_is_at_the_root(client):
    response = client.get("/")
    assert response.status_code == 200
    assert response.headers["content-type"].startswith("text/html")


def test_physics_keeps_to_the_wall_clock_while_observations_are_made():
    """Through the robogym command, at its real priorities."""
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    url = f"http://127.0.0.1:{port}"
    command = [sys.executable, "-m", "robogym", "serve", "so101_dual_arm"]
    command += ["--port", str(port)]
    server = subprocess.Popen(command, stderr=subprocess.DEVNULL)
    try:
        for _ in range(150):
            try:
                httpx2.post(f"{url}/resume")
                break
            except httpx2.ConnectError:
                time.sleep(0.2)
        httpx2.post(f"{url}/action", json={"joints": {"left_shoulder_lift": 0.0}})
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            assert httpx2.get(f"{url}/observation").status_code == 200
        assert httpx2.get(f"{url}/status").json()["late_steps"] == 0
    finally:
        server.send_signal(signal.SIGINT)
        server.wait()
