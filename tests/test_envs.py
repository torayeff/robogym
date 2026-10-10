"""Tests that every environment in ENVS keeps robogym.env's contract."""

import asyncio
import time

import jsonschema
import numpy as np
import pytest
from fastapi.testclient import TestClient

from robogym.env import VIEW_HEIGHT, VIEW_WIDTH
from robogym.envs import ENVS, load
from robogym.server import create_app


@pytest.fixture(params=ENVS)
def client(request):
    with TestClient(create_app(load(request.param))) as client:
        yield client


def status(client: TestClient) -> dict:
    return client.get("/status").json()


def test_observations_match_the_spec(client):
    spec = client.get("/spec").json()
    jsonschema.Draft202012Validator.check_schema(spec["observation"])
    jsonschema.Draft202012Validator.check_schema(spec["action"])
    jsonschema.validate(client.get("/observation").json(), spec["observation"])


def test_time_waits_for_resume_then_keeps_to_the_wall_clock(client):
    time.sleep(0.2)
    assert status(client)["time"] == 0.0
    client.post("/resume")
    start, first = time.monotonic(), status(client)["time"]
    time.sleep(1.0)
    now = status(client)
    assert now["running"]
    assert now["time"] - first == pytest.approx(time.monotonic() - start, abs=0.05)


def test_pause_freezes_time_and_resume_goes_on_from_there(client):
    client.post("/resume")
    time.sleep(0.2)
    assert client.post("/pause").status_code == 204
    frozen = status(client)
    time.sleep(0.3)
    assert status(client) == frozen and not frozen["running"]
    assert client.post("/resume").status_code == 204
    assert frozen["time"] <= status(client)["time"] < frozen["time"] + 0.1


def test_reset_goes_back_to_time_zero_paused(client):
    client.post("/resume")
    time.sleep(0.2)
    assert client.post("/reset").status_code == 204
    time.sleep(0.2)
    after = status(client)
    assert (after["running"], after["time"], after["resets"]) == (False, 0.0, 1)


@pytest.mark.parametrize("name", ENVS)
def test_a_view_comes_only_when_the_simulation_changes(name):
    async def check():
        with load(name)() as env:
            view = await env.view()
            assert view.shape == (VIEW_HEIGHT, VIEW_WIDTH, 3) and view.dtype == np.uint8
            assert await env.view() is None
            await env.resume()
            await asyncio.sleep(0.1)
            assert await env.view() is not None

    asyncio.run(check())
