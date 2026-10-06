"""Serves any environment in envs/ over HTTP, running it in real time from /start.

Run it from the repository root with the environment's name from envs/__init__.py:

    uv run python -m robogym.server <name>    # e.g. so101_dual_arm

Every endpoint runs on the event loop's thread, one at a time, so that thread makes
every call to the environment and owns any OpenGL context it has.
"""

import argparse
import asyncio
import contextlib
import importlib
import time
from collections.abc import Callable

import uvicorn
from fastapi import FastAPI, HTTPException

from envs import ENVS
from robogym.env import Env

# How often a running simulation steps to keep up with the wall clock.
TICK = 0.01
# A longer stall, such as the computer sleeping, is skipped rather than simulated.
MAX_LAG = 0.5


async def run_in_real_time(env: Env) -> None:
    """Step env forever, keeping its time in line with the wall clock."""
    origin = time.monotonic() - env.time  # the wall-clock time at which env.time was 0
    while True:
        await asyncio.sleep(TICK)
        lag = time.monotonic() - origin - env.time
        if lag > MAX_LAG:
            origin += lag
        else:
            env.step(lag)


def create_app(make_env: Callable[[], Env]) -> FastAPI:
    env: Env
    clock: asyncio.Task | None = None  # runs the simulation, from /start to /pause

    @contextlib.asynccontextmanager
    async def lifespan(app: FastAPI):
        nonlocal env
        with make_env() as env:
            yield
            await pause()

    app = FastAPI(title="robogym", lifespan=lifespan)

    @app.get("/spec")
    async def spec() -> dict:
        """What the environment is, and the formats of /observation and /action."""
        return env.spec()

    @app.get("/observation")
    async def observe() -> dict:
        """The environment as it is now, in the format /spec gives."""
        return env.observe()

    @app.post("/action", status_code=204)
    async def act(action: dict) -> None:
        """Apply an action in the format /spec gives; a running simulation carries on."""
        try:
            env.act(action)
        except ValueError as error:
            raise HTTPException(422, str(error)) from error

    @app.post("/start", status_code=204)
    async def start() -> None:
        """Run the simulation in real time, from where it is."""
        nonlocal clock
        if clock is None:
            clock = asyncio.create_task(run_in_real_time(env))

    @app.post("/pause", status_code=204)
    async def pause() -> None:
        """Freeze simulation time; /start goes on from there."""
        nonlocal clock
        if clock is not None:
            clock.cancel()
            clock = None

    @app.post("/reset", status_code=204)
    async def reset() -> None:
        """Stop the simulation and put it back to its starting state."""
        await pause()
        env.reset()

    return app


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("env", choices=ENVS, help="the environment to serve")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()
    module, _, name = ENVS[args.env].partition(":")
    make_env = getattr(importlib.import_module(module), name)
    uvicorn.run(create_app(make_env), host=args.host, port=args.port)


if __name__ == "__main__":
    main()
