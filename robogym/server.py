"""Serves a robogym environment over HTTP, running it in real time from /start.

    uv run robogym so101_dual_arm

Every endpoint runs on the event loop's thread, one at a time, so that thread makes
every call to the environment and owns any OpenGL context it has. The viewer at /
renders on its own thread.
"""

import argparse
import asyncio
import contextlib
import logging
import time
from collections.abc import Callable, Coroutine
from pathlib import Path

import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, StreamingResponse

from robogym.env import Env
from robogym.envs import ENVS
from robogym.viewer import Viewer

# How often a running simulation steps to keep up with the wall clock.
TICK = 0.01
# A longer stall, such as the computer sleeping, is skipped rather than simulated.
MAX_LAG = 0.5
VIEWER_PAGE = Path(__file__).with_name("viewer.html")

logger = logging.getLogger(__name__)


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


def background(coroutine: Coroutine, name: str) -> asyncio.Task:
    """Run coroutine as a task that logs the exception it fails with."""

    def log_failure(task: asyncio.Task) -> None:
        if not task.cancelled() and (error := task.exception()):
            logger.error("The %s stopped", name, exc_info=error)

    task = asyncio.create_task(coroutine)
    task.add_done_callback(log_failure)
    return task


def create_app(make_env: Callable[[], Env]) -> FastAPI:
    env: Env
    viewer: Viewer
    clock: asyncio.Task | None = None  # runs the simulation, from /start to /pause

    @contextlib.asynccontextmanager
    async def lifespan(app: FastAPI):
        nonlocal env, viewer
        with make_env() as env, Viewer(env) as viewer:
            streaming = background(viewer.run(), "viewer")
            yield
            streaming.cancel()
            await pause()

    app = FastAPI(title="robogym", lifespan=lifespan)

    @app.get("/spec")
    async def spec() -> dict:
        """Name, description, and the JSON Schemas of /observation and /action."""
        return env.spec()

    @app.get("/observation")
    async def observe() -> dict:
        """The current observation, in the format /spec gives."""
        return env.observe()

    @app.post("/action", status_code=204)
    async def act(action: dict) -> None:
        """Apply an action in the format /spec gives, without waiting for its effect."""
        try:
            env.act(action)
        except ValueError as error:
            raise HTTPException(422, str(error)) from error

    @app.post("/start", status_code=204)
    async def start() -> None:
        """Run the simulation in real time, from where it is."""
        nonlocal clock
        if clock is None or clock.done():  # done: a step failed
            clock = background(run_in_real_time(env), "simulation")

    @app.post("/pause", status_code=204)
    async def pause() -> None:
        """Freeze simulation time; /start resumes from there."""
        nonlocal clock
        if clock is not None:
            clock.cancel()
            clock = None

    @app.post("/reset", status_code=204)
    async def reset() -> None:
        """Stop the simulation and put it back to its starting state."""
        await pause()
        env.reset()

    @app.get("/", include_in_schema=False)
    async def page() -> FileResponse:
        return FileResponse(VIEWER_PAGE)

    @app.get("/video", include_in_schema=False)
    async def video() -> StreamingResponse:
        return StreamingResponse(viewer.watch(), media_type="application/octet-stream")

    return app


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("env", choices=ENVS, help="the environment to serve")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()
    uvicorn.run(create_app(ENVS[args.env]), host=args.host, port=args.port)


if __name__ == "__main__":
    main()
