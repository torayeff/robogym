"""Serves a robogym environment over HTTP, running it in real time from /start.

    uv run robogym so101_dual_arm

The environment runs in a process of its own (robogym/simulation.py), so nothing the
server does can hold it up. Observations and the viewer at / are rendered from its
states on threads of their own.
"""

import argparse
import asyncio
import concurrent.futures
import contextlib
import logging
from collections.abc import Callable, Coroutine
from pathlib import Path

import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, StreamingResponse

from robogym.env import Env, Observer
from robogym.envs import ENVS
from robogym.simulation import Simulation
from robogym.viewer import Viewer

VIEWER_PAGE = Path(__file__).with_name("viewer.html")

logger = logging.getLogger(__name__)


def background(coroutine: Coroutine, name: str) -> asyncio.Task:
    """Run coroutine as a task that logs the exception it fails with."""

    def log_failure(task: asyncio.Task) -> None:
        if not task.cancelled() and (error := task.exception()):
            logger.error("The %s stopped", name, exc_info=error)

    task = asyncio.create_task(coroutine)
    task.add_done_callback(log_failure)
    return task


def create_app(make_env: Callable[[], Env]) -> FastAPI:
    """Serve make_env's environment; make_env must be picklable, such as an Env class."""
    env: Env
    simulation: Simulation
    viewer: Viewer
    observer: Observer
    # Makes every observation, so it owns the observer's OpenGL context.
    observing = concurrent.futures.ThreadPoolExecutor(max_workers=1)

    @contextlib.asynccontextmanager
    async def lifespan(app: FastAPI):
        nonlocal env, simulation, viewer, observer
        with (
            make_env() as env,
            Simulation(make_env) as simulation,
            Viewer(env, simulation.state) as viewer,
        ):
            observer = env.observer()
            streaming = background(viewer.run(), "viewer")
            yield
            streaming.cancel()
            observing.submit(observer.close)
            observing.shutdown()

    app = FastAPI(title="robogym", lifespan=lifespan)

    @app.get("/spec")
    async def spec() -> dict:
        """Name, description, and the JSON Schemas of /observation and /action."""
        return env.spec()

    @app.get("/observation")
    async def observe() -> dict:
        """The observation as the request arrives, in the format /spec gives."""
        state = await simulation.state()
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(observing, observer.observe, state)

    @app.post("/action", status_code=204)
    async def act(action: dict) -> None:
        """Apply an action in the format /spec gives, without waiting for its effect."""
        try:
            await simulation.call("act", action)
        except ValueError as error:
            raise HTTPException(422, str(error)) from error

    @app.post("/start", status_code=204)
    async def start() -> None:
        """Run the simulation in real time, from where it is."""
        await simulation.call("start")

    @app.post("/pause", status_code=204)
    async def pause() -> None:
        """Freeze simulation time; /start resumes from there."""
        await simulation.call("pause")

    @app.post("/reset", status_code=204)
    async def reset() -> None:
        """Stop the simulation and put it back to its starting state."""
        await simulation.call("reset")

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
