"""Serves a robogym environment over HTTP."""

import asyncio
import contextlib
import logging
import os
import signal
import sys
from collections.abc import Callable, Coroutine
from pathlib import Path

import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, StreamingResponse

from robogym.env import Env
from robogym.envs import load
from robogym.viewer import Viewer

VIEWER_PAGE = Path(__file__).with_name("viewer.html")
NICE = 10

logger = logging.getLogger(__name__)


def lower_priority(by: int) -> None:
    """Lower every thread of this process by `by` nice levels."""
    if sys.platform == "linux":
        for thread in map(int, os.listdir("/proc/self/task")):
            nice = os.getpriority(os.PRIO_PROCESS, thread)
            os.setpriority(os.PRIO_PROCESS, thread, nice + by)
    elif hasattr(os, "nice"):
        os.nice(by)


def background(coroutine: Coroutine, name: str) -> asyncio.Task:
    """Run coroutine as a task that logs the exception it fails with."""

    def log_failure(task: asyncio.Task) -> None:
        if not task.cancelled() and (error := task.exception()):
            logger.error("The %s stopped", name, exc_info=error)

    task = asyncio.create_task(coroutine)
    task.add_done_callback(log_failure)
    return task


async def stop_if_dead(env: Env) -> None:
    code = await env.wait()
    logger.critical("The simulation died (exit code %s); stopping", code)
    signal.raise_signal(signal.SIGTERM)


class Server(uvicorn.Server):
    """Ends the /video streams as shutdown starts, which uvicorn would wait on."""

    async def shutdown(self, sockets=None) -> None:
        self.config.app.state.viewer.end_streams()
        await super().shutdown(sockets)


def create_app(make_env: Callable[[], Env], nice: int = 0) -> FastAPI:
    """Serve make_env(), lowering the server's priority by `nice`."""
    env: Env

    @contextlib.asynccontextmanager
    async def lifespan(app: FastAPI):
        nonlocal env
        with make_env() as env:
            lower_priority(nice)
            app.state.viewer = Viewer(env.view)
            tasks = [
                background(app.state.viewer.run(), "viewer"),
                background(stop_if_dead(env), "watch on the simulation"),
            ]
            yield
            for task in tasks:
                task.cancel()

    app = FastAPI(title="robogym", lifespan=lifespan)

    @app.get("/spec")
    async def spec() -> dict:
        """Name, description, and the JSON Schemas of /observation and /action."""
        return env.spec()

    @app.get("/observation")
    async def observe() -> dict:
        """An observation of one instant, in the format /spec gives."""
        return await env.observe()

    @app.get("/status")
    async def status() -> dict:
        """Whether the simulation runs, its time and resets, and how late steps ran."""
        return await env.status()

    @app.post("/action", status_code=204)
    async def act(action: dict) -> None:
        """Apply an action in the format /spec gives, without waiting for its effect."""
        try:
            await env.act(action)
        except ValueError as error:
            raise HTTPException(422, str(error)) from error

    @app.post("/pause", status_code=204)
    async def pause() -> None:
        """Freeze simulation time."""
        await env.pause()

    @app.post("/resume", status_code=204)
    async def resume() -> None:
        """Run the simulation in real time from where it is."""
        await env.resume()

    @app.post("/reset", status_code=204)
    async def reset() -> None:
        """Put everything back to its starting state at time 0, paused."""
        await env.reset()

    @app.get("/", include_in_schema=False)
    async def page() -> FileResponse:
        return FileResponse(VIEWER_PAGE)

    @app.get("/video", include_in_schema=False)
    async def video() -> StreamingResponse:
        return StreamingResponse(
            app.state.viewer.watch(), media_type="application/octet-stream"
        )

    return app


def serve(env_name: str, host: str, port: int) -> None:
    app = create_app(load(env_name), NICE)
    server = Server(uvicorn.Config(app, host=host, port=port))
    with contextlib.suppress(KeyboardInterrupt):
        server.run()
