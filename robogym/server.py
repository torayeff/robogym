"""Serves an environment over HTTP, running it in real time from the start.

Run it from the repository root:

    uv run python -m robogym.server so101_dual_arm
"""

import argparse
import contextlib
import importlib
from collections.abc import Callable

import uvicorn
from fastapi import FastAPI, HTTPException

from envs import ENVS
from robogym.env import Env
from robogym.simulation import Simulation


def create_app(make_env: Callable[[], Env]) -> FastAPI:
    simulation = Simulation(make_env)

    @contextlib.asynccontextmanager
    async def lifespan(app: FastAPI):
        simulation.start()
        yield
        simulation.stop()

    app = FastAPI(title="robogym", lifespan=lifespan)

    @app.get("/spec")
    def spec() -> dict:
        """What the environment is, and the formats of /observation and /action."""
        return simulation.call(lambda env: env.spec())

    @app.get("/observation")
    def observe() -> dict:
        """The environment as it is now, in the format /spec gives."""
        return simulation.call(lambda env: env.observe())

    @app.post("/action", status_code=204)
    def act(action: dict) -> None:
        """Apply an action in the format /spec gives; the simulation carries on."""
        try:
            simulation.call(lambda env: env.act(action))
        except ValueError as error:
            raise HTTPException(422, str(error)) from error

    @app.post("/pause", status_code=204)
    def pause() -> None:
        """Freeze simulation time."""
        simulation.pause()

    @app.post("/resume", status_code=204)
    def resume() -> None:
        """Let simulation time run again, from where it was paused."""
        simulation.resume()

    @app.post("/reset", status_code=204)
    def reset() -> None:
        """Put the whole simulation back to its starting state, paused or not."""
        simulation.reset()

    return app


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("env", choices=ENVS, help="the environment to serve")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()
    module, _, name = ENVS[args.env].partition(":")
    env = getattr(importlib.import_module(module), name)
    uvicorn.run(create_app(env), host=args.host, port=args.port)


if __name__ == "__main__":
    main()
