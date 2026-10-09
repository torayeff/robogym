"""Runs an environment in real time, in a process that does nothing else."""

import asyncio
import concurrent.futures
import gc
import logging
import multiprocessing
import signal
import time
from collections.abc import Callable

import numpy as np

from robogym.env import Env

# A longer stall, such as the computer sleeping, is skipped rather than simulated.
MAX_LAG = 0.5
# A step this much later than it fell due is logged, at most once a second.
WARN_LATE = 0.005

logger = logging.getLogger(__name__)


class Simulation:
    """Runs make_env's environment in real time, in a process of its own.

    make_env must be picklable, such as an Env class.
    """

    def __init__(self, make_env: Callable[[], Env]):
        context = multiprocessing.get_context("spawn")
        self._connection, connection = context.Pipe()
        self._process = context.Process(
            target=_run, args=(make_env, connection), daemon=True
        )
        self._process.start()
        connection.close()
        self._connection.recv()  # started
        self._thread = concurrent.futures.ThreadPoolExecutor(max_workers=1)

    async def state(self) -> np.ndarray:
        """The environment's state() now, to within a physics step."""
        return await self.call("state")

    async def call(self, command: str, argument=None):
        """Have the process act, start, pause, reset or give its state."""
        loop = asyncio.get_running_loop()
        reply = await loop.run_in_executor(self._thread, self._call, command, argument)
        if isinstance(reply, Exception):
            raise reply
        return reply

    def close(self) -> None:
        self._connection.send(("stop", None))
        self._process.join()
        self._thread.shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _call(self, command: str, argument):
        self._connection.send((command, argument))
        return self._connection.recv()


def _run(make_env: Callable[[], Env], connection) -> None:
    """Step the environment in real time, and carry out commands between steps."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)  # on Ctrl+C, the server stops us
    with make_env() as env:
        connection.send(None)
        gc.freeze()  # keeps garbage collections, which would hold steps up, short
        running = False
        origin = 0.0  # the wall-clock time at which env.time was 0
        warned = 0.0
        while True:
            if running and not connection.poll():
                due = origin + env.time + env.timestep
                time.sleep(max(0.0, due - time.monotonic()))
                now = time.monotonic()
                if now - due > WARN_LATE and now - warned > 1:
                    logger.warning(
                        "The simulation fell %.1f ms behind the wall clock",
                        1e3 * (now - due),
                    )
                    warned = now
                lag = now - origin - env.time
                if lag > MAX_LAG:
                    origin += lag
                    continue
                try:
                    env.step(lag // env.timestep * env.timestep)
                except Exception:
                    logger.exception("The simulation stopped")
                    running = False
                continue
            command, argument = connection.recv()
            if command == "stop":
                return
            reply = None
            try:
                match command:
                    case "state":
                        reply = env.state()
                    case "act":
                        env.act(argument)
                    case "start" if not running:
                        origin = time.monotonic() - env.time
                        running = True
                    case "pause":
                        running = False
                    case "reset":
                        running = False
                        env.reset()
            except ValueError as error:  # a bad action
                reply = error
            except Exception as error:
                logger.exception("The simulation failed to %s", command)
                reply = error
            connection.send(reply)
