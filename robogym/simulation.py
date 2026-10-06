"""Runs an environment in real time, on a thread of its own."""

import concurrent.futures
import queue
import threading
import time
from collections.abc import Callable

from robogym.env import Env

# How often the thread wakes to step the simulation while no calls come in.
TICK = 0.01
# A longer stall, such as the computer sleeping, is skipped rather than simulated.
MAX_LAG = 0.5

_STOP = object()


class Simulation:
    """Runs an environment in real time, on a thread of its own.

    The thread creates the environment, so it owns any OpenGL context, and runs every
    call to it, one at a time and in order. Before each call it brings the simulation
    up to the present, so the call sees the environment as it is now.
    """

    def __init__(self, make_env: Callable[[], Env]):
        self._make_env = make_env
        self._calls = queue.SimpleQueue()
        self._thread = None
        self._paused = False
        self._origin = 0.0  # the wall-clock time at which simulation time was 0

    def start(self) -> None:
        """Create the environment and start its clock.

        Raises whatever creating the environment raises.
        """
        started = concurrent.futures.Future()
        self._thread = threading.Thread(target=self._run, args=(started,), daemon=True)
        self._thread.start()
        started.result()

    def stop(self) -> None:
        """Stop the clock and close the environment."""
        self._calls.put(_STOP)
        self._thread.join()

    def call[T](self, fn: Callable[[Env], T]) -> T:
        """Run fn(env) on the simulation thread and return what it returns."""
        done = concurrent.futures.Future()
        self._calls.put((fn, done))
        return done.result()

    def pause(self) -> None:
        """Freeze simulation time."""
        self.call(self._pause)

    def resume(self) -> None:
        """Let simulation time run again, from where it was paused."""
        self.call(self._resume)

    def reset(self) -> None:
        """Reset the environment and restart its clock from 0, paused or not."""
        self.call(self._reset)

    def _run(self, started: concurrent.futures.Future) -> None:
        try:
            env = self._make_env()
        except Exception as error:
            started.set_exception(error)
            return
        with env:
            self._sync(env)
            started.set_result(None)
            while (call := self._next_call()) is not _STOP:
                if not self._paused:
                    self._catch_up(env)
                if call is not None:
                    fn, done = call
                    try:
                        done.set_result(fn(env))
                    except Exception as error:
                        done.set_exception(error)

    def _next_call(self):
        """The next call, or None if none came within a tick while running."""
        try:
            return self._calls.get(timeout=None if self._paused else TICK)
        except queue.Empty:
            return None

    def _catch_up(self, env: Env) -> None:
        lag = time.monotonic() - self._origin - env.time
        if lag > MAX_LAG:
            self._sync(env)
        else:
            env.step(lag)

    def _sync(self, env: Env) -> None:
        """Line the clock up with the simulation as it is now."""
        self._origin = time.monotonic() - env.time

    def _pause(self, env: Env) -> None:
        self._paused = True

    def _resume(self, env: Env) -> None:
        if self._paused:
            self._paused = False
            self._sync(env)

    def _reset(self, env: Env) -> None:
        env.reset()
        self._sync(env)
