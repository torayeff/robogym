"""For engines that robogym steps itself: the Physics they implement, and the
PhysicsProcess that steps one in real time, in a process of its own."""

import abc
import asyncio
import concurrent.futures
import ctypes
import ctypes.util
import gc
import logging
import multiprocessing
import os
import signal
import sys
import time
from collections.abc import Callable
from multiprocessing.connection import Connection

import numpy as np

# A longer stall, such as the computer sleeping, is skipped rather than caught up.
MAX_LAG = 0.5
THREAD_TIME_CONSTRAINT_POLICY = 2

logger = logging.getLogger(__name__)


class Physics(abc.ABC):
    """A simulation for PhysicsProcess to step. Physics waits for every call."""

    @property
    @abc.abstractmethod
    def time(self) -> float:
        """Simulation seconds since the last reset."""

    @property
    @abc.abstractmethod
    def timestep(self) -> float:
        """Simulation seconds per step."""

    @abc.abstractmethod
    def step(self, steps: int) -> None:
        """Advance by `steps` steps, or raise if the simulation breaks down."""

    @abc.abstractmethod
    def act(self, action: dict) -> None:
        """Apply an action, or raise ValueError and change nothing."""

    @abc.abstractmethod
    def state(self) -> np.ndarray:
        """Everything that drawing the simulation as it is now depends on."""

    @abc.abstractmethod
    def reset(self) -> None:
        """Go back to the starting state at time 0."""


class PhysicsProcess:
    """Steps make_physics() in real time between "resume" and "pause" or "reset".

    make_physics must be picklable, such as a Physics class.
    """

    def __init__(self, make_physics: Callable[[], Physics]):
        context = multiprocessing.get_context("spawn")
        self._connection, connection = context.Pipe()
        self._process = context.Process(
            target=_run, args=(make_physics, connection), daemon=True
        )
        self._process.start()
        connection.close()
        self._connection.recv()
        self._thread = concurrent.futures.ThreadPoolExecutor(max_workers=1)

    async def call(self, command: str, argument=None):
        """Have physics act, pause, resume, reset, or give its state or status."""
        loop = asyncio.get_running_loop()
        return await loop.run_in_executor(self._thread, self._call, command, argument)

    async def wait(self) -> int:
        """The exit code, once the process has ended."""
        while self._process.is_alive():
            await asyncio.sleep(0.1)
        return self._process.exitcode

    def close(self) -> None:
        self._thread.shutdown()
        if self._process.is_alive():
            self._connection.send(("stop", None))
        self._process.join()

    def _call(self, command: str, argument):
        self._connection.send((command, argument))
        reply = self._connection.recv()
        if isinstance(reply, Exception):
            raise reply
        return reply


def _run(make_physics: Callable[[], Physics], connection: Connection) -> None:
    signal.signal(signal.SIGINT, signal.SIG_IGN)  # the server stops us
    physics = make_physics()
    _raise_priority(physics.timestep)
    connection.send(None)
    gc.freeze()
    running, origin, resets = False, 0.0, 0
    late_steps, max_lateness, warned = 0, 0.0, 0.0
    while True:
        if running and not connection.poll():
            due = origin + physics.time + physics.timestep
            time.sleep(max(0.0, due - time.monotonic()))
            now = time.monotonic()
            max_lateness = max(max_lateness, now - due)
            if now - due > physics.timestep:
                late_steps += 1
                if now - warned > 1:
                    logger.warning("A physics step ran %.1f ms late", 1e3 * (now - due))
                    warned = now
            lag = now - origin - physics.time
            if lag > MAX_LAG:
                origin += lag
                continue
            try:
                physics.step(int(lag // physics.timestep))
            except Exception:
                logger.exception("The simulation broke down, so it was reset")
                physics.reset()
                running, resets = False, resets + 1
            continue
        command, argument = connection.recv()
        reply = None
        try:
            match command:
                case "act":
                    physics.act(argument)
                case "state":
                    reply = physics.state()
                case "status":
                    reply = {
                        "running": running,
                        "time": physics.time,
                        "resets": resets,
                        "late_steps": late_steps,
                        "max_lateness": max_lateness,
                    }
                case "pause":
                    running = False
                case "resume" if not running:
                    origin = time.monotonic() - physics.time
                    running = True
                case "reset":
                    physics.reset()
                    running, resets = False, resets + 1
                case "stop":
                    return
        except ValueError as error:
            reply = error
        except Exception as error:
            logger.exception("The simulation failed to %s", command)
            reply = error
        connection.send(reply)


def _raise_priority(period: float) -> None:
    """Have the OS run this thread ahead of all others, where it allows that."""
    if sys.platform == "darwin":
        _time_constraint(period)
    elif hasattr(os, "sched_setscheduler"):
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(1))
        except PermissionError:
            logger.warning("Physics runs at normal priority: no CAP_SYS_NICE")


class _TimeConstraintPolicy(ctypes.Structure):
    _fields_ = [
        ("period", ctypes.c_uint32),
        ("computation", ctypes.c_uint32),
        ("constraint", ctypes.c_uint32),
        ("preemptible", ctypes.c_int),
    ]


def _time_constraint(period: float) -> None:
    """macOS's real-time scheduling, which needs no privileges."""
    libc = ctypes.CDLL(ctypes.util.find_library("c"))
    timebase = (ctypes.c_uint32 * 2)()
    libc.mach_timebase_info(timebase)
    ticks = period * 1e9 * timebase[1] / timebase[0]
    policy = _TimeConstraintPolicy(round(ticks), round(ticks / 2), round(ticks), 1)
    libc.mach_thread_self.restype = ctypes.c_uint32
    thread = libc.mach_thread_self()
    policy_set = libc.thread_policy_set
    if policy_set(thread, THREAD_TIME_CONSTRAINT_POLICY, ctypes.byref(policy), 4):
        logger.warning("Physics runs at normal priority: macOS refused real time")
