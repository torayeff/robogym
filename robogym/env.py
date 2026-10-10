"""The interface between the server and the environments it serves."""

import abc

import numpy as np

VIEW_WIDTH, VIEW_HEIGHT = 1280, 720


class Env(abc.ABC):
    """A simulation that runs in real time between resume() and pause() or reset().

    It starts paused at time 0. Its methods may be called concurrently, and act()
    never waits for drawing.
    """

    @abc.abstractmethod
    def spec(self) -> dict:
        """Its "name", "description", and the JSON Schemas "observation" and "action"."""

    @abc.abstractmethod
    async def observe(self) -> dict:
        """An observation of one instant, in the spec's format."""

    @abc.abstractmethod
    async def act(self, action: dict) -> None:
        """Apply an action in the spec's format, or raise ValueError and change nothing."""

    @abc.abstractmethod
    async def pause(self) -> None:
        """Freeze simulation time."""

    @abc.abstractmethod
    async def resume(self) -> None:
        """Run in real time from where it is."""

    @abc.abstractmethod
    async def reset(self) -> None:
        """Go back to the starting state at time 0, paused."""

    @abc.abstractmethod
    async def view(self) -> np.ndarray | None:
        """The default view as a (VIEW_HEIGHT, VIEW_WIDTH, 3) uint8 image, or None.

        None means there's no new view to show.
        """

    @abc.abstractmethod
    async def status(self) -> dict:
        """{"running", "time", "resets", "late_steps", "max_lateness"}.

        late_steps counts the steps that ran a whole step late; max_lateness is in
        seconds.
        """

    @abc.abstractmethod
    async def wait(self) -> int:
        """The exit code, once the simulation has died."""

    @abc.abstractmethod
    def close(self) -> None:
        """Stop the simulation and release what it holds."""

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
