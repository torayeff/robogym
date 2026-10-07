"""The interface every robogym environment implements.

The server runs an environment in real time: it calls step() to keep simulation time
in line with the wall clock, and act(), observe() and reset() as requests arrive.
spec() tells clients what observe() returns and what act() takes. The browser viewer
renders positions() with a view_renderer() on its own thread.
"""

import abc

import numpy as np


class Env(abc.ABC):
    """A simulation that the server runs and exposes over HTTP.

    Not thread-safe: call every method from the same thread, which also owns the
    OpenGL context of any renderer observe() uses.
    """

    @abc.abstractmethod
    def spec(self) -> dict:
        """Name, description, and the JSON Schemas of observe() and act().

        Keys: "name", "description", "observation" and "action".
        """

    @property
    @abc.abstractmethod
    def time(self) -> float:
        """Simulation time in seconds since the last reset."""

    @abc.abstractmethod
    def step(self, seconds: float) -> None:
        """Advance the simulation by `seconds`, rounded to whole physics steps."""

    @abc.abstractmethod
    def act(self, action: dict) -> None:
        """Apply an action in the spec's "action" format.

        Raises ValueError, and changes nothing, if the action doesn't fit the format.
        """

    @abc.abstractmethod
    def observe(self) -> dict:
        """The current observation, in the spec's "observation" format."""

    @abc.abstractmethod
    def reset(self) -> None:
        """Put the whole simulation back to its starting state."""

    @abc.abstractmethod
    def positions(self) -> np.ndarray:
        """A copy of everything the default view depends on, in metres and radians."""

    @abc.abstractmethod
    def view_renderer(self, width: int, height: int) -> "ViewRenderer":
        """A renderer of the environment's default view, width x height pixels."""

    def close(self) -> None:
        """Release resources such as renderers."""

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class ViewRenderer(abc.ABC):
    """Draws an environment's default view from its positions().

    Created on the environment's thread; render() and close() then run on a single
    render thread, which owns the OpenGL context.
    """

    @abc.abstractmethod
    def render(self, positions: np.ndarray) -> np.ndarray:
        """The view at `positions`, as a (height, width, 3) uint8 RGB image."""

    def close(self) -> None:
        """Release resources such as the OpenGL context."""
