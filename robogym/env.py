"""The interface every robogym environment implements.

The server makes each environment twice. One runs in a process of its own, which steps
it in real time and calls act(), reset() and state() as requests arrive. The other, in
the server's process, gives spec() and makes observations and the default view from
those states.
"""

import abc

import numpy as np


class Env(abc.ABC):
    """A simulation that the server runs and exposes over HTTP.

    Not thread-safe: call every method from the same thread.
    """

    @abc.abstractmethod
    def spec(self) -> dict:
        """Name, description, and the JSON Schemas of observations and of act().

        Keys: "name", "description", "observation" and "action".
        """

    @property
    @abc.abstractmethod
    def time(self) -> float:
        """Simulation time in seconds since the last reset."""

    @property
    @abc.abstractmethod
    def timestep(self) -> float:
        """The simulation seconds in one physics step."""

    @abc.abstractmethod
    def step(self, seconds: float) -> None:
        """Advance the simulation by `seconds`, rounded to whole physics steps."""

    @abc.abstractmethod
    def act(self, action: dict) -> None:
        """Apply an action in the spec's "action" format.

        Raises ValueError, and changes nothing, if the action doesn't fit the format.
        """

    @abc.abstractmethod
    def state(self) -> np.ndarray:
        """A copy of the whole simulation state, which observations are made from."""

    @abc.abstractmethod
    def reset(self) -> None:
        """Put the whole simulation back to its starting state."""

    @abc.abstractmethod
    def positions(self, state: np.ndarray) -> np.ndarray:
        """Everything in `state` the default view depends on, in metres and radians."""

    @abc.abstractmethod
    def observer(self) -> "Observer":
        """An Observer of this environment."""

    @abc.abstractmethod
    def view_renderer(self, width: int, height: int) -> "ViewRenderer":
        """A renderer of the environment's default view, width x height pixels."""

    def close(self) -> None:
        """Release resources."""

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class Observer(abc.ABC):
    """Makes an environment's observations from its states.

    Created on the environment's thread; observe() and close() then run on a single
    render thread, which owns any OpenGL context.
    """

    @abc.abstractmethod
    def observe(self, state: np.ndarray) -> dict:
        """The observation at `state`, in the spec's "observation" format."""

    def close(self) -> None:
        """Release resources such as the OpenGL context."""


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
