"""The interface every robogym environment implements.

The server runs an environment in real time: it calls step() to keep simulation time
in line with the wall clock, and act(), observe() and reset() as requests arrive.
spec() tells clients what observe() returns and what act() takes.
"""

import abc


class Env(abc.ABC):
    """A simulation that the server runs and exposes over HTTP.

    Not thread-safe: call every method from the same thread, which also owns any
    renderer's OpenGL context.
    """

    @abc.abstractmethod
    def spec(self) -> dict:
        """What the environment is, what observe() returns and what act() takes.

        Keys: "name", "description", and the JSON Schemas "observation" and "action".
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

    def close(self) -> None:
        """Release resources such as renderers."""

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
