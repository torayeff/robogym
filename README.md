# robogym

Simulated robots for AI agents to control over HTTP.

```bash
uv run python -m robogym.server so101_dual_arm
```

Serves the environment on http://127.0.0.1:8000 (`--host` and `--port` change that), for one agent. The simulation starts at once and runs in real time; the cameras are rendered only when an observation is requested. Interactive API docs are at http://127.0.0.1:8000/docs.

| Method | Path | Does |
| --- | --- | --- |
| GET | `/spec` | what the environment is, and the JSON Schemas of `/observation` and `/action` |
| GET | `/observation` | the environment as it is now |
| POST | `/action` | applies an action; returns at once while the simulation carries on |
| POST | `/pause` | freezes simulation time |
| POST | `/resume` | lets it run again from where it was paused |
| POST | `/reset` | puts the whole simulation back to its starting state, paused or not |

```bash
curl localhost:8000/spec
curl --json '{"joints": {"left_shoulder_lift": -0.75}}' localhost:8000/action
curl localhost:8000/observation
```

An action that doesn't fit `/spec` gets a 422 whose `detail` says what is wrong.

- `robogym/env.py`: the interface every environment implements. `spec()` describes the environment and gives JSON Schemas for what `observe()` returns and `act()` takes; `step()` advances the simulation and `reset()` restarts it.
- `robogym/simulation.py`: runs an environment in real time on a thread of its own, which makes every call to it.
- `robogym/server.py`: the HTTP API over a running simulation.
- `envs/`: the environments, listed by name in `envs/__init__.py`. `so101_dual_arm/` has two SO-101 arms on a table with a prism and a storage tray.
- `tests/`: run them with `uv run pytest`.
