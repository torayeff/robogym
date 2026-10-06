# robogym

Simulated robots for AI agents to control over HTTP.

```bash
uv run python -m robogym.server so101_dual_arm
```

Serves the environment on http://127.0.0.1:8000 (`--host` and `--port` change that), for one agent. The simulation waits at its starting state until `/start`, then runs in real time until `/pause` or `/reset`; the cameras are rendered only when an observation is requested. Interactive API docs are at http://127.0.0.1:8000/docs.

| Method | Path | Does |
| --- | --- | --- |
| GET | `/spec` | what the environment is, and the JSON Schemas of `/observation` and `/action` |
| GET | `/observation` | the environment as it is now |
| POST | `/action` | applies an action; returns at once while a running simulation carries on |
| POST | `/start` | runs the simulation in real time, from where it is |
| POST | `/pause` | freezes simulation time |
| POST | `/reset` | stops the simulation and puts it back to its starting state |

```bash
curl localhost:8000/spec
curl -X POST localhost:8000/start
curl --json '{"joints": {"left_shoulder_lift": -0.75}}' localhost:8000/action
curl localhost:8000/observation
```

An action that doesn't fit `/spec` gets a 422 whose `detail` says what is wrong.

- `robogym/env.py`: the interface every environment implements. `spec()` describes the environment and gives JSON Schemas for what `observe()` returns and `act()` takes; `step()` advances the simulation and `reset()` restarts it.
- `robogym/server.py`: the HTTP API, which runs the environment in real time between `/start` and `/pause`.
- `envs/`: the environments, listed by name in `envs/__init__.py`. `so101_dual_arm/` has two SO-101 arms on a table with a prism and a storage tray.
- `tests/`: run them with `uv run pytest`.
