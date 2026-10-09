# robogym

Simulated robots for AI agents to control over HTTP.

```bash
uv run robogym so101_dual_arm
```

Serves the environment on http://127.0.0.1:8000 (`--host` and `--port` change that), for one agent. The simulation waits at its starting state until `/start`, then runs in real time until `/pause` or `/reset`. It runs in a process of its own that does nothing else, so serving requests and rendering never hold it up; if the computer itself does, it logs how far behind the wall clock it fell. Interactive API docs are at http://127.0.0.1:8000/docs.

| Method | Path | Does |
| --- | --- | --- |
| GET | `/spec` | name, description, and the JSON Schemas of `/observation` and `/action` |
| GET | `/observation` | the observation as the request arrives: its time, joint angles and images are all of that moment |
| POST | `/action` | applies an action before the next physics step and returns; a running simulation carries on |
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

Open http://127.0.0.1:8000 in a browser to watch. Browsers decode the video only on pages from localhost or over HTTPS, so to watch from another machine, put the server behind an HTTPS proxy. On macOS, the startup warning that PyAV and OpenCV both bundle FFmpeg's camera classes is harmless. On a Linux server without a display, set `MUJOCO_GL=egl` for MuJoCo to render on the GPU, or `MUJOCO_GL=osmesa` without one.

## Adding an environment

Subclass `robogym.env.Env` and add it to `ENVS` in `robogym/envs/__init__.py`; [so101_dual_arm](robogym/envs/so101_dual_arm/) is a complete example. The server makes it twice, once in the simulation's process; [robogym/env.py](robogym/env.py) says what each does. Run the tests with `uv run pytest`.
