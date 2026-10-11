# robogym

Simulated robots for AI agents to control over HTTP.

```bash
uv run robogym serve so101_dual_arm
```

Serves the environment on http://127.0.0.1:8000 (`--host` and `--port` change that), for one agent. The simulation waits paused at time 0 until `/resume`, then runs in real time until `/pause` or `/reset`. Physics runs in a process of its own with real-time scheduling, so neither the server nor other programs hold it up, and `/status` reports any step that ran late. Interactive API docs are at http://127.0.0.1:8000/docs.

| Method | Path | Does |
| --- | --- | --- |
| GET | `/spec` | name, description, and the JSON Schemas of `/observation` and `/action` |
| GET | `/observation` | an observation: its time, joint angles and images are all of one instant |
| GET | `/status` | whether the simulation runs, its time, its resets, and how late its steps ran |
| POST | `/action` | applies an action from the next physics step, and returns |
| POST | `/pause` | freezes simulation time |
| POST | `/resume` | runs the simulation in real time from where it is |
| POST | `/reset` | puts everything back to its starting state at time 0, paused |

```bash
curl localhost:8000/spec
curl -X POST localhost:8000/resume
curl --json '{"joints": {"left_shoulder_lift": -0.75}}' localhost:8000/action
curl localhost:8000/observation
```

An action that doesn't fit `/spec` gets a 422 whose `detail` says what is wrong.

Open http://127.0.0.1:8000 in a browser to watch. Browsers decode the video only on pages from localhost or over HTTPS, so to watch from another machine, put the server behind an HTTPS proxy. On macOS, the startup warning that PyAV and OpenCV both bundle FFmpeg's camera classes is harmless. On Linux, physics gets real-time scheduling only with `CAP_SYS_NICE` (in Docker, `--cap-add=SYS_NICE`); without it, it warns and runs at normal priority, with the server below it. On a Linux server without a display, set `MUJOCO_GL=egl` for MuJoCo to render on the GPU, or `MUJOCO_GL=osmesa` without one.

## Serving from a Modal sandbox

```bash
uv run robogym sandbox create so101_dual_arm
uv run robogym sandbox list
uv run robogym sandbox delete <id>
```

`create` serves the environment from a new [Modal](https://modal.com) sandbox, and prints the sandbox's ID and its URL once it serves. The URL is public and HTTPS, so anyone who has it can use the API and watch, from anywhere. A sandbox stops after 30 minutes with nothing connected to its URL, and after 2 hours in any case. The command reads `MODAL_TOKEN_ID` and `MODAL_TOKEN_SECRET` from `.env`.

## Adding an environment

Subclass `robogym.env.Env`, and add its import path to `ENVS` in `robogym/envs/__init__.py`. To serve it from a sandbox, give its package a `Dockerfile` and a `sandbox.toml`. [so101_dual_arm](robogym/envs/so101_dual_arm/) is a complete example, and [ARCHITECTURE.md](ARCHITECTURE.md) explains the design. Run the tests with `uv run pytest`; [tests/test_envs.py](tests/test_envs.py) checks every environment in `ENVS` against the contract.
