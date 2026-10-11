# Architecture

robogym serves a simulated robot to an AI agent over HTTP, and the scene to people watching in a browser. Today there is one environment, two SO-101 arms at a table simulated with MuJoCo, but the design is meant to take others, Isaac Sim included.

The main requirement is that nothing delays physics while the simulation runs. The SO-101 scene steps every 2.08 ms, and each simulated second must take one real second, however busy the server is. Drawing one observation takes about 33 ms, and Python runs one thread at a time, so physics runs in a process of its own. [Why it's built this way](#why-its-built-this-way) gives the measurements behind each choice.

## Processes and threads

```
              agent                         browsers
                ▲ │ HTTP                      ▲ H.264 on /video
                │ ▼                           │
┌─────────────────── server process ────────────────────┐
│ event loop      HTTP routes, the viewer's loop        │
│ pipe thread     one command at a time, to physics     │
│ render thread   the one GL context: observations      │
│                 first, then the browser view          │
└───────────────────────────┬───────────────────────────┘
                            │ pipe: commands and states
┌───────────────────────────┴───────────────────────────┐
│ physics process · real-time scheduling                │
│ steps on the wall clock; runs commands between steps  │
└───────────────────────────────────────────────────────┘
```

The server process runs the HTTP API, draws images and streams video. The physics process steps physics on the wall clock and, between two steps, carries out short commands from the server: apply an action, hand over its state, pause, resume or reset.

Inside the server, three threads matter. The event loop handles HTTP and never blocks. The pipe thread sends one command at a time to physics and waits for the reply, so that the event loop doesn't have to. The render thread owns the server's only OpenGL context, which works only on the thread that made it, and does all the drawing.

## The contract

The server knows nothing about MuJoCo or any other engine. It talks to one object, an `Env`, whose methods mirror the HTTP routes; [robogym/env.py](robogym/env.py) defines them. The contract says what an environment does, not how, so it fits two kinds of engine:

- Engines that can be stepped from Python, such as MuJoCo, implement a `Physics` and run it in a `PhysicsProcess`, described next. Their state is a short list of numbers, from which the server draws.
- Engines that run their own loop and draw in their own process, such as Isaac Sim or Gazebo, run in a process of their own, and their `Env` is a client to it. By default, Isaac Sim takes several physics steps per rendered frame, so an action would take effect at the next frame, 17–33 ms later, rather than at the next step. That needs measuring before relying on it.

## Keeping physics on the wall clock

`PhysicsProcess` spawns a fresh Python process. The process builds the environment's `Physics`, asks the operating system to run it ahead of everything else, and tells the server it's ready. On macOS it asks for the time-constraint policy, which needs no privileges; on Linux, for `SCHED_FIFO`, which needs `CAP_SYS_NICE`. If the OS refuses, it logs a warning and runs at normal priority, still ahead of the server, which lowers its own. It then takes everything made so far out of the garbage collector's view, which keeps collections short, and starts its loop.

The loop keeps the wall-clock moment at which simulation time was 0. The next step falls due at that moment, plus the simulation time, plus one step, so a step that wakes a little late doesn't push back the ones after it. The loop sleeps until then. If it wakes late, it takes every whole step that has fallen due at once, and simulation time catches up with the wall clock. If it's more than half a second behind, because the laptop slept, say, it skips the gap rather than replay it. Each step more than a whole step late is counted, `status()` reports the count and the worst lateness, and a warning is logged at most once a second.

Between two steps, the loop carries out the next command waiting in the pipe. Physics waits for each one, so every command must be quick, and the environment checks actions before sending them. `resume()` moves the wall-clock moment so that time carries on from where it stopped. If a step fails, or the simulation diverges, the loop resets the simulation, pauses it, and counts the reset in `status()`. If the whole process dies, the server notices within a tenth of a second and shuts down, so that a supervisor such as systemd can restart it.

## The SO-101 environment

The scene is `scene.xml`, with the robot it attaches from `robot/`: a rig with an overhead camera, two arms with a wrist camera each, a table, a block and a tray. Each arm has six servos, which MuJoCo models as position actuators. The scene is compiled twice, and neither compile changes the mesh files:

- `So101DualArmPhysics`, in the physics process, leaves out what only drawing uses: the detailed meshes, which collide with nothing.
- `So101DualArm`, the environment's `Env`, compiles the whole scene in the server. `<attach>` copies a model's meshes for each prefix, so this compile points the two arms and three cameras at one copy of each mesh.

Only the state crosses from physics to drawing, so anything a reset changed outside the state, such as an object's colour, would never reach the pictures. A reset folds the arms and draws a new layout for the block and the tray; the environment's [README](robogym/envs/so101_dual_arm/README.md) explains the layouts and the cameras' lens.

The spec comes from the compiled model: the joint names and goal ranges, the objects' sizes and the camera's height. So the description an agent reads matches the scene, and `act()` checks actions against the same ranges.

## Following a request

**An observation.** `GET /observation` calls `observe()` on the event loop. The pipe thread sends "state" to the physics process, which answers between two steps. The render thread loads the state into the whole scene, draws the three robot cameras through their lens and encodes them as JPEG, while physics carries on stepping. The reply holds the time, the joint angles and the images, all of the moment the request arrived. It takes about 35 ms, of which physics spends a few tens of microseconds.

**An action.** `POST /action` calls `act()`, which checks the action in the server. If it's wrong, the route answers 422 with what is wrong, and nothing reaches physics. Otherwise the action goes down the same pipe, physics sets the goals between two steps, and the route answers 204. The request waits neither for the arm to move nor for any drawing: the simulated servos move the joints over the following steps, as real servos would. `/pause`, `/resume` and `/reset` take the same path, and `/status` reads the loop's counters.

**Watching.** A browser loads the viewer page, which opens `/video`, a response that never ends. While anyone watches, the viewer asks the environment for its view up to 30 times a second. The environment draws one only when the state has changed and no observation is in progress, so observations go first, and frames stop while the simulation is paused, apart from one after each action, since servo goals are part of the state. Each new view is encoded once as H.264, and the same bytes go to every browser. A browser that joins late first gets the frames since the last keyframe, so it can start decoding at once, and one that falls too far behind drops its backlog and resumes at the next keyframe. The page decodes the frames with the browser's WebCodecs.

## Starting and stopping

`uv run robogym serve so101_dual_arm` runs the command in `robogym/__main__.py`, which imports the server only after reading its arguments. As the server starts, it makes the environment, which spawns physics, compiles the whole scene and starts the render thread. Then the server lowers its own priority, and starts the viewer's loop and a watch on the physics process.

Ctrl+C makes uvicorn shut down. First the server ends the video streams, since uvicorn would otherwise wait for the browsers to close them. Then the viewer's loop stops, the render thread frees the OpenGL context, and the server stops the physics process, which ignores Ctrl+C itself.

## Serving from a sandbox

`robogym sandbox create <env>`, in `robogym/sandbox.py`, runs `robogym serve <env>` in a Modal sandbox of its own, and Modal gives its port a public HTTPS URL. The environment's package says what the sandbox needs, in two files:

- `Dockerfile` makes the image. Modal builds it with the repository as its context, and uploads only the files it copies, less those in `.dockerignore`.
- `sandbox.toml` gives the `vcpus`, the `memory_gib` and, if it draws on a GPU, the Modal `gpu` type.

Modal runs a sandbox either under gVisor or in a VM. gVisor, which stands in for the Linux kernel, refuses real-time scheduling; a VM allows it but has no GPUs. So a sandbox without a GPU runs in a VM, and one with a GPU runs under gVisor, where physics runs at normal priority. The simulation lives only in the sandbox's memory, so it's gone when the sandbox stops.

## Why it's built this way

Measured on an M2 Mac, and the last two rows in Modal sandboxes with 4 vCPUs:

| Choice | Rather than | Because |
| --- | --- | --- |
| physics in a process of its own | a thread, or the event loop | within one process, the GIL and the event loop delayed steps by 3–118 ms |
| spawning that process | forking the server | forking a process that already runs threads and OpenGL is unsafe |
| real-time scheduling | lowering the server's priority alone | that puts physics ahead of the server only: with every core busy, steps ran up to 12 ms late, and under real-time scheduling at most 0.1 ms |
| sleeping until a step is due | waiting on the pipe with a timeout | the pipe's timeout rounds up to whole milliseconds, and steps are 2.08 ms apart |
| a contract of what an environment does | a contract of stepping here and drawing there | an engine that draws in its own process, such as Isaac Sim, would need a second copy of itself just to draw |
| drawing in the server, from states | drawing in the physics process | drawing takes milliseconds that physics can't spare |
| states through the pipe | shared memory | a state of 237 numbers crosses in about 10 µs, and even 256 KB would take 0.2 ms |
| checking actions in the server | checking them in the physics process | physics unpickles whatever crosses the pipe between two steps, and a 2 MB action held it up 12 ms |
| the physics scene without its visual-only parts | the whole scene in both processes | the physics process holds a 7 MB model rather than 65 MB |
| meshes shared across `<attach>` prefixes | MuJoCo's copy of each mesh per prefix | it halves the drawing model, from 125 to 65 MB, without touching the mesh files |
| one GL context | one per renderer | a context costs about 150 MB on macOS before it holds a scene, so one saves about 300 MB, and an observation waits at most for one view frame, about 9 ms |
| H.264, encoded once for every browser | MJPEG, or 3D in the browser | it shows anything the simulation can draw, costs about 0.6 cores whether 1 or 300 browsers watch, and needs far less bandwidth than MJPEG |
| the server ending the video streams as it shuts down | uvicorn's graceful-shutdown timeout | the timeout cancels the streams and logs a traceback |
| the server imported inside the command's `main()` | at the top of its module | the spawned physics process re-imports that module, and would load FastAPI, PyAV and OpenCV |
| a T4 for so101_dual_arm's sandbox | drawing on its CPUs, with Mesa | an observation took 244 ms rather than 25, the view ran at 7 frames a second rather than 29, and with the CPUs free, every step ran on time without real-time scheduling |
| a VM for a sandbox without a GPU | gVisor, Modal's default | under gVisor, without real-time scheduling, steps ran up to 18 ms late while drawing used the CPUs; in a VM, none did after its first 30 s |

## Adding an environment

1. Make a package in `robogym/envs/` with a subclass of `Env`, and add `"<name>": "robogym.envs.<name>.env:<Class>"` to `ENVS` in `robogym/envs/__init__.py`. Only the environment being served is imported, so different engines' dependencies never meet.
2. If robogym is to step the engine, subclass `Physics` and run it in a `PhysicsProcess`, as so101_dual_arm does. The physics process imports the class by name, so it must be importable, and its module mustn't load what only drawing needs. Its methods run between steps, so they must be quick, and the `Env` checks actions before they reach it. Its state must hold everything that drawing depends on.
3. To serve it from a sandbox, give its package a `Dockerfile` and a `sandbox.toml`, as [Serving from a sandbox](#serving-from-a-sandbox) describes.
4. `tests/test_envs.py` checks every environment in `ENVS` against the contract. Add tests for what is particular to yours.

## Where things are

| Path | What it is |
| --- | --- |
| `robogym/__main__.py` | the `robogym` command |
| `robogym/server.py` | the HTTP API, and the wiring of the server's parts |
| `robogym/env.py` | `Env`, the contract between the server and an environment |
| `robogym/physics.py` | `Physics` and `PhysicsProcess`, for engines that robogym steps itself |
| `robogym/viewer.py`, `robogym/viewer.html` | the browser view, as H.264, and the page that shows it |
| `robogym/sandbox.py` | serving environments from Modal sandboxes |
| `robogym/envs/__init__.py` | `ENVS`, the environments by name |
| `robogym/envs/so101_dual_arm/` | the SO-101 environment, whose README lists its files |
| `tests/` | `test_envs.py` for the contract, and a file each for the server, the viewer and the SO-101 environment |

To read the code after this, a good order is `env.py`, `physics.py`, then the SO-101 environment's `physics.py` and `env.py`, then `server.py` and `viewer.py`.
