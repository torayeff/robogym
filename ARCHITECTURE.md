# Architecture

robogym serves a simulated robot to an AI agent over HTTP. The agent asks what the environment is (`/spec`), reads what its cameras and joints sense (`/observation`), sends servo goals (`/action`), and controls time (`/pause`, `/resume`, `/reset`). Meanwhile, people can watch the scene in a browser. Today there is one environment, two SO-101 arms at a table simulated with MuJoCo, but the design is meant to take others, Isaac Sim included.

One rule shapes everything: *while the simulation runs, nothing may delay physics*. The SO-101 scene takes a physics step every 2.08 ms, 480 a second, and every simulated second must take one real second, however busy the server is. Drawing one observation takes about 33 ms, and Python runs only one thread at a time, so physics can't share a process with the server or with drawing. Most of what follows comes from that. The last section maps each file.

## The big picture

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

There are two processes. The server process runs the HTTP API, draws images and streams video. The physics process does nothing but step physics on the wall clock and, between two steps, carry out short commands from the server: apply an action, hand over its state, pause, resume or reset. A pipe connects them, and what crosses it is small, so physics never waits on the server for long.

Inside the server, three threads matter. The event loop handles HTTP and never blocks. The pipe thread sends one command at a time to physics and waits for the reply, so that the event loop doesn't have to. The render thread owns the server's only OpenGL context and does all the drawing.

## The contract every environment keeps

The server knows nothing about MuJoCo or any other engine. It talks to one object, an `Env`, whose methods mirror the HTTP routes:

- `spec()` describes the environment: its name, a description for the agent, and the JSON Schemas of its observations and actions.
- `observe()` returns an observation of one instant, and `view()` the default view as an image, or nothing when there is no new one.
- `act()` applies an action, or rejects it with the reason and changes nothing; the server turns that into a 422.
- `pause()`, `resume()` and `reset()` control time. An environment starts paused at time 0 and runs in real time from `resume()` until `pause()`. `reset()` puts everything back to time 0, paused.
- `status()` reports whether it runs, its time, how many resets it has had, and how late its steps have run.
- `wait()` returns if the simulation dies, and `close()` stops it.

These methods may be called at the same time: an action can arrive while an observation is being drawn, and it never waits for the drawing.

The contract says what an environment does, not how, because engines differ. MuJoCo can be stepped from Python a step at a time, and its state is a short list of numbers from which anything can draw. Isaac Sim runs its own loop and draws inside its own process, so a contract that said "step here, draw there" would need a second copy of Isaac just to draw. So there are two kinds of environment:

- Engines that robogym steps itself, such as MuJoCo, implement a `Physics` and run it in a `PhysicsProcess`, the real-time runner described next.
- Engines that run themselves, such as Isaac Sim or Gazebo, run in a process of their own, and their `Env` is a client to it. By default, Isaac Sim takes several physics steps per rendered frame, so an action takes effect at the next frame, 17–33 ms later, rather than at the next step. That needs measuring before relying on it.

## Keeping physics on the wall clock

`PhysicsProcess` spawns a fresh Python process. The process builds the environment's `Physics`, asks the operating system to run it ahead of everything else, and tells the server it's ready. On macOS it asks for the time-constraint policy, which needs no privileges; on Linux, for `SCHED_FIFO`, which needs `CAP_SYS_NICE`. If the OS refuses, it logs a warning and runs at normal priority, still ahead of the server, which lowers its own. It then takes everything made so far out of the garbage collector's view, which keeps collections short, and starts its loop.

The loop keeps the wall-clock moment at which simulation time was 0. The next step falls due at that moment, plus the simulation time, plus one step, so a step that wakes a little late doesn't push back the ones after it. The loop sleeps until then. If it wakes late, it takes every whole step that has fallen due at once, and simulation time catches up with the wall clock. If it's more than half a second behind, because the laptop slept, say, it skips the gap rather than replay it. A step that runs more than a whole step late is counted and logged, at most once a second, and `status()` reports the count and the worst lateness.

Between two steps, the loop carries out the next command waiting in the pipe. Physics waits for each one, so every command must be quick: setting servo goals, or handing over a state of 237 numbers, takes microseconds. `resume()` moves the wall-clock moment so that time carries on from where it stopped. If a step fails, or MuJoCo's simulation blows up, the loop resets the simulation, pauses it, and counts the reset in `status()`. If the whole process dies, the server notices within a tenth of a second and shuts down, so that a supervisor such as systemd can restart it.

## The SO-101 environment

The SO-101 environment is two halves of one scene, joined by the state.

The scene is `scene.xml`, with the robot it attaches from `robot/`: a rig with an overhead camera, two arms with a wrist camera each, a table, a block and a tray. It's compiled twice. The physics half, `So101DualArmPhysics`, leaves out what only drawing uses, the detailed meshes that collide with nothing, and runs in the physics process. Each arm has six servos, which MuJoCo models as position actuators. An action sets their goal angles, all checked before any is applied, and the simulated servos move the joints there. A reset folds the arms and draws a new layout for the block and the tray, which the environment's README explains.

The drawing half lives in the server. `So101DualArm`, the environment's `Env`, holds the whole scene and draws it from states it fetches from physics. To observe, it fetches the state as the request arrives and draws it while physics carries on, so the time, the joint angles and the three camera images all show that moment. Only the state crosses between the halves, and that is the one thing to keep in mind when changing the environment: anything a reset changed outside the state, such as an object's colour, would never reach the pictures.

All drawing happens on one render thread, which owns the server's only OpenGL context. A context works only on the thread that made it, and each costs about 150 MB on macOS before it holds any scene. Observations go first: while one is being drawn, the browser view skips its frame. The robot cameras have a wide-angle lens that MuJoCo's pinhole cameras can't draw, so each is drawn as a wider pinhole image and mapped through the lens model in `camera.py`.

Two details keep memory down without touching the designer's mesh files. MuJoCo copies every mesh of an attached model for each prefix, so the two arms and three cameras would hold several copies of the same meshes; compiling the scene makes them share one. And the physics half leaves the visual meshes out altogether.

The spec comes from the compiled model: the joint names and ranges, the objects' sizes and the camera's height. So the description an agent reads always matches the scene.

## Following a request

**An observation.** `GET /observation` calls `observe()` on the event loop. The pipe thread sends "state" to the physics process, which answers between two steps. The render thread loads the state into the whole scene, draws the three robot cameras through their lens and encodes them as JPEG, while physics carries on stepping. The reply holds the time, the joint angles and the images, all of the moment the request arrived. It takes about 35 ms, of which physics spends a few tens of microseconds.

**An action.** `POST /action` sends the action down the same pipe. Between two steps, physics checks every goal and sets all of them or none, and the route answers 204, or 422 with what is wrong. The request waits neither for the arm to move nor for any drawing: the simulated servos move the joints over the following steps, as real servos would. `/pause`, `/resume` and `/reset` take the same path, and `/status` reads the loop's counters.

**Watching.** A browser loads the viewer page, which opens `/video`, a response that never ends. While anyone watches, the viewer asks the environment for its view up to 30 times a second. The environment draws one only when the state has changed and no observation is waiting, so frames stop while the simulation is paused, apart from one after each action, since servo goals are part of the state. Each new view is encoded once as H.264, and the same bytes go to every browser, so one browser costs the server as much as a hundred. A browser that joins late first gets the frames since the last keyframe, which comes every 2 s, so it can start decoding at once. A browser that falls too far behind drops its backlog and resumes at the next keyframe. The page decodes the frames with the browser's WebCodecs, which browsers offer only on localhost or over HTTPS.

## Starting and stopping

`uv run robogym serve so101_dual_arm` runs the command in `robogym/__main__.py`. It reads its arguments and only then imports the server, because the spawned physics process re-imports the command's module, and would otherwise load the whole server too. As the server starts, it makes the environment, which spawns physics, compiles the whole scene and starts the render thread. Then the server lowers its own priority, and starts the viewer's loop and a watch on the physics process. The simulation waits at time 0 for `/resume`.

Ctrl+C makes uvicorn shut down. First the server ends the video streams, since uvicorn would otherwise wait for the browsers to close them. Then the viewer's loop stops, the render thread frees the OpenGL context, and the physics process stops.

## Serving from a sandbox

`robogym sandbox create <env>`, in `robogym/sandbox.py`, serves an environment from a Modal sandbox of its own. The sandbox runs `robogym serve <env>`, and Modal gives its port a public HTTPS URL. The environment's package says what the sandbox needs, in two files:

- `Dockerfile` makes the image. Modal builds it with the repository as its context, and uploads only the files it copies, less those in `.dockerignore`.
- `sandbox.toml` gives the vCPUs, the memory, and the GPU if it draws on one.

The sandbox's tags hold its environment's name, which `list` shows next to Modal's ID for the sandbox; `delete` takes that ID.

Modal runs a sandbox either under gVisor or in a VM. gVisor stands in for the Linux kernel and refuses real-time scheduling; a VM allows it but has no GPUs. So a sandbox without a GPU runs in a VM, where physics gets real-time scheduling as on Linux with `CAP_SYS_NICE`. One with a GPU runs under gVisor, at normal priority. That kept every step on time in measurement, because drawing on the GPU leaves the CPUs free.

A sandbox is idle while nothing is connected to its URL. It stops after 30 idle minutes, or after 2 hours in any case, and since the simulation lives only in its memory, a stopped sandbox is gone.

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
| the physics scene without its visual-only parts | the whole scene in both processes | the physics process holds a 7 MB model rather than 65 MB |
| meshes shared across `<attach>` prefixes | MuJoCo's copy of each mesh per prefix | it halves the drawing model, from 125 to 65 MB, without touching the mesh files |
| one GL context | one per renderer | it saves about 300 MB, and an observation waits at most for one view frame, about 9 ms |
| H.264, encoded once for every browser | MJPEG, or 3D in the browser | it shows anything the simulation can draw, costs about 0.6 cores whether 1 or 300 browsers watch, and needs far less bandwidth than MJPEG |
| the server ending the video streams as it shuts down | uvicorn's graceful-shutdown timeout | the timeout cancels the streams and logs a traceback |
| the server imported inside the command's `main()` | at the top of its module | the spawned physics process re-imports that module, and would load FastAPI, PyAV and OpenCV |
| a T4 for so101_dual_arm's sandbox | drawing on its CPUs, with Mesa | an observation took 244 ms rather than 25, and the view ran at 7 frames a second rather than 29 |
| a VM for a sandbox without a GPU | gVisor, Modal's default | under gVisor, without real-time scheduling, steps ran up to 18 ms late while drawing used the CPUs; in a VM, none did after its first 30 s |

## Adding an environment

1. Make a package in `robogym/envs/` with a subclass of `Env`, and add `"<name>": "robogym.envs.<name>.env:<Class>"` to `ENVS` in `robogym/envs/__init__.py`. Only the environment being served is imported, so different engines' dependencies never meet.
2. If robogym is to step the engine, subclass `Physics` and run it in a `PhysicsProcess`, as so101_dual_arm does. The physics process imports the class by name, so it must be importable, and its module mustn't load what only drawing needs. Its methods run between steps, so they must be quick, and its state must hold everything that drawing depends on.
3. To serve it from a sandbox, give its package two files, as so101_dual_arm does:
   - a `Dockerfile` that installs what it needs alongside the repository;
   - a `sandbox.toml` with its `vcpus`, its `memory_gib` and, if it draws on a GPU, the Modal `gpu` type.
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
| `robogym/envs/so101_dual_arm/` | the SO-101 environment: `physics.py`, `env.py`, `camera.py`, the scene, its sandbox's `Dockerfile` and `sandbox.toml`, and a README of its own |
| `tests/` | `test_envs.py` for the contract, and a file each for the server, the viewer and the SO-101 environment |

To read the code after this, a good order is `env.py`, `physics.py`, then the SO-101 environment's `physics.py` and `env.py`, then `server.py` and `viewer.py`.
