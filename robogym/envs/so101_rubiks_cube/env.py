"""The SO-101 dual-arm Rubik's cube scene as a robogym environment."""

import asyncio
import base64
import concurrent.futures
from collections.abc import Callable

import cv2
import mujoco
import numpy as np

from robogym.env import VIEW_HEIGHT, VIEW_WIDTH, Env
from robogym.physics import PhysicsProcess

from .camera import U20CAM, LensMap
from .physics import STATE, So101RubiksCubePhysics, compile_scene, table_height

DESCRIPTION = (
    "Two SO-101 robot arms side by side at the back edge of a table, facing forward. "
    "An articulated 3x3x3 Rubik's cube ({cube}) rests on the table between them, "
    "where both arms can reach it to hold it and turn its faces. Every reset places "
    "the cube at a random position and yaw within reach of both arms and scrambles "
    "its faces. Each arm has six position-controlled servos: an action sets their "
    "goal angles, and the simulated servos move the joints there. After a reset "
    "both arms are folded, grippers just above the table; turning a folded arm more "
    "than about 0.8 rad can swing it into the camera mast between the arms."
)
SERVOS = {
    "shoulder_pan": "Turns the arm about the vertical axis; positive turns it right.",
    "shoulder_lift": "Tilts the upper arm; positive swings it forward and down.",
    "elbow_flex": "Bends the elbow; positive swings the forearm down.",
    "wrist_flex": "Bends the wrist; positive tips the gripper down.",
    "wrist_roll": "Rolls the gripper; positive is anticlockwise seen from the wrist.",
    "gripper": "Opens the jaws: the minimum closes them, the maximum opens them wide.",
}
CAMERAS = {
    "top_camera": (
        "On the mast between the arms, {height:.0f} cm above the table, looking "
        "forward and down at the table and both arms. The image's left is the "
        "robot's left."
    ),
    "left_wrist_camera": "On the left gripper, looking along its jaws.",
    "right_wrist_camera": "On the right gripper, looking along its jaws.",
}
JPEG_QUALITY = 90


class So101RubiksCube(Env):
    """Steps physics in a process of its own, and draws from its states here."""

    def __init__(
        self,
        make_physics: Callable[[], So101RubiksCubePhysics] = So101RubiksCubePhysics,
    ):
        self._physics = PhysicsProcess(make_physics)
        model = compile_scene(visual=True)
        self._spec = _spec(model)
        self._goal_ranges = _goal_ranges(model)
        self._renderer = Renderer(model)
        self._observing = 0
        self._viewed = None

    def spec(self) -> dict:
        return self._spec

    async def observe(self) -> dict:
        self._observing += 1
        try:
            state = await self._physics.call("state")
            return await asyncio.wrap_future(self._renderer.observe(state))
        finally:
            self._observing -= 1

    async def act(self, action: dict) -> None:
        _check(action, self._goal_ranges)
        await self._physics.call("act", action)

    async def pause(self) -> None:
        await self._physics.call("pause")

    async def resume(self) -> None:
        await self._physics.call("resume")

    async def reset(self) -> None:
        await self._physics.call("reset")

    async def view(self) -> np.ndarray | None:
        if self._observing:
            return None
        state = await self._physics.call("state")
        if self._viewed is not None and np.array_equal(state, self._viewed):
            return None
        self._viewed = state
        return await asyncio.wrap_future(self._renderer.view(state))

    async def status(self) -> dict:
        return await self._physics.call("status")

    async def wait(self) -> int:
        return await self._physics.wait()

    def close(self) -> None:
        self._renderer.close()
        self._physics.close()


class Renderer:
    """Draws observations and the view from states, on one GL context and thread."""

    def __init__(self, model: mujoco.MjModel):
        self._render_thread = concurrent.futures.ThreadPoolExecutor(max_workers=1)
        self._render_thread.submit(self._open, model).result()

    def observe(self, state: np.ndarray) -> concurrent.futures.Future:
        return self._render_thread.submit(self._observe, state)

    def view(self, state: np.ndarray) -> concurrent.futures.Future:
        return self._render_thread.submit(self._view, state)

    def close(self) -> None:
        self._render_thread.submit(self._close)
        self._render_thread.shutdown()

    def _open(self, model: mujoco.MjModel) -> None:
        self._lens = LensMap(U20CAM)
        width = max(self._lens.pinhole_width, VIEW_WIDTH)
        height = max(self._lens.pinhole_height, VIEW_HEIGHT)
        model.vis.global_.offwidth, model.vis.global_.offheight = width, height
        self._model = model
        self._data = mujoco.MjData(model)
        self._joints = [model.actuator(i).name for i in range(model.nu)]
        self._gl = mujoco.GLContext(width, height)
        self._gl.make_current()
        self._context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self._context)
        self._scene = mujoco.MjvScene(model, maxgeom=10_000)
        self._option = mujoco.MjvOption()
        self._default_camera = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(model, self._default_camera)
        self._cameras = {}
        for name in CAMERAS:
            self._cameras[name] = camera = mujoco.MjvCamera()
            camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
            camera.fixedcamid = model.camera(name).id

    def _observe(self, state: np.ndarray) -> dict:
        self._load(state)
        return {
            "time": self._data.time,
            "joints": {
                joint: self._data.joint(joint).qpos.item() for joint in self._joints
            },
            "images": {
                name: self._jpeg(camera) for name, camera in self._cameras.items()
            },
        }

    def _view(self, state: np.ndarray) -> np.ndarray:
        self._load(state)
        return self._draw(self._default_camera, VIEW_WIDTH, VIEW_HEIGHT)

    def _load(self, state: np.ndarray) -> None:
        mujoco.mj_setState(self._model, self._data, state, STATE)
        mujoco.mj_forward(self._model, self._data)

    def _jpeg(self, camera: mujoco.MjvCamera) -> str:
        lens = self._lens
        pinhole = self._draw(
            camera, lens.pinhole_width, lens.pinhole_height, lens.tan_half_fovy
        )
        bgr = cv2.cvtColor(lens.apply(pinhole), cv2.COLOR_RGB2BGR)
        _, jpeg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        return base64.b64encode(jpeg).decode()

    def _draw(
        self,
        camera: mujoco.MjvCamera,
        width: int,
        height: int,
        tan_half_fovy: float | None = None,
    ) -> np.ndarray:
        mujoco.mjv_updateScene(
            self._model,
            self._data,
            self._option,
            None,
            camera,
            mujoco.mjtCatBit.mjCAT_ALL,
            self._scene,
        )
        if tan_half_fovy:
            for eye in self._scene.camera:
                eye.frustum_bottom = -eye.frustum_near * tan_half_fovy
                eye.frustum_top = eye.frustum_near * tan_half_fovy
        viewport = mujoco.MjrRect(0, 0, width, height)
        mujoco.mjr_render(viewport, self._scene, self._context)
        image = np.empty((height, width, 3), np.uint8)
        mujoco.mjr_readPixels(image, None, viewport, self._context)
        return np.flipud(image).copy()

    def _close(self) -> None:
        self._gl.free()
        self._context.free()


def _spec(model: mujoco.MjModel) -> dict:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    camera_height = 100 * (data.cam("top_camera").xpos[2] - table_height(model))
    ranges = _goal_ranges(model)
    images = {
        name: {
            "type": "string",
            "contentEncoding": "base64",
            "contentMediaType": "image/jpeg",
            "description": f"{U20CAM.width} x {U20CAM.height}, wide-angle lens. "
            + text.format(height=camera_height),
        }
        for name, text in CAMERAS.items()
    }
    goals = {
        joint: {
            "type": "number",
            "minimum": low,
            "maximum": high,
            "description": SERVOS[joint.split("_", 1)[1]],
        }
        for joint, (low, high) in ranges.items()
    }
    return {
        "name": "so101_rubiks_cube",
        "description": DESCRIPTION.format(cube=_size(model, "cube")),
        "observation": {
            "type": "object",
            "properties": {
                "time": {
                    "type": "number",
                    "description": "Simulation seconds since the last reset.",
                },
                "joints": {
                    "type": "object",
                    "description": "Measured joint angles in radians.",
                    "properties": {joint: {"type": "number"} for joint in ranges},
                    "required": list(ranges),
                },
                "images": {
                    "type": "object",
                    "properties": images,
                    "required": list(CAMERAS),
                },
            },
            "required": ["time", "joints", "images"],
        },
        "action": {
            "type": "object",
            "properties": {
                "joints": {
                    "type": "object",
                    "description": "Goal angles in radians. Joints left out keep "
                    "their goal.",
                    "properties": goals,
                    "additionalProperties": False,
                }
            },
            "required": ["joints"],
            "additionalProperties": False,
        },
    }


def _goal_ranges(model: mujoco.MjModel) -> dict[str, tuple[float, float]]:
    """Each servo's joint, and the range its goal angle can take."""
    return {
        model.actuator(i).name: tuple(model.actuator_ctrlrange[i].tolist())
        for i in range(model.nu)
    }


def _size(model: mujoco.MjModel, body: str) -> str:
    """The extent of a body and its child cubelets' mesh geoms, as 'W x D x H cm'."""
    root = model.body(body).id
    verts = []
    for g in range(model.ngeom):
        b = model.geom_bodyid[g]
        if b == root or model.body_parentid[b] == root:
            mid = model.geom_dataid[g]
            if mid >= 0:
                start = model.mesh_vertadr[mid]
                end = start + model.mesh_vertnum[mid]
                verts.append(model.mesh_vert[start:end] + model.geom_pos[g])
    pts = np.vstack(verts)
    extent = 100 * (pts.max(axis=0) - pts.min(axis=0))
    return " x ".join(f"{x:.1f}" for x in extent) + " cm"


def _check(action: dict, goal_ranges: dict[str, tuple[float, float]]) -> None:
    """Raise ValueError, saying what is wrong, unless the action fits the spec."""
    goals = action.get("joints")
    if action.keys() != {"joints"} or not isinstance(goals, dict):
        raise ValueError('An action is {"joints": {joint: goal in radians, ...}}.')
    for joint, goal in goals.items():
        if joint not in goal_ranges:
            joints = ", ".join(goal_ranges)
            raise ValueError(f"Unknown joint {joint!r}; the joints are {joints}.")
        low, high = goal_ranges[joint]
        is_number = isinstance(goal, int | float) and not isinstance(goal, bool)
        if not (is_number and low <= goal <= high):
            raise ValueError(
                f"The goal for {joint} must be a number from {low} to {high}, "
                f"not {goal!r}."
            )
