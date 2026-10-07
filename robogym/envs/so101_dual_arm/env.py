"""The SO-101 dual-arm scene as a robogym environment.

Actions set goal angles for the 12 servos; observations hold the joint angles and the
images of the three robot cameras. Each reset puts the prism and the tray in new places.

    with So101DualArm() as env:
        env.act({"joints": {"left_shoulder_lift": -0.75}})
        env.step(1.0)
        observation = env.observe()  # {"time": ..., "joints": {...}, "images": {...}}
"""

import base64
from pathlib import Path

import cv2
import mujoco
import numpy as np

from robogym.env import Env, ViewRenderer

from .camera import LensRenderer

SCENE = Path(__file__).parent / "scene.xml"

DESCRIPTION = (
    "Two SO-101 robot arms side by side at the back edge of a table, facing forward. "
    "A red prism (2 x 2 x 7 cm) and a red open tray (10 x 10 x 3 cm) are out to the "
    "sides, one beside each arm: the other arm can't pick up the prism or get its "
    "gripper over the tray. Every reset picks which is on which side, where each is "
    "and which way it faces, and whether the prism stands on end or lies on a side; "
    "the tray is always upright. Each arm has six position-controlled servos: an "
    "action sets their goal angles, and the simulated servos move the joints there. "
    "After a reset both arms are folded, grippers just above the table; turning a "
    "folded arm more than about 0.8 rad can swing it into the camera mast between "
    "the arms."
)

# What each servo does, the same on both arms. The signs are measured from the model.
SERVOS = {
    "shoulder_pan": "Turns the arm about the vertical axis; positive turns it right.",
    "shoulder_lift": "Tilts the upper arm; positive swings it forward and down.",
    "elbow_flex": "Bends the elbow; positive swings the forearm down.",
    "wrist_flex": "Bends the wrist; positive tips the gripper down.",
    "wrist_roll": "Rolls the gripper; positive is anticlockwise seen from the wrist.",
    "gripper": (
        "Opens the jaws: -0.17 is closed (fingertips 4 mm apart), "
        "1.75 is wide open (133 mm)."
    ),
}
# Each servo's actuator and the joint it drives share this name.
JOINTS = {
    f"{arm}_{servo}": description
    for arm in ("left", "right")
    for servo, description in SERVOS.items()
}

CAMERAS = {
    "top_camera": (
        "On the mast between the arms, 54 cm above the table, looking forward and "
        "down at the table and both arms. The image's left is the robot's left."
    ),
    "left_wrist_camera": "On the left gripper, looking along its jaws.",
    "right_wrist_camera": "On the right gripper, looking along its jaws.",
}

JPEG_QUALITY = 90

# Where reset() puts the objects. Each one's centre is
# - within NEAR of its own arm's shoulder-pan axis, so that arm can pick the prism up
#   with its gripper within 25° of vertical, and get over the tray within 40°;
# - at least FAR from the other arm's, beyond its grasp point's reach of 0.436 m at
#   prism heights and 0.441 m over a tray;
# - at x >= FRONT, which keeps all of it in the top camera's view.
NEAR = {"prism": 0.33, "storage": 0.36}
FAR = {"prism": 0.47, "storage": 0.52}
FRONT = {"prism": 0.20, "storage": 0.26}
TABLE_TOP = 0.74
# A quarter turn about the x axis, which lays the prism on a side.
ON_SIDE = np.array([np.sqrt(0.5), np.sqrt(0.5), 0, 0])


class So101DualArm(Env):
    """scene.xml from the folded rest pose, with cameras rendered through their lens.

    `seed` makes the layouts that resets draw repeatable.
    """

    def __init__(self, seed: int | None = None):
        self.model = mujoco.MjModel.from_xml_path(str(SCENE))
        self.data = mujoco.MjData(self.model)
        self._renderer = LensRenderer(self.model)
        self._rng = np.random.default_rng(seed)
        mujoco.mj_kinematics(self.model, self.data)
        # The xy of each arm's vertical shoulder-pan axis, by side: 1 is the left arm.
        self._pan_axes = {
            side: self.data.joint(f"{arm}_shoulder_pan").xanchor[:2].copy()
            for side, arm in ((1, "left"), (-1, "right"))
        }
        self.reset()

    def spec(self) -> dict:
        return {
            "name": "so101_dual_arm",
            "description": DESCRIPTION,
            "observation": self._observation_schema(),
            "action": self._action_schema(),
        }

    @property
    def time(self) -> float:
        return self.data.time

    def step(self, seconds: float) -> None:
        steps = round(seconds / self.model.opt.timestep)
        if steps > 0:
            mujoco.mj_step(self.model, self.data, nstep=steps)

    def act(self, action: dict) -> None:
        goals = action.get("joints") if isinstance(action, dict) else None
        if not isinstance(goals, dict) or action.keys() != {"joints"}:
            raise ValueError('An action is {"joints": {joint: goal in radians, ...}}.')
        # Check every goal before applying any, so a bad action changes nothing.
        for joint, goal in goals.items():
            if joint not in JOINTS:
                raise ValueError(
                    f"Unknown joint {joint!r}; the joints are {', '.join(JOINTS)}."
                )
            low, high = self.model.actuator(joint).ctrlrange.tolist()
            is_number = isinstance(goal, int | float) and not isinstance(goal, bool)
            if not (is_number and low <= goal <= high):
                raise ValueError(
                    f"The goal for {joint} must be a number from {low} to {high}, "
                    f"not {goal!r}."
                )
        for joint, goal in goals.items():
            self.data.ctrl[self.model.actuator(joint).id] = goal

    def observe(self) -> dict:
        # mj_step leaves body positions one physics step behind the joint angles;
        # bring them up to date for rendering.
        mujoco.mj_forward(self.model, self.data)
        return {
            "time": self.data.time,
            "joints": {joint: self.data.joint(joint).qpos.item() for joint in JOINTS},
            "images": {camera: self._render_jpeg(camera) for camera in CAMERAS},
        }

    def reset(self) -> None:
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("rest").id)
        sides = self._rng.permutation([1, -1])  # 1 is beside the left arm
        for name, side in zip(("prism", "storage"), sides):
            self.data.joint(name).qpos = self._resting_pose(name, side)

    def positions(self) -> np.ndarray:
        return self.data.qpos.copy()

    def view_renderer(self, width: int, height: int) -> ViewRenderer:
        return DefaultViewRenderer(self.model, width, height)

    def close(self) -> None:
        self._renderer.close()

    def _resting_pose(self, name: str, side: int) -> list[float]:
        """A free-joint qpos for the object, beside the arm at `side`."""
        near, far = self._pan_axes[side], self._pan_axes[-side]
        while True:  # sample NEAR's bounding box, cut at FRONT, until a centre fits
            centre = self._rng.uniform(
                [FRONT[name], near[1] - NEAR[name]], near + NEAR[name]
            )
            if (
                np.linalg.norm(centre - near) <= NEAR[name]
                and np.linalg.norm(centre - far) >= FAR[name]
            ):
                break
        x, y = centre
        yaw = self._rng.uniform(-np.pi, np.pi)
        upright = np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])
        if name == "prism" and self._rng.random() < 0.5:
            # Lying on a side. The prism's origin is at one end, so it goes half the
            # prism's length along the table from the centre, and half its width up.
            half_width, _, half_length = self.model.geom("prism").size
            lying = np.empty(4)
            mujoco.mju_mulQuat(lying, upright, ON_SIDE)
            x, y = x - half_length * np.sin(yaw), y + half_length * np.cos(yaw)
            return [x, y, TABLE_TOP + half_width, *lying]
        return [x, y, TABLE_TOP, *upright]

    def _observation_schema(self) -> dict:
        lens = self._renderer.lens
        images = {
            camera: {
                "type": "string",
                "contentEncoding": "base64",
                "contentMediaType": "image/jpeg",
                "description": f"{lens.width} x {lens.height}, wide-angle lens. {text}",
            }
            for camera, text in CAMERAS.items()
        }
        return {
            "type": "object",
            "properties": {
                "time": {
                    "type": "number",
                    "description": "Simulation seconds since the last reset.",
                },
                "joints": {
                    "type": "object",
                    "description": "Measured joint angles in radians.",
                    "properties": {joint: {"type": "number"} for joint in JOINTS},
                    "required": list(JOINTS),
                },
                "images": {
                    "type": "object",
                    "properties": images,
                    "required": list(CAMERAS),
                },
            },
            "required": ["time", "joints", "images"],
        }

    def _action_schema(self) -> dict:
        goals = {}
        for joint, text in JOINTS.items():
            low, high = self.model.actuator(joint).ctrlrange.tolist()
            goals[joint] = {
                "type": "number",
                "minimum": low,
                "maximum": high,
                "description": text,
            }
        joints = {
            "type": "object",
            "description": "Goal angles in radians. Joints left out keep their goal.",
            "properties": goals,
            "additionalProperties": False,
        }
        return {
            "type": "object",
            "properties": {"joints": joints},
            "required": ["joints"],
            "additionalProperties": False,
        }

    def _render_jpeg(self, camera: str) -> str:
        bgr = cv2.cvtColor(self._renderer.render(self.data, camera), cv2.COLOR_RGB2BGR)
        _, jpeg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        return base64.b64encode(jpeg).decode()


class DefaultViewRenderer(ViewRenderer):
    """Renders MuJoCo's default free camera, framed by scene.xml's <statistic>."""

    def __init__(self, model: mujoco.MjModel, width: int, height: int):
        model.vis.global_.offwidth = max(model.vis.global_.offwidth, width)
        model.vis.global_.offheight = max(model.vis.global_.offheight, height)
        self._model = model
        self._data = mujoco.MjData(model)
        self._camera = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(model, self._camera)
        self._size = height, width
        self._renderer: mujoco.Renderer | None = None

    def render(self, positions: np.ndarray) -> np.ndarray:
        if self._renderer is None:
            # Created on first use, so its OpenGL context belongs to the render thread.
            self._renderer = mujoco.Renderer(self._model, *self._size)
        self._data.qpos[:] = positions
        mujoco.mj_forward(self._model, self._data)
        self._renderer.update_scene(self._data, self._camera)
        return self._renderer.render()

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
