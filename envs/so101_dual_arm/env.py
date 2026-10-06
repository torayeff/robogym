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

from robogym.env import Env

from .camera import LensRenderer

SCENE = Path(__file__).parent / "scene.xml"

DESCRIPTION = (
    "Two SO-101 robot arms side by side at the back edge of a table, facing forward. "
    "A red prism (2 x 2 x 7 cm) and a red open tray (10 x 10 x 3 cm) are out to the "
    "sides, one beside each arm: the other arm can't pick up the prism or get its "
    "gripper over the tray. Every reset picks which is on which side, where each is "
    "and which way it faces, and whether the prism stands on end or lies on a side; "
    "the tray is always upright. Each arm has "
    "six position-controlled servos: an action sets their goal angles, and the "
    "simulated servos move the joints there. After a reset both arms are folded, "
    "grippers just above the table; turning a folded arm more than about 0.8 rad can "
    "swing it into the camera mast between the arms."
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

# Where reset() puts the objects (README.md, Layouts). Each one's centre is within NEAR
# of its own arm's shoulder-pan axis, so that arm can pick it up or drop into it; at
# least FAR from the other arm's, beyond where that arm's grasp point gets; and at
# x >= FRONT, which keeps all of it in the top camera's view.
PAN_AXIS_X, PAN_AXIS_Y = 0.0103, 0.116  # the left arm's is at +y, the right arm's at -y
NEAR = {"prism": 0.33, "storage": 0.36}
FAR = {"prism": 0.47, "storage": 0.52}
FRONT = {"prism": 0.20, "storage": 0.26}
TABLE_TOP = 0.74


class So101DualArm(Env):
    """scene.xml from the folded rest pose, with cameras rendered through their lens.

    `seed` seeds where each reset puts the objects.
    """

    def __init__(self, seed: int | None = None):
        self.model = mujoco.MjModel.from_xml_path(str(SCENE))
        self.data = mujoco.MjData(self.model)
        self._renderer = LensRenderer(self.model)
        self._rng = np.random.default_rng(seed)
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

    def close(self) -> None:
        self._renderer.close()

    def _resting_pose(self, name: str, side: int) -> list[float]:
        """A free-joint qpos for the object, beside the arm at `side`."""
        near = np.array([PAN_AXIS_X, side * PAN_AXIS_Y])
        far = np.array([PAN_AXIS_X, -side * PAN_AXIS_Y])
        while True:  # try centres on the arm's half of the table until one fits
            x, y = self._rng.uniform([FRONT[name], 0], [0.55, 0.5]) * [1, side]
            if (
                np.hypot(x - near[0], y - near[1]) <= NEAR[name]
                and np.hypot(x - far[0], y - far[1]) >= FAR[name]
            ):
                break
        yaw = self._rng.uniform(-np.pi, np.pi)
        c, s = np.cos(yaw / 2), np.sin(yaw / 2)
        if name == "prism" and self._rng.random() < 0.5:
            # Lying on a side, turned 90° about its own x axis: its origin, at one end,
            # is 3.5 cm along the table from its centre, which is 1 cm above the table.
            x, y = x - 0.035 * np.sin(yaw), y + 0.035 * np.cos(yaw)
            r = np.sqrt(0.5)
            return [x, y, TABLE_TOP + 0.01, r * c, r * c, r * s, r * s]
        return [x, y, TABLE_TOP, c, 0, 0, s]

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
