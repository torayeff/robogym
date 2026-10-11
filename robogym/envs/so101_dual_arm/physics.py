"""The physics of the SO-101 dual-arm scene."""

from pathlib import Path

import mujoco
import numpy as np

from robogym.physics import Physics

SCENE = Path(__file__).with_name("scene.xml")
STATE = mujoco.mjtState.mjSTATE_INTEGRATION
DIVERGED = (
    mujoco.mjtWarning.mjWARN_BADQPOS,
    mujoco.mjtWarning.mjWARN_BADQVEL,
    mujoco.mjtWarning.mjWARN_BADQACC,
)
PAN_JOINTS = ("left_shoulder_pan", "right_shoulder_pan")
NEAR = {"block": 0.33, "tray": 0.36}
FAR = {"block": 0.47, "tray": 0.52}
FRONT = {"block": 0.20, "tray": 0.26}
ON_SIDE = np.array([np.sqrt(0.5), np.sqrt(0.5), 0, 0])


def compile_scene(visual: bool) -> mujoco.MjModel:
    """scene.xml, whole or without what only drawing uses."""
    spec = mujoco.MjSpec.from_file(str(SCENE))
    if not visual:
        for geom in spec.geoms:
            if geom.contype == 0 and geom.conaffinity == 0:
                spec.delete(geom)
        used = {geom.meshname for geom in spec.geoms}
        for mesh in spec.meshes:
            if mesh.name not in used:
                spec.delete(mesh)
    _share_meshes(spec)
    return spec.compile()


def table_height(model: mujoco.MjModel) -> float:
    tabletop = model.geom("tabletop")
    return tabletop.pos[2] + tabletop.size[2]


def _share_meshes(spec: mujoco.MjSpec) -> None:
    """Point the geoms of identical meshes at one of them."""
    model = spec.compile()
    kept, copies = {}, {}
    for mesh in spec.meshes:
        key = (mesh.maxhullvert, mesh.material, *_mesh_data(model, mesh.name))
        if key in kept:
            copies[mesh.name] = kept[key]
        else:
            kept[key] = mesh.name
    for geom in spec.geoms:
        geom.meshname = copies.get(geom.meshname, geom.meshname)
    for name in copies:
        spec.delete(spec.mesh(name))


def _mesh_data(model: mujoco.MjModel, name: str) -> tuple[bytes, ...]:
    """A compiled mesh's vertices, normals, texture coordinates and faces."""
    i = model.mesh(name).id

    def rows(field: str) -> bytes:
        start = getattr(model, f"mesh_{field}adr")[i]
        count = getattr(model, f"mesh_{field}num")[i]
        return getattr(model, f"mesh_{field}")[start : start + count].tobytes()

    return tuple(rows(field) for field in ("vert", "normal", "texcoord", "face"))


class So101DualArmPhysics(Physics):
    """scene.xml without what only drawing uses; `seed` repeats the reset layouts."""

    def __init__(self, seed: int | None = None):
        self.model = compile_scene(visual=False)
        self.data = mujoco.MjData(self.model)
        self._rng = np.random.default_rng(seed)
        self._table_height = table_height(self.model)
        mujoco.mj_kinematics(self.model, self.data)
        self._pan_axes = np.array(
            [self.data.joint(joint).xanchor[:2] for joint in PAN_JOINTS]
        )
        self.reset()

    @property
    def time(self) -> float:
        return self.data.time

    @property
    def timestep(self) -> float:
        return self.model.opt.timestep

    def step(self, steps: int) -> None:
        mujoco.mj_step(self.model, self.data, nstep=steps)
        if any(self.data.warning[warning].number for warning in DIVERGED):
            raise FloatingPointError(f"The simulation diverged at {self.time:.3f} s")

    def act(self, action: dict) -> None:
        for joint, goal in action["joints"].items():
            self.data.actuator(joint).ctrl = goal

    def state(self) -> np.ndarray:
        state = np.empty(mujoco.mj_stateSize(self.model, STATE))
        mujoco.mj_getState(self.model, self.data, state, STATE)
        return state

    def reset(self) -> None:
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("rest").id)
        near, far = self._rng.permutation(self._pan_axes)
        self.data.joint("block").qpos = self._resting_pose("block", near, far)
        self.data.joint("tray").qpos = self._resting_pose("tray", far, near)

    def _resting_pose(
        self, name: str, near: np.ndarray, far: np.ndarray
    ) -> list[float]:
        """A free-joint qpos for the object, beside the pan axis `near`."""
        while True:
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
        if name == "block" and self._rng.random() < 0.5:
            half_width, _, half_length = self.model.geom("block").size
            lying = np.empty(4)
            mujoco.mju_mulQuat(lying, upright, ON_SIDE)
            x, y = x - half_length * np.sin(yaw), y + half_length * np.cos(yaw)
            return [x, y, self._table_height + half_width, *lying]
        return [x, y, self._table_height, *upright]
