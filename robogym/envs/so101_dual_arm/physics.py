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

# reset() puts each object's centre within NEAR of its own arm's shoulder-pan axis,
# at least FAR from the other arm's, and at x >= FRONT, in the top camera's view.
NEAR = {"block": 0.33, "tray": 0.36}
FAR = {"block": 0.47, "tray": 0.52}
FRONT = {"block": 0.20, "tray": 0.26}
ON_SIDE = np.array([np.sqrt(0.5), np.sqrt(0.5), 0, 0])  # a quarter turn about x


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
    """Point the geoms of identical meshes at one of them.

    <attach> copies a model's meshes for every prefix it's attached with.
    """
    model = spec.compile()
    kept, copies = {}, {}
    for mesh in spec.meshes:
        key = (mesh.maxhullvert, mesh.material, *_mesh_data(model, mesh.name))
        if (original := kept.setdefault(key, mesh.name)) != mesh.name:
            copies[mesh.name] = original
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
        return getattr(model, f"mesh_{field}")[start:][:count].tobytes()

    return tuple(rows(field) for field in ("vert", "normal", "texcoord", "face"))


class So101DualArmPhysics(Physics):
    """scene.xml without what only drawing uses, from the folded rest pose.

    `seed` repeats the layouts that resets draw.
    """

    def __init__(self, seed: int | None = None):
        self.model = compile_scene(visual=False)
        self.data = mujoco.MjData(self.model)
        self._rng = np.random.default_rng(seed)
        self._table_height = table_height(self.model)
        self._ranges = {
            self.model.actuator(i).name: self.model.actuator_ctrlrange[i].tolist()
            for i in range(self.model.nu)
        }
        mujoco.mj_kinematics(self.model, self.data)
        self._pan_axes = {  # the xy of each arm's shoulder-pan axis; 1 is the left
            side: self.data.joint(f"{arm}_shoulder_pan").xanchor[:2].copy()
            for side, arm in ((1, "left"), (-1, "right"))
        }
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
        goals = action.get("joints") if isinstance(action, dict) else None
        if not isinstance(goals, dict) or action.keys() != {"joints"}:
            raise ValueError('An action is {"joints": {joint: goal in radians, ...}}.')
        for joint, goal in goals.items():
            if joint not in self._ranges:
                joints = ", ".join(self._ranges)
                raise ValueError(f"Unknown joint {joint!r}; the joints are {joints}.")
            low, high = self._ranges[joint]
            is_number = isinstance(goal, int | float) and not isinstance(goal, bool)
            if not (is_number and low <= goal <= high):
                raise ValueError(
                    f"The goal for {joint} must be a number from {low} to {high}, "
                    f"not {goal!r}."
                )
        for joint, goal in goals.items():
            self.data.actuator(joint).ctrl = goal

    def state(self) -> np.ndarray:
        state = np.empty(mujoco.mj_stateSize(self.model, STATE))
        mujoco.mj_getState(self.model, self.data, state, STATE)
        return state

    def reset(self) -> None:
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("rest").id)
        for name, side in zip(("block", "tray"), self._rng.permutation([1, -1])):
            self.data.joint(name).qpos = self._resting_pose(name, side)

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
        if name == "block" and self._rng.random() < 0.5:
            # Lying on a side: its origin is at one end, so it moves half its length
            # along the table and rises by half its width.
            half_width, _, half_length = self.model.geom("block").size
            lying = np.empty(4)
            mujoco.mju_mulQuat(lying, upright, ON_SIDE)
            x, y = x - half_length * np.sin(yaw), y + half_length * np.cos(yaw)
            return [x, y, self._table_height + half_width, *lying]
        return [x, y, self._table_height, *upright]
