"""The physics of the SO-101 dual-arm Rubik's cube scene."""

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
NEAR = 0.33
FRONT = 0.20
CUBE_HALF_EXTENT = 0.0285
SCRAMBLE_MOVES = 20
FACES = (
    ("pX", np.array([1.0, 0.0, 0.0])),
    ("nX", np.array([-1.0, 0.0, 0.0])),
    ("pY", np.array([0.0, 1.0, 0.0])),
    ("nY", np.array([0.0, -1.0, 0.0])),
    ("pZ", np.array([0.0, 0.0, 1.0])),
    ("nZ", np.array([0.0, 0.0, -1.0])),
)


def compile_scene(visual: bool) -> mujoco.MjModel:
    """scene.xml, whole or without what only drawing uses."""
    spec = mujoco.MjSpec.from_file(str(SCENE))
    if not visual:
        for geom in spec.geoms:
            if geom.contype == 0 and geom.conaffinity == 0:
                spec.delete(geom)
        used = {geom.meshname for geom in spec.geoms}
        for mesh in list(spec.meshes):
            if mesh.name not in used:
                spec.delete(mesh)
            else:
                mesh.usertexcoord = []
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


class So101RubiksCubePhysics(Physics):
    """scene.xml without what only drawing uses; `seed` repeats the reset states."""

    def __init__(self, seed: int | None = None):
        self.model = compile_scene(visual=False)
        self.data = mujoco.MjData(self.model)
        self._rng = np.random.default_rng(seed)
        self._table_height = table_height(self.model)
        mujoco.mj_kinematics(self.model, self.data)
        self._pan_axes = np.array(
            [self.data.joint(joint).xanchor[:2] for joint in PAN_JOINTS]
        )
        self._ball_joints = [
            (
                self.model.jnt_qposadr[j].item(),
                self.model.geom_pos[self.model.body_geomadr[self.model.jnt_bodyid[j]]],
            )
            for j in range(self.model.njnt)
            if self.model.jnt_type[j] == mujoco.mjtJoint.mjJNT_BALL
        ]
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
        self.data.joint("cube").qpos = self._resting_pose()
        self._scramble(SCRAMBLE_MOVES)

    def _resting_pose(self) -> list[float]:
        """A free-joint qpos for the cube, on the table where both arms can reach."""
        high_y = self._pan_axes[:, 1].min() + NEAR
        low_y = self._pan_axes[:, 1].max() - NEAR
        high_x = self._pan_axes[:, 0].min() + NEAR
        while True:
            centre = self._rng.uniform([FRONT, low_y], [high_x, high_y])
            if all(np.linalg.norm(centre - axis) <= NEAR for axis in self._pan_axes):
                break
        x, y = centre
        yaw = self._rng.uniform(-np.pi, np.pi)
        quat = [np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]
        return [x, y, self._table_height + CUBE_HALF_EXTENT, *quat]

    def _scramble(self, moves: int) -> None:
        """Apply `moves` non-cancelling quarter or half face turns to the cube."""
        last_axis = -1
        for _ in range(moves):
            axis_group = int(self._rng.integers(2 if last_axis >= 0 else 3))
            if last_axis >= 0 and axis_group >= last_axis:
                axis_group += 1
            last_axis = axis_group
            face_name, axis = FACES[2 * axis_group + int(self._rng.integers(2))]
            angle = int(self._rng.choice((-1, 1, 2))) * (np.pi / 2)
            self._turn_face(face_name, axis, angle)

    def _turn_face(self, face_name: str, axis: np.ndarray, angle: float) -> None:
        """Turn `face_name` by `angle` radians about `axis` in the cube frame."""
        q_rot = np.empty(4)
        mujoco.mju_axisAngle2Quat(q_rot, axis, angle)
        self.data.joint(face_name).qpos += angle
        cur_pos = np.empty(3)
        q_new = np.empty(4)
        for qposadr, pos0 in self._ball_joints:
            q_curr = self.data.qpos[qposadr : qposadr + 4]
            mujoco.mju_rotVecQuat(cur_pos, pos0, q_curr)
            if np.dot(cur_pos, axis) > 0.01:
                mujoco.mju_mulQuat(q_new, q_rot, q_curr)
                self.data.qpos[qposadr : qposadr + 4] = q_new
