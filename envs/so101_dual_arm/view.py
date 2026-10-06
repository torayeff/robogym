"""Open the SO-101 dual-arm scene in the MuJoCo viewer, starting from the folded rest pose.

    uv run envs/so101_dual_arm/view.py
"""

from pathlib import Path

import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path(str(Path(__file__).parent / "scene.xml"))
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, model.key("rest").id)
mujoco.viewer.launch(model, data)
