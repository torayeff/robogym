"""Open the bimanual SO-101 scene in the MuJoCo viewer, starting from the folded rest pose.

    uv run so101_dual_arm/sim.py
"""

from pathlib import Path

import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path(str(Path(__file__).parent / "scene.xml"))
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, 0)
mujoco.viewer.launch(model, data)
