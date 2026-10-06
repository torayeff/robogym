"""The environments robogym can serve, by name.

Each maps to the "module:class" of its Env, imported only when it is served.
"""

ENVS = {"so101_dual_arm": "envs.so101_dual_arm.env:So101DualArm"}
