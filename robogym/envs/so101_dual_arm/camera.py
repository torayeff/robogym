"""Render the robot cameras as the U20CAM's wide-angle lens sees them.

MuJoCo cameras are pinhole, so LensRenderer renders a wider pinhole image and remaps it
through a lens model in OpenCV's fisheye (Kannala-Brandt) convention; the K and D of a
cv2.fisheye calibration drop in unchanged.

    with LensRenderer(model) as renderer:
        image = renderer.render(data, "left_wrist_camera")  # (480, 640, 3) uint8
"""

import dataclasses
import math

import cv2
import mujoco
import numpy as np


@dataclasses.dataclass(frozen=True)
class Lens:
    """Image size and intrinsics as cv2.fisheye takes them.

    K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]] and D = k.
    """

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    k: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)


# Nominal values from the U20CAM-1080P-S1 user manual (v1.1), not a calibration. Its
# 106° horizontal field of view across the 1920 x 1080 sensor is more than a 2.8 mm
# pinhole lens covers on a 1/3" sensor (about 86°), but what an equidistant lens
# (r = f θ, k = 0) covers. 640 x 480 is taken to be the 4:3 centre crop scaled by
# 480/1080, which keeps 79.5° x 59.625°. The manual's other figures don't all fit one
# lens, so the true vertical field of view could be anywhere from about 54° to 66°.
_F = 960 / math.radians(53) * 480 / 1080  # pixels per radian
U20CAM = Lens(640, 480, _F, _F, 319.5, 239.5)


class LensRenderer:
    """Renders model cameras through `lens` as (height, width, 3) uint8 images.

    Raises the model's offscreen buffer (vis.global_.offwidth, offheight) to fit the
    wider pinhole render.
    """

    def __init__(self, model: mujoco.MjModel, lens: Lens = U20CAM):
        self.lens = lens
        u, v = np.meshgrid(np.arange(lens.width), np.arange(lens.height))
        pixels = np.stack([u, v], -1).reshape(-1, 1, 2).astype(np.float64)
        K = np.array([[lens.fx, 0, lens.cx], [0, lens.fy, lens.cy], [0, 0, 1]])
        rays = cv2.fisheye.undistortPoints(pixels, K, np.array(lens.k)).reshape(
            lens.height, lens.width, 2
        )
        # The pinhole image resolves 1.5 times as finely as the lens at the image
        # centre, where the lens is sharpest, and is just big enough to hold every ray
        # the lens sees.
        f = 1.5 * max(lens.fx, lens.fy)
        half_width = math.ceil(f * np.abs(rays[..., 0]).max()) + 1
        half_height = math.ceil(f * np.abs(rays[..., 1]).max()) + 1
        model.vis.global_.offwidth = max(model.vis.global_.offwidth, 2 * half_width)
        model.vis.global_.offheight = max(model.vis.global_.offheight, 2 * half_height)
        self._renderer = mujoco.Renderer(model, 2 * half_height, 2 * half_width)
        self._tan_half_fovy = half_height / f
        maps = (f * rays + [half_width - 0.5, half_height - 0.5]).astype(np.float32)
        self._map1, self._map2 = cv2.convertMaps(
            maps[..., 0], maps[..., 1], cv2.CV_16SC2
        )

    def render(self, data: mujoco.MjData, camera: int | str) -> np.ndarray:
        self._renderer.update_scene(data, camera=camera)
        # Widen the frustum to fit the pinhole image, leaving the model's fovy as it is.
        for eye in self._renderer.scene.camera:
            eye.frustum_bottom = -eye.frustum_near * self._tan_half_fovy
            eye.frustum_top = eye.frustum_near * self._tan_half_fovy
        return cv2.remap(
            self._renderer.render(), self._map1, self._map2, cv2.INTER_LINEAR
        )

    def close(self) -> None:
        self._renderer.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
