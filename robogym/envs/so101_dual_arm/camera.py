"""The U20CAM's wide-angle lens, in OpenCV's fisheye (Kannala-Brandt) convention."""

import dataclasses
import math

import cv2
import numpy as np


@dataclasses.dataclass(frozen=True)
class Lens:
    """An image size, and the K = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]] and D = k of a
    cv2.fisheye calibration."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    k: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)

    def rays(self, pixels: np.ndarray) -> np.ndarray:
        """The directions (x/z, y/z) that pixels (u, v) look in."""
        K = np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]])
        points = pixels.reshape(-1, 1, 2).astype(np.float64)
        rays = cv2.fisheye.undistortPoints(points, K, np.array(self.k))
        return rays.reshape(pixels.shape)

    @property
    def fovy(self) -> float:
        """The vertical field of view in degrees."""
        _, top = self.rays(np.array([self.cx, -0.5]))
        return math.degrees(2 * math.atan(abs(top)))


# Nominal, from the U20CAM-1080P-S1 manual (v1.1), not a calibration: an equidistant
# lens (k = 0) with 106° across the sensor's 1920 pixels, cropped to 4:3 and scaled
# to 640 x 480.
_FOCAL_PX = 960 / math.radians(53) * 480 / 1080
U20CAM = Lens(640, 480, _FOCAL_PX, _FOCAL_PX, 319.5, 239.5)


class LensMap:
    """Maps a wider pinhole image, as a MuJoCo camera draws it, onto a lens's image.

    The pinhole image is pinhole_width x pinhole_height, and tan_half_fovy is the
    tangent of half its vertical field of view.
    """

    def __init__(self, lens: Lens):
        u, v = np.meshgrid(np.arange(lens.width), np.arange(lens.height))
        rays = lens.rays(np.stack([u, v], -1))
        # 1.5 times as fine as the lens at its sharpest, the centre; wide enough to
        # hold every ray.
        f = 1.5 * max(lens.fx, lens.fy)
        half_width = math.ceil(f * np.abs(rays[..., 0]).max()) + 1
        half_height = math.ceil(f * np.abs(rays[..., 1]).max()) + 1
        self.pinhole_width, self.pinhole_height = 2 * half_width, 2 * half_height
        self.tan_half_fovy = half_height / f
        maps = (f * rays + [half_width - 0.5, half_height - 0.5]).astype(np.float32)
        self._map1, self._map2 = cv2.convertMaps(
            maps[..., 0], maps[..., 1], cv2.CV_16SC2
        )

    def apply(self, pinhole: np.ndarray) -> np.ndarray:
        return cv2.remap(pinhole, self._map1, self._map2, cv2.INTER_LINEAR)
