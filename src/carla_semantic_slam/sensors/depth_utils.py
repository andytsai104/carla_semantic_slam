"""Depth image conversion utilities."""
from __future__ import annotations

import carla
import numpy as np


def carla_depth_to_meters(image: carla.Image) -> np.ndarray:
    """Decode CARLA depth camera output into metric depth.

    CARLA encodes normalized depth in the RGB channels over a 0 to 1000 meter range.
    The raw memory layout is BGRA, so B/G/R are channels 0/1/2.
    """
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4)).astype(np.float32)
    b = array[:, :, 0]
    g = array[:, :, 1]
    r = array[:, :, 2]
    normalized = (r + g * 256.0 + b * 256.0 * 256.0) / (256.0**3 - 1.0)
    return normalized * 1000.0
