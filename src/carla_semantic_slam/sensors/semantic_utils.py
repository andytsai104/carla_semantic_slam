"""Semantic segmentation conversion and color utilities.

The mapping stage only needs the label dictionaries and palettes. The CARLA
Python API is imported lazily/optionally so offline reconstruction can still run
as long as RGB/depth/semantic files were already collected.
"""
from __future__ import annotations

from typing import Any

import numpy as np

try:  # CARLA is only required during live data collection.
    import carla  # type: ignore
except Exception:  # pragma: no cover - depends on local CARLA install
    carla = None  # type: ignore


CARLA_LABEL_NAMES = {
    0: "Unlabeled",
    1: "Building",
    2: "Fence",
    3: "Other",
    4: "Pedestrian",
    5: "Pole",
    6: "RoadLine",
    7: "Road",
    8: "Sidewalk",
    9: "Vegetation",
    10: "Vehicle",
    11: "Wall",
    12: "TrafficSign",
    13: "Sky",
    14: "Ground",
    15: "Bridge",
    16: "RailTrack",
    17: "GuardRail",
    18: "TrafficLight",
    19: "Static",
    20: "Dynamic",
    21: "Water",
    22: "Terrain",
}


CARLA_CITYSCAPES_PALETTE = np.zeros((256, 3), dtype=np.uint8)
CARLA_CITYSCAPES_PALETTE[0] = [0, 0, 0]
CARLA_CITYSCAPES_PALETTE[1] = [70, 70, 70]
CARLA_CITYSCAPES_PALETTE[2] = [100, 40, 40]
CARLA_CITYSCAPES_PALETTE[3] = [55, 90, 80]
CARLA_CITYSCAPES_PALETTE[4] = [220, 20, 60]
CARLA_CITYSCAPES_PALETTE[5] = [153, 153, 153]
CARLA_CITYSCAPES_PALETTE[6] = [157, 234, 50]
CARLA_CITYSCAPES_PALETTE[7] = [128, 64, 128]
CARLA_CITYSCAPES_PALETTE[8] = [244, 35, 232]
CARLA_CITYSCAPES_PALETTE[9] = [107, 142, 35]
CARLA_CITYSCAPES_PALETTE[10] = [0, 0, 142]
CARLA_CITYSCAPES_PALETTE[11] = [102, 102, 156]
CARLA_CITYSCAPES_PALETTE[12] = [220, 220, 0]
CARLA_CITYSCAPES_PALETTE[13] = [70, 130, 180]
CARLA_CITYSCAPES_PALETTE[14] = [81, 0, 81]
CARLA_CITYSCAPES_PALETTE[15] = [150, 100, 100]
CARLA_CITYSCAPES_PALETTE[16] = [230, 150, 140]
CARLA_CITYSCAPES_PALETTE[17] = [180, 165, 180]
CARLA_CITYSCAPES_PALETTE[18] = [250, 170, 30]
CARLA_CITYSCAPES_PALETTE[19] = [110, 190, 160]
CARLA_CITYSCAPES_PALETTE[20] = [170, 120, 50]
CARLA_CITYSCAPES_PALETTE[21] = [45, 60, 150]
CARLA_CITYSCAPES_PALETTE[22] = [145, 170, 100]


def carla_semantic_to_labels(image: Any) -> np.ndarray:
    """Extract raw semantic class IDs from a CARLA semantic segmentation image."""
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))
    return array[:, :, 2].copy()  # red channel in BGRA memory layout


def semantic_labels_to_palette(labels: np.ndarray) -> np.ndarray:
    """Return an RGB visualization for semantic labels."""
    return CARLA_CITYSCAPES_PALETTE[labels]


def label_name(label_id: int) -> str:
    return CARLA_LABEL_NAMES.get(int(label_id), f"Class_{int(label_id)}")
