"""Camera geometry helpers."""
from __future__ import annotations

import math
from typing import Dict

import carla
import numpy as np


def make_transform(transform_cfg: Dict[str, float]) -> carla.Transform:
    return carla.Transform(
        carla.Location(
            x=float(transform_cfg.get("x", 0.0)),
            y=float(transform_cfg.get("y", 0.0)),
            z=float(transform_cfg.get("z", 0.0)),
        ),
        carla.Rotation(
            roll=float(transform_cfg.get("roll", 0.0)),
            pitch=float(transform_cfg.get("pitch", 0.0)),
            yaw=float(transform_cfg.get("yaw", 0.0)),
        ),
    )


def camera_intrinsics(width: int, height: int, fov_deg: float) -> Dict[str, float]:
    fx = width / (2.0 * math.tan(math.radians(fov_deg) / 2.0))
    fy = fx
    cx = width / 2.0
    cy = height / 2.0
    return {
        "fx": float(fx),
        "fy": float(fy),
        "cx": float(cx),
        "cy": float(cy),
        "width": int(width),
        "height": int(height),
        "fov": float(fov_deg),
    }


def carla_transform_to_matrix(transform: carla.Transform) -> np.ndarray:
    return np.array(transform.get_matrix(), dtype=np.float64)


def transform_to_dict(transform: carla.Transform, prefix: str = "") -> Dict[str, float]:
    p = f"{prefix}_" if prefix else ""
    loc = transform.location
    rot = transform.rotation
    return {
        f"{p}x": float(loc.x),
        f"{p}y": float(loc.y),
        f"{p}z": float(loc.z),
        f"{p}roll": float(rot.roll),
        f"{p}pitch": float(rot.pitch),
        f"{p}yaw": float(rot.yaw),
    }
