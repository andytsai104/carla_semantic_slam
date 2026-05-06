"""RGB-D semantic back-projection utilities."""
from __future__ import annotations

from typing import Iterable, Optional, Tuple

import numpy as np


def backproject_semantic_depth(
    depth_m: np.ndarray,
    semantic: np.ndarray,
    intrinsics: dict,
    pixel_stride: int = 4,
    min_depth_m: float = 0.5,
    max_depth_m: float = 80.0,
    ignore_labels: Iterable[int] = (0, 13),
    max_points: Optional[int] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """Back-project depth pixels into camera-frame 3D points with semantic labels.

    The input camera model follows the standard pinhole convention:
        x_cv = right, y_cv = down, z_cv = forward

    CARLA actor local coordinates follow:
        x = forward, y = right, z = up

    Therefore this function converts camera-frame points into CARLA camera actor
    coordinates before the camera world transform is applied.
    """
    if depth_m.shape[:2] != semantic.shape[:2]:
        raise ValueError(f"Depth shape {depth_m.shape} and semantic shape {semantic.shape} do not match.")

    fx, fy = float(intrinsics["fx"]), float(intrinsics["fy"])
    cx, cy = float(intrinsics["cx"]), float(intrinsics["cy"])
    ignore = set(int(x) for x in ignore_labels)

    stride = max(1, int(pixel_stride))
    v_coords, u_coords = np.mgrid[0 : depth_m.shape[0] : stride, 0 : depth_m.shape[1] : stride]
    z = depth_m[v_coords, u_coords]
    labels = semantic[v_coords, u_coords].astype(np.uint8)

    valid = np.isfinite(z) & (z >= min_depth_m) & (z <= max_depth_m)
    for label in ignore:
        valid &= labels != label

    u = u_coords[valid].astype(np.float64)
    v = v_coords[valid].astype(np.float64)
    z = z[valid].astype(np.float64)
    labels = labels[valid]

    if max_points is not None and max_points > 0 and z.size > max_points:
        # Deterministic uniform subsampling across the valid set.
        keep = np.linspace(0, z.size - 1, int(max_points), dtype=np.int64)
        u, v, z, labels = u[keep], v[keep], z[keep], labels[keep]

    x_cv = (u - cx) * z / fx
    y_cv = (v - cy) * z / fy

    points_camera_carla = np.column_stack([z, x_cv, -y_cv])
    return points_camera_carla.astype(np.float32), labels.astype(np.uint8)


def transform_points(points: np.ndarray, transform_4x4: np.ndarray) -> np.ndarray:
    """Apply a 4x4 homogeneous transform to Nx3 points."""
    if points.size == 0:
        return points.reshape(0, 3)
    homogeneous = np.column_stack([points.astype(np.float64), np.ones(points.shape[0])])
    transformed = (transform_4x4 @ homogeneous.T).T
    return transformed[:, :3].astype(np.float32)
