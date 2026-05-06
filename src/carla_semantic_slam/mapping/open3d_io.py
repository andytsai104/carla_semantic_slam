"""Open3D and array I/O helpers for semantic maps."""
from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
import open3d as o3d


def make_pointcloud(points_xyz: np.ndarray, colors_rgb: np.ndarray) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_xyz.astype(np.float64))
    pcd.colors = o3d.utility.Vector3dVector(colors_rgb.astype(np.float64) / 255.0)
    return pcd


def save_pointcloud(
    path: str | Path,
    points_xyz: np.ndarray,
    colors_rgb: np.ndarray,
    voxel_size: Optional[float] = None,
) -> o3d.geometry.PointCloud:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    pcd = make_pointcloud(points_xyz, colors_rgb)
    if voxel_size and voxel_size > 0:
        pcd = pcd.voxel_down_sample(voxel_size=float(voxel_size))
    o3d.io.write_point_cloud(str(path), pcd)
    return pcd


def save_semantic_npz(path: str | Path, points_xyz: np.ndarray, labels: np.ndarray, colors_rgb: np.ndarray) -> None:
    """Save full semantic map arrays. Unlike PLY, this preserves label IDs."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        path,
        points=points_xyz.astype(np.float32),
        labels=labels.astype(np.uint8),
        colors=colors_rgb.astype(np.uint8),
    )
