"""Open3D visualization helpers."""
from __future__ import annotations

import open3d as o3d


def visualize_pointcloud(path: str) -> None:
    pcd = o3d.io.read_point_cloud(path)
    o3d.visualization.draw_geometries([pcd])
