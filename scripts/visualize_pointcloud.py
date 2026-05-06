#!/usr/bin/env python3
"""Visualize a saved point cloud using Open3D."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from carla_semantic_slam.visualization.open3d_viz import visualize_pointcloud


def main() -> None:
    parser = argparse.ArgumentParser(description="Visualize a semantic point cloud.")
    parser.add_argument("path", nargs="?", default="outputs/maps/semantic_map_run_001.ply")
    args = parser.parse_args()
    path = Path(args.path)
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    visualize_pointcloud(str(path))


if __name__ == "__main__":
    main()
