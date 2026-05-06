#!/usr/bin/env python3
"""Build a semantic point cloud using Phase 3 RTAB-Map/SLAM estimated poses.

This script mirrors scripts/reconstruct_semantic_map.py, but replaces the CARLA
saved ground-truth pose with odometry poses exported by the ROS 2 Phase 3 launch.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path
from typing import Any

from tqdm import tqdm

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from carla_semantic_slam.data.dataset_reader import iter_samples, load_metadata, load_pose_rows
from carla_semantic_slam.mapping.backprojection import backproject_semantic_depth, transform_points
from carla_semantic_slam.mapping.open3d_io import save_pointcloud, save_semantic_npz
from carla_semantic_slam.mapping.semantic_pointcloud import SemanticPointCloud
from carla_semantic_slam.slam.slam_pose_reader import load_slam_pose_rows, nearest_slam_pose, pose_row_to_matrix
from carla_semantic_slam.utils.config import load_yaml
from carla_semantic_slam.utils.logging_utils import get_logger

log = get_logger()


def _resolve_path(path_str: str | Path) -> Path:
    path = Path(path_str)
    return path if path.is_absolute() else PROJECT_ROOT / path


def _write_summary_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["label_id", "label_name", "point_count", "percentage"])
        writer.writeheader()
        writer.writerows(rows)


def _write_summary_json(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(rows, f, indent=2)


def main() -> None:
    parser = argparse.ArgumentParser(description="Reconstruct semantic map using RTAB-Map estimated poses.")
    parser.add_argument("--config", type=str, default="configs/slam.yaml", help="Path to Phase 3 SLAM config.")
    parser.add_argument("--run-dir", type=str, default=None, help="Override input.run_dir from config.")
    parser.add_argument("--slam-pose-csv", type=str, default=None, help="Override input.slam_pose_csv from config.")
    parser.add_argument("--out", type=str, default=None, help="Override output.pointcloud_path from config.")
    parser.add_argument("--max-samples", type=int, default=None, help="Override processing.max_samples.")
    args = parser.parse_args()

    cfg = load_yaml(_resolve_path(args.config))
    input_cfg = cfg.get("input", {})
    projection_cfg = cfg.get("projection", {})
    processing_cfg = cfg.get("processing", {})
    output_cfg = cfg.get("output", {})
    visualization_cfg = cfg.get("visualization", {})

    run_dir = _resolve_path(args.run_dir or input_cfg.get("run_dir", "data/raw/run_001"))
    slam_pose_csv = _resolve_path(args.slam_pose_csv or input_cfg.get("slam_pose_csv", "outputs/slam/run_001/slam_poses.csv"))
    out_path = _resolve_path(args.out or output_cfg.get("pointcloud_path", "outputs/maps/semantic_map_slam.ply"))
    npz_path = _resolve_path(output_cfg.get("npz_path", "outputs/maps/semantic_map_slam.npz"))
    csv_path = _resolve_path(output_cfg.get("label_summary_csv", "outputs/maps/semantic_map_slam_label_summary.csv"))
    json_path = _resolve_path(output_cfg.get("label_summary_json", "outputs/maps/semantic_map_slam_label_summary.json"))

    sample_stride = int(processing_cfg.get("sample_stride", 1))
    max_samples = args.max_samples if args.max_samples is not None else processing_cfg.get("max_samples", None)
    max_time_diff = processing_cfg.get("max_time_diff_sec", 0.20)

    metadata = load_metadata(run_dir)
    dataset_rows = load_pose_rows(run_dir)
    slam_rows = load_slam_pose_rows(slam_pose_csv)
    intrinsics = metadata["camera_intrinsics"]

    log.info("Input run: %s", run_dir)
    log.info("Dataset rows: %d", len(dataset_rows))
    log.info("SLAM poses: %d from %s", len(slam_rows), slam_pose_csv)

    cloud = SemanticPointCloud()
    sample_count = 0
    point_count = 0
    skipped = 0

    for sample in tqdm(iter_samples(run_dir, sample_stride=sample_stride, max_samples=max_samples), desc="SLAM-pose semantic mapping"):
        try:
            slam_row, time_diff = nearest_slam_pose(sample["row"], slam_rows, max_time_diff_sec=max_time_diff)
        except Exception as exc:
            skipped += 1
            if skipped <= 5:
                log.warning("Skipping sample because no matched SLAM pose was found: %s", exc)
            continue

        points_camera, labels = backproject_semantic_depth(
            depth_m=sample["depth"],
            semantic=sample["semantic"],
            intrinsics=intrinsics,
            pixel_stride=int(projection_cfg.get("pixel_stride", 4)),
            min_depth_m=float(projection_cfg.get("min_depth_m", 0.5)),
            max_depth_m=float(projection_cfg.get("max_depth_m", 80.0)),
            ignore_labels=projection_cfg.get("ignore_labels", [0, 13]),
            max_points=projection_cfg.get("max_points_per_frame", None),
        )

        slam_T_camera = pose_row_to_matrix(slam_row)
        points_map = transform_points(points_camera, slam_T_camera)
        cloud.add(points_map, labels)
        sample_count += 1
        point_count += int(points_map.shape[0])

    points, labels, colors = cloud.as_arrays()
    if points.shape[0] == 0:
        raise RuntimeError("No semantic points were generated. Check SLAM pose CSV timestamps and depth settings.")

    pcd = save_pointcloud(out_path, points, colors, voxel_size=float(visualization_cfg.get("voxel_size", 0.15)))
    save_semantic_npz(npz_path, points, labels, colors)
    summary_rows = cloud.label_summary()
    _write_summary_csv(csv_path, summary_rows)
    _write_summary_json(json_path, summary_rows)

    log.info("Processed %d samples; skipped %d samples", sample_count, skipped)
    log.info("Accumulated %d semantic points before voxel downsampling", point_count)
    log.info("Saved SLAM-pose semantic PLY: %s", out_path)
    log.info("Saved full arrays: %s", npz_path)
    log.info("PLY point count after optional voxel downsampling: %d", len(pcd.points))


if __name__ == "__main__":
    main()
