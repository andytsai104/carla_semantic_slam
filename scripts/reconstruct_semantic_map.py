#!/usr/bin/env python3
"""Build a baseline semantic point cloud from a collected CARLA run.

This is the Phase 2 baseline: it uses CARLA ground-truth camera poses saved
from data collection. The mapping modules are intentionally pose-source
agnostic so the same back-projection and accumulation code can later use
RTAB-Map camera poses.
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
from carla_semantic_slam.data.transforms import row_camera_matrix
from carla_semantic_slam.mapping.backprojection import backproject_semantic_depth, transform_points
from carla_semantic_slam.mapping.open3d_io import save_pointcloud, save_semantic_npz
from carla_semantic_slam.mapping.semantic_pointcloud import SemanticPointCloud
from carla_semantic_slam.utils.config import load_yaml
from carla_semantic_slam.utils.logging_utils import get_logger

log = get_logger()


def _resolve_path(path_str: str | Path) -> Path:
    path = Path(path_str)
    return path if path.is_absolute() else PROJECT_ROOT / path


def _write_summary_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = ["label_id", "label_name", "point_count", "percentage"]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _write_summary_json(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(rows, f, indent=2)


def main() -> None:
    parser = argparse.ArgumentParser(description="Reconstruct a semantic point cloud from collected CARLA data.")
    parser.add_argument("--config", type=str, default="configs/mapping.yaml", help="Path to mapping config.")
    parser.add_argument("--run-dir", type=str, default=None, help="Override input.run_dir from config.")
    parser.add_argument("--out", type=str, default=None, help="Override output.pointcloud_path from config.")
    parser.add_argument("--max-samples", type=int, default=None, help="Override processing.max_samples.")
    parser.add_argument("--sample-stride", type=int, default=None, help="Override processing.sample_stride.")
    args = parser.parse_args()

    cfg_path = _resolve_path(args.config)
    cfg = load_yaml(cfg_path)

    input_cfg = cfg.get("input", {})
    projection_cfg = cfg.get("projection", {})
    processing_cfg = cfg.get("processing", {})
    output_cfg = cfg.get("output", {})

    run_dir = _resolve_path(args.run_dir or input_cfg.get("run_dir", "data/raw/run_001"))
    out_path = _resolve_path(args.out or output_cfg.get("pointcloud_path", "outputs/maps/semantic_map.ply"))
    npz_path = _resolve_path(output_cfg.get("npz_path", "outputs/maps/semantic_map.npz"))
    csv_path = _resolve_path(output_cfg.get("label_summary_csv", "outputs/maps/semantic_map_label_summary.csv"))
    json_path = _resolve_path(output_cfg.get("label_summary_json", "outputs/maps/semantic_map_label_summary.json"))

    sample_stride = int(args.sample_stride or processing_cfg.get("sample_stride", 1))
    max_samples = args.max_samples if args.max_samples is not None else processing_cfg.get("max_samples", None)
    if max_samples in ("", "null"):
        max_samples = None

    metadata = load_metadata(run_dir)
    rows = load_pose_rows(run_dir)
    intrinsics = metadata["camera_intrinsics"]

    log.info("Input run: %s", run_dir)
    log.info("Found %d pose rows", len(rows))
    log.info("Camera intrinsics: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", intrinsics["fx"], intrinsics["fy"], intrinsics["cx"], intrinsics["cy"])

    cloud = SemanticPointCloud()
    sample_count = 0
    point_count = 0

    samples = iter_samples(run_dir, sample_stride=sample_stride, max_samples=max_samples)
    for sample in tqdm(samples, desc="Back-projecting semantic RGB-D samples"):
        row = sample["row"]
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
        camera_T_world = row_camera_matrix(row)
        points_world = transform_points(points_camera, camera_T_world)
        cloud.add(points_world, labels)
        sample_count += 1
        point_count += int(points_world.shape[0])

    points, labels, colors = cloud.as_arrays()
    if points.shape[0] == 0:
        raise RuntimeError("No valid semantic points were generated. Check depth range, ignored labels, and dataset paths.")

    pcd = save_pointcloud(out_path, points, colors, voxel_size=float(output_cfg.get("voxel_size", 0.15)))
    save_semantic_npz(npz_path, points, labels, colors)

    summary_rows = cloud.label_summary()
    _write_summary_csv(csv_path, summary_rows)
    _write_summary_json(json_path, summary_rows)

    log.info("Processed %d samples", sample_count)
    log.info("Accumulated %d semantic points before voxel downsampling", point_count)
    log.info("Saved full semantic arrays: %s", npz_path)
    log.info("Saved visualization PLY: %s", out_path)
    log.info("PLY point count after optional voxel downsampling: %d", len(pcd.points))
    log.info("Saved label summaries: %s and %s", csv_path, json_path)
    log.info("Top semantic classes:")
    for row in summary_rows[:8]:
        log.info("  %s (%s): %d points, %.2f%%", row["label_name"], row["label_id"], row["point_count"], row["percentage"])


if __name__ == "__main__":
    main()
