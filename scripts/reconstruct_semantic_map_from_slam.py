#!/usr/bin/env python3
"""Build a semantic point cloud using RTAB-Map/SLAM odometry poses.

Key fixes in this version:
  1) Each image frame is matched to the nearest /odom pose by replay timestamp
     using outputs/slam/run_001/frame_timestamps.csv.
  2) /odom is treated as T_odom_base, not T_odom_camera.
     Therefore points are transformed as:

         P_odom = T_odom_base @ T_base_camera @ P_camera

     where T_base_camera is read from the original CARLA camera mounting transform
     saved in metadata.json, or from configs/slam.yaml if provided.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any, Optional

import numpy as np
import pandas as pd
from tqdm import tqdm

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from carla_semantic_slam.data.dataset_reader import iter_samples, load_metadata, load_pose_rows
from carla_semantic_slam.mapping.backprojection import backproject_semantic_depth, transform_points
from carla_semantic_slam.mapping.open3d_io import save_pointcloud, save_semantic_npz
from carla_semantic_slam.mapping.semantic_pointcloud import SemanticPointCloud
from carla_semantic_slam.slam.slam_pose_reader import load_slam_pose_rows, pose_row_to_matrix
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


def _sample_index_from_row(row: dict[str, str], fallback: int) -> int:
    for key in ["sample_index", "index", "frame_index", "sample", "frame"]:
        if key in row and row[key] not in (None, ""):
            return int(float(row[key]))
    return fallback


def _time_column(df: pd.DataFrame) -> str:
    for c in ["time_sec", "stamp", "timestamp", "time", "t"]:
        if c in df.columns:
            return c
    if "stamp_sec" in df.columns and "stamp_nanosec" in df.columns:
        df["time_sec"] = df["stamp_sec"].astype(float) + df["stamp_nanosec"].astype(float) * 1e-9
        return "time_sec"
    raise ValueError(f"No timestamp column found. Columns: {df.columns.tolist()}")


def _load_frame_timestamps(path: Path) -> pd.DataFrame:
    if not path.exists():
        raise FileNotFoundError(
            f"Missing frame timestamp CSV: {path}\n"
            "Run replay_rtabmap.launch.py with frame_timestamps_csv:=outputs/slam/run_001/frame_timestamps.csv first."
        )
    df = pd.read_csv(path)
    if df.empty:
        raise ValueError(f"Frame timestamp CSV is empty: {path}")
    if "sample_index" not in df.columns:
        raise ValueError(f"frame_timestamps.csv must contain sample_index. Columns: {df.columns.tolist()}")
    _time_column(df)
    return df


def _nearest_slam_pose_by_time(
    image_time: float,
    slam_df: pd.DataFrame,
    slam_times: np.ndarray,
    max_dt: Optional[float],
) -> tuple[pd.Series, float]:
    idx = int(np.searchsorted(slam_times, image_time))
    candidates = []
    if idx > 0:
        candidates.append(idx - 1)
    if idx < len(slam_times):
        candidates.append(idx)
    if not candidates:
        raise ValueError("No SLAM pose candidates available.")

    best_idx = min(candidates, key=lambda j: abs(float(slam_times[j]) - image_time))
    dt = abs(float(slam_times[best_idx]) - image_time)
    if max_dt is not None and dt > float(max_dt):
        raise ValueError(f"Nearest SLAM pose is {dt:.3f}s away from image timestamp {image_time:.3f}")
    return slam_df.iloc[best_idx], dt


def _rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)


def _rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)


def _rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


def _make_transform_xyz_rpy(x: float, y: float, z: float, roll: float, pitch: float, yaw: float, degrees: bool = True) -> np.ndarray:
    if degrees:
        roll, pitch, yaw = math.radians(roll), math.radians(pitch), math.radians(yaw)
    # Fixed-axis xyz convention. With current camera roll/pitch/yaw = 0 this is equivalent either way.
    R = _rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def _camera_extrinsic_from_metadata(metadata: dict) -> dict[str, float]:
    """Read CARLA camera mount from metadata.json.

    The Phase 1 collection config saved camera transform under:
      metadata['config']['sensors']['camera']['transform']
    """
    try:
        tf = metadata["config"]["sensors"]["camera"]["transform"]
        return {
            "x": float(tf.get("x", 0.0)),
            "y": float(tf.get("y", 0.0)),
            "z": float(tf.get("z", 0.0)),
            "roll": float(tf.get("roll", 0.0)),
            "pitch": float(tf.get("pitch", 0.0)),
            "yaw": float(tf.get("yaw", 0.0)),
        }
    except Exception:
        # Safe fallback for your current collection.yaml.
        return {"x": 1.5, "y": 0.0, "z": 2.4, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}


def _camera_extrinsic_from_config_or_metadata(cfg: dict, metadata: dict) -> np.ndarray:
    slam_pose_cfg = cfg.get("slam_pose", {})
    extrinsic_cfg = slam_pose_cfg.get("camera_extrinsic", {})

    if extrinsic_cfg.get("source", "metadata") == "manual":
        tf = extrinsic_cfg
    else:
        tf = _camera_extrinsic_from_metadata(metadata)
        # Allow individual overrides even when source=metadata.
        for k in ["x", "y", "z", "roll", "pitch", "yaw"]:
            if k in extrinsic_cfg and extrinsic_cfg[k] is not None:
                tf[k] = float(extrinsic_cfg[k])

    T_base_camera = _make_transform_xyz_rpy(
        x=float(tf.get("x", 0.0)),
        y=float(tf.get("y", 0.0)),
        z=float(tf.get("z", 0.0)),
        roll=float(tf.get("roll", 0.0)),
        pitch=float(tf.get("pitch", 0.0)),
        yaw=float(tf.get("yaw", 0.0)),
        degrees=True,
    )

    log.info(
        "Using T_base_camera from %s: x=%.3f, y=%.3f, z=%.3f, roll=%.3f, pitch=%.3f, yaw=%.3f",
        extrinsic_cfg.get("source", "metadata"),
        float(tf.get("x", 0.0)),
        float(tf.get("y", 0.0)),
        float(tf.get("z", 0.0)),
        float(tf.get("roll", 0.0)),
        float(tf.get("pitch", 0.0)),
        float(tf.get("yaw", 0.0)),
    )
    return T_base_camera


def main() -> None:
    parser = argparse.ArgumentParser(description="Reconstruct semantic map using RTAB-Map /odom poses.")
    parser.add_argument("--config", type=str, default="configs/slam.yaml", help="Path to Phase 3 SLAM config.")
    parser.add_argument("--run-dir", type=str, default=None, help="Override input.run_dir from config.")
    parser.add_argument("--slam-pose-csv", type=str, default=None, help="Override input.slam_pose_csv from config.")
    parser.add_argument("--frame-timestamps-csv", type=str, default=None, help="Override input.frame_timestamps_csv from config.")
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
    frame_timestamps_csv = _resolve_path(
        args.frame_timestamps_csv or input_cfg.get("frame_timestamps_csv", "outputs/slam/run_001/frame_timestamps.csv")
    )
    out_path = _resolve_path(args.out or output_cfg.get("pointcloud_path", "outputs/maps/semantic_map_slam.ply"))
    npz_path = _resolve_path(output_cfg.get("npz_path", "outputs/maps/semantic_map_slam.npz"))
    csv_path = _resolve_path(output_cfg.get("label_summary_csv", "outputs/maps/semantic_map_slam_label_summary.csv"))
    json_path = _resolve_path(output_cfg.get("label_summary_json", "outputs/maps/semantic_map_slam_label_summary.json"))

    sample_stride = int(processing_cfg.get("sample_stride", 1))
    max_samples = args.max_samples if args.max_samples is not None else processing_cfg.get("max_samples", None)
    max_time_diff = processing_cfg.get("max_time_diff_sec", 0.20)

    metadata = load_metadata(run_dir)
    dataset_rows = load_pose_rows(run_dir)
    intrinsics = metadata["camera_intrinsics"]

    frame_ts = _load_frame_timestamps(frame_timestamps_csv)
    frame_time_col = _time_column(frame_ts)
    frame_ts_by_sample = {
        int(row["sample_index"]): float(row[frame_time_col])
        for _, row in frame_ts.iterrows()
    }

    slam_rows = load_slam_pose_rows(slam_pose_csv)
    slam_df = pd.DataFrame(slam_rows)
    slam_time_col = _time_column(slam_df)
    slam_df[slam_time_col] = slam_df[slam_time_col].astype(float)
    slam_df = slam_df.sort_values(slam_time_col).reset_index(drop=True)
    slam_times = slam_df[slam_time_col].to_numpy(dtype=np.float64)

    T_base_camera = _camera_extrinsic_from_config_or_metadata(cfg, metadata)

    log.info("Input run: %s", run_dir)
    log.info("Dataset rows: %d", len(dataset_rows))
    log.info("Frame timestamps: %d from %s", len(frame_ts), frame_timestamps_csv)
    log.info("SLAM poses: %d from %s", len(slam_df), slam_pose_csv)
    log.info("Transform chain: P_odom = T_odom_base @ T_base_camera @ P_camera")

    cloud = SemanticPointCloud()
    sample_count = 0
    point_count = 0
    skipped = 0
    first_skip_messages = 0

    for sample in tqdm(iter_samples(run_dir, sample_stride=sample_stride, max_samples=max_samples), desc="SLAM-pose semantic mapping"):
        row = sample["row"]
        sample_index = _sample_index_from_row(row, sample_count)

        if sample_index not in frame_ts_by_sample:
            skipped += 1
            if first_skip_messages < 5:
                log.warning("Skipping sample_index=%s because no replay timestamp was found", sample_index)
                first_skip_messages += 1
            continue

        image_time = frame_ts_by_sample[sample_index]
        try:
            slam_pose_row, time_diff = _nearest_slam_pose_by_time(
                image_time=image_time,
                slam_df=slam_df,
                slam_times=slam_times,
                max_dt=max_time_diff,
            )
        except Exception as exc:
            skipped += 1
            if first_skip_messages < 5:
                log.warning("Skipping sample_index=%s because no matched SLAM pose was found: %s", sample_index, exc)
                first_skip_messages += 1
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

        T_odom_base = pose_row_to_matrix(slam_pose_row.to_dict())
        T_odom_camera = T_odom_base @ T_base_camera
        points_odom = transform_points(points_camera, T_odom_camera)
        cloud.add(points_odom, labels)
        sample_count += 1
        point_count += int(points_odom.shape[0])

    points, labels, colors = cloud.as_arrays()
    if points.shape[0] == 0:
        raise RuntimeError(
            "No semantic points were generated. Check frame_timestamps.csv, slam_poses.csv, "
            "max_time_diff_sec, and camera extrinsic settings."
        )

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
