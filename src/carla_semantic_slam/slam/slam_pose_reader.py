"""Utilities for loading estimated SLAM/odometry poses from Phase 3."""
from __future__ import annotations

import csv
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np


def quaternion_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.eye(3, dtype=np.float64)
    qx, qy, qz, qw = q / norm
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ], dtype=np.float64)


def pose_row_to_matrix(row: Dict[str, str]) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = quaternion_to_matrix(
        float(row["qx"]), float(row["qy"]), float(row["qz"]), float(row["qw"])
    )
    transform[:3, 3] = [float(row["x"]), float(row["y"]), float(row["z"])]
    return transform


def load_slam_pose_rows(csv_path: str | Path) -> List[Dict[str, str]]:
    path = Path(csv_path)
    if not path.exists():
        raise FileNotFoundError(f"Missing SLAM pose CSV: {path}")
    with path.open("r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"SLAM pose CSV is empty: {path}")
    return rows


def _row_time(row: Dict[str, str]) -> float:
    if "time_sec" in row and row["time_sec"] not in (None, ""):
        return float(row["time_sec"])
    return float(row["stamp_sec"]) + float(row["stamp_nanosec"]) * 1e-9


def nearest_slam_pose(
    dataset_row: Dict[str, str],
    slam_rows: List[Dict[str, str]],
    max_time_diff_sec: Optional[float] = 0.20,
) -> tuple[Dict[str, str], float]:
    """Return a SLAM pose matched to a dataset row.

    Preferred matching is timestamp-based when the CARLA dataset has a timestamp.
    The current Phase 1 writer may not save timestamps, so the fallback is order-
    based matching using sample_index. That fallback is useful for the replay mode
    where the dataset publisher uses live ROS time to avoid TF extrapolation.
    """
    if "timestamp" in dataset_row and dataset_row["timestamp"] not in (None, ""):
        target_t = float(dataset_row["timestamp"])
        times = np.array([_row_time(r) for r in slam_rows], dtype=np.float64)
        idx = int(np.argmin(np.abs(times - target_t)))
        diff = float(abs(times[idx] - target_t))
        if max_time_diff_sec is not None and diff > float(max_time_diff_sec):
            raise ValueError(
                f"Nearest SLAM pose is {diff:.3f}s away from dataset timestamp {target_t:.3f}; "
                f"increase max_time_diff_sec if this is expected."
            )
        return slam_rows[idx], diff

    if "time_sec" in dataset_row and dataset_row["time_sec"] not in (None, ""):
        target_t = float(dataset_row["time_sec"])
        times = np.array([_row_time(r) for r in slam_rows], dtype=np.float64)
        idx = int(np.argmin(np.abs(times - target_t)))
        diff = float(abs(times[idx] - target_t))
        if max_time_diff_sec is not None and diff > float(max_time_diff_sec):
            raise ValueError(
                f"Nearest SLAM pose is {diff:.3f}s away from dataset time {target_t:.3f}; "
                f"increase max_time_diff_sec if this is expected."
            )
        return slam_rows[idx], diff

    # Fallback for Phase 1 datasets without timestamps. Use sample_index to pick
    # the closest available odometry row. If odometry has fewer rows than frames,
    # this still gives a rough baseline instead of failing immediately.
    sample_idx = int(dataset_row.get("sample_index", 0))
    if len(slam_rows) == 1:
        return slam_rows[0], 0.0
    idx = min(max(sample_idx, 0), len(slam_rows) - 1)
    return slam_rows[idx], 0.0
