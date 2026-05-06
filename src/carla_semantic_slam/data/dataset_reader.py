"""Utilities for loading collected CARLA semantic mapping runs."""
from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Dict, Iterator, Optional

import cv2
import numpy as np


def load_metadata(run_dir: str | Path) -> Dict:
    run_dir = Path(run_dir)
    metadata_path = run_dir / "metadata.json"
    if not metadata_path.exists():
        raise FileNotFoundError(f"Missing metadata file: {metadata_path}")
    with metadata_path.open("r", encoding="utf-8") as f:
        return json.load(f)


def load_pose_rows(run_dir: str | Path) -> list[Dict[str, str]]:
    run_dir = Path(run_dir)
    poses_path = run_dir / "poses.csv"
    if not poses_path.exists():
        raise FileNotFoundError(f"Missing poses file: {poses_path}")
    with poses_path.open("r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def iter_samples(
    run_dir: str | Path,
    sample_stride: int = 1,
    max_samples: Optional[int] = None,
) -> Iterator[Dict]:
    """Yield synchronized RGB/depth/semantic samples from a collected run.

    Args:
        run_dir: Directory containing rgb/, depth/, semantic/, poses.csv, metadata.json.
        sample_stride: Use every nth saved row from poses.csv.
        max_samples: Optional maximum number of yielded samples.
    """
    run_dir = Path(run_dir)
    rows = load_pose_rows(run_dir)
    emitted = 0

    for row_idx, row in enumerate(rows):
        if sample_stride > 1 and row_idx % sample_stride != 0:
            continue
        if max_samples is not None and emitted >= max_samples:
            break

        rgb_path = run_dir / row["rgb_path"]
        depth_path = run_dir / row["depth_path"]
        semantic_path = run_dir / row["semantic_path"]

        if not rgb_path.exists() or not depth_path.exists() or not semantic_path.exists():
            missing = [str(p) for p in [rgb_path, depth_path, semantic_path] if not p.exists()]
            raise FileNotFoundError(f"Missing sample file(s): {missing}")

        rgb_bgr = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
        semantic = cv2.imread(str(semantic_path), cv2.IMREAD_UNCHANGED)
        if rgb_bgr is None:
            raise ValueError(f"Could not read RGB image: {rgb_path}")
        if semantic is None:
            raise ValueError(f"Could not read semantic image: {semantic_path}")

        rgb = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2RGB)
        depth = np.load(depth_path).astype(np.float32)

        yield {
            "row": row,
            "rgb": rgb,
            "depth": depth,
            "semantic": semantic.astype(np.uint8),
        }
        emitted += 1
