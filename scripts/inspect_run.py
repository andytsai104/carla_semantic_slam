#!/usr/bin/env python3
"""Quick sanity check for a collected CARLA semantic mapping run."""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from carla_semantic_slam.data.dataset_reader import iter_samples, load_metadata, load_pose_rows
from carla_semantic_slam.sensors.semantic_utils import label_name


def main() -> None:
    parser = argparse.ArgumentParser(description="Inspect a collected CARLA RGB-D-semantic run.")
    parser.add_argument("run_dir", type=str, nargs="?", default="data/raw/run_001")
    parser.add_argument("--max-samples", type=int, default=5)
    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    if not run_dir.is_absolute():
        run_dir = PROJECT_ROOT / run_dir

    metadata = load_metadata(run_dir)
    rows = load_pose_rows(run_dir)
    print(f"Run directory: {run_dir}")
    print(f"Pose rows: {len(rows)}")
    print(f"Map: {metadata.get('map_name', 'unknown')}")
    print(f"Camera intrinsics: {metadata.get('camera_intrinsics')}")

    all_labels: list[int] = []
    for idx, sample in enumerate(iter_samples(run_dir, max_samples=args.max_samples)):
        depth = sample["depth"]
        sem = sample["semantic"]
        unique = np.unique(sem)
        all_labels.extend([int(x) for x in unique])
        print(
            f"Sample {idx:03d}: rgb={sample['rgb'].shape}, depth={depth.shape}, "
            f"depth_range=({np.nanmin(depth):.2f}, {np.nanmax(depth):.2f}), "
            f"semantic_labels={[label_name(int(x)) for x in unique[:12]]}"
        )

    unique_all = sorted(set(all_labels))
    print("Observed labels in inspected samples:")
    for label_id in unique_all:
        print(f"  {label_id:02d}: {label_name(label_id)}")


if __name__ == "__main__":
    main()
