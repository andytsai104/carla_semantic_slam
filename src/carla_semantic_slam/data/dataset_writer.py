"""Dataset writer for synchronized CARLA RGB-D-semantic samples."""
from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Dict, Optional

import cv2
import numpy as np

from carla_semantic_slam.sensors.camera_utils import carla_transform_to_matrix, transform_to_dict
from carla_semantic_slam.sensors.semantic_utils import semantic_labels_to_palette


class DatasetWriter:
    def __init__(self, output_dir: str | Path, run_name: str, metadata: Dict):
        self.run_dir = Path(output_dir) / run_name
        self.rgb_dir = self.run_dir / "rgb"
        self.depth_dir = self.run_dir / "depth"
        self.depth_viz_dir = self.run_dir / "depth_viz"
        self.semantic_dir = self.run_dir / "semantic"
        self.semantic_viz_dir = self.run_dir / "semantic_viz"

        for directory in [self.rgb_dir, self.depth_dir, self.depth_viz_dir, self.semantic_dir, self.semantic_viz_dir]:
            directory.mkdir(parents=True, exist_ok=True)

        self.metadata_path = self.run_dir / "metadata.json"
        self.poses_path = self.run_dir / "poses.csv"
        self._pose_file = self.poses_path.open("w", newline="", encoding="utf-8")
        self._pose_writer: Optional[csv.DictWriter] = None

        with self.metadata_path.open("w", encoding="utf-8") as f:
            json.dump(metadata, f, indent=2)

    def save_sample(self, sample_index: int, carla_frame: int, sample: Dict, vehicle_transform, camera_transform) -> None:
        stem = f"{sample_index:06d}"
        rgb = sample["rgb"]
        depth_m = sample["depth"]
        semantic = sample["semantic"]

        cv2.imwrite(str(self.rgb_dir / f"{stem}.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        np.save(self.depth_dir / f"{stem}.npy", depth_m.astype(np.float32))

        depth_clip = np.clip(depth_m, 0.0, 80.0)
        depth_viz = (255.0 * (depth_clip / 80.0)).astype(np.uint8)
        cv2.imwrite(str(self.depth_viz_dir / f"{stem}.png"), depth_viz)

        cv2.imwrite(str(self.semantic_dir / f"{stem}.png"), semantic.astype(np.uint8))
        semantic_viz = semantic_labels_to_palette(semantic)
        cv2.imwrite(str(self.semantic_viz_dir / f"{stem}.png"), cv2.cvtColor(semantic_viz, cv2.COLOR_RGB2BGR))

        row = {
            "sample_index": sample_index,
            "carla_frame": carla_frame,
            "rgb_path": f"rgb/{stem}.png",
            "depth_path": f"depth/{stem}.npy",
            "semantic_path": f"semantic/{stem}.png",
            **transform_to_dict(vehicle_transform, prefix="vehicle"),
            **transform_to_dict(camera_transform, prefix="camera"),
        }

        camera_matrix = carla_transform_to_matrix(camera_transform).reshape(-1)
        for i, value in enumerate(camera_matrix):
            row[f"camera_T_world_{i}"] = float(value)

        if self._pose_writer is None:
            self._pose_writer = csv.DictWriter(self._pose_file, fieldnames=list(row.keys()))
            self._pose_writer.writeheader()

        self._pose_writer.writerow(row)
        self._pose_file.flush()

    def close(self) -> None:
        if not self._pose_file.closed:
            self._pose_file.close()
