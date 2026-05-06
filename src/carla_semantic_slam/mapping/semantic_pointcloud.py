"""Semantic point-cloud accumulation and summaries."""
from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
from typing import Dict

import numpy as np

from carla_semantic_slam.sensors.semantic_utils import label_name, semantic_labels_to_palette


@dataclass
class SemanticPointCloud:
    points: list[np.ndarray] = field(default_factory=list)
    labels: list[np.ndarray] = field(default_factory=list)

    def add(self, points_xyz: np.ndarray, labels: np.ndarray) -> None:
        if points_xyz.size == 0:
            return
        if points_xyz.shape[0] != labels.shape[0]:
            raise ValueError("points and labels must have the same length")
        self.points.append(points_xyz.astype(np.float32))
        self.labels.append(labels.astype(np.uint8))

    def as_arrays(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if not self.points:
            return (
                np.zeros((0, 3), dtype=np.float32),
                np.zeros((0,), dtype=np.uint8),
                np.zeros((0, 3), dtype=np.uint8),
            )
        points = np.vstack(self.points)
        labels = np.concatenate(self.labels)
        colors = semantic_labels_to_palette(labels)
        return points, labels, colors

    def label_summary(self) -> list[Dict]:
        _, labels, _ = self.as_arrays()
        counts = Counter(int(x) for x in labels.tolist())
        total = int(labels.size)
        rows = []
        for label_id, count in sorted(counts.items(), key=lambda x: (-x[1], x[0])):
            rows.append(
                {
                    "label_id": label_id,
                    "label_name": label_name(label_id),
                    "point_count": int(count),
                    "percentage": float(count / total * 100.0) if total else 0.0,
                }
            )
        return rows
