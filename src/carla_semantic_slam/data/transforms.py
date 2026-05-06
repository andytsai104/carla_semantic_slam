"""Transform conversion helpers for mapping."""
from __future__ import annotations

from typing import Dict

import numpy as np


def row_camera_matrix(row: Dict[str, str]) -> np.ndarray:
    values = [float(row[f"camera_T_world_{i}"]) for i in range(16)]
    return np.array(values, dtype=np.float64).reshape(4, 4)
