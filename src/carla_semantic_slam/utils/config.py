"""Configuration helpers."""
from __future__ import annotations

from pathlib import Path
from typing import Any, Dict

import yaml


def load_yaml(path: str | Path) -> Dict[str, Any]:
    path = Path(path)
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def project_root() -> Path:
    return Path(__file__).resolve().parents[3]
