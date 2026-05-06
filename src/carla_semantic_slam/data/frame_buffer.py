"""Synchronize CARLA sensor frames by simulation frame id."""
from __future__ import annotations

import threading
from collections import defaultdict
from typing import Any, Dict, Iterable, Optional


class FrameBuffer:
    def __init__(self, required_keys: Iterable[str]):
        self.required_keys = set(required_keys)
        self._data: Dict[int, Dict[str, Any]] = defaultdict(dict)
        self._lock = threading.Lock()

    def add(self, frame_id: int, key: str, value: Any) -> None:
        with self._lock:
            self._data[int(frame_id)][key] = value

    def get_complete(self, frame_id: int) -> Optional[Dict[str, Any]]:
        with self._lock:
            sample = self._data.get(int(frame_id))
            if sample is None:
                return None
            if self.required_keys.issubset(sample.keys()):
                return dict(sample)
            return None

    def pop_complete(self, frame_id: int) -> Optional[Dict[str, Any]]:
        with self._lock:
            sample = self._data.get(int(frame_id))
            if sample is None:
                return None
            if self.required_keys.issubset(sample.keys()):
                return dict(self._data.pop(int(frame_id)))
            return None

    def prune_before(self, frame_id: int) -> None:
        with self._lock:
            for key in [k for k in self._data if k < frame_id]:
                self._data.pop(key, None)
