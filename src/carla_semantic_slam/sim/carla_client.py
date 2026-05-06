"""CARLA client connection helpers."""
from __future__ import annotations

from typing import Optional

import carla


def connect_client(host: str = "localhost", port: int = 2000, timeout: float = 10.0) -> carla.Client:
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    return client


def get_or_load_world(client: carla.Client, town: Optional[str] = None) -> carla.World:
    if town:
        return client.load_world(town)
    return client.get_world()
