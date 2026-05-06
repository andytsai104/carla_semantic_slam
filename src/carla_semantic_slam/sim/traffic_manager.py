"""Traffic manager setup."""
from __future__ import annotations

from typing import Any, Dict, Optional

import carla


def setup_traffic_manager(client: carla.Client, cfg: Dict[str, Any]) -> Optional[carla.TrafficManager]:
    if not cfg.get("enabled", True):
        return None
    port = int(cfg.get("port", 8000))
    tm = client.get_trafficmanager(port)
    tm.set_global_distance_to_leading_vehicle(float(cfg.get("global_distance_to_leading_vehicle", 2.5)))
    tm.set_synchronous_mode(bool(cfg.get("synchronous_mode", True)))
    if "seed" in cfg:
        tm.set_random_device_seed(int(cfg["seed"]))
    return tm
