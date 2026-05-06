"""Vehicle control helpers.

For the first working pipeline, CARLA autopilot is enough. This file exists so
waypoint exploration can be added later without changing the main collection script.
"""
from __future__ import annotations

import carla


def enable_autopilot(vehicle: carla.Vehicle, enabled: bool = True, traffic_manager_port: int = 8000) -> None:
    vehicle.set_autopilot(enabled, traffic_manager_port)
