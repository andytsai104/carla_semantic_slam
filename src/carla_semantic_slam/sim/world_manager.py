"""World-level CARLA settings.

This module is intentionally small for now. It keeps synchronous-mode setup separate
from the main script so the project can grow cleanly when RTAB-Map/ROS integration is added.
"""
from __future__ import annotations

import carla


class WorldManager:
    def __init__(self, world: carla.World):
        self.world = world
        self.previous_settings: carla.WorldSettings | None = None

    def enable_synchronous_mode(self, fixed_delta_seconds: float = 0.05) -> None:
        self.previous_settings = self.world.get_settings()
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = fixed_delta_seconds
        self.world.apply_settings(settings)

    def restore(self) -> None:
        if self.previous_settings is not None:
            self.world.apply_settings(self.previous_settings)
            self.previous_settings = None
