"""Actor spawning and cleanup helpers."""
from __future__ import annotations

from typing import List

import carla


class ActorManager:
    def __init__(self, world: carla.World):
        self.world = world
        self.actors: List[carla.Actor] = []

    def spawn_ego_vehicle(self, blueprint_filter: str, spawn_index: int = 0) -> carla.Vehicle:
        blueprint_library = self.world.get_blueprint_library()
        blueprints = blueprint_library.filter(blueprint_filter)
        if not blueprints:
            raise RuntimeError(f"No vehicle blueprint matched: {blueprint_filter}")

        vehicle_bp = blueprints[0]
        spawn_points = self.world.get_map().get_spawn_points()
        if not spawn_points:
            raise RuntimeError("No spawn points available in this CARLA map.")

        preferred = spawn_points[spawn_index % len(spawn_points)]
        vehicle = self.world.try_spawn_actor(vehicle_bp, preferred)
        if vehicle is None:
            for spawn_point in spawn_points:
                vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_point)
                if vehicle is not None:
                    break

        if vehicle is None:
            raise RuntimeError("Failed to spawn ego vehicle. Try changing ego_vehicle.spawn_index.")

        self.actors.append(vehicle)
        return vehicle

    def register(self, actor: carla.Actor) -> None:
        self.actors.append(actor)

    def cleanup(self) -> None:
        for actor in reversed(self.actors):
            try:
                if actor is not None and actor.is_alive:
                    actor.destroy()
            except RuntimeError:
                pass
        self.actors.clear()
