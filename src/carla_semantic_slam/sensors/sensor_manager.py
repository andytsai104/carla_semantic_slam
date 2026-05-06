"""Sensor creation, callbacks, and cleanup."""
from __future__ import annotations

from typing import Dict, List

import carla
import numpy as np

from carla_semantic_slam.data.frame_buffer import FrameBuffer
from carla_semantic_slam.sensors.camera_utils import make_transform
from carla_semantic_slam.sensors.depth_utils import carla_depth_to_meters
from carla_semantic_slam.sensors.semantic_utils import carla_semantic_to_labels


def carla_image_to_rgb(image: carla.Image) -> np.ndarray:
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))
    return array[:, :, :3][:, :, ::-1].copy()  # BGRA -> RGB


class SensorManager:
    def __init__(self, world: carla.World, parent_actor: carla.Actor, frame_buffer: FrameBuffer):
        self.world = world
        self.parent_actor = parent_actor
        self.frame_buffer = frame_buffer
        self.sensors: List[carla.Sensor] = []

    def _create_camera_blueprint(self, sensor_type: str, camera_cfg: Dict) -> carla.ActorBlueprint:
        bp = self.world.get_blueprint_library().find(sensor_type)
        bp.set_attribute("image_size_x", str(int(camera_cfg["width"])))
        bp.set_attribute("image_size_y", str(int(camera_cfg["height"])))
        bp.set_attribute("fov", str(float(camera_cfg["fov"])))
        bp.set_attribute("sensor_tick", str(float(camera_cfg.get("sensor_tick", 0.0))))
        return bp

    def attach_camera_stack(self, camera_cfg: Dict) -> List[carla.Sensor]:
        transform = make_transform(camera_cfg["transform"])
        specs = [
            ("rgb", "sensor.camera.rgb"),
            ("depth", "sensor.camera.depth"),
            ("semantic", "sensor.camera.semantic_segmentation"),
        ]
        for key, sensor_type in specs:
            bp = self._create_camera_blueprint(sensor_type, camera_cfg)
            sensor = self.world.spawn_actor(bp, transform, attach_to=self.parent_actor)
            sensor.listen(self._make_callback(key))
            self.sensors.append(sensor)
        return self.sensors

    def _make_callback(self, key: str):
        def callback(image: carla.Image) -> None:
            if key == "rgb":
                value = carla_image_to_rgb(image)
            elif key == "depth":
                value = carla_depth_to_meters(image)
            elif key == "semantic":
                value = carla_semantic_to_labels(image)
            else:
                raise ValueError(f"Unknown sensor key: {key}")
            self.frame_buffer.add(image.frame, key, value)
        return callback

    def cleanup(self) -> None:
        for sensor in reversed(self.sensors):
            try:
                if sensor is not None and sensor.is_alive:
                    sensor.stop()
                    sensor.destroy()
            except RuntimeError:
                pass
        self.sensors.clear()
