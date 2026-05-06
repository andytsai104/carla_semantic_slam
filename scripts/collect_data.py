#!/usr/bin/env python3
"""Collect synchronized RGB, depth, semantic segmentation, and pose data from CARLA.

Run from the project root:
    python scripts/collect_data.py --config configs/collection.yaml
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import carla
from tqdm import tqdm

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from carla_semantic_slam.data.dataset_writer import DatasetWriter
from carla_semantic_slam.data.frame_buffer import FrameBuffer
from carla_semantic_slam.sensors.camera_utils import camera_intrinsics
from carla_semantic_slam.sensors.sensor_manager import SensorManager
from carla_semantic_slam.sim.actor_manager import ActorManager
from carla_semantic_slam.sim.carla_client import connect_client, get_or_load_world
from carla_semantic_slam.sim.traffic_manager import setup_traffic_manager
from carla_semantic_slam.sim.vehicle_controller import enable_autopilot
from carla_semantic_slam.sim.world_manager import WorldManager
from carla_semantic_slam.utils.config import load_yaml
from carla_semantic_slam.utils.logging_utils import get_logger

log = get_logger()


def main() -> None:
    parser = argparse.ArgumentParser(description="Collect synchronized CARLA RGB-D-semantic data.")
    parser.add_argument("--config", type=str, default="configs/collection.yaml", help="Path to YAML config file.")
    args = parser.parse_args()

    cfg_path = Path(args.config)
    if not cfg_path.is_absolute():
        cfg_path = PROJECT_ROOT / cfg_path
    cfg = load_yaml(cfg_path)

    carla_cfg = cfg["carla"]
    traffic_cfg = cfg.get("traffic_manager", {})
    ego_cfg = cfg["ego_vehicle"]
    camera_cfg = cfg["sensors"]["camera"]
    collection_cfg = cfg["collection"]

    client = connect_client(
        host=str(carla_cfg.get("host", "localhost")),
        port=int(carla_cfg.get("port", 2000)),
        timeout=float(carla_cfg.get("timeout", 10.0)),
    )
    world = get_or_load_world(client, carla_cfg.get("town"))

    world_manager = WorldManager(world)
    actor_manager: ActorManager | None = None
    sensor_manager: SensorManager | None = None
    writer: DatasetWriter | None = None
    ego_vehicle: carla.Vehicle | None = None

    try:
        if bool(carla_cfg.get("synchronous_mode", True)):
            world_manager.enable_synchronous_mode(float(carla_cfg.get("fixed_delta_seconds", 0.05)))

        setup_traffic_manager(client, traffic_cfg)

        actor_manager = ActorManager(world)
        ego_vehicle = actor_manager.spawn_ego_vehicle(
            blueprint_filter=str(ego_cfg.get("blueprint", "vehicle.tesla.model3")),
            spawn_index=int(ego_cfg.get("spawn_index", 0)),
        )

        if bool(ego_cfg.get("autopilot", True)):
            enable_autopilot(ego_vehicle, True, int(traffic_cfg.get("port", 8000)))

        frame_buffer = FrameBuffer(required_keys=["rgb", "depth", "semantic"])
        sensor_manager = SensorManager(world, ego_vehicle, frame_buffer)
        sensors = sensor_manager.attach_camera_stack(camera_cfg)
        rgb_sensor = sensors[0]

        intrinsics = camera_intrinsics(
            width=int(camera_cfg["width"]),
            height=int(camera_cfg["height"]),
            fov_deg=float(camera_cfg["fov"]),
        )

        metadata = {
            "carla_map": world.get_map().name,
            "config": cfg,
            "camera_intrinsics": intrinsics,
            "camera_transform_relative_to_vehicle": camera_cfg["transform"],
            "pose_source": "carla_ground_truth",
            "description": "Synchronized CARLA RGB/depth/semantic data with ground-truth vehicle and camera poses.",
        }

        writer = DatasetWriter(
            output_dir=PROJECT_ROOT / str(collection_cfg.get("output_dir", "data/raw")),
            run_name=str(collection_cfg.get("run_name", "run_001")),
            metadata=metadata,
        )

        log.info("Connected to CARLA map: %s", world.get_map().name)
        log.info("Ego vehicle spawned: %s", ego_vehicle.type_id)
        log.info("Camera stack attached: RGB + depth + semantic")
        log.info("Writing dataset to: %s", writer.run_dir)

        for _ in range(int(collection_cfg.get("warmup_ticks", 20))):
            world.tick()

        num_samples = int(collection_cfg.get("num_samples", 500))
        save_every_n = max(1, int(collection_cfg.get("save_every_n_frames", 1)))
        max_wait_seconds = float(collection_cfg.get("max_sync_wait_seconds", 0.5))

        saved = 0
        pbar = tqdm(total=num_samples, desc="Collecting synchronized samples")
        while saved < num_samples:
            frame_id = world.tick()

            if frame_id % save_every_n != 0:
                frame_buffer.prune_before(frame_id - 20)
                continue

            sample = frame_buffer.get_complete(frame_id)
            start_time = time.time()
            while sample is None and (time.time() - start_time) < max_wait_seconds:
                time.sleep(0.002)
                sample = frame_buffer.get_complete(frame_id)

            if sample is None:
                log.warning("Skipping CARLA frame %s because the synchronized sample is incomplete", frame_id)
                frame_buffer.prune_before(frame_id - 20)
                continue

            writer.save_sample(
                sample_index=saved,
                carla_frame=frame_id,
                sample=sample,
                vehicle_transform=ego_vehicle.get_transform(),
                camera_transform=rgb_sensor.get_transform(),
            )

            frame_buffer.prune_before(frame_id - 20)
            saved += 1
            pbar.update(1)

        pbar.close()
        log.info("Saved %d synchronized samples", saved)
        log.info("Dataset complete: %s", writer.run_dir)
        log.info("Metadata: %s", writer.metadata_path)
        log.info("Poses: %s", writer.poses_path)

    finally:
        if writer is not None:
            writer.close()
        if ego_vehicle is not None:
            try:
                enable_autopilot(ego_vehicle, False, int(traffic_cfg.get("port", 8000)))
            except RuntimeError:
                pass
        if sensor_manager is not None:
            sensor_manager.cleanup()
        if actor_manager is not None:
            actor_manager.cleanup()
        world_manager.restore()
        log.info("Cleanup complete")


if __name__ == "__main__":
    main()
