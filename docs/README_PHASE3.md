# Phase 3 Files Only — RTAB-Map RGB-D SLAM Layer

This package contains only the new files for Phase 3. Copy these files into the Phase 1 + Phase 2 project package.

## What Phase 3 adds

Phase 3 adds a ROS 2 replay package that publishes the saved CARLA RGB-D dataset to RTAB-Map. RTAB-Map estimates odometry / SLAM poses, and the semantic reconstruction script uses those estimated poses instead of CARLA ground-truth poses.

## Files included

```text
configs/slam.yaml
scripts/reconstruct_semantic_map_from_slam.py
src/carla_semantic_slam/slam/slam_pose_reader.py
ros2_ws/src/carla_semantic_slam_ros/package.xml
ros2_ws/src/carla_semantic_slam_ros/setup.py
ros2_ws/src/carla_semantic_slam_ros/setup.cfg
ros2_ws/src/carla_semantic_slam_ros/launch/replay_rtabmap.launch.py
ros2_ws/src/carla_semantic_slam_ros/carla_semantic_slam_ros/dataset_rgbd_publisher.py
ros2_ws/src/carla_semantic_slam_ros/carla_semantic_slam_ros/slam_pose_logger.py
docs/phase3_rtabmap_integration.md
```

## Main commands

Build the ROS package:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Run RTAB-Map replay:

```bash
cd /path/to/carla_semantic_slam_full
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 launch carla_semantic_slam_ros replay_rtabmap.launch.py \
  run_dir:=$(pwd)/data/raw/run_001 \
  pose_csv:=$(pwd)/outputs/slam/run_001/slam_poses.csv \
  rate_hz:=10.0
```

Reconstruct semantic map with SLAM poses:

```bash
python scripts/reconstruct_semantic_map_from_slam.py --config configs/slam.yaml
python scripts/visualize_pointcloud.py outputs/maps/semantic_map_slam_run_001.ply
```
