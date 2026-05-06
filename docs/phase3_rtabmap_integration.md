# Phase 3 — RTAB-Map RGB-D SLAM Integration

Phase 3 replaces the Phase 2 baseline pose source with an estimated RGB-D odometry / SLAM pose from RTAB-Map.

The overall flow is:

```text
saved CARLA RGB-D dataset
  → ROS 2 dataset replay node
  → RTAB-Map rgbd_odometry + rtabmap
  → slam_poses.csv
  → semantic point cloud reconstruction using SLAM poses
```

## 1. Install ROS 2 dependencies

This assumes ROS 2 Humble is already installed.

```bash
sudo apt update
sudo apt install ros-humble-rtabmap-ros ros-humble-cv-bridge ros-humble-tf2-ros
```

## 2. Copy the Phase 3 files into your project

From your project root:

```bash
cp -r /path/to/carla_semantic_slam_phase3_only/* .
```

This will add:

```text
configs/slam.yaml
scripts/reconstruct_semantic_map_from_slam.py
src/carla_semantic_slam/slam/slam_pose_reader.py
ros2_ws/src/carla_semantic_slam_ros/...
docs/phase3_rtabmap_integration.md
```

## 3. Build the ROS 2 replay package

From the project root:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 4. Replay a collected run into RTAB-Map

Use an absolute path for `run_dir` to avoid ROS launch path confusion.

```bash
cd /path/to/carla_semantic_slam_full
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 launch carla_semantic_slam_ros replay_rtabmap.launch.py \
  run_dir:=$(pwd)/data/raw/run_001 \
  pose_csv:=$(pwd)/outputs/slam/run_001/slam_poses.csv \
  rate_hz:=10.0
```

The launch file starts:

- `dataset_rgbd_publisher`: publishes saved RGB, depth, and camera info topics
- `rgbd_odometry`: estimates RGB-D odometry
- `rtabmap`: builds the graph/map
- `slam_pose_logger`: writes `/odom` poses to CSV

## 5. Check topics

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 topic list
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_rect_raw
ros2 topic hz /odom
```

Expected key topics:

```text
/camera/color/image_raw
/camera/depth/image_rect_raw
/camera/color/camera_info
/odom
/rtabmap/mapData
/rtabmap/cloud_map
```

## 6. Reconstruct semantic map using SLAM poses

After the launch finishes or after enough poses are logged:

```bash
python scripts/reconstruct_semantic_map_from_slam.py --config configs/slam.yaml
```

Output:

```text
outputs/maps/semantic_map_slam_run_001.ply
outputs/maps/semantic_map_slam_run_001.npz
outputs/maps/semantic_map_slam_run_001_label_summary.csv
outputs/maps/semantic_map_slam_run_001_label_summary.json
```

Visualize:

```bash
python scripts/visualize_pointcloud.py outputs/maps/semantic_map_slam_run_001.ply
```

## 7. Important notes

The Phase 2 version uses CARLA ground-truth camera poses. The Phase 3 version uses RTAB-Map / RGB-D odometry poses saved from `/odom`.

If the SLAM-pose point cloud looks rotated, mirrored, or shifted, check:

1. Whether `/odom` is estimating `base_link` or `camera_link` motion.
2. Whether the static transform from `base_link` to `camera_link` should match your CARLA camera mount.
3. Whether the RGB and depth frames are synchronized.
4. Whether RTAB-Map has enough visual features in the selected CARLA route.

For the final report, use Phase 2 as the controlled photogeometric baseline and Phase 3 as the real SLAM-pose extension.
