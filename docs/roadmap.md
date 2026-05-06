# Project Roadmap

## Project title

**Autonomous Exploration and Photogeometric Semantic Mapping in CARLA**

## Goal

Build a simulated autonomous mapping pipeline that uses RGB-D perception and semantic segmentation in CARLA to produce a 3D semantic map of the environment.

Final output:

```text
Where are meaningful objects and regions in 3D space?
```

## System overview

```text
CARLA ego vehicle
→ synchronized RGB + depth + semantic cameras
→ saved dataset
→ RGB-D back-projection
→ semantic point cloud
→ optional RTAB-Map SLAM pose replacement
```

## Phase 1: CARLA data collection

Status: implemented.

### Objective

Collect synchronized RGB, depth, semantic segmentation, and camera pose data from CARLA.

### Main files

```text
configs/collection.yaml
scripts/collect_data.py
src/carla_semantic_slam/sim/
src/carla_semantic_slam/sensors/
src/carla_semantic_slam/data/dataset_writer.py
```

### Output

```text
data/raw/run_001/
├── rgb/
├── depth/
├── semantic/
├── poses.csv
└── metadata.json
```

## Phase 2: Ground-truth-pose semantic mapping

Status: implemented.

### Objective

Use CARLA ground-truth camera pose to back-project semantic depth pixels into the global frame.

### Why this matters

This is the stable baseline. Even if RTAB-Map is unstable, this proves the semantic mapping pipeline works.

### Main files

```text
configs/mapping.yaml
scripts/reconstruct_semantic_map.py
src/carla_semantic_slam/mapping/backprojection.py
src/carla_semantic_slam/mapping/semantic_pointcloud.py
```

### Output

```text
outputs/maps/semantic_map_run_001.ply
outputs/maps/semantic_map_run_001.npz
outputs/maps/semantic_map_run_001_label_summary.csv
```

## Phase 3: ROS 2 + RTAB-Map integration

Status: implemented as an integration layer.

### Objective

Replay the saved RGB-D dataset into ROS 2, estimate motion using RTAB-Map RGB-D odometry, save `/odom`, and reconstruct the semantic map using estimated poses.

### Main files

```text
ros2_ws/src/carla_semantic_slam_ros/carla_semantic_slam_ros/dataset_rgbd_publisher.py
ros2_ws/src/carla_semantic_slam_ros/carla_semantic_slam_ros/slam_pose_logger.py
ros2_ws/src/carla_semantic_slam_ros/launch/replay_rtabmap.launch.py
scripts/reconstruct_semantic_map_from_slam.py
src/carla_semantic_slam/slam/slam_pose_reader.py
```

### Output

```text
outputs/slam/run_001/slam_poses.csv
outputs/maps/semantic_map_slam_run_001.ply
```

## Recommended final report framing

Use this hierarchy:

1. **Main result**: semantic point cloud from RGB-D + semantic segmentation + CARLA GT pose.
2. **SLAM extension**: RTAB-Map RGB-D odometry pipeline and preliminary SLAM-pose semantic map.
3. **Limitations**: RTAB-Map sensitivity to synchronization, texture, depth scale, frame conventions, and replay rate.

## Future work

- Replace identity camera extrinsic in ROS launch with the actual CARLA camera transform.
- Add direct CARLA-to-ROS live bridge.
- Publish semantic segmentation as a ROS topic.
- Fuse semantic labels incrementally instead of post-processing.
- Compare CARLA ground-truth trajectory against RTAB-Map trajectory.
- Try loop closure evaluation across multiple CARLA towns.
