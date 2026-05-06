# Autonomous Exploration and Photogeometric Semantic Mapping in CARLA

This repository implements a staged CARLA-based semantic SLAM pipeline for building a 3D semantic map from RGB-D and semantic segmentation observations.

The current project is organized around three working stages:

1. **Data collection in CARLA**: spawn an ego vehicle, attach synchronized RGB/depth/semantic cameras, and save frames with camera/vehicle poses.
2. **Baseline semantic reconstruction**: use CARLA ground-truth camera poses to back-project depth + semantic labels into a global semantic point cloud.
3. **ROS 2 + RTAB-Map integration**: replay the saved RGB-D dataset into ROS 2, estimate RGB-D odometry/SLAM poses using RTAB-Map, log `/odom`, and reconstruct the semantic point cloud using estimated poses.

The safest project strategy is to treat Phase 2 as the stable baseline and Phase 3 as the realistic SLAM extension.

---

## Repository structure

```text
carla_semantic_slam/
├── configs/
│   ├── collection.yaml          # CARLA sensor/data collection settings
│   ├── mapping.yaml             # GT-pose semantic reconstruction settings
│   └── slam.yaml                # SLAM-pose reconstruction settings
├── scripts/
│   ├── collect_data.py          # Phase 1: collect RGB/depth/semantic/pose data
│   ├── inspect_run.py           # Check saved dataset integrity
│   ├── reconstruct_semantic_map.py
│   ├── reconstruct_semantic_map_from_slam.py
│   └── visualize_pointcloud.py
├── src/carla_semantic_slam/
│   ├── sim/                     # CARLA client, world, actors, traffic, vehicle control
│   ├── sensors/                 # Camera/depth/semantic utilities
│   ├── data/                    # Dataset reader/writer, frame buffer, transforms
│   ├── mapping/                 # Back-projection and semantic point-cloud creation
│   ├── slam/                    # SLAM pose reader and future RTAB-Map utilities
│   ├── visualization/           # Open3D visualization helpers
│   └── utils/                   # Config and logging helpers
├── ros2_ws/
│   └── src/carla_semantic_slam_ros/
│       ├── carla_semantic_slam_ros/
│       │   ├── dataset_rgbd_publisher.py
│       │   └── slam_pose_logger.py
│       └── launch/replay_rtabmap.launch.py
├── data/
│   ├── raw/                     # Collected CARLA runs
│   └── processed/
├── outputs/
│   ├── maps/                    # Semantic point clouds
│   ├── figures/
│   ├── videos/
│   ├── logs/
│   └── slam/                    # RTAB-Map/odometry pose CSVs
├── docs/
│   ├── roadmap.md
│   └── phase3_rtabmap_integration.md
├── requirements.txt
├── pyproject.toml
└── README.md
```

---

## Environment setup

Recommended environment:

- Ubuntu 22.04
- Python 3.10
- CARLA 0.9.16
- ROS 2 Humble
- RTAB-Map ROS packages

Create the Python environment:

```bash
conda create -n carla_semantic python=3.10 -y
conda activate carla_semantic
pip install -r requirements.txt
```

Install the CARLA Python wheel, adjusting the path if needed:

```bash
pip install ~/CARLA_0.9.16/PythonAPI/carla/dist/carla-0.9.16-cp310-cp310-linux_x86_64.whl
```

If your wheel name is different, check with:

```bash
ls ~/CARLA_0.9.16/PythonAPI/carla/dist/
```

Install ROS 2 / RTAB-Map dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-rtabmap-ros \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-tf2-ros
```

Build the ROS 2 workspace:

```bash
cd /home/andy/ros2_projects/carla_semantic_slam/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Phase 1: collect CARLA RGB-D-semantic data

Start CARLA in one terminal:

```bash
cd ~/CARLA_0.9.16
./CarlaUE4.sh
```

Then collect data from the project root:

```bash
cd /home/andy/ros2_projects/carla_semantic_slam
conda activate carla_semantic
python scripts/collect_data.py --config configs/collection.yaml
```

Expected output:

```text
data/raw/run_001/
├── rgb/
├── depth/
├── depth_viz/
├── semantic/
├── semantic_viz/
├── poses.csv
└── metadata.json
```

Check the run:

```bash
python scripts/inspect_run.py data/raw/run_001 --max-samples 5
```

CARLA is only needed for this collection step. After the dataset is saved, Phase 2 and Phase 3 can run without CARLA.

---

## Phase 2: baseline semantic mapping with CARLA ground-truth pose

This stage uses the saved CARLA camera pose in `poses.csv` as the camera trajectory. It is the stable baseline for the final report/demo.

Run a small test first:

```bash
cd /home/andy/ros2_projects/carla_semantic_slam
conda activate carla_semantic
python scripts/reconstruct_semantic_map.py --config configs/mapping.yaml --max-samples 30
```

Then visualize:

```bash
python scripts/visualize_pointcloud.py outputs/maps/semantic_map_run_001.ply
```

Run the full dataset when the small test looks correct:

```bash
python scripts/reconstruct_semantic_map.py --config configs/mapping.yaml
```

Expected outputs:

```text
outputs/maps/semantic_map_run_001.ply
outputs/maps/semantic_map_run_001.npz
outputs/maps/semantic_map_run_001_label_summary.csv
outputs/maps/semantic_map_run_001_label_summary.json
```

---

## Phase 3: ROS 2 replay + RTAB-Map RGB-D SLAM

Phase 3 replays the saved RGB-D dataset into ROS 2 and lets RTAB-Map estimate odometry/SLAM poses.

You do **not** need to start CARLA for Phase 3.

### 1. Build/source the ROS 2 package

```bash
cd /home/andy/ros2_projects/carla_semantic_slam/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Test the dataset publisher only

Before launching RTAB-Map, make sure the replay topics publish at a stable rate.

Terminal 1:

```bash
cd /home/andy/ros2_projects/carla_semantic_slam
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 run carla_semantic_slam_ros dataset_rgbd_publisher \
  --ros-args \
  -p run_dir:=/home/andy/ros2_projects/carla_semantic_slam/data/raw/run_001 \
  -p rate_hz:=2.0 \
  -p use_wall_time:=true
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source /home/andy/ros2_projects/carla_semantic_slam/ros2_ws/install/setup.bash

timeout 10 ros2 topic hz /camera/color/image_raw
timeout 10 ros2 topic hz /camera/depth/image_rect_raw
timeout 10 ros2 topic hz /camera/color/camera_info
```

Target result:

```text
/camera/color/image_raw        about 2 Hz
/camera/depth/image_rect_raw   about 2 Hz
/camera/color/camera_info      about 2 Hz
```

Also check headers:

```bash
ros2 topic echo /camera/color/image_raw/header --once
ros2 topic echo /camera/depth/image_rect_raw/header --once
ros2 topic echo /camera/color/camera_info/header --once
```

For the current replay setup, timestamps should be live ROS time, not tiny dataset times like `31 sec`.

### 3. Launch RTAB-Map replay

Stop the standalone publisher first, then run:

```bash
cd /home/andy/ros2_projects/carla_semantic_slam
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

mkdir -p outputs/slam/run_001

ros2 launch carla_semantic_slam_ros replay_rtabmap.launch.py \
  run_dir:=/home/andy/ros2_projects/carla_semantic_slam/data/raw/run_001 \
  pose_csv:=/home/andy/ros2_projects/carla_semantic_slam/outputs/slam/run_001/slam_poses.csv \
  rate_hz:=2.0 \
  use_wall_time:=true
```

Check odometry:

```bash
timeout 15 ros2 topic hz /odom
ros2 topic echo /odom --once
```

Check the pose log:

```bash
head outputs/slam/run_001/slam_poses.csv
tail outputs/slam/run_001/slam_poses.csv
```

### 4. Reconstruct semantic map using SLAM poses

```bash
conda activate carla_semantic
python scripts/reconstruct_semantic_map_from_slam.py --config configs/slam.yaml
python scripts/visualize_pointcloud.py outputs/maps/semantic_map_slam_run_001.ply
```

Expected outputs:

```text
outputs/slam/run_001/slam_poses.csv
outputs/maps/semantic_map_slam_run_001.ply
outputs/maps/semantic_map_slam_run_001.npz
outputs/maps/semantic_map_slam_run_001_label_summary.csv
outputs/maps/semantic_map_slam_run_001_label_summary.json
```

---

## Core projection idea

For each depth pixel with semantic label:

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth(u, v)
```

Then transform the point into the global map frame:

```text
P_map = T_map_camera @ P_camera
```

Phase 2 gets `T_map_camera` from CARLA ground truth. Phase 3 gets the pose from RTAB-Map/odometry.

---

## Troubleshooting

### CARLA Python API import fails

Check Python version and wheel:

```bash
python --version
python -c "import carla; print(carla.__file__)"
```

### `Missing poses.csv`

Make sure the launch path points to the project root dataset, not inside `ros2_ws`:

```bash
ls /home/andy/ros2_projects/carla_semantic_slam/data/raw/run_001/poses.csv
```

Use absolute paths in the launch command.

### Camera topics are unstable

Test only the publisher first:

```bash
ros2 run carla_semantic_slam_ros dataset_rgbd_publisher \
  --ros-args \
  -p run_dir:=/home/andy/ros2_projects/carla_semantic_slam/data/raw/run_001 \
  -p rate_hz:=2.0 \
  -p use_wall_time:=true
```

Then check topic rates. Do not debug RTAB-Map until RGB, depth, and camera info are stable.

### RTAB-Map says it did not receive synchronized data

The launch file uses:

```text
approx_sync = true
sync_queue_size = 30
topic_queue_size = 30
```

If it still fails, reduce replay speed:

```bash
rate_hz:=1.0
```

### `/odom` does not publish

Possible causes:

- RGB/depth/camera_info topics are not stable.
- Depth encoding or scale is wrong.
- The visual scene does not have enough texture/features.
- The vehicle motion is too fast between frames.
- RTAB-Map optical/base frame assumptions need adjustment.

For the final project, keep Phase 2 as the reliable result and present Phase 3 as an integration extension if odometry is unstable.

---

## Suggested final demo flow

1. Show CARLA environment and ego vehicle data collection.
2. Show saved RGB/depth/semantic frames.
3. Show Phase 2 semantic point cloud using CARLA ground-truth pose.
4. Show ROS 2 replay topics.
5. Show RTAB-Map odometry/trajectory if stable.
6. Show Phase 3 SLAM-pose semantic map as an extension.
