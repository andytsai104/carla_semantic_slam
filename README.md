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

Phase 3 replaces the CARLA ground-truth pose used in Phase 2 with an estimated pose from a ROS 2 RGB-D SLAM pipeline. The saved CARLA dataset is replayed as ROS 2 topics, RTAB-Map estimates odometry, the odometry is logged to CSV, and the semantic point cloud is reconstructed again using the estimated SLAM poses.

Important: **CARLA is not needed for Phase 3** if `data/raw/run_001` has already been collected. CARLA is only needed for Phase 1 data collection.

### Phase 3 pipeline

```text
saved CARLA RGB-D dataset
→ dataset_rgbd_publisher.py
→ /camera/color/image_raw
→ /camera/depth/image_rect_raw
→ /camera/color/camera_info
→ rgbd_odometry
→ /odom
→ rtabmap
→ slam_pose_logger.py
→ outputs/slam/run_001/slam_poses.csv
→ reconstruct_semantic_map_from_slam.py
→ outputs/maps/semantic_map_slam_run_001.ply
```

### Phase 3 files

```text
ros2_ws/src/carla_semantic_slam_ros/
├── package.xml
├── setup.py
├── setup.cfg
├── launch/
│   └── replay_rtabmap.launch.py
└── carla_semantic_slam_ros/
    ├── dataset_rgbd_publisher.py
    └── slam_pose_logger.py

configs/slam.yaml
scripts/reconstruct_semantic_map_from_slam.py
src/carla_semantic_slam/slam/slam_pose_reader.py
docs/phase3_rtabmap_integration.md
```

### Step 0: make sure Phase 1 data exists

From the project root:

```bash
cd /home/andy/ros2_projects/carla_semantic_slam

ls data/raw/run_001
ls data/raw/run_001/rgb | head
ls data/raw/run_001/depth | head
ls data/raw/run_001/semantic | head

find data/raw/run_001/rgb -type f | wc -l
find data/raw/run_001/depth -type f | wc -l
find data/raw/run_001/semantic -type f | wc -l
```

Expected structure:

```text
data/raw/run_001/
├── rgb/              # RGB PNG files
├── depth/            # depth NPY files in meters
├── depth_viz/
├── semantic/         # raw semantic label PNG files
├── semantic_viz/
├── poses.csv
└── metadata.json
```

The RGB, depth, and semantic folders should have the same number of files.

### Step 1: install RTAB-Map ROS dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-rtabmap-ros \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-tf2-ros
```

### Step 2: build the ROS 2 workspace

```bash
cd /home/andy/ros2_projects/carla_semantic_slam/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If you edit any file inside `ros2_ws/src/carla_semantic_slam_ros`, rebuild with the same commands.

### Step 3: test the dataset replay publisher only

Do this before running RTAB-Map. The goal is to prove that RGB, depth, and camera info are published at the same stable rate.

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

ros2 topic list | grep camera

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

If camera info is stable but RGB/depth are slow or missing, debug `dataset_rgbd_publisher.py` before launching RTAB-Map.

### Step 4: check replay timestamps

RTAB-Map is sensitive to timestamps. For this replay setup, all three messages should use the same wall-time ROS timestamp for each synchronized frame.

```bash
ros2 topic echo /camera/color/image_raw/header --once
ros2 topic echo /camera/depth/image_rect_raw/header --once
ros2 topic echo /camera/color/camera_info/header --once
```

Good example:

```text
stamp:
  sec: 177807xxxx
  nanosec: ...
frame_id: camera_link
```

Potentially problematic example:

```text
stamp:
  sec: 31
  nanosec: ...
```

If the timestamp looks like tiny dataset time, launch with:

```bash
-p use_wall_time:=true
```

### Step 5: launch RTAB-Map replay

Stop the standalone publisher first. Then launch the full Phase 3 pipeline:

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

The launch file starts:

```text
1. dataset_rgbd_publisher
2. static TF publisher
3. rgbd_odometry
4. rtabmap
5. slam_pose_logger
```

The launch file is configured with approximate synchronization:

```text
approx_sync = true
sync_queue_size = 30
topic_queue_size = 30
```

### Step 6: check odometry and pose logging

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/andy/ros2_projects/carla_semantic_slam/ros2_ws/install/setup.bash

timeout 15 ros2 topic hz /odom
ros2 topic echo /odom --once
```

Check the pose CSV:

```bash
cd /home/andy/ros2_projects/carla_semantic_slam
head outputs/slam/run_001/slam_poses.csv
tail outputs/slam/run_001/slam_poses.csv
```

If `/odom` is publishing and `slam_poses.csv` is filling with rows, Phase 3 replay is working.

### Step 7: reconstruct semantic map using SLAM poses

After the replay has produced `outputs/slam/run_001/slam_poses.csv`, run:

```bash
cd /home/andy/ros2_projects/carla_semantic_slam
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

### Phase 3 troubleshooting

#### `Missing poses.csv`

You are probably launching from inside `ros2_ws`, so `$(pwd)` points to the wrong folder. Use absolute paths:

```bash
run_dir:=/home/andy/ros2_projects/carla_semantic_slam/data/raw/run_001
```

#### Camera info is 2 Hz, but RGB/depth are slow

This means the replay publisher is not publishing a synchronized triplet correctly. Test only the publisher first and check:

```bash
timeout 10 ros2 topic hz /camera/color/image_raw
timeout 10 ros2 topic hz /camera/depth/image_rect_raw
timeout 10 ros2 topic hz /camera/color/camera_info
```

All three should be around the same rate.

#### RTAB-Map says it did not receive synchronized data

Make sure the launch file uses:

```text
approx_sync = true
sync_queue_size = 30
topic_queue_size = 30
```

Also reduce replay speed:

```bash
rate_hz:=1.0
```

#### TF extrapolation error

Use wall-time replay:

```bash
use_wall_time:=true
```

This avoids mixing dataset timestamps with live TF timestamps.

#### `/odom` does not publish

Possible causes:

- RGB/depth/camera_info are not stable.
- RGB/depth/camera_info timestamps do not match.
- Depth encoding/scale is wrong.
- Scene texture is too weak for visual odometry.
- Motion between frames is too large.
- Frame convention or static TF needs adjustment.

For the final report, keep Phase 2 as the reliable mapping result and present Phase 3 as the SLAM integration extension.


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
