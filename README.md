# Autonomous Exploration and Photogeometric Semantic Mapping in CARLA

This repository implements a staged semantic SLAM pipeline in CARLA. The system collects synchronized RGB, depth, semantic segmentation, and pose data from a simulated ego vehicle, reconstructs a baseline semantic 3D point cloud using CARLA ground-truth poses, and optionally replaces the ground-truth trajectory with RTAB-Map RGB-D SLAM poses through ROS 2 replay.

The project is designed to be reproducible in three main stages:

1. **CARLA RGB-D-semantic data collection**
2. **Ground-truth-pose semantic 3D reconstruction**
3. **ROS 2 replay + RTAB-Map SLAM-pose semantic reconstruction**

For the most reliable final demo/report result, use **Phase 2** as the stable baseline and present **Phase 3** as the realistic SLAM extension.

---

## Table of contents

- [Project overview](#project-overview)
- [System pipeline](#system-pipeline)
- [Repository structure](#repository-structure)
- [Requirements](#requirements)
- [Environment setup](#environment-setup)
- [Phase 1: collect CARLA RGB-D-semantic data](#phase-1-collect-carla-rgb-d-semantic-data)
- [Phase 2: reconstruct semantic map using CARLA ground-truth poses](#phase-2-reconstruct-semantic-map-using-carla-ground-truth-poses)
- [Phase 3: replay dataset in ROS 2 and reconstruct using RTAB-Map poses](#phase-3-replay-dataset-in-ros-2-and-reconstruct-using-rtab-map-poses)
- [Core photogeometric projection](#core-photogeometric-projection)
- [Troubleshooting](#troubleshooting)
- [Suggested final demo flow](#suggested-final-demo-flow)

---

## Project overview

The goal of this project is to build a semantic 3D mapping pipeline for a simulated exploration platform. Instead of only creating a geometric map, the system also assigns semantic labels to 3D points, producing a semantic point cloud that shows meaningful scene regions such as roads, sidewalks, buildings, vegetation, vehicles, poles, signs, and other CARLA semantic classes.

The main idea is:

```text
RGB-D sensing + semantic segmentation + camera pose
→ 3D back-projection
→ semantic point accumulation
→ global semantic point cloud
```

The repository supports two mapping modes:

| Mode | Pose source | Purpose |
|---|---|---|
| Ground-truth baseline | CARLA camera/vehicle poses | Stable semantic reconstruction and final demo baseline |
| SLAM extension | RTAB-Map RGB-D odometry/SLAM poses | More realistic mapping pipeline using estimated poses |

---

## System pipeline

```text
CARLA ego vehicle
      │
      ├── RGB camera
      ├── Depth camera
      └── Semantic segmentation camera
      │
      ▼
Saved synchronized dataset
      │
      ├── Phase 2: CARLA ground-truth pose
      │       └── semantic 3D reconstruction
      │
      └── Phase 3: ROS 2 replay
              ├── RGB-D topics
              ├── RTAB-Map RGB-D odometry
              ├── logged SLAM poses
              └── semantic 3D reconstruction
```

---

## Repository structure

```text
carla_semantic_slam/
├── configs/
│   ├── collection.yaml          # CARLA sensor and data-collection settings
│   ├── mapping.yaml             # Ground-truth-pose reconstruction settings
│   └── slam.yaml                # SLAM-pose reconstruction settings
│
├── scripts/
│   ├── collect_data.py                          # Phase 1: collect RGB/depth/semantic/pose data
│   ├── inspect_run.py                           # Check saved dataset integrity
│   ├── reconstruct_semantic_map.py              # Phase 2: reconstruct using CARLA poses
│   ├── reconstruct_semantic_map_from_slam.py    # Phase 3: reconstruct using SLAM poses
│   └── visualize_pointcloud.py                  # Visualize PLY point clouds
│
├── src/carla_semantic_slam/
│   ├── sim/                     # CARLA client, world, actors, traffic, and vehicle control
│   ├── sensors/                 # RGB, depth, and semantic camera utilities
│   ├── data/                    # Dataset reader/writer, frame buffer, and transforms
│   ├── mapping/                 # Back-projection and semantic point-cloud generation
│   ├── slam/                    # SLAM pose reader and related utilities
│   ├── visualization/           # Open3D visualization helpers
│   └── utils/                   # Config and logging helpers
│
├── ros2_ws/
│   └── src/carla_semantic_slam_ros/
│       ├── carla_semantic_slam_ros/
│       │   ├── dataset_rgbd_publisher.py        # Replays saved dataset as ROS 2 RGB-D topics
│       │   └── slam_pose_logger.py              # Logs /odom poses to CSV
│       └── launch/
│           └── replay_rtabmap.launch.py         # Full ROS 2 replay + RTAB-Map launch file
│
├── data/
│   ├── raw/                     # Collected CARLA runs
│   └── processed/
│
├── outputs/
│   ├── maps/                    # Generated semantic point clouds and summaries
│   ├── figures/
│   ├── videos/
│   ├── logs/
│   └── slam/                    # RTAB-Map/odometry pose CSVs
│
├── docs/
│   ├── roadmap.md
│   └── phase3_rtabmap_integration.md
│
├── requirements.txt
├── pyproject.toml
└── README.md
```

---

## Requirements

Recommended system:

- Ubuntu 22.04
- Python 3.10
- CARLA 0.9.16
- ROS 2 Humble
- RTAB-Map ROS packages
- Conda or Miniconda

The Python scripts use the CARLA Python API and common scientific/visualization packages such as NumPy, OpenCV, Open3D, Matplotlib, and tqdm.

---

## Environment setup

### 1. Clone the repository

```bash
git clone <your-repository-url>
cd carla_semantic_slam
```

For convenience, define the project root:

```bash
export PROJECT_ROOT=$PWD
```

If you open a new terminal later, run this again from the repository root:

```bash
cd /path/to/carla_semantic_slam
export PROJECT_ROOT=$PWD
```

---

### 2. Create the Python environment

```bash
conda create -n carla_semantic python=3.10 -y
conda activate carla_semantic
pip install -r requirements.txt
```

---

### 3. Install the CARLA Python API

Adjust the CARLA path if your installation is somewhere else.

```bash
pip install ~/CARLA_0.9.16/PythonAPI/carla/dist/carla-0.9.16-cp310-cp310-linux_x86_64.whl
```

If the wheel name is different, check the available file:

```bash
ls ~/CARLA_0.9.16/PythonAPI/carla/dist/
```

Verify the CARLA Python API:

```bash
python -c "import carla; print('CARLA API loaded from:', carla.__file__)"
```

---

### 4. Install ROS 2 and RTAB-Map dependencies

This project assumes ROS 2 Humble is already installed.

```bash
sudo apt update
sudo apt install -y \
  ros-humble-rtabmap-ros \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-tf2-ros
```

---

### 5. Build the ROS 2 workspace

```bash
cd $PROJECT_ROOT/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If you edit files inside `ros2_ws/src/carla_semantic_slam_ros`, rebuild the workspace:

```bash
cd $PROJECT_ROOT/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Phase 1: collect CARLA RGB-D-semantic data

Phase 1 launches CARLA, spawns an ego vehicle, attaches synchronized RGB/depth/semantic cameras, and saves frames with camera/vehicle poses.

### 1. Start CARLA

In Terminal 1:

```bash
cd ~/CARLA_0.9.16
./CarlaUE4.sh
```

Wait until the CARLA simulator window has loaded.

---

### 2. Run data collection

In Terminal 2:

```bash
cd $PROJECT_ROOT
conda activate carla_semantic

python scripts/collect_data.py --config configs/collection.yaml
```

Expected output structure:

```text
data/raw/run_001/
├── rgb/              # RGB PNG images
├── depth/            # depth arrays, usually NPY files in meters
├── depth_viz/        # visualized depth images
├── semantic/         # raw semantic label PNG images
├── semantic_viz/     # visualized semantic segmentation images
├── poses.csv         # saved camera/vehicle poses
└── metadata.json     # run metadata and camera parameters
```

---

### 3. Inspect the collected run

```bash
cd $PROJECT_ROOT
conda activate carla_semantic

python scripts/inspect_run.py data/raw/run_001 --max-samples 5
```

A healthy run should have:

- RGB, depth, and semantic files saved for the same frames
- A valid `poses.csv`
- A valid `metadata.json`
- Matching or nearly matching frame counts across `rgb/`, `depth/`, and `semantic/`

You can also check counts manually:

```bash
find data/raw/run_001/rgb -type f | wc -l
find data/raw/run_001/depth -type f | wc -l
find data/raw/run_001/semantic -type f | wc -l
```

After the dataset is saved, CARLA is no longer needed for Phase 2 or Phase 3.

---

## Phase 2: reconstruct semantic map using CARLA ground-truth poses

Phase 2 is the stable baseline. It uses the saved CARLA camera pose from `poses.csv` to transform semantic depth points into a global point cloud.

### 1. Run a small reconstruction test

```bash
cd $PROJECT_ROOT
conda activate carla_semantic

python scripts/reconstruct_semantic_map.py \
  --config configs/mapping.yaml \
  --max-samples 30
```

---

### 2. Visualize the output

```bash
python scripts/visualize_pointcloud.py outputs/maps/semantic_map_run_001.ply
```

Check that the point cloud has a reasonable 3D structure and that semantic colors/classes appear in plausible locations.

---

### 3. Run the full reconstruction

```bash
cd $PROJECT_ROOT
conda activate carla_semantic

python scripts/reconstruct_semantic_map.py --config configs/mapping.yaml
```

Expected outputs:

```text
outputs/maps/semantic_map_run_001.ply
outputs/maps/semantic_map_run_001.npz
outputs/maps/semantic_map_run_001_label_summary.csv
outputs/maps/semantic_map_run_001_label_summary.json
```

The `.ply` file is used for visualization. The `.csv` and `.json` summaries are useful for reporting semantic class counts.

---

## Phase 3: replay dataset in ROS 2 and reconstruct using RTAB-Map poses

Phase 3 replaces CARLA ground-truth poses with estimated poses from RTAB-Map RGB-D odometry/SLAM. This makes the pipeline more realistic, but the result may be less stable depending on texture quality, camera motion, timestamp synchronization, and depth scaling.

CARLA is **not required** for Phase 3 if `data/raw/run_001` already exists.

### Phase 3 pipeline

```text
Saved CARLA RGB-D dataset
      │
      ▼
dataset_rgbd_publisher.py
      │
      ├── /camera/color/image_raw
      ├── /camera/depth/image_rect_raw
      └── /camera/color/camera_info
      │
      ▼
RTAB-Map rgbd_odometry
      │
      ▼
/odom
      │
      ▼
slam_pose_logger.py
      │
      ▼
outputs/slam/run_001/slam_poses.csv
      │
      ▼
reconstruct_semantic_map_from_slam.py
      │
      ▼
outputs/maps/semantic_map_slam_run_001.ply
```

---

### Step 0: confirm Phase 1 data exists

```bash
cd $PROJECT_ROOT

ls data/raw/run_001
ls data/raw/run_001/rgb | head
ls data/raw/run_001/depth | head
ls data/raw/run_001/semantic | head
```

Expected structure:

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

---

### Step 1: test the dataset replay publisher only

Before launching RTAB-Map, first verify that the saved dataset can be replayed as stable ROS 2 topics.

Terminal 1:

```bash
cd $PROJECT_ROOT
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

ros2 run carla_semantic_slam_ros dataset_rgbd_publisher \
  --ros-args \
  -p run_dir:=$PROJECT_ROOT/data/raw/run_001 \
  -p rate_hz:=2.0 \
  -p use_wall_time:=true
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source $PROJECT_ROOT/ros2_ws/install/setup.bash

ros2 topic list | grep camera

timeout 10 ros2 topic hz /camera/color/image_raw
timeout 10 ros2 topic hz /camera/depth/image_rect_raw
timeout 10 ros2 topic hz /camera/color/camera_info
```

Target result:

```text
/camera/color/image_raw        approximately 2 Hz
/camera/depth/image_rect_raw   approximately 2 Hz
/camera/color/camera_info      approximately 2 Hz
```

If camera info is stable but RGB/depth are slow or missing, debug `dataset_rgbd_publisher.py` before running RTAB-Map.

---

### Step 2: check replay timestamps

RTAB-Map is sensitive to timestamp synchronization. For this replay setup, RGB, depth, and camera info should share the same wall-time ROS timestamp for each synchronized frame.

```bash
ros2 topic echo /camera/color/image_raw/header --once
ros2 topic echo /camera/depth/image_rect_raw/header --once
ros2 topic echo /camera/color/camera_info/header --once
```

A good wall-time replay stamp usually looks like a large current-time value:

```text
stamp:
  sec: 177807xxxx
  nanosec: ...
frame_id: camera_link
```

A potentially problematic dataset-time stamp may look very small:

```text
stamp:
  sec: 31
  nanosec: ...
```

If timestamps look too small or inconsistent, run the replay with:

```bash
-p use_wall_time:=true
```

---

### Step 3: launch the full RTAB-Map replay pipeline

Stop the standalone replay publisher first. Then run the full launch file:

```bash
cd $PROJECT_ROOT
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

mkdir -p outputs/slam/run_001

ros2 launch carla_semantic_slam_ros replay_rtabmap.launch.py \
  run_dir:=$PROJECT_ROOT/data/raw/run_001 \
  pose_csv:=$PROJECT_ROOT/outputs/slam/run_001/slam_poses.csv \
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

The launch file is configured for approximate synchronization:

```text
approx_sync = true
sync_queue_size = 30
topic_queue_size = 30
```

---

### Step 4: check odometry and pose logging

In a separate terminal:

```bash
source /opt/ros/humble/setup.bash
source $PROJECT_ROOT/ros2_ws/install/setup.bash

timeout 15 ros2 topic hz /odom
ros2 topic echo /odom --once
```

Then check whether the SLAM pose CSV is being written:

```bash
cd $PROJECT_ROOT

head outputs/slam/run_001/slam_poses.csv
tail outputs/slam/run_001/slam_poses.csv
```

If `/odom` is publishing and `slam_poses.csv` is filling with rows, the Phase 3 replay pipeline is working.

---

### Step 5: reconstruct the semantic map using SLAM poses

After `outputs/slam/run_001/slam_poses.csv` has been generated, run:

```bash
cd $PROJECT_ROOT
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

## Core photogeometric projection

For each valid depth pixel with a semantic label, the mapper converts the pixel into a 3D camera-frame point:

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth(u, v)
```

Then the point is transformed into the global map frame:

```text
P_map = T_map_camera @ P_camera
```

The difference between Phase 2 and Phase 3 is the source of `T_map_camera`:

| Phase | Pose transform source |
|---|---|
| Phase 2 | CARLA ground-truth camera pose |
| Phase 3 | RTAB-Map/odometry estimated pose |

The semantic label for each 3D point comes from the semantic segmentation image at the same pixel location.

---

## Troubleshooting

### CARLA Python API import fails

Check the Python version and installed CARLA wheel:

```bash
python --version
python -c "import carla; print(carla.__file__)"
```

Make sure the wheel matches Python 3.10 and CARLA 0.9.16.

---

### `Missing poses.csv`

This usually means the command is being run from the wrong directory or the launch file is using a relative path that points inside `ros2_ws`.

Check that the file exists:

```bash
ls $PROJECT_ROOT/data/raw/run_001/poses.csv
```

Use absolute paths or the `$PROJECT_ROOT` variable in ROS 2 launch commands.

---

### Camera topics are unstable

Test only the publisher before debugging RTAB-Map:

```bash
ros2 run carla_semantic_slam_ros dataset_rgbd_publisher \
  --ros-args \
  -p run_dir:=$PROJECT_ROOT/data/raw/run_001 \
  -p rate_hz:=2.0 \
  -p use_wall_time:=true
```

Then check topic rates:

```bash
timeout 10 ros2 topic hz /camera/color/image_raw
timeout 10 ros2 topic hz /camera/depth/image_rect_raw
timeout 10 ros2 topic hz /camera/color/camera_info
```

All three should publish at approximately the same rate.

---

### RTAB-Map does not receive synchronized data

Make sure approximate synchronization is enabled in the launch configuration:

```text
approx_sync = true
sync_queue_size = 30
topic_queue_size = 30
```

If synchronization still fails, reduce the replay speed:

```bash
rate_hz:=1.0
```

---

### TF extrapolation error

Use wall-time replay:

```bash
use_wall_time:=true
```

This avoids mixing saved dataset timestamps with live TF timestamps.

---

### `/odom` does not publish

Possible causes:

- RGB/depth/camera info topics are not stable
- RGB/depth/camera info timestamps do not match
- Depth encoding or depth scale is incorrect
- Scene texture is too weak for RGB-D visual odometry
- Motion between frames is too large
- Static TF or camera frame convention needs adjustment

For the final report, use Phase 2 as the reliable semantic mapping result and describe Phase 3 as the SLAM integration extension if odometry remains unstable.

---

### SLAM semantic map looks worse than ground-truth semantic map

This is expected if the estimated SLAM trajectory drifts or fails intermittently. The ground-truth map uses perfect CARLA pose, while the SLAM map depends on RTAB-Map odometry quality.

Common reasons include:

- Missing or unstable `/odom`
- Poor visual features in the scene
- Fast ego-vehicle motion
- Large frame-to-frame viewpoint changes
- Incorrect depth scale
- Timestamp mismatch
- Frame convention mismatch

Recommended report interpretation:

> The ground-truth-pose reconstruction demonstrates the correctness of the semantic projection and mapping module. The SLAM-pose reconstruction demonstrates integration with a realistic RGB-D SLAM pipeline, but mapping quality depends strongly on odometry stability.

<!-- ---

## Suggested final demo flow

A clear final demo can follow this order:

1. Show the CARLA environment and ego vehicle exploration.
2. Show examples of saved RGB, depth, and semantic segmentation frames.
3. Show the Phase 2 semantic point cloud reconstructed with CARLA ground-truth poses.
4. Show ROS 2 replay topics for RGB, depth, and camera info.
5. Show RTAB-Map `/odom` or trajectory output if stable.
6. Show the Phase 3 SLAM-pose semantic map as an extension.
7. Briefly compare the stable ground-truth-pose map with the realistic SLAM-pose map. -->

---

## Project status

Current recommended project framing:

- **Main working contribution:** photogeometric semantic 3D mapping from CARLA RGB-D and semantic observations.
- **Stable final result:** semantic point cloud using CARLA ground-truth pose.
- **Realistic extension:** ROS 2 replay with RTAB-Map RGB-D SLAM and SLAM-pose semantic reconstruction.
- **Future work:** improve odometry stability, add object-level landmarks, compare SLAM trajectory to CARLA ground truth, and extend the same mapping pipeline to UAV-style exploration.
