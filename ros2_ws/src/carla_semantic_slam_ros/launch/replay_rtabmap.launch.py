"""Replay saved CARLA RGB-D data into RTAB-Map RGB-D odometry and mapping.

This launch file uses rtabmap_msgs/msg/RGBDImage on /rgbd_image for RTAB-Map.
It also asks dataset_rgbd_publisher to write frame_timestamps.csv, which is later
used to match each image frame to the nearest /odom pose by timestamp.

Run from the project root after building the ROS package:

ros2 launch carla_semantic_slam_ros replay_rtabmap.launch.py \
  run_dir:=/absolute/path/to/data/raw/run_001 \
  pose_csv:=/absolute/path/to/outputs/slam/run_001/slam_poses.csv \
  frame_timestamps_csv:=/absolute/path/to/outputs/slam/run_001/frame_timestamps.csv \
  rate_hz:=1.0 \
  use_wall_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    run_dir = LaunchConfiguration("run_dir")
    pose_csv = LaunchConfiguration("pose_csv")
    frame_timestamps_csv = LaunchConfiguration("frame_timestamps_csv")
    rate_hz = LaunchConfiguration("rate_hz")
    loop = LaunchConfiguration("loop")
    delete_db_on_start = LaunchConfiguration("delete_db_on_start")
    use_wall_time = LaunchConfiguration("use_wall_time")
    frame_id = LaunchConfiguration("frame_id")
    max_frames = LaunchConfiguration("max_frames")

    return LaunchDescription([
        DeclareLaunchArgument(
            "run_dir",
            description="Absolute path to a collected CARLA run, e.g., data/raw/run_001",
        ),
        DeclareLaunchArgument(
            "pose_csv",
            default_value="outputs/slam/run_001/slam_poses.csv",
            description="CSV path for estimated odometry poses.",
        ),
        DeclareLaunchArgument(
            "frame_timestamps_csv",
            default_value="outputs/slam/run_001/frame_timestamps.csv",
            description="CSV path for replayed RGB-D image timestamps.",
        ),
        DeclareLaunchArgument(
            "rate_hz",
            default_value="1.0",
            description="Replay rate for saved RGB-D frames.",
        ),
        DeclareLaunchArgument(
            "loop",
            default_value="false",
            description="Replay dataset continuously.",
        ),
        DeclareLaunchArgument(
            "delete_db_on_start",
            default_value="true",
            description="Delete RTAB-Map database on start.",
        ),
        DeclareLaunchArgument(
            "use_wall_time",
            default_value="true",
            description="Use current ROS time for replayed image headers.",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="camera_link",
            description="Frame id used in RGB-D headers.",
        ),
        DeclareLaunchArgument(
            "max_frames",
            default_value="0",
            description="Maximum number of frames to replay. 0 means all frames.",
        ),

        Node(
            package="carla_semantic_slam_ros",
            executable="dataset_rgbd_publisher",
            name="dataset_rgbd_publisher",
            output="screen",
            parameters=[{
                "run_dir": run_dir,
                "rate_hz": rate_hz,
                "loop": loop,
                "use_wall_time": use_wall_time,
                "frame_id": frame_id,
                "rgb_topic": "/camera/color/image_raw",
                "depth_topic": "/camera/depth/image_rect_raw",
                "camera_info_topic": "/camera/color/camera_info",
                "rgbd_topic": "/rgbd_image",
                "max_frames": max_frames,
                "frame_timestamps_csv": frame_timestamps_csv,
            }],
        ),

        # Baseline static TF for offline replay.
        # For RGBDImage replay, this mostly defines the camera frame relation used by RTAB-Map.
        # The semantic reconstruction script separately applies the CARLA camera mounting
        # transform T_base_camera when projecting points with /odom poses.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_camera_tf",
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "camera_link"],
        ),

        Node(
            package="rtabmap_odom",
            executable="rgbd_odometry",
            name="rgbd_odometry",
            output="screen",
            parameters=[{
                "frame_id": "base_link",
                "odom_frame_id": "odom",
                "publish_tf": True,
                "subscribe_rgbd": True,
                "approx_sync": False,
                "topic_queue_size": 10,
                "sync_queue_size": 10,
                "Odom/ResetCountdown": "1",
                "Vis/MinInliers": "10",
            }],
            remappings=[
                ("rgbd_image", "/rgbd_image"),
                ("odom", "/odom"),
            ],
        ),

        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[{
                "frame_id": "base_link",
                "map_frame_id": "map",
                "subscribe_rgbd": True,
                "subscribe_rgb": False,
                "subscribe_depth": False,
                "subscribe_scan": False,
                "subscribe_odom_info": False,
                "approx_sync": True,
                "approx_sync_max_interval": 0.10,
                "sync_queue_size": 50,
                "topic_queue_size": 50,
                "delete_db_on_start": delete_db_on_start,
                "Mem/IncrementalMemory": "true",
                "RGBD/OptimizeFromGraphEnd": "false",
                "Reg/Force3DoF": "true",
                "Mem/DepthCompressionFormat": ".png",
            }],
            remappings=[
                ("rgbd_image", "/rgbd_image"),
                ("odom", "/odom"),
            ],
        ),

        Node(
            package="carla_semantic_slam_ros",
            executable="slam_pose_logger",
            name="slam_pose_logger",
            output="screen",
            parameters=[{
                "odom_topic": "/odom",
                "pose_csv": pose_csv,
            }],
        ),
    ])
