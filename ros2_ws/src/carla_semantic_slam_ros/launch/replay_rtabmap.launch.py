"""Replay saved CARLA RGB-D data into RTAB-Map RGB-D odometry and mapping.

Run from the project root after building the ROS package:

ros2 launch carla_semantic_slam_ros replay_rtabmap.launch.py \
  run_dir:=/absolute/path/to/data/raw/run_001 \
  pose_csv:=/absolute/path/to/outputs/slam/run_001/slam_poses.csv \
  rate_hz:=2.0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    run_dir = LaunchConfiguration("run_dir")
    pose_csv = LaunchConfiguration("pose_csv")
    rate_hz = LaunchConfiguration("rate_hz")
    loop = LaunchConfiguration("loop")
    delete_db_on_start = LaunchConfiguration("delete_db_on_start")
    use_wall_time = LaunchConfiguration("use_wall_time")

    return LaunchDescription([
        DeclareLaunchArgument("run_dir", description="Absolute path to a collected CARLA run, e.g., data/raw/run_001"),
        DeclareLaunchArgument("pose_csv", default_value="outputs/slam/run_001/slam_poses.csv", description="CSV path for estimated odometry poses."),
        DeclareLaunchArgument("rate_hz", default_value="2.0", description="Replay rate for saved RGB-D frames."),
        DeclareLaunchArgument("loop", default_value="false", description="Replay dataset continuously."),
        DeclareLaunchArgument("delete_db_on_start", default_value="true", description="Delete RTAB-Map database on start."),
        DeclareLaunchArgument("use_wall_time", default_value="true", description="Use current ROS time for replayed image headers."),

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
                "frame_id": "camera_link",
                "rgb_topic": "/camera/color/image_raw",
                "depth_topic": "/camera/depth/image_rect_raw",
                "camera_info_topic": "/camera/color/camera_info",
            }],
        ),

        # Baseline static TF. For a simple offline replay, use identity from base to camera.
        # If the estimated trajectory looks rotated, replace this with the actual CARLA camera extrinsic.
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
                "approx_sync": True,
                "sync_queue_size": 30,
                "topic_queue_size": 30,
                "Odom/ResetCountdown": "1",
                "Vis/MinInliers": "10",
            }],
            remappings=[
                ("rgb/image", "/camera/color/image_raw"),
                ("depth/image", "/camera/depth/image_rect_raw"),
                ("rgb/camera_info", "/camera/color/camera_info"),
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
                "subscribe_depth": True,
                "subscribe_rgb": True,
                "subscribe_scan": False,
                "subscribe_odom_info": False,
                "approx_sync": True,
                "sync_queue_size": 30,
                "topic_queue_size": 30,
                "delete_db_on_start": delete_db_on_start,
                "Mem/IncrementalMemory": "true",
                "RGBD/OptimizeFromGraphEnd": "false",
                "Reg/Force3DoF": "true",
                "Mem/DepthCompressionFormat": ".png",
            }],
            remappings=[
                ("rgb/image", "/camera/color/image_raw"),
                ("depth/image", "/camera/depth/image_rect_raw"),
                ("rgb/camera_info", "/camera/color/camera_info"),
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
