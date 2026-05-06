from setuptools import setup, find_packages

package_name = "carla_semantic_slam_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/replay_rtabmap.launch.py"]),
    ],
    install_requires=["setuptools", "numpy", "opencv-python"],
    zip_safe=True,
    maintainer="CARLA Semantic SLAM",
    maintainer_email="user@example.com",
    description="Dataset replay and pose logging utilities for CARLA RGB-D RTAB-Map SLAM.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "dataset_rgbd_publisher = carla_semantic_slam_ros.dataset_rgbd_publisher:main",
            "slam_pose_logger = carla_semantic_slam_ros.slam_pose_logger:main",
        ],
    },
)
