#!/usr/bin/env python3
"""Log RTAB-Map RGB-D odometry poses from nav_msgs/Odometry to CSV."""
from __future__ import annotations

import csv
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class SlamPoseLogger(Node):
    def __init__(self) -> None:
        super().__init__("slam_pose_logger")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("pose_csv", "outputs/slam/run_001/slam_poses.csv")

        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.pose_csv = Path(str(self.get_parameter("pose_csv").value)).expanduser().resolve()
        self.pose_csv.parent.mkdir(parents=True, exist_ok=True)

        self.file = self.pose_csv.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(
            self.file,
            fieldnames=[
                "stamp_sec",
                "stamp_nanosec",
                "time_sec",
                "pose_index",
                "frame_id",
                "child_frame_id",
                "x",
                "y",
                "z",
                "qx",
                "qy",
                "qz",
                "qw",
            ],
        )
        self.writer.writeheader()
        self.count = 0
        self.sub = self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 50)
        self.get_logger().info(f"Logging odometry from {self.odom_topic} to {self.pose_csv}")

    def _odom_cb(self, msg: Odometry) -> None:
        stamp_sec = int(msg.header.stamp.sec)
        stamp_nanosec = int(msg.header.stamp.nanosec)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.writer.writerow({
            "stamp_sec": stamp_sec,
            "stamp_nanosec": stamp_nanosec,
            "time_sec": stamp_sec + stamp_nanosec * 1e-9,
            "pose_index": self.count,
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "x": p.x,
            "y": p.y,
            "z": p.z,
            "qx": q.x,
            "qy": q.y,
            "qz": q.z,
            "qw": q.w,
        })
        self.count += 1
        if self.count % 30 == 0:
            self.file.flush()
            self.get_logger().info(f"Logged {self.count} odometry poses")

    def destroy_node(self) -> bool:
        try:
            self.file.flush()
            self.file.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamPoseLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
