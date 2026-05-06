#!/usr/bin/env python3
"""Replay saved CARLA RGB-D frames as ROS 2 Image + CameraInfo topics.

This node publishes one synchronized RGB/depth/camera_info triplet per timer tick.
It is intentionally independent from CARLA: Phase 3 replays the saved Phase 1
run into RTAB-Map.
"""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Dict, List

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class DatasetRGBDPublisher(Node):
    def __init__(self) -> None:
        super().__init__("dataset_rgbd_publisher")

        self.declare_parameter("run_dir", "data/raw/run_001")
        self.declare_parameter("rate_hz", 2.0)
        self.declare_parameter("loop", False)
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("rgb_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_rect_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("use_wall_time", True)
        self.declare_parameter("depth_min_m", 0.0)
        self.declare_parameter("depth_max_m", 1000.0)
        self.declare_parameter("log_every_n", 30)

        self.run_dir = Path(str(self.get_parameter("run_dir").value)).expanduser().resolve()
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.loop = bool(self.get_parameter("loop").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.use_wall_time = bool(self.get_parameter("use_wall_time").value)
        self.depth_min_m = float(self.get_parameter("depth_min_m").value)
        self.depth_max_m = float(self.get_parameter("depth_max_m").value)
        self.log_every_n = int(self.get_parameter("log_every_n").value)

        self.rows = self._load_rows(self.run_dir)
        self.metadata = self._load_metadata(self.run_dir)
        self.camera_info_template = self._build_camera_info()
        self.index = 0

        # Keep queue sizes modest. The replay is offline and deterministic, so
        # huge queues can hide timing problems during debugging.
        qos_depth = 10
        self.rgb_pub = self.create_publisher(Image, str(self.get_parameter("rgb_topic").value), qos_depth)
        self.depth_pub = self.create_publisher(Image, str(self.get_parameter("depth_topic").value), qos_depth)
        self.info_pub = self.create_publisher(CameraInfo, str(self.get_parameter("camera_info_topic").value), qos_depth)

        period = 1.0 / max(self.rate_hz, 0.1)
        self.timer = self.create_timer(period, self._timer_cb)
        self.get_logger().info(f"Replaying {len(self.rows)} RGB-D samples from {self.run_dir}")
        self.get_logger().info(
            f"rate_hz={self.rate_hz:.2f}, use_wall_time={self.use_wall_time}, frame_id='{self.frame_id}'"
        )

    def _load_rows(self, run_dir: Path) -> List[Dict[str, str]]:
        poses_path = run_dir / "poses.csv"
        if not poses_path.exists():
            raise FileNotFoundError(f"Missing poses.csv: {poses_path}")
        with poses_path.open("r", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        if not rows:
            raise ValueError(f"poses.csv is empty: {poses_path}")
        return rows

    def _load_metadata(self, run_dir: Path) -> Dict:
        metadata_path = run_dir / "metadata.json"
        if not metadata_path.exists():
            raise FileNotFoundError(f"Missing metadata.json: {metadata_path}")
        with metadata_path.open("r", encoding="utf-8") as f:
            return json.load(f)

    def _build_camera_info(self) -> CameraInfo:
        intr = self.metadata["camera_intrinsics"]
        width = int(intr.get("width", self.metadata.get("camera", {}).get("width", 640)))
        height = int(intr.get("height", self.metadata.get("camera", {}).get("height", 480)))
        fx, fy, cx, cy = float(intr["fx"]), float(intr["fy"]), float(intr["cx"]), float(intr["cy"])

        msg = CameraInfo()
        msg.header.frame_id = self.frame_id
        msg.width = width
        msg.height = height
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def _stamp_from_dataset_or_sequence(self, row: Dict[str, str]) -> TimeMsg:
        """Fallback timestamp if use_wall_time is false."""
        stamp = TimeMsg()
        try:
            t = float(row.get("timestamp", row.get("time_sec", "")))
            if not math.isfinite(t):
                raise ValueError
        except Exception:
            t = float(self.index) / max(self.rate_hz, 0.1)
        stamp.sec = int(math.floor(t))
        stamp.nanosec = int(round((t - stamp.sec) * 1e9))
        if stamp.nanosec >= 1_000_000_000:
            stamp.sec += 1
            stamp.nanosec -= 1_000_000_000
        return stamp

    def _current_stamp(self, row: Dict[str, str]) -> TimeMsg:
        # Wall time avoids TF extrapolation when RTAB-Map and tf2 are using live ROS time.
        if self.use_wall_time:
            return self.get_clock().now().to_msg()
        return self._stamp_from_dataset_or_sequence(row)

    def _timer_cb(self) -> None:
        if self.index >= len(self.rows):
            if self.loop:
                self.index = 0
            else:
                self.get_logger().info("Finished replaying dataset.")
                self.timer.cancel()
                return

        row = self.rows[self.index]
        rgb_path = self.run_dir / row["rgb_path"]
        depth_path = self.run_dir / row["depth_path"]

        try:
            rgb_bgr = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
            if rgb_bgr is None:
                raise RuntimeError(f"Could not read RGB image: {rgb_path}")
            rgb = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2RGB)

            if depth_path.suffix.lower() == ".npy":
                depth = np.load(depth_path).astype(np.float32)
            else:
                depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED).astype(np.float32)
            depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0).astype(np.float32)
            depth[(depth < self.depth_min_m) | (depth > self.depth_max_m)] = 0.0
        except Exception as exc:
            self.get_logger().error(f"Failed to load frame index={self.index}: {exc}")
            self.index += 1
            return

        # One shared timestamp for the synchronized triplet.
        stamp = self._current_stamp(row)

        rgb_msg = self._numpy_to_image(rgb, "rgb8", stamp)
        depth_msg = self._numpy_to_image(depth, "32FC1", stamp)
        info_msg = CameraInfo()
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = self.frame_id
        info_msg.width = self.camera_info_template.width
        info_msg.height = self.camera_info_template.height
        info_msg.distortion_model = self.camera_info_template.distortion_model
        info_msg.d = list(self.camera_info_template.d)
        info_msg.k = list(self.camera_info_template.k)
        info_msg.r = list(self.camera_info_template.r)
        info_msg.p = list(self.camera_info_template.p)

        # Publish all three together. RTAB-Map synchronization depends on this.
        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        self.info_pub.publish(info_msg)

        if self.log_every_n > 0 and self.index % self.log_every_n == 0:
            self.get_logger().info(
                f"Published synchronized frame {self.index + 1}/{len(self.rows)} "
                f"rgb={rgb.shape} depth={depth.shape} "
                f"depth_range=[{float(np.min(depth)):.2f}, {float(np.max(depth)):.2f}]"
            )
        self.index += 1

    def _numpy_to_image(self, array: np.ndarray, encoding: str, stamp: TimeMsg) -> Image:
        contiguous = np.ascontiguousarray(array)
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = int(contiguous.shape[0])
        msg.width = int(contiguous.shape[1])
        msg.encoding = encoding
        msg.is_bigendian = False
        if contiguous.ndim == 2:
            msg.step = int(contiguous.shape[1] * contiguous.dtype.itemsize)
        else:
            msg.step = int(contiguous.shape[1] * contiguous.shape[2] * contiguous.dtype.itemsize)
        msg.data = contiguous.tobytes()
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DatasetRGBDPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
