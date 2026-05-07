#!/usr/bin/env python3
"""Replay saved CARLA RGB-D data as ROS 2 topics for RTAB-Map.

This publisher sends both:
  1) Separate debugging topics:
     /camera/color/image_raw
     /camera/depth/image_rect_raw
     /camera/color/camera_info
  2) A packed RTAB-Map RGBDImage topic:
     /rgbd_image

It also writes the actual ROS replay timestamp for each sample to a CSV so that
semantic reconstruction can match image frames to the nearest /odom pose by time.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rtabmap_msgs.msg import RGBDImage
from sensor_msgs.msg import CameraInfo, Image


class DatasetRGBDPublisher(Node):
    def __init__(self):
        super().__init__("dataset_rgbd_publisher")

        self.declare_parameter("run_dir", "")
        self.declare_parameter("rate_hz", 2.0)
        self.declare_parameter("use_wall_time", True)
        self.declare_parameter("loop", False)
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("rgb_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_rect_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("rgbd_topic", "/rgbd_image")
        self.declare_parameter("max_frames", 0)
        self.declare_parameter("frame_timestamps_csv", "")

        self.run_dir = Path(self.get_parameter("run_dir").get_parameter_value().string_value).expanduser()
        self.rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value
        self.use_wall_time = self.get_parameter("use_wall_time").get_parameter_value().bool_value
        self.loop = self.get_parameter("loop").get_parameter_value().bool_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.rgb_topic = self.get_parameter("rgb_topic").get_parameter_value().string_value
        self.depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.rgbd_topic = self.get_parameter("rgbd_topic").get_parameter_value().string_value
        self.max_frames = self.get_parameter("max_frames").get_parameter_value().integer_value
        self.frame_timestamps_csv = self.get_parameter("frame_timestamps_csv").get_parameter_value().string_value

        if not self.run_dir.exists():
            raise FileNotFoundError(f"run_dir does not exist: {self.run_dir}")

        self.rgb_dir = self.run_dir / "rgb"
        self.depth_dir = self.run_dir / "depth"
        self.metadata_path = self.run_dir / "metadata.json"
        self.poses_path = self.run_dir / "poses.csv"

        if not self.rgb_dir.exists():
            raise FileNotFoundError(f"Missing RGB directory: {self.rgb_dir}")
        if not self.depth_dir.exists():
            raise FileNotFoundError(f"Missing depth directory: {self.depth_dir}")
        if not self.metadata_path.exists():
            raise FileNotFoundError(f"Missing metadata.json: {self.metadata_path}")
        if not self.poses_path.exists():
            raise FileNotFoundError(f"Missing poses.csv: {self.poses_path}")

        self.metadata = self._load_metadata(self.metadata_path)
        self.rows = self._load_rows(self.poses_path)
        if self.max_frames and self.max_frames > 0:
            self.rows = self.rows[: self.max_frames]

        self.width, self.height, self.fx, self.fy, self.cx, self.cy = self._read_intrinsics(self.metadata)

        self.bridge = CvBridge()
        self.index = 0

        self.rgb_pub = self.create_publisher(Image, self.rgb_topic, 10)
        self.depth_pub = self.create_publisher(Image, self.depth_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, self.camera_info_topic, 10)
        self.rgbd_pub = self.create_publisher(RGBDImage, self.rgbd_topic, 10)

        self.frame_ts_file = None
        self.frame_ts_writer = None
        if self.frame_timestamps_csv:
            ts_path = Path(self.frame_timestamps_csv).expanduser().resolve()
            ts_path.parent.mkdir(parents=True, exist_ok=True)
            self.frame_ts_file = ts_path.open("w", newline="", encoding="utf-8")
            self.frame_ts_writer = csv.DictWriter(
                self.frame_ts_file,
                fieldnames=[
                    "sample_index",
                    "replay_index",
                    "stamp_sec",
                    "stamp_nanosec",
                    "time_sec",
                    "frame_id",
                ],
            )
            self.frame_ts_writer.writeheader()
            self.frame_ts_file.flush()
            self.get_logger().info(f"Writing replay frame timestamps to {ts_path}")

        period = 1.0 / max(self.rate_hz, 0.1)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(f"Replaying {len(self.rows)} RGB-D samples from {self.run_dir}")
        self.get_logger().info(
            f"Publishing at {self.rate_hz:.2f} Hz, frame_id='{self.frame_id}', "
            f"use_wall_time={self.use_wall_time}, loop={self.loop}"
        )
        self.get_logger().info(
            f"Topics: rgb={self.rgb_topic}, depth={self.depth_topic}, "
            f"camera_info={self.camera_info_topic}, rgbd={self.rgbd_topic}"
        )

    def _load_metadata(self, metadata_path: Path) -> dict:
        with metadata_path.open("r", encoding="utf-8") as f:
            return json.load(f)

    def _load_rows(self, poses_path: Path) -> list[dict]:
        with poses_path.open("r", newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        if not rows:
            raise RuntimeError(f"No rows found in {poses_path}")
        return rows

    def _read_intrinsics(self, metadata: dict):
        """Support the metadata layouts used in this project."""
        if "camera_intrinsics" in metadata:
            intr = metadata["camera_intrinsics"]
            fx = float(intr["fx"])
            fy = float(intr["fy"])
            cx = float(intr["cx"])
            cy = float(intr["cy"])
        elif "camera" in metadata and "intrinsics" in metadata["camera"]:
            intr = metadata["camera"]["intrinsics"]
            fx = float(intr["fx"])
            fy = float(intr["fy"])
            cx = float(intr["cx"])
            cy = float(intr["cy"])
        else:
            raise KeyError("Could not find camera intrinsics in metadata.json")

        if "config" in metadata and "sensors" in metadata["config"]:
            cam_cfg = metadata["config"]["sensors"]["camera"]
            width = int(cam_cfg["width"])
            height = int(cam_cfg["height"])
        elif "camera" in metadata and "width" in metadata["camera"]:
            width = int(metadata["camera"]["width"])
            height = int(metadata["camera"]["height"])
        else:
            width = int(round(cx * 2.0))
            height = int(round(cy * 2.0))

        return width, height, fx, fy, cx, cy

    def _sample_index_from_row(self, row: dict, fallback_index: int) -> int:
        for key in ["sample_index", "index", "frame_index", "sample", "frame"]:
            if key in row and row[key] not in (None, ""):
                return int(float(row[key]))
        return fallback_index

    def _make_camera_info(self, stamp) -> CameraInfo:
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.width = self.width
        msg.height = self.height
        msg.k = [
            self.fx, 0.0, self.cx,
            0.0, self.fy, self.cy,
            0.0, 0.0, 1.0,
        ]
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.distortion_model = "plumb_bob"
        msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]
        msg.p = [
            self.fx, 0.0, self.cx, 0.0,
            0.0, self.fy, self.cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return msg

    def _load_rgb(self, sample_index: int) -> np.ndarray:
        rgb_path = self.rgb_dir / f"{sample_index:06d}.png"
        rgb_bgr = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
        if rgb_bgr is None:
            raise FileNotFoundError(f"Failed to read RGB image: {rgb_path}")
        return rgb_bgr

    def _load_depth(self, sample_index: int) -> np.ndarray:
        depth_npy_path = self.depth_dir / f"{sample_index:06d}.npy"
        depth_png_path = self.depth_dir / f"{sample_index:06d}.png"

        if depth_npy_path.exists():
            depth = np.load(str(depth_npy_path)).astype(np.float32)
        elif depth_png_path.exists():
            depth = cv2.imread(str(depth_png_path), cv2.IMREAD_UNCHANGED)
            if depth is None:
                raise FileNotFoundError(f"Failed to read depth PNG: {depth_png_path}")
            depth = depth.astype(np.float32)
        else:
            raise FileNotFoundError(f"Missing depth file: {depth_npy_path} or {depth_png_path}")

        if depth.ndim == 3:
            depth = depth[:, :, 0]
        if depth.ndim != 2:
            raise ValueError(f"Depth image should be HxW, got shape={depth.shape}")

        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
        return depth.astype(np.float32)

    def _make_stamp(self, row: dict):
        if self.use_wall_time:
            return self.get_clock().now().to_msg()

        for key in ["timestamp", "time", "t", "sim_time"]:
            if key in row and row[key] not in (None, ""):
                t = float(row[key])
                sec = int(t)
                nanosec = int((t - sec) * 1e9)
                from builtin_interfaces.msg import Time
                return Time(sec=sec, nanosec=nanosec)

        return self.get_clock().now().to_msg()

    def _write_frame_timestamp(self, sample_index: int, stamp) -> None:
        if self.frame_ts_writer is None:
            return
        time_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        self.frame_ts_writer.writerow({
            "sample_index": sample_index,
            "replay_index": self.index,
            "stamp_sec": int(stamp.sec),
            "stamp_nanosec": int(stamp.nanosec),
            "time_sec": time_sec,
            "frame_id": self.frame_id,
        })
        self.frame_ts_file.flush()

    def _on_timer(self):
        if self.index >= len(self.rows):
            if self.loop:
                self.index = 0
            else:
                self.get_logger().info("Finished replaying dataset.")
                self.timer.cancel()
                return

        row = self.rows[self.index]
        sample_index = self._sample_index_from_row(row, self.index)

        try:
            rgb_bgr = self._load_rgb(sample_index)
            depth = self._load_depth(sample_index)
        except Exception as exc:
            self.get_logger().error(f"Skipping sample_index={sample_index}: {exc}")
            self.index += 1
            return

        stamp = self._make_stamp(row)

        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_bgr, encoding="bgr8")
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = self.frame_id

        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="32FC1")
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = self.frame_id

        camera_info_msg = self._make_camera_info(stamp)

        rgbd_msg = RGBDImage()
        rgbd_msg.header.stamp = stamp
        rgbd_msg.header.frame_id = self.frame_id
        rgbd_msg.rgb = rgb_msg
        rgbd_msg.depth = depth_msg
        rgbd_msg.rgb_camera_info = camera_info_msg
        rgbd_msg.depth_camera_info = camera_info_msg

        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        self.camera_info_pub.publish(camera_info_msg)
        self.rgbd_pub.publish(rgbd_msg)
        self._write_frame_timestamp(sample_index, stamp)

        if self.index == 0 or self.index % 30 == 0:
            self.get_logger().info(
                f"Published synchronized RGBD frame {self.index + 1}/{len(self.rows)} "
                f"sample_index={sample_index}, rgb_shape={rgb_bgr.shape}, "
                f"depth_shape={depth.shape}, depth_min={float(np.min(depth)):.3f}, "
                f"depth_max={float(np.max(depth)):.3f}"
            )

        self.index += 1

    def destroy_node(self) -> bool:
        try:
            if self.frame_ts_file is not None:
                self.frame_ts_file.flush()
                self.frame_ts_file.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DatasetRGBDPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
