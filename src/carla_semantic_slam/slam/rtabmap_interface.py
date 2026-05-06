"""Placeholder interface for later RTAB-Map integration.

The current pipeline uses CARLA ground-truth camera pose for the baseline semantic map.
Later, this module can provide the camera pose estimated by RTAB-Map in the map frame.
"""
from __future__ import annotations


def get_slam_pose_for_frame(frame_id: int):
    raise NotImplementedError("RTAB-Map pose lookup will be added after the baseline map works.")
