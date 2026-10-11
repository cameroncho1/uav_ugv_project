"""Check the public fused-pose message contract."""

import importlib.util
from pathlib import Path

import pytest
from nav_msgs.msg import Odometry


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "ugv_fused_pose.py"
SPEC = importlib.util.spec_from_file_location("ugv_fused_pose", MODULE_PATH)
fused_pose = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(fused_pose)


def test_preserves_filter_timestamp_pose_and_covariance():
    odometry = Odometry()
    odometry.header.frame_id = "map"
    odometry.header.stamp.sec = 12
    odometry.pose.pose.position.x = 3.0
    odometry.pose.covariance[0] = 0.25

    output = fused_pose.pose_from_odometry(odometry)

    assert output.header.frame_id == "map"
    assert output.header.stamp.sec == 12
    assert output.pose.pose.position.x == 3.0
    assert output.pose.covariance[0] == 0.25


def test_rejects_non_map_frame():
    odometry = Odometry()
    odometry.header.frame_id = "odom"

    with pytest.raises(ValueError, match="map frame"):
        fused_pose.pose_from_odometry(odometry)
