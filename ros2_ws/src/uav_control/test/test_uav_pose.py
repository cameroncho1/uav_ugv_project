"""PX4 anchor timing, initialization and reset contract."""

from types import SimpleNamespace
import numpy as np
import rclpy
from px4_msgs.msg import VehicleLandDetected, VehicleOdometry
from uav_pose import UavPose


def test_px4_adapter_rejects_clock_mismatch_unknown_frame_and_reset():
    rclpy.init()
    node = UavPose()
    output = []
    node.publisher = SimpleNamespace(publish=output.append)
    message = VehicleOdometry()
    message.timestamp_sample = node.get_clock().now().nanoseconds // 1000
    message.q = np.array([1., 0., 0., 0.], dtype=np.float32)
    node.on_odometry(message)
    assert not output
    message.pose_frame = VehicleOdometry.POSE_FRAME_NED
    node.on_odometry(message)
    assert len(output) == 1
    message.timestamp_sample += 1000
    message.reset_counter = 1
    node.on_odometry(message)
    assert len(output) == 1
    message.timestamp_sample = 10
    node.on_odometry(message)
    assert len(output) == 1
    node.destroy_node()
    rclpy.shutdown()


def test_live_anchor_waits_for_takeoff_and_stable_reference():
    rclpy.init()
    node = UavPose()
    node.require_airborne = True
    node.settle_time = .1
    output = []
    node.publisher = SimpleNamespace(publish=output.append)
    stamp = node.get_clock().now().nanoseconds // 1000 - 300000
    message = VehicleOdometry(timestamp_sample=stamp, pose_frame=VehicleOdometry.POSE_FRAME_NED)
    message.q = np.array([1., 0., 0., 0.], dtype=np.float32)
    node.on_odometry(message)
    assert not output
    node.on_landing(VehicleLandDetected(timestamp=stamp, landed=False))
    node.on_odometry(message)
    assert not output
    message.timestamp_sample += 50000
    message.reset_counter = 1
    node.on_odometry(message)
    assert not output
    message.timestamp_sample += 150000
    node.on_odometry(message)
    assert len(output) == 1
    message.timestamp_sample += 1000
    message.reset_counter = 2
    node.on_odometry(message)
    assert len(output) == 1
    node.destroy_node()
    rclpy.shutdown()
