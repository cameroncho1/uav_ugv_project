"""Explicit simulation bias and relative-sensor contracts."""

import numpy as np
import pytest
import rclpy
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import Pose, PoseArray
from sim_anchor_fault import biased_anchor
from simulation_faults import common_position_bias
from sim_uav_sensor import SimUavSensor


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_shared_fault_preserves_time_and_converts_enu_bias_to_ned():
    message = VehicleOdometry()
    message.pose_frame = VehicleOdometry.POSE_FRAME_NED
    message.position = np.array([1., 2., -4.], np.float32)
    message.position_variance = np.array([0.1, 0.2, 0.3], np.float32)
    message.timestamp_sample = 12000000
    output = biased_anchor(message, [3., 5., 0.], 0.8)
    np.testing.assert_allclose(output.position, [6., 5., -4.])
    np.testing.assert_allclose(output.position_variance, [0.74, 0.84, 0.3])
    assert output.timestamp_sample == 12000000
    np.testing.assert_allclose(message.position, [1., 2., -4.])
    np.testing.assert_array_equal(common_position_bias(513, 0.8), common_position_bias(513, 0.8))
    with pytest.raises(ValueError):
        common_position_bias(513, -1)


def test_vector_fixture_reports_chassis_point_in_uav_body():
    rclpy.init()
    sensor = SimUavSensor()
    try:
        sensor.relative_pub = Capture()
        sensor.publish_anchor = False
        sensor.relative_std = np.zeros(3)
        message = PoseArray()
        message.header.frame_id = 'map'
        message.header.stamp.sec = 10
        uav, ugv = Pose(), Pose()
        uav.position.z = 4.0
        uav.orientation.w = ugv.orientation.w = 1.0
        message.poses = [uav, ugv]
        sensor.on_poses(message)
        point = sensor.relative_pub.messages[0].pose.pose.position
        assert (point.x, point.y, point.z) == (0, 0, -3.75)
    finally:
        sensor.destroy_node()
        rclpy.try_shutdown()
