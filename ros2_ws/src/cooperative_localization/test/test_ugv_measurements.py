"""Preserve measurement validity, acquisition time and supplied uncertainty."""

from types import SimpleNamespace
import numpy as np
import pytest
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from ugv_measurements import UgvMeasurements


@pytest.fixture
def adapter():
    rclpy.init()
    node = UgvMeasurements()
    clock = SimpleNamespace(nanoseconds=10_000_000_000)
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=clock.nanoseconds))
    outputs = {key: [] for key in ("gps", "imu", "odom")}
    for key, value in outputs.items():
        setattr(node, key + "_pub", SimpleNamespace(publish=value.append))
    yield node, clock, outputs
    node.destroy_node()
    rclpy.try_shutdown()


def fix():
    m = NavSatFix()
    m.header.frame_id = "simple_ugv/chassis/gps"
    m.header.stamp.sec = 10
    m.latitude = 47.397971057728974
    m.longitude = 8.546163739800146
    return m


def test_known_gps_covariance_and_time_survive(adapter):
    node, _, out = adapter
    m = fix()
    m.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
    m.position_covariance = np.diag([100., 64., 9.]).ravel().tolist()
    node.on_gps(m)
    c = np.array(out["gps"][0].pose.covariance).reshape(6, 6)
    np.testing.assert_allclose(c[:3, :3], np.diag([100., 64., 9.]), atol=1e-8)
    assert out["gps"][0].header.stamp == m.header.stamp


def test_unknown_gps_uses_explicit_fallback(adapter):
    node, _, out = adapter
    node.on_gps(fix())
    assert out["gps"][0].pose.covariance[0] == pytest.approx(2.25)


@pytest.mark.parametrize("fault", ["no_fix", "stale", "frame", "negative", "zero", "nan", "latitude"])
def test_bad_gps_is_not_forwarded(adapter, fault):
    node, _, out = adapter
    m = fix()
    if fault == "no_fix": m.status.status = NavSatStatus.STATUS_NO_FIX
    elif fault == "stale": m.header.stamp.sec = 1
    elif fault == "frame": m.header.frame_id = "wrong"
    elif fault == "nan": m.latitude = float("nan")
    elif fault == "latitude": m.latitude = 91.
    else:
        m.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
        m.position_covariance = np.diag([-1. if fault == "negative" else 0., 1., 1.]).ravel().tolist()
    node.on_gps(m)
    assert not out["gps"]


@pytest.mark.parametrize("fault", ["unavailable", "zero_quaternion", "nan_quaternion"])
def test_missing_heading_keeps_valid_turn_rate(adapter, fault):
    node, _, out = adapter
    m = Imu()
    m.header.frame_id = "simple_ugv/chassis/imu"
    m.header.stamp.sec = 10
    m.angular_velocity.z = .2
    if fault == "unavailable":
        m.orientation.w = 1.
        m.orientation_covariance[0] = -1.
    elif fault == "nan_quaternion": m.orientation.w = float("nan")
    elif fault == "zero_quaternion": m.orientation.w = 0.
    node.on_imu(m)
    assert out["imu"][0].orientation_covariance[0] == -1.
    assert out["imu"][0].angular_velocity.z == .2
    assert out["imu"][0].angular_velocity_covariance[0] > 0


def test_imu_known_covariance_and_unavailable_vectors(adapter):
    node, _, out = adapter
    m = Imu()
    m.header.frame_id = "simple_ugv/chassis/imu"
    m.header.stamp.sec = 10
    m.orientation.w = 1.
    m.orientation_covariance = np.diag([.1, .1, .2]).ravel().tolist()
    m.angular_velocity_covariance[0] = -1.
    m.linear_acceleration.x = float("nan")
    node.on_imu(m)
    assert out["imu"][0].orientation_covariance[8] == .2
    assert out["imu"][0].angular_velocity_covariance[0] == -1.
    assert out["imu"][0].linear_acceleration_covariance[0] == -1.


def test_wheel_uncertainty_and_clock_epoch(adapter):
    node, clock, out = adapter
    m = Odometry()
    m.header.frame_id = "simple_ugv/odom"
    m.child_frame_id = "simple_ugv/chassis"
    m.header.stamp.sec = 10
    m.twist.twist.linear.x = .3
    m.twist.covariance[0] = .4
    node.on_odom(m)
    node.on_odom(m)
    assert len(out["odom"]) == 1
    assert out["odom"][0].twist.covariance[0] == .4
    clock.nanoseconds = 2_000_000_000
    m.header.stamp.sec = 2
    node.on_odom(m)
    assert len(out["odom"]) == 2
