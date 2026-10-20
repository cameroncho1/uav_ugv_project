"""Check geometry, covariance propagation, acquisition time, and fault rejection."""

import math
from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import SetBool

from localization_common.frames import (FRD_TO_FLU, NED_TO_ENU, covariance, interpolate,
                              px4_pose, quaternion, rotation, ugv_position)
from uav_correction import UavCorrection


def test_px4_conversion_includes_world_and_body_axes_and_origin():
    position, q, c = px4_pose([2., 3., -4.], [1., 0., 0., 0.],
                              [.1, .2, .3], [.01, .02, .03], [10., 20., 1.])
    np.testing.assert_allclose(position, [13., 22., 5.])
    np.testing.assert_allclose(rotation(q) @ [1., 0., 0.], [0., 1., 0.], atol=1e-12)
    np.testing.assert_allclose(rotation(q) @ [0., 1., 0.], [-1., 0., 0.], atol=1e-12)
    np.testing.assert_allclose(np.diag(c)[:3], [.2, .1, .3])


@pytest.mark.parametrize("angle", [0., .4, math.pi, -math.pi/2])
def test_quaternion_and_px4_pose_round_trip(angle):
    q = [math.cos(angle/2), math.sin(angle/2)/math.sqrt(3),
         math.sin(angle/2)/math.sqrt(3), math.sin(angle/2)/math.sqrt(3)]
    r = rotation(q)
    np.testing.assert_allclose(rotation(quaternion(r)), r, atol=1e-12)
    ned_q = quaternion(NED_TO_ENU @ r @ FRD_TO_FLU)
    _, result, _ = px4_pose([0., 0., 0.], ned_q, [1.]*3, [.01]*3, [0.]*3)
    np.testing.assert_allclose(rotation(result), r, atol=1e-12)


def test_interpolation_handles_opposite_quaternion_signs():
    first = (np.zeros(3), np.array([1., 0., 0., 0.]), np.eye(6))
    second = (np.ones(3)*2, np.array([-1., 0., 0., 0.]), np.eye(6)*3)
    p, q, c = interpolate(first, second, .5)
    np.testing.assert_allclose(p, [1.]*3)
    np.testing.assert_allclose(q, first[1])
    np.testing.assert_allclose(c, np.eye(6)*2)


def test_attitude_uncertainty_grows_with_observation_lever_arm():
    anchor = (np.zeros(3), [1., 0., 0., 0.], np.diag([.1]*3 + [.01]*3))
    _, near = ugv_position(anchor, [1., 0., 0.], np.eye(3)*.04)
    _, far = ugv_position(anchor, [10., 0., 0.], np.eye(3)*.04)
    assert far[1, 1] == pytest.approx(1.14)
    assert near[1, 1] == pytest.approx(.15)


def test_covariance_matches_monte_carlo_independent_position_and_attitude_errors():
    rng = np.random.default_rng(513)
    relative = np.array([4., -2., -3.])
    q = [math.cos(.3), 0., 0., math.sin(.3)]
    r = rotation(q)
    anchor_cov = np.diag([.09, .09, .25, .0001, .0001, .0004])
    relative_cov = np.diag([.0625, .0225, .0625])
    _, predicted = ugv_position((np.zeros(3), q, anchor_cov), relative, relative_cov)
    errors = []
    # Factor each covariance once; repeated single draws spend most of the
    # test doing identical decompositions on constrained simulation hosts.
    anchor_errors = rng.multivariate_normal(np.zeros(6), anchor_cov, 10000)
    relative_errors = rng.multivariate_normal(np.zeros(3), relative_cov, 10000)
    for anchor_error, relative_error in zip(anchor_errors, relative_errors):
        theta = anchor_error[3:]
        a = np.linalg.norm(theta)
        error_q = [math.cos(a/2), *(theta/a*math.sin(a/2))]
        noisy_relative = relative + relative_error
        errors.append(anchor_error[:3] + rotation(error_q) @ r @ noisy_relative - r @ relative)
    np.testing.assert_allclose(np.diag(np.cov(np.array(errors).T)), np.diag(predicted), rtol=.05)


@pytest.mark.parametrize("values", [[float("nan")]*9, [-1., 0., 0., 0., 1., 0., 0., 0., 1.],
                                    [1., 1., 0., 0., 1., 0., 0., 0., 1.]])
def test_invalid_covariance_is_rejected(values):
    with pytest.raises(ValueError):
        covariance(values)


@pytest.fixture
def correction():
    rclpy.init()
    node = UavCorrection()
    messages = []
    node.publisher = SimpleNamespace(publish=messages.append)
    yield node, messages
    node.destroy_node()
    rclpy.shutdown()


def reading(node, stamp, position, frame):
    message = PoseWithCovarianceStamped()
    message.header.frame_id = frame
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1000000000)
    p = message.pose.pose.position
    p.x, p.y, p.z = (float(value) for value in position)
    message.pose.pose.orientation.w = 1.
    message.pose.covariance = np.diag([.04]*3 + [.0001]*3).flatten().tolist()
    return message


def test_relative_waits_for_anchor_and_keeps_acquisition_time(correction):
    node, output = correction
    stamp = node.get_clock().now().nanoseconds - 100000000
    node.on_relative(reading(node, stamp, [2., 0., -4.], "uav/base_link"))
    assert not output
    node.on_anchor(reading(node, stamp, [3., 1., 4.], "map"))
    assert len(output) == 1
    assert output[0].pose.pose.position.x == 5.
    assert output[0].header.stamp.nanosec == stamp % 1000000000
    assert output[0].pose.covariance[35] == 1e6


def test_missing_anchor_expires_without_extrapolation(correction):
    node, output = correction
    stamp = node.get_clock().now().nanoseconds - 100000000
    node.on_relative(reading(node, stamp, [2., 0., -4.], "uav/base_link"))
    node.on_anchor(reading(node, stamp+50000000, [3., 1., 4.], "map"))
    assert not output
    node.max_age = .01
    node.process()
    assert not node.pending
    assert node.counters["expired"] == 1


def test_outlier_is_rejected_and_normal_reading_recovers(correction):
    node, output = correction
    stamp = node.get_clock().now().nanoseconds - 300000000
    for offset, relative_x in [(0, 1.), (100000000, 9.), (200000000, 1.05)]:
        node.on_anchor(reading(node, stamp+offset, [0., 0., 4.], "map"))
        node.on_relative(reading(node, stamp+offset, [relative_x, 0., -4.], "uav/base_link"))
    assert len(output) == 2
    assert node.counters["jump_rejected"] == 1


def test_disabled_correction_and_wrong_frame_do_not_publish(correction):
    node, output = correction
    stamp = node.get_clock().now().nanoseconds - 100000000
    node.on_anchor(reading(node, stamp, [0., 0., 4.], "map"))
    node.on_relative(reading(node, stamp, [1., 0., -4.], "camera"))
    node.set_enabled(SetBool.Request(data=False), SetBool.Response())
    node.on_relative(reading(node, stamp, [1., 0., -4.], "uav/base_link"))
    assert not output
    assert node.counters["invalid_relative"] == 1




def test_multiple_pending_readings_rewind_without_crash_and_recover(correction):
    from rclpy.time import Time
    node, output = correction
    now = SimpleNamespace(nanoseconds=10_000_000_000)
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=now.nanoseconds))
    for stamp in (9_800_000_000, 9_900_000_000):
        node.on_relative(reading(node, stamp, [1., 0., -4.], "uav/base_link"))
    assert len(node.pending) == 2
    now.nanoseconds = 2_000_000_000
    node.process()
    assert not node.pending and not output
    node.on_anchor(reading(node, now.nanoseconds, [0., 0., 4.], "map"))
    node.on_relative(reading(node, now.nanoseconds, [1., 0., -4.], "uav/base_link"))
    assert len(output) == 1


def test_heading_update_preserves_joint_covariance_without_duplicate_position(correction):
    node, position_only = correction
    node.heading_enabled = True
    joint = []
    node.heading_publisher = SimpleNamespace(publish=joint.append)
    stamp = node.get_clock().now().nanoseconds - 100000000
    anchor = reading(node, stamp, [3., 1., 4.], 'map')
    anchor.pose.pose.orientation.w = math.cos(math.pi/4)
    anchor.pose.pose.orientation.z = math.sin(math.pi/4)
    relative = reading(node, stamp, [2., 0., -4.], 'uav/base_link')
    c = np.array(relative.pose.covariance).reshape(6, 6)
    c[1, 5] = c[5, 1] = .001
    relative.pose.covariance = c.ravel().tolist()
    node.on_relative(relative)
    node.on_anchor(anchor)
    assert not position_only
    assert len(joint) == 1
    measured = joint[0]
    assert measured.header.stamp == relative.header.stamp
    q = measured.pose.pose.orientation
    assert math.atan2(2*q.w*q.z, 1-2*q.z*q.z) == pytest.approx(math.pi/2)
    out = np.array(measured.pose.covariance).reshape(6, 6)
    assert abs(out[0, 5]) > .0001
    assert np.linalg.eigvalsh(out[np.ix_([0, 1, 5], [0, 1, 5])]).min() > 0
    node.on_relative(relative)
    assert len(joint) == 1


@pytest.mark.parametrize('fault', ['missing_orientation', 'unknown_uncertainty', 'vertical'])
def test_unusable_heading_keeps_position_only_correction(correction, fault):
    node, position_only = correction
    node.heading_enabled = True
    joint = []
    node.heading_publisher = SimpleNamespace(publish=joint.append)
    stamp = node.get_clock().now().nanoseconds - 100000000
    relative = reading(node, stamp, [1., 0., -4.], 'uav/base_link')
    if fault == 'missing_orientation':
        relative.pose.pose.orientation.w = 0.
    elif fault == 'unknown_uncertainty':
        relative.pose.covariance[35] = 1e6
    else:
        relative.pose.pose.orientation.w = math.cos(math.pi/4)
        relative.pose.pose.orientation.y = math.sin(math.pi/4)
    node.on_anchor(reading(node, stamp, [0., 0., 4.], 'map'))
    node.on_relative(relative)
    assert len(position_only) == 1
    assert not joint
    assert position_only[0].pose.covariance[35] == 1e6
    assert node.counters['heading_unavailable'] == 1
