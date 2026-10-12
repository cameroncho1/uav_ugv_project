"""Acquisition-matched anchors and metric ground projection."""

import importlib.machinery
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import rclpy
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import CameraInfo, Image
from grid_projection import project_grid, robot_projection_mask

LOADER = importlib.machinery.SourceFileLoader("mapper", str(
    Path(__file__).resolve().parents[1] / "scripts/traversability_mapper"))
SPEC = importlib.util.spec_from_loader(LOADER.name, LOADER)
mapper = importlib.util.module_from_spec(SPEC)
LOADER.exec_module(mapper)


def test_ground_projection_applies_camera_position_and_axes():
    gray = np.full((80, 100), 255, np.uint8)
    k = np.array([[50., 0., 50.], [0., 50., 40.], [0., 0., 1.]])
    r = np.diag([1., -1., -1.])
    origin, data, x, y = project_grid(gray, k, [], np.array([2., 3., 4.]), r, .25, 32.)
    assert origin[0] == -2. and origin[1] < 3.
    assert np.any(data == 0)
    assert x.min() < 2. < x.max() and y.min() < 3. < y.max()


def test_own_roof_projection_is_removed_without_masking_unrelated_ground():
    # An elevated roof at x=-4 projects farther from a camera at x=0.
    # A ground-only radius would leave this tag pixel as a false obstacle.
    x = np.array([[-4.5, -6., 0.]])
    mask = robot_projection_mask(x, np.zeros(x.shape), np.array([0., 0., 4.]),
                                 np.array([-4., 0., 0.]), .5, .5)
    assert mask.tolist() == [[True, False, False]]


def test_grid_waits_for_matching_anchor_and_preserves_image_time():
    rclpy.init()
    node = mapper.TraversabilityMapper()
    try:
        out = []
        node.grid_pub = SimpleNamespace(publish=out.append)
        clock = SimpleNamespace(nanoseconds=10_000_000_000)
        node.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=clock.nanoseconds))
        info = CameraInfo()
        info.header.frame_id = "uav/camera_optical_frame"
        info.width, info.height = 100, 80
        info.k = [50., 0., 50., 0., 50., 40., 0., 0., 1.]
        info.distortion_model = "plumb_bob"
        node.on_info(info)
        image = Image()
        image.header = info.header
        image.header.stamp.sec = 10
        image.width, image.height, image.step = 100, 80, 100
        image.encoding = "mono8"
        image.data = [255] * 8000
        vehicle = PoseWithCovarianceStamped()
        vehicle.header.frame_id = "map"
        vehicle.header.stamp.sec = 9
        vehicle.header.stamp.nanosec = 900_000_000
        vehicle.pose.pose.orientation.w = 1.
        vehicle.pose.covariance = (np.eye(6) * .01).ravel().tolist()
        node.on_ugv(vehicle)
        node.on_image(image)
        assert not out
        anchor = PoseWithCovarianceStamped()
        anchor.header.frame_id = "map"
        anchor.header.stamp.sec = 10
        anchor.pose.pose.position.z = 4.
        anchor.pose.pose.orientation.w = 1.
        anchor.pose.covariance = np.eye(6).ravel().tolist()
        node.on_anchor(anchor)
        assert not out  # Self masking also waits for acquisition-time coverage.
        vehicle.header.stamp.sec = 10
        vehicle.header.stamp.nanosec = 0
        node.on_ugv(vehicle)
        assert len(out) == 1 and out[0].header.stamp == image.header.stamp
        clock.nanoseconds += 1_000_000_000
        image.header.stamp.sec = 11
        node.on_image(image)
        assert len(out) == 1  # Old pose cannot authorize a fresh map.
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
