"""Known geometry, image visibility, and correlated calibration error checks."""

import sys
import math
import copy
from types import SimpleNamespace
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
import pytest
import rclpy
import yaml
from sensor_msgs.msg import CameraInfo, Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from fiducial_math import Calibration, detect_marker, marker_points, solve_marker  # noqa: E402
from uav_fiducial_sensor import FiducialSensor  # noqa: E402

K = np.array([[540., 0., 640.], [0., 540., 480.], [0., 0., 1.]])
BODY_ROTATION = np.array([[0., -1., 0.], [-1., 0., 0.], [0., 0., -1.]])
MOUNT = np.array([0., 0., -0.18])
OFFSET = np.array([0., 0., -0.12])


def project(rvec, tvec):
    return cv2.projectPoints(marker_points(0.4), rvec, tvec, K, np.zeros(5))[0].reshape(4, 2)


@pytest.mark.parametrize('translation', [[0.2, -0.4, 3.5], [-2.5, 1.0, 4.0], [0.8, 0.3, 8.0]])
def test_known_projection_and_mount(translation):
    rvec = np.array([2.9, 0.2, 0.1])
    tvec = np.array(translation)
    point, covariance, error = solve_marker(project(rvec, tvec), K, np.zeros(5),
                                            BODY_ROTATION, MOUNT, OFFSET)
    expected = BODY_ROTATION @ (cv2.Rodrigues(rvec)[0] @ OFFSET + tvec) + MOUNT
    np.testing.assert_allclose(point, expected, atol=1e-6)
    assert error < 1e-6
    assert np.linalg.eigvalsh(covariance).min() > 0


def marker_image(corners):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = (cv2.aruco.generateImageMarker(dictionary, 7, 400)
              if hasattr(cv2.aruco, 'generateImageMarker') else
              cv2.aruco.drawMarker(dictionary, 7, 400))
    transform = cv2.getPerspectiveTransform(
        np.array([[0, 0], [399, 0], [399, 399], [0, 399]], np.float32),
        corners.astype(np.float32))
    return cv2.warpPerspective(marker, transform, (1280, 960), borderValue=255)


def test_image_detection_occlusion_and_absent_marker():
    corners = project(np.array([2.9, 0.2, 0.1]), np.array([0.2, -0.4, 3.5]))
    image = marker_image(corners)
    detected = detect_marker(image)
    assert detected is not None
    np.testing.assert_allclose(detected, corners, atol=1.5)
    left, top = np.floor(corners.min(axis=0)).astype(int)
    right, bottom = np.ceil(corners.max(axis=0)).astype(int)
    image[top:bottom, (left + right) // 2:right + 2] = 255
    assert detect_marker(image) is None
    assert detect_marker(np.full((960, 1280), 255, np.uint8)) is None


def test_rendered_off_axis_border_and_partial_occlusion():
    image = cv2.imread(str(Path(__file__).parent / 'fixtures/off_axis_tag.png'))
    assert image is not None
    corners = detect_marker(image)
    assert corners is not None
    # The rendered target is one metre forward and half a metre right.
    assert 675 < corners[:, 0].mean() < 765
    assert 275 < corners[:, 1].mean() < 370
    left, top = np.floor(corners.min(axis=0)).astype(int)
    right, bottom = np.ceil(corners.max(axis=0)).astype(int)
    image[top:bottom + 2, (left + right) // 2:right + 2] = 204
    assert detect_marker(image) is None


def test_duplicate_target_ids_are_ambiguous():
    corners = np.array([[200., 200.], [400., 200.], [400., 400.], [200., 400.]])
    first = marker_image(corners)
    second = marker_image(corners + [600., 0.])
    assert detect_marker(first) is not None
    assert detect_marker(second) is not None
    assert detect_marker(np.minimum(first, second)) is None


def test_rejects_invalid_calibration_and_small_targets():
    corners = project(np.array([2.9, 0.2, 0.1]), np.array([0., 0., 4.]))
    with pytest.raises(ValueError):
        solve_marker(corners, K, [float('nan')], BODY_ROTATION, MOUNT, OFFSET)
    with pytest.raises(ValueError):
        solve_marker(corners, np.zeros((3, 3)), [], BODY_ROTATION, MOUNT, OFFSET)
    with pytest.raises(ValueError, match='small'):
        solve_marker(corners * 0.01, K, [], BODY_ROTATION, MOUNT, OFFSET)
    with pytest.raises(ValueError):
        Calibration(pixel_stddev=0)


def test_front_parallel_camera():
    rvec = cv2.Rodrigues(BODY_ROTATION)[0].ravel()
    corners = project(rvec, np.array([0., 0., 3.45]))
    point, _, error = solve_marker(corners, K, [], BODY_ROTATION, MOUNT, OFFSET)
    np.testing.assert_allclose(point, [0., 0., -3.75], atol=1e-5)
    assert error < 1e-5


def test_shared_principal_error_does_not_average_away():
    rvec, tvec = np.array([2.7, 0.3, 0.1]), np.array([1.2, -0.8, 4.])
    corners = project(rvec, tvec)
    config = Calibration(pixel_stddev=0.2, principal_point_stddev=2.0,
                         focal_length_stddev=1e-8,
                         mount_position_stddev=1e-8, mount_angle_stddev=1e-8,
                         marker_size_stddev=1e-8)
    center, covariance, _ = solve_marker(corners, K, [], BODY_ROTATION, MOUNT, OFFSET, config)
    random = np.random.default_rng(513)
    errors = []
    for _ in range(600):
        noisy = corners + random.normal(0, 0.2, (4, 2)) + random.normal(0, 2.0, (1, 2))
        point, _, _ = solve_marker(noisy, K, [], BODY_ROTATION, MOUNT, OFFSET, config)
        errors.append(point - center)
    empirical = np.cov(np.array(errors).T)
    np.testing.assert_allclose(np.diag(empirical), np.diag(covariance), rtol=0.35, atol=1e-5)


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture
def sensor():
    rclpy.init()
    node = FiducialSensor()
    # Callback tests control acquisition time independently of OpenCV runtime.
    # Rendered trials exercise the actual processing deadline.
    from rclpy.time import Time
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=10_000_000_000))
    node.publisher = Capture()
    yield node
    node.destroy_node()
    rclpy.try_shutdown()


def camera_message(sensor):
    info = CameraInfo()
    info.header.frame_id = sensor.input_frame
    info.width, info.height = 1280, 960
    info.k = K.ravel().tolist()
    info.d = [0.] * 5
    info.distortion_model = 'plumb_bob'
    sensor.on_info(info)
    image = Image()
    image.header.frame_id = sensor.input_frame
    image.header.stamp = sensor.get_clock().now().to_msg()
    image.width, image.height = 1280, 960
    image.encoding = 'mono8'
    image.step = 1280
    pixels = marker_image(project(np.array([2.9, 0.2, 0.1]), np.array([0.2, -0.4, 3.5])))
    image.data = pixels.ravel().tolist()
    return image


def test_camera_callback_preserves_acquisition_and_body_contract(sensor):
    message = camera_message(sensor)
    sensor.on_image(message)
    assert len(sensor.publisher.messages) == 1
    output = sensor.publisher.messages[0]
    assert output.header.frame_id == 'uav/base_link'
    assert output.header.stamp == message.header.stamp
    assert output.pose.covariance[35] == 1e6
    assert output.pose.covariance[0] > 0
    sensor.on_image(message)
    assert len(sensor.publisher.messages) == 1


def test_exact_five_hz_images_are_not_dropped_by_float_rounding(sensor):
    message = camera_message(sensor)
    clock = SimpleNamespace(nanoseconds=10000000000)
    sensor.get_clock = lambda: SimpleNamespace(now=lambda: clock)
    message.header.stamp.sec = 10
    message.header.stamp.nanosec = 0
    sensor.on_image(message)
    second = copy.deepcopy(message)
    second.header.stamp.nanosec = 200000000
    clock.nanoseconds = 10200000000
    sensor.on_image(second)
    assert len(sensor.publisher.messages) == 2


@pytest.mark.parametrize('fault', [
    'stale', 'missing_info', 'wrong_frame', 'invalid_data', 'empty_image', 'absent'])
def test_camera_callback_rejects_invalid_inputs(sensor, fault):
    message = camera_message(sensor)
    if fault == 'stale':
        message.header.stamp.sec -= 2
    elif fault == 'missing_info':
        sensor.info = None
    elif fault == 'wrong_frame':
        message.header.frame_id = 'map'
    elif fault == 'invalid_data':
        message.data = [0]
    elif fault == 'empty_image':
        message.width = sensor.info.width = 0
        message.height = sensor.info.height = 0
        message.step = 0
        message.data = []
    else:
        message.data = [255] * (1280 * 960)
    sensor.on_image(message)
    assert not sensor.publisher.messages


def test_calibration_matches_owned_camera_and_roof_mount():
    root = Path(__file__).resolve().parents[4]
    config = yaml.safe_load((root / 'ros2_ws/src/uav_sensing/config/'
                             'camera_calibration.yaml').read_text())
    values = config['uav_fiducial_sensor']['ros__parameters']
    model = ET.parse(root / 'simulation/models/cooperative_camera_uav/model.sdf')
    assert model.findtext("./model/link/sensor/gz_frame_id") == values['input_frame']
    pose = list(map(float, model.find("./model/link[@name='camera_link']/pose").text.split()))
    np.testing.assert_allclose(pose[:3], values['body_from_optical_translation'])
    assert pose[3] == pose[5] == 0
    c, s = math.cos(pose[4]), math.sin(pose[4])
    mount = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    optical_to_camera = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    np.testing.assert_allclose((mount @ optical_to_camera).ravel(),
                               values['body_from_optical_rotation'], atol=1e-12)
    for world in ['uav_ugv', 'simple_goal_demo']:
        tree = ET.parse(root / f'simulation/worlds/{world}.sdf')
        model = tree.find("./world/model[@name='simple_ugv']")
        assert model.attrib['canonical_link'] == 'chassis'
        drive = model.find("./plugin[@name='gz::sim::systems::DiffDrive']")
        assert drive.findtext('frame_id') == 'simple_ugv/odom'
        assert drive.findtext('child_frame_id') == 'simple_ugv/chassis'
        includes = tree.findall("./world/model[@name='simple_ugv']/include")
        tag = next(item for item in includes if item.findtext('uri') == 'model://ugv_fiducial')
        assert tag.find('pose').attrib['relative_to'] == 'chassis'
        tag_pose = list(map(float, tag.findtext('pose').split()))
        np.testing.assert_allclose(-np.array(tag_pose[:3]), values['marker_to_chassis'])




def test_old_image_is_rejected_but_real_clock_reset_recovers(sensor):
    from rclpy.time import Time
    clock = SimpleNamespace(nanoseconds=10_000_000_000)
    sensor.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=clock.nanoseconds))
    image = camera_message(sensor)
    sensor.on_image(image)
    old = copy.deepcopy(image)
    old.header.stamp.sec = 9
    old.header.stamp.nanosec = 900_000_000
    sensor.on_image(old)
    assert len(sensor.publisher.messages) == 1
    clock.nanoseconds = 2_000_000_000
    reset = copy.deepcopy(image)
    reset.header.stamp.sec = 2
    sensor.on_image(reset)
    assert len(sensor.publisher.messages) == 2


def test_rejected_future_image_does_not_poison_monotonic_state(sensor):
    from rclpy.time import Time
    clock = SimpleNamespace(nanoseconds=10_000_000_000)
    sensor.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=clock.nanoseconds))
    valid = camera_message(sensor)
    future = copy.deepcopy(valid)
    future.header.stamp.sec = 11
    sensor.on_image(future)
    sensor.on_image(valid)
    assert len(sensor.publisher.messages) == 1
