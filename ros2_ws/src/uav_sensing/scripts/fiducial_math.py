"""Calibrated square-marker pose and uncertainty, independent of ROS and truth."""

from dataclasses import dataclass

import cv2
import numpy as np

from localization_common.frames import quaternion, skew


@dataclass(frozen=True)
class Calibration:
    marker_size: float = 0.4
    pixel_stddev: float = 0.5
    principal_point_stddev: float = 0.25
    focal_length_stddev: float = 0.25
    marker_size_stddev: float = 0.001
    mount_position_stddev: float = 0.01
    mount_angle_stddev: float = 0.005
    marker_mount_angle_stddev: float = 0.01
    max_reprojection_error: float = 1.5
    min_edge_pixels: float = 12.0

    def __post_init__(self):
        if any(not np.isfinite(v) or v <= 0 for v in vars(self).values()):
            raise ValueError('Calibration values must be finite and positive')


def marker_points(size):
    half = size / 2
    # OpenCV IPPE_SQUARE requires this order: image top-left clockwise.
    return np.array([[-half, half, 0], [half, half, 0],
                     [half, -half, 0], [-half, -half, 0]], dtype=np.float64)


@dataclass(frozen=True)
class MarkerPose:
    """Chassis pose in UAV FLU, joint fixed-axis covariance, and image forward axis."""

    position: np.ndarray
    attitude: np.ndarray
    covariance: np.ndarray
    error: float
    image_axis: np.ndarray


def solve_marker(corners, intrinsic, distortion, body_rotation, body_translation,
                 marker_to_chassis, calibration=Calibration()):
    """Return position-only observations for consumers that do not use heading."""
    pose = solve_marker_pose(corners, intrinsic, distortion, body_rotation, body_translation,
                             marker_to_chassis, calibration=calibration)
    return pose.position, pose.covariance[:3, :3], pose.error


def solve_marker_pose(corners, intrinsic, distortion, body_rotation, body_translation,
                      marker_to_chassis, marker_from_chassis_rotation=None,
                      calibration=Calibration()):
    """
    Return FLU chassis pose with joint position/rotation covariance.

    Pixel noise includes independent corner errors and a shared principal-point
    error. Mount and marker-size uncertainty remain common to all corners.
    This covariance is a local approximation, conditional on correct marker ID.
    """
    corners = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    intrinsic = np.asarray(intrinsic, dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(distortion, dtype=np.float64).reshape(-1)
    rotation = np.asarray(body_rotation, dtype=np.float64).reshape(3, 3)
    translation = np.asarray(body_translation, dtype=np.float64).reshape(3)
    offset = np.asarray(marker_to_chassis, dtype=np.float64).reshape(3)
    chassis_rotation = (np.eye(3) if marker_from_chassis_rotation is None else
                        np.asarray(marker_from_chassis_rotation, dtype=float).reshape(3, 3))
    if not all(np.isfinite(v).all() for v in
               [corners, intrinsic, distortion, rotation, translation, offset, chassis_rotation]):
        raise ValueError('Nonfinite camera calibration or corners')
    if (intrinsic[0, 0] <= 0 or intrinsic[1, 1] <= 0 or
            not np.allclose(intrinsic[2], [0, 0, 1]) or
            not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or
            not np.isclose(np.linalg.det(rotation), 1.0) or
            not np.allclose(chassis_rotation.T @ chassis_rotation, np.eye(3), atol=1e-6) or
            not np.isclose(np.linalg.det(chassis_rotation), 1.0)):
        raise ValueError('Invalid camera calibration or rotation')
    edges = np.linalg.norm(corners - np.roll(corners, -1, axis=0), axis=1)
    if edges.min() < calibration.min_edge_pixels:
        raise ValueError('Marker is too small')
    objects = marker_points(calibration.marker_size)
    _, rotations, translations, _ = cv2.solvePnPGeneric(
        objects, corners, intrinsic, distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    # Iterative homography initialization covers IPPE's numerical singularity
    # at an exactly front-parallel square. Every candidate uses the same gates.
    solved, iterative_r, iterative_t = cv2.solvePnP(
        objects, corners, intrinsic, distortion, flags=cv2.SOLVEPNP_ITERATIVE)
    if solved:
        rotations = (*rotations, iterative_r)
        translations = (*translations, iterative_t)
    candidates = []
    for rvec, tvec in zip(rotations, translations):
        if not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
            continue
        r = cv2.Rodrigues(rvec)[0]
        t = tvec.reshape(3)
        if np.any((objects @ r.T + t)[:, 2] <= 0):
            continue
        pixels, jacobian = cv2.projectPoints(objects, rvec, tvec, intrinsic, distortion)
        error = float(np.sqrt(np.mean((pixels.reshape(4, 2) - corners)**2)))
        candidates.append((error, rvec.reshape(3), t, jacobian))
    if not candidates:
        raise ValueError('Marker pose has no positive-depth solution')
    error, rvec, tvec, jacobian = min(candidates, key=lambda c: c[0])
    if error > calibration.max_reprojection_error:
        raise ValueError('Marker reprojection error exceeds the limit')
    if np.linalg.matrix_rank(jacobian[:, :6]) < 6:
        raise ValueError('Marker pose is geometrically degenerate')
    pixel_cov = calibration.pixel_stddev**2 * np.eye(8)
    pixel_cov += calibration.principal_point_stddev**2 * np.kron(np.ones((4, 4)), np.eye(2))
    focal_jacobian = jacobian[:, 6] + jacobian[:, 7]
    pixel_cov += calibration.focal_length_stddev**2 * np.outer(focal_jacobian, focal_jacobian)
    inverse = np.linalg.pinv(jacobian[:, :6])
    pose_cov = inverse @ pixel_cov @ inverse.T
    optical_from_marker = cv2.Rodrigues(rvec)[0]
    body_from_chassis = rotation @ optical_from_marker @ chassis_rotation
    camera_point = optical_from_marker @ offset + tvec
    # OpenCV uses additive Rodrigues-vector errors; ROS pose covariance uses
    # small fixed-parent-axis rotations. Transform both parts jointly.
    pose_jacobian = np.zeros((6, 6))
    step = 1e-5
    for axis in range(3):
        delta = np.eye(3)[axis] * step
        pose_jacobian[:3, axis] = rotation @ (
            cv2.Rodrigues(rvec + delta)[0] @ offset -
            cv2.Rodrigues(rvec - delta)[0] @ offset) / (2 * step)
        plus = rotation @ cv2.Rodrigues(rvec + delta)[0] @ chassis_rotation
        minus = rotation @ cv2.Rodrigues(rvec - delta)[0] @ chassis_rotation
        derivative = ((plus - minus) / (2 * step)) @ body_from_chassis.T
        pose_jacobian[3:, axis] = [derivative[2, 1], derivative[0, 2], derivative[1, 0]]
    pose_jacobian[:3, 3:] = rotation
    body_point = rotation @ camera_point
    covariance = pose_jacobian @ pose_cov @ pose_jacobian.T
    covariance[:3, :3] += calibration.mount_position_stddev**2 * np.eye(3)
    mount_jacobian = np.vstack((-skew(body_point), np.eye(3)))
    covariance += calibration.mount_angle_stddev**2 * mount_jacobian @ mount_jacobian.T
    covariance[3:, 3:] += calibration.marker_mount_angle_stddev**2 * np.eye(3)
    scale_error = rotation @ tvec * calibration.marker_size_stddev / calibration.marker_size
    covariance[:3, :3] += np.outer(scale_error, scale_error)
    covariance = (covariance + covariance.T) / 2
    if not np.isfinite(covariance).all() or np.linalg.eigvalsh(covariance).min() <= 0:
        raise ValueError('Invalid propagated covariance')
    optical_forward = optical_from_marker @ chassis_rotation[:, 0]
    axis_points = np.array([tvec, tvec + optical_forward * calibration.marker_size * 0.75])
    image_axis = cv2.projectPoints(axis_points, np.zeros(3), np.zeros(3),
                                   intrinsic, distortion)[0].reshape(2, 2)
    return MarkerPose(body_point + translation, quaternion(body_from_chassis),
                      covariance, error, image_axis)


def annotate_heading(image, observation=None):
    """Draw only a current camera measurement; the overlay is never a sensor input."""
    annotated = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    label = 'Target unavailable'
    if observation is not None:
        start, end = np.rint(observation.image_axis).astype(int)
        cv2.arrowedLine(annotated, tuple(start), tuple(end), (0, 220, 0), 3,
                       cv2.LINE_AA, tipLength=0.3)
        label = 'UGV front (camera)'
    cv2.putText(annotated, label, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 220, 0) if observation is not None else (0, 180, 255), 2, cv2.LINE_AA)
    return annotated


def detect_marker(image, marker_id=7):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    # A second threshold recovers the measured off-axis rendered border case.
    # Both attempts require the full dictionary code; partial tags stay absent.
    for threshold in (7, 15):
        modern = hasattr(cv2.aruco, 'ArucoDetector')
        params = (cv2.aruco.DetectorParameters() if modern else
                  cv2.aruco.DetectorParameters_create())
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.adaptiveThreshConstant = threshold
        if modern:
            corners, ids, _ = cv2.aruco.ArucoDetector(dictionary, params).detectMarkers(image)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(image, dictionary, parameters=params)
        if ids is None:
            continue
        matches = [corner.reshape(4, 2) for corner, tag in zip(corners, ids.ravel())
                   if tag == marker_id]
        if matches:
            # Duplicate IDs make the target identity ambiguous.
            return matches[0] if len(matches) == 1 else None
    return None
