"""Frame and uncertainty operations shared by localization adapters and tests."""

import math

import numpy as np


NED_TO_ENU = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
FRD_TO_FLU = np.diag([1., -1., -1.])


def rotation(quaternion):
    """Return the body-to-world matrix for a Hamilton quaternion (w, x, y, z)."""
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or abs(np.linalg.norm(q) - 1.) > .05:
        raise ValueError("Invalid unit quaternion")
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def quaternion(matrix):
    """Return a Hamilton quaternion from a proper rotation matrix."""
    # Select the largest component to avoid division by zero near half turns.
    m = np.asarray(matrix)
    values = [1 + np.trace(m), 1 + m[0, 0] - m[1, 1] - m[2, 2],
              1 - m[0, 0] + m[1, 1] - m[2, 2], 1 - m[0, 0] - m[1, 1] + m[2, 2]]
    i = int(np.argmax(values))
    q = np.zeros(4)
    q[i] = math.sqrt(max(0., values[i])) / 2
    d = 4*q[i]
    if i == 0:
        q[1:] = [(m[2, 1]-m[1, 2])/d, (m[0, 2]-m[2, 0])/d, (m[1, 0]-m[0, 1])/d]
    else:
        a, b, c = i-1, i % 3, (i+1) % 3
        q[0] = (m[c, b]-m[b, c])/d
        q[b+1] = (m[b, a]+m[a, b])/d
        q[c+1] = (m[c, a]+m[a, c])/d
    return q / np.linalg.norm(q)


def skew(vector):
    x, y, z = vector
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def covariance(values, size=3):
    matrix = np.asarray(values, dtype=float).reshape(size, size)
    if not np.all(np.isfinite(matrix)) or not np.allclose(matrix, matrix.T, atol=1e-8):
        raise ValueError("Nonfinite or asymmetric covariance")
    if np.linalg.eigvalsh(matrix).min() < -1e-8:
        raise ValueError("Covariance must be positive semidefinite")
    return matrix


def px4_pose(position, attitude, position_variance, attitude_variance, origin):
    """Convert NED/FRD pose and body attitude uncertainty to map ENU/FLU."""
    position = np.asarray(position, dtype=float)
    variances = np.asarray([position_variance, attitude_variance], dtype=float)
    if not np.all(np.isfinite(position)) or not np.all(np.isfinite(variances)):
        raise ValueError("Nonfinite PX4 state")
    if np.any(variances < 0):
        raise ValueError("Negative PX4 variance")
    r = NED_TO_ENU @ rotation(attitude) @ FRD_TO_FLU
    c = np.zeros((6, 6))
    c[:3, :3] = NED_TO_ENU @ np.diag(position_variance) @ NED_TO_ENU.T
    body_cov = FRD_TO_FLU @ np.diag(attitude_variance) @ FRD_TO_FLU.T
    c[3:, 3:] = r @ body_cov @ r.T
    return NED_TO_ENU @ position + origin, quaternion(r), c


def interpolate(first, second, alpha):
    """Interpolate (position, quaternion, covariance) at a measurement time."""
    p0, q0, c0 = first
    p1, q1, c1 = second
    dot = float(np.dot(q0, q1))
    if dot < 0:
        q1, dot = -q1, -dot
    if dot > .9995:
        q = (1-alpha)*q0 + alpha*q1
    else:
        angle = math.acos(min(1., dot))
        q = (math.sin((1-alpha)*angle)*q0 + math.sin(alpha*angle)*q1) / math.sin(angle)
    return (1-alpha)*p0 + alpha*p1, q/np.linalg.norm(q), (1-alpha)*c0 + alpha*c1


def ugv_position(anchor, relative, relative_covariance):
    """Propagate independent anchor and relative errors through p_g = p_a + R*r."""
    position, attitude, anchor_covariance = anchor
    r = rotation(attitude)
    offset = r @ relative
    # Small attitude errors are expressed about fixed map axes, as in ROS poses.
    jacobian = np.hstack((np.eye(3), -skew(offset)))
    result_cov = (jacobian @ covariance(anchor_covariance, 6) @ jacobian.T
                  + r @ covariance(relative_covariance) @ r.T)
    return position + offset, (result_cov + result_cov.T) / 2


def ugv_pose(anchor, relative_position, relative_attitude, relative_covariance):
    """Compose independent poses with fixed-axis rotation errors in their parent frames.

    The full covariance retains position/rotation correlations from both sources.
    Relative attitude rotates chassis axes into the UAV body frame.
    """
    position, attitude, anchor_covariance = anchor
    r = rotation(attitude)
    offset = r @ np.asarray(relative_position)
    anchor_jacobian = np.eye(6)
    anchor_jacobian[:3, 3:] = -skew(offset)
    relative_jacobian = np.zeros((6, 6))
    relative_jacobian[:3, :3] = relative_jacobian[3:, 3:] = r
    result_cov = (anchor_jacobian @ covariance(anchor_covariance, 6) @ anchor_jacobian.T
                  + relative_jacobian @ covariance(relative_covariance, 6)
                  @ relative_jacobian.T)
    return (position + offset, quaternion(r @ rotation(relative_attitude)),
            (result_cov + result_cov.T) / 2)


def planar_heading_pose(attitude, pose_covariance, min_horizontal=0.5):
    """Return yaw-only attitude and covariance retaining position/yaw correlations.

    Heading is the map-plane direction of chassis +X, not direction of travel.
    Reject near-vertical forward axes where this direction is poorly defined.
    """
    forward = rotation(attitude)[:, 0]
    horizontal_squared = float(forward[:2] @ forward[:2])
    if horizontal_squared < min_horizontal**2:
        raise ValueError("Chassis forward axis has no reliable planar heading")
    yaw = math.atan2(forward[1], forward[0])
    jacobian = np.zeros((6, 6))
    jacobian[:3, :3] = np.eye(3)
    jacobian[5, 3:] = [-forward[0]*forward[2]/horizontal_squared,
                       -forward[1]*forward[2]/horizontal_squared, 1.]
    result = jacobian @ covariance(pose_covariance, 6) @ jacobian.T
    result[3, 3] = result[4, 4] = 1e6
    return np.array([math.cos(yaw/2), 0., 0., math.sin(yaw/2)]), (result + result.T)/2
