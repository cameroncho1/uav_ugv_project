"""Planar proxy geometry; these margins do not assert physical contact."""

from __future__ import annotations

import math

import numpy as np


def clearance(point: tuple[float, float], shape: dict, radius: float) -> float:
    dx, dy = point[0] - shape["x"], point[1] - shape["y"]
    if shape["kind"] == "cylinder":
        return math.hypot(dx, dy) - shape["radius"] - radius
    yaw = shape.get("yaw", 0)
    x, y = math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy
    qx, qy = abs(x) - shape["half_x"], abs(y) - shape["half_y"]
    return math.hypot(max(qx, 0), max(qy, 0)) + min(max(qx, qy), 0) - radius


def _segment_distance(a, b, center) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    t = (
        max(0.0, min(1.0, ((center[0] - a[0]) * dx + (center[1] - a[1]) * dy) / length2))
        if length2
        else 0.0
    )
    return math.hypot(a[0] + t * dx - center[0], a[1] + t * dy - center[1])


def _segments_intersect(a, b, c, d) -> bool:
    def orientation(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    o1, o2, o3, o4 = (
        orientation(a, b, c),
        orientation(a, b, d),
        orientation(c, d, a),
        orientation(c, d, b),
    )
    if o1 * o2 < 0 and o3 * o4 < 0:
        return True
    for value, p, q, r in ((o1, a, b, c), (o2, a, b, d), (o3, c, d, a), (o4, c, d, b)):
        if (
            value == 0
            and min(p[0], q[0]) <= r[0] <= max(p[0], q[0])
            and min(p[1], q[1]) <= r[1] <= max(p[1], q[1])
        ):
            return True
    return False


def segment_clearance(a, b, shape: dict, radius: float) -> float:
    if shape["kind"] == "cylinder":
        return _segment_distance(a, b, (shape["x"], shape["y"])) - shape["radius"] - radius
    if min(clearance(a, shape, 0), clearance(b, shape, 0)) <= 0:
        return -radius
    yaw = shape.get("yaw", 0)
    corners = [
        (
            shape["x"] + math.cos(yaw) * x - math.sin(yaw) * y,
            shape["y"] + math.sin(yaw) * x + math.cos(yaw) * y,
        )
        for x, y in (
            (-shape["half_x"], -shape["half_y"]),
            (shape["half_x"], -shape["half_y"]),
            (shape["half_x"], shape["half_y"]),
            (-shape["half_x"], shape["half_y"]),
        )
    ]
    minimum = math.inf
    for c, d in zip(corners, corners[1:] + corners[:1], strict=True):
        if _segments_intersect(a, b, c, d):
            return -radius
        minimum = min(
            minimum,
            _segment_distance(a, b, c),
            _segment_distance(a, b, d),
            _segment_distance(c, d, a),
            _segment_distance(c, d, b),
        )
    return minimum - radius


def position_stddev(covariance: list[list[float]]) -> float:
    matrix = np.asarray(covariance, dtype=float)
    if (
        matrix.shape != (3, 3)
        or not np.isfinite(matrix).all()
        or not np.allclose(matrix, matrix.T, atol=1e-8, rtol=0)
        or np.linalg.eigvalsh(matrix).min() <= 0
    ):
        raise ValueError("x/y/yaw covariance must be symmetric positive definite")
    return float(np.sqrt(np.linalg.eigvalsh(matrix[:2, :2]).max()))
