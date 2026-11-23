"""Dashboard helpers without ROS or Gazebo imports: world geometry, error statistics, controls."""

import base64
import math
import xml.etree.ElementTree as ET
from collections import deque

import numpy as np


def _pose(element, default=(0.0, 0.0, 0.0)):
    text = element.findtext("pose") if element is not None else None
    values = [float(v) for v in text.split()] if text else []
    values += [0.0] * (6 - len(values))
    return values[0] or default[0], values[1] or default[1], values[5]


def world_geometry(path):
    """Static obstacles, goal marker and UGV spawn from a goal-demo world file."""
    root = ET.parse(path).getroot()
    world = root.find("world")
    obstacles, goal, start = [], None, None
    for model in world.iter("model"):
        name = model.get("name", "")
        x, y, yaw = _pose(model)
        if name.startswith("obstacle_"):
            geometry = model.find(".//collision/geometry")
            box = geometry.find("box/size") if geometry is not None else None
            cylinder = geometry.find("cylinder/radius") if geometry is not None else None
            if box is not None:
                sx, sy = (float(v) for v in box.text.split()[:2])
                obstacles.append(dict(kind="box", x=x, y=y, half_x=sx / 2, half_y=sy / 2, yaw=yaw))
            elif cylinder is not None:
                obstacles.append(dict(kind="cylinder", x=x, y=y, radius=float(cylinder.text)))
        elif name == "global_goal_marker":
            goal = (x, y)
    for include in world.iter("include"):
        if "simple_ugv" in (include.findtext("uri") or ""):
            start = _pose(include)[:2]
    return dict(obstacles=obstacles, goal=goal, start=start)


def position_error(estimate, truth):
    """Euclidean error between two (x, y) points, or None if either is missing."""
    if estimate is None or truth is None:
        return None
    return math.hypot(estimate[0] - truth[0], estimate[1] - truth[1])


def stddev_from_covariance(covariance):
    """Horizontal one-sigma radius from a 6x6 row-major pose covariance."""
    if covariance is None or len(covariance) < 8:
        return None
    variance = 0.5 * (covariance[0] + covariance[7])
    return math.sqrt(variance) if variance >= 0.0 and math.isfinite(variance) else None


class ErrorStats:
    """Rolling error history and running RMS for one estimator."""

    def __init__(self, window=900):
        self.history = deque(maxlen=window)
        self.sum_squares = 0.0
        self.count = 0
        self.maximum = 0.0

    def add(self, time_s, error):
        if error is None or not math.isfinite(error):
            return
        self.history.append((time_s, error))
        self.sum_squares += error * error
        self.count += 1
        self.maximum = max(self.maximum, error)

    @property
    def rms(self):
        return math.sqrt(self.sum_squares / self.count) if self.count else None

    @property
    def latest(self):
        return self.history[-1][1] if self.history else None

    def reset(self):
        self.history.clear()
        self.sum_squares = 0.0
        self.count = 0
        self.maximum = 0.0


# name -> (kind, bounds) for every control the page may set
CONTROLS = {
    "gps_enabled": ("bool", None),
    "uav_assistance": ("bool", None),
    "noise_scale": ("float", (0.0, 10.0)),
    "gps_horizontal_stddev": ("float", (0.0, 20.0)),
    "wheel_speed_scale": ("float", (0.5, 1.5)),
    "imu_yaw_bias_rate": ("float", (-0.05, 0.05)),
}


def validate_control(name, value):
    """Return the value coerced to its type, or raise ValueError."""
    if name not in CONTROLS:
        raise ValueError(f"unknown control {name!r}")
    kind, bounds = CONTROLS[name]
    if kind == "bool":
        if not isinstance(value, bool):
            raise ValueError(f"{name} must be true or false")
        return value
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    if not bounds[0] <= value <= bounds[1]:
        raise ValueError(f"{name} must be within {bounds[0]} and {bounds[1]}")
    return float(value)


def point_gap(obstacle, x, y):
    """Distance from (x, y) to the edge of an obstacle (boxes are axis-aligned)."""
    if obstacle["kind"] == "cylinder":
        return max(math.hypot(x - obstacle["x"], y - obstacle["y"]) - obstacle["radius"], 0.0)
    return math.hypot(max(abs(x - obstacle["x"]) - obstacle["half_x"], 0.0),
                      max(abs(y - obstacle["y"]) - obstacle["half_y"], 0.0))


GOAL_LIMIT_M = 50.0
GOAL_OBSTACLE_MARGIN_M = 0.8  # footprint radius 0.5 m plus clearance and grid cell


def goal_error(world, x, y):
    """Return why (x, y) is not an acceptable goal, or None."""
    if (isinstance(x, bool) or isinstance(y, bool)
            or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (x, y))):
        return "goal must be finite numbers"
    if abs(x) > GOAL_LIMIT_M or abs(y) > GOAL_LIMIT_M:
        return "goal is outside the map"
    if any(point_gap(o, x, y) < GOAL_OBSTACLE_MARGIN_M for o in world["obstacles"]):
        return "goal is too close to an obstacle"
    return None


UNSEEN, FREE, OBSTACLE, HALO = 0, 1, 2, 3


def classify_layers(values, width, height, safety=None, resolution=0.1, reach_m=0.65):
    """Classes for the dashboard map: 0 unseen, 1 seen free, 2 obstacle, 3 inflated halo.

    `values` is a row-major occupancy grid (-1 unseen, 0 free, 100 occupied). `safety` is
    the planner's inflated grid on the same cells. The halo is its occupied cells within
    `reach_m` of a detected obstacle: the planner also pads unseen ground and the map
    edge, which is not drawn.
    """
    grid = np.asarray(values, dtype=np.int16).reshape(height, width)
    classes = np.zeros(grid.shape, np.uint8)
    classes[grid >= 0] = FREE
    classes[grid >= 50] = OBSTACLE
    if safety is not None:
        inflated = np.asarray(safety, dtype=np.int16).reshape(height, width) >= 50
        obstacle = classes == OBSTACLE
        near = np.zeros_like(obstacle)
        reach = int(math.ceil(reach_m / resolution))
        for dy in range(-reach, reach + 1):
            for dx in range(-reach, reach + 1):
                if (math.hypot(dx, dy) * resolution > reach_m
                        or abs(dy) >= height or abs(dx) >= width):
                    continue
                shifted = np.zeros_like(obstacle)
                shifted[max(dy, 0):height + min(dy, 0), max(dx, 0):width + min(dx, 0)] =                     obstacle[max(-dy, 0):height + min(-dy, 0), max(-dx, 0):width + min(-dx, 0)]
                near |= shifted
        classes[inflated & near & ~obstacle] = HALO
    return classes


def reduce_layers(classes, factor=2):
    """Shrink by `factor`; a block takes its most significant class (obstacle over halo)."""
    height, width = classes.shape
    padded = np.zeros((-(-height // factor) * factor, -(-width // factor) * factor), np.uint8)
    padded[:height, :width] = classes
    ranked = np.array([0, 1, 3, 2], np.uint8)[padded]  # obstacle outranks halo outranks free
    blocks = ranked.reshape(padded.shape[0] // factor, factor,
                            padded.shape[1] // factor, factor).max(axis=(1, 3))
    return np.array([0, 1, 3, 2], np.uint8)[blocks]  # the ranking permutation is its own inverse


def encode_layers(classes, origin_x, origin_y, resolution, factor=2):
    reduced = reduce_layers(classes, factor)
    return dict(ox=origin_x, oy=origin_y, res=resolution * factor,
                w=int(reduced.shape[1]), h=int(reduced.shape[0]),
                data=base64.b64encode(reduced.tobytes()).decode("ascii"))


def point_along(path, distance):
    """Point `distance` metres along a polyline, or None past its end."""
    for (ax, ay), (bx, by) in zip(path, path[1:]):
        length = math.hypot(bx - ax, by - ay)
        if distance <= length:
            fraction = distance / length if length else 0.0
            return ax + fraction * (bx - ax), ay + fraction * (by - ay)
        distance -= length
    return None


def block_site(route, here, obstacles, goal=None, ahead_m=2.2, half_m=0.3,
               clearance_m=1.2, goal_margin_m=1.5):
    """Where to drop a block on the vehicle's route, or None if there is no room.

    Tries `ahead_m` along the route from the vehicle and then further out. A site needs
    `clearance_m` between the block and every obstacle (so a route can still pass around
    it) and `goal_margin_m` to the goal.
    """
    if here is None or len(route) < 2:
        return None
    nearest = min(range(len(route)),
                  key=lambda i: math.hypot(route[i][0] - here[0], route[i][1] - here[1]))
    path = [tuple(here)] + [tuple(p) for p in route[nearest + 1:]]
    for attempt in range(6):
        site = point_along(path, ahead_m + 0.6 * attempt)
        if site is None:
            return None
        if goal is not None and math.hypot(site[0] - goal[0], site[1] - goal[1]) < goal_margin_m:
            return None
        # The block's farthest corner is half_m * sqrt(2) from its centre.
        if all(point_gap(o, *site) - half_m * math.sqrt(2) >= clearance_m for o in obstacles):
            return site
    return None
