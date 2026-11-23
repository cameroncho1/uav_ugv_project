"""Dashboard helpers: world geometry, error statistics and control validation."""

import base64
import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest
from dashboard_core import (ErrorStats, block_site, classify_layers, encode_layers,
                            goal_error, point_along, position_at, position_error, reduce_layers,
                            stddev_from_covariance, validate_control, world_geometry)

WORLD = Path(__file__).resolve().parents[4] / "simulation/worlds/simple_goal_demo_complex.sdf"


def test_world_geometry_reads_obstacles_goal_and_start():
    world = world_geometry(WORLD)
    assert world["goal"] == (9.0, 3.0)
    assert world["start"] == (-4.0, 0.0)
    kinds = {o["kind"] for o in world["obstacles"]}
    assert kinds == {"box", "cylinder"}
    assert len(world["obstacles"]) == 12
    wall = next(o for o in world["obstacles"] if o["kind"] == "box" and o["half_y"] > 2.5)
    assert wall["x"] == 3.0


def test_error_statistics_ignore_missing_values():
    stats = ErrorStats()
    for time_s, error in enumerate([3.0, None, 4.0, float("nan")]):
        stats.add(time_s, error)
    assert stats.count == 2 and stats.latest == 4.0
    assert stats.rms == pytest.approx(math.sqrt(12.5))
    stats.reset()
    assert stats.rms is None and stats.latest is None


def test_position_error_and_sigma():
    assert position_error((3.0, 4.0), (0.0, 0.0)) == 5.0
    assert position_error(None, (0.0, 0.0)) is None
    covariance = [0.0] * 36
    covariance[0], covariance[7] = 0.04, 0.16
    assert stddev_from_covariance(covariance) == pytest.approx(0.4)
    covariance[1] = covariance[6] = 0.1  # indefinite covariance is not an uncertainty radius
    assert stddev_from_covariance(covariance) is None
    assert stddev_from_covariance([]) is None


def test_display_errors_match_acquisition_times_and_leave_gaps():
    truth = [(10.0, (0.0, 0.0)), (10.08, (0.08, 0.0))]
    estimate = [(10.02, (0.12, 0.0)), (10.10, (0.20, 0.0))]
    assert position_error(position_at(estimate, 10.04), position_at(truth, 10.04)) == pytest.approx(0.1)
    assert position_at(estimate, 10.01) is None  # no extrapolation before the first sample
    assert position_at(estimate, 10.11) is None  # no extrapolation after the last sample
    assert position_at([(1.0, (0.0, 0.0)), (1.2, (1.0, 0.0))], 1.1) is None


def test_goal_clearance_uses_rotated_box_geometry():
    world = dict(obstacles=[dict(kind="box", x=0.0, y=0.0,
                                half_x=2.0, half_y=0.1, yaw=math.pi / 2)])
    assert goal_error(world, 0.0, 1.5)  # inside the rotated wall
    assert goal_error(world, 1.5, 0.0) is None


@pytest.mark.parametrize("x", [0.0, 0.4, 0.9])
def test_course_rejects_obstacles_covering_or_near_uav_spawn(x):
    script = WORLD.parents[2] / "scripts/generate_obstacle_worlds.py"
    spec = importlib.util.spec_from_file_location("obstacle_generator", script)
    generator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(generator)
    problems, _, _ = generator.check_layout("complex", [("box", x, 0.0, 1.0, 1.0)])
    assert any("UAV spawn" in problem for problem in problems)


@pytest.mark.parametrize("name,value", [
    ("gps_enabled", 1), ("noise_scale", True), ("noise_scale", float("nan")),
    ("noise_scale", -0.1), ("wheel_speed_scale", 3.0), ("unknown", 1.0),
])
def test_invalid_controls_are_rejected(name, value):
    with pytest.raises(ValueError):
        validate_control(name, value)


def test_valid_controls_are_coerced():
    assert validate_control("gps_enabled", False) is False
    assert validate_control("noise_scale", 2) == 2.0
    assert validate_control("imu_yaw_bias_rate", -0.01) == -0.01


def test_goal_validation_rejects_obstacles_and_bad_values():
    world = world_geometry(WORLD)
    assert goal_error(world, 9.0, 3.0) is None
    assert goal_error(world, -4.0, 0.0) is None
    assert "obstacle" in goal_error(world, 5.2, 1.0)          # on a cylinder
    assert "obstacle" in goal_error(world, 3.0, 2.0)          # inside the wall
    assert "obstacle" in goal_error(world, 3.4, 2.0)          # 0.2 m from the wall
    assert goal_error(world, float("nan"), 0.0)
    assert goal_error(world, 0.0, True)
    assert "outside" in goal_error(world, 500.0, 0.0)


def test_layers_classify_seen_obstacle_and_halo():
    values = [-1, 0, 0, 100,
              -1, 0, 0, 0]
    safety = [-1, 0, 100, 100,
              -1, 0, 100, 100]
    classes = classify_layers(values, 4, 2, safety)
    assert classes.tolist() == [[0, 1, 3, 2], [0, 1, 3, 3]]
    assert classify_layers(values, 4, 2).tolist() == [[0, 1, 1, 2], [0, 1, 1, 1]]


def test_reduce_keeps_obstacles_over_halo_over_free():
    classes = np.array([[1, 3, 0, 0],
                        [2, 1, 1, 1],
                        [0, 0, 3, 1],
                        [0, 0, 1, 1]], np.uint8)
    assert reduce_layers(classes, 2).tolist() == [[2, 1], [0, 3]]
    odd = reduce_layers(np.ones((3, 5), np.uint8), 2)
    assert odd.shape == (2, 3) and odd[0, 0] == 1 and odd[1, 2] == 1


def test_encoded_layers_round_trip():
    classes = np.array([[0, 1], [2, 3]], np.uint8)
    encoded = encode_layers(classes, -12.0, -10.0, 0.1, factor=1)
    assert (encoded["w"], encoded["h"], encoded["res"]) == (2, 2, 0.1)
    assert base64.b64decode(encoded["data"]) == bytes([0, 1, 2, 3])


def test_halo_covers_only_ground_near_detected_obstacles():
    width = 30
    values = [100] + [0] * (width - 1)
    classes = classify_layers(values, width, 1, [100] * width, resolution=0.1, reach_m=0.35)
    assert classes[0, 0] == 2
    assert classes[0, 1:4].tolist() == [3, 3, 3]
    assert (classes[0, 4:] == 1).all()


def test_point_along_walks_the_polyline():
    path = [(0.0, 0.0), (3.0, 0.0), (3.0, 4.0)]
    assert point_along(path, 1.0) == (1.0, 0.0)
    assert point_along(path, 5.0) == (3.0, 2.0)
    assert point_along(path, 7.5) is None


def test_block_site_sits_ahead_on_the_route_with_room_to_pass():
    route = [(float(x), 0.0) for x in range(-4, 10)]
    site = block_site(route, (-3.0, 0.0), [], goal=(9.0, 3.0))
    assert site == pytest.approx((-0.8, 0.0))
    wall = dict(kind="box", x=-0.8, y=1.2, half_x=0.2, half_y=0.4)
    moved = block_site(route, (-3.0, 0.0), [wall], goal=(9.0, 3.0))
    assert moved is not None and moved[0] > -0.8              # slides on to a clear spot
    assert block_site(route, (8.0, 0.0), [], goal=(9.0, 0.0)) is None   # near the goal
    assert block_site([], (0.0, 0.0), []) is None
