"""Seeded open-loop maneuver schedule contracts for clean dataset runs."""

import pytest
from ugv_dataset_recorder import (MAX_ANGULAR, MAX_LINEAR, maneuver_schedule, step_toward,
                                  target_command)


def test_schedule_is_seeded_contiguous_and_starts_stationary():
    segments = maneuver_schedule(7, 120.0, 3.0)
    assert segments == maneuver_schedule(7, 120.0, 3.0)
    assert segments != maneuver_schedule(8, 120.0, 3.0)
    assert segments[0] == (0.0, 3.0, "stop", {})
    assert segments[-1][1] == pytest.approx(120.0)
    for previous, current in zip(segments, segments[1:]):
        assert current[0] == previous[1]


def test_commands_stay_within_vehicle_limits():
    for seed in range(20):
        for segment in maneuver_schedule(seed, 200.0, 3.0):
            for fraction in (0.0, 0.3, 0.7, 0.99):
                linear, angular = target_command(
                    segment, segment[0] + fraction * (segment[1] - segment[0]))
                assert abs(linear) <= MAX_LINEAR + 1e-9
                assert abs(angular) <= MAX_ANGULAR + 1e-9


def test_ramp_interpolates_and_rate_limit_bounds_steps():
    segment = (10.0, 14.0, "ramp", {"from": 0.2, "to": 0.6})
    assert target_command(segment, 12.0) == pytest.approx((0.4, 0.0))
    assert step_toward(0.0, 1.0, 0.04) == pytest.approx(0.04)
    assert step_toward(0.5, 0.48, 0.04) == pytest.approx(0.48)
