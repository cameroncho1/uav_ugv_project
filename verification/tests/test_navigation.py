import math

import pytest
from conftest import NS, command, guard, result, sample

from uav_ugv_verification import evaluate
from uav_ugv_verification.signals.geometry import clearance, position_stddev, segment_clearance


@pytest.mark.parametrize("point,expected", [((2, 0), 0.5), ((1.5, 0), 0), ((0, 0), -1.5)])
def test_box_reference_distances(point, expected):
    assert (
        clearance(point, {"kind": "box", "x": 0, "y": 0, "half_x": 1, "half_y": 1}, 0.5) == expected
    )


def test_cylinder_and_rigid_transform_invariance():
    assert clearance(
        (1, 0), {"kind": "cylinder", "x": 0, "y": 0, "radius": 0.4}, 0.5
    ) == pytest.approx(0.1)
    angle = 0.71
    shape = {"kind": "box", "x": 3, "y": -2, "half_x": 1, "half_y": 1, "yaw": angle}
    point = (3 + 2 * math.cos(angle), -2 + 2 * math.sin(angle))
    assert clearance(point, shape, 0.5) == pytest.approx(0.5)


def test_segment_checks_interior_and_rounded_corners():
    box = {"kind": "box", "x": 0, "y": 0, "half_x": 1, "half_y": 1}
    assert segment_clearance((-2, 0), (2, 0), box, 0.5) < 0
    assert segment_clearance((-2, 2), (2, 2), box, 0.5) == 0.5
    assert segment_clearance((1.4, 1.4), (2, 2), box, 0.5) > 0  # Rounded footprint, not a square.


def test_position_uncertainty_uses_correlations():
    assert position_stddev([[0.2, 0.15, 0], [0.15, 0.2, 0], [0, 0, 0.01]]) == pytest.approx(
        math.sqrt(0.35)
    )
    with pytest.raises(ValueError):
        position_stddev([[0.2, 0.3, 0], [0.3, 0.2, 0], [0, 0, 0.01]])


@pytest.mark.parametrize("x,verdict", [(2, "PASS"), (1.4, "FAIL")])
def test_r1_proxy(run_factory, profile_factory, x, verdict):
    run = run_factory(
        {
            "truth_pose": [sample(0, x=x, y=0), sample(3 * NS, x=x, y=0)],
            "obstacles": [
                sample(
                    0,
                    static=True,
                    shapes=[{"kind": "box", "x": 0, "y": 0, "half_x": 1, "half_y": 1}],
                )
            ],
        }
    )
    checked = result(evaluate(run, profile_factory("R1")), "R1")
    assert checked.verdict == verdict
    assert checked.metrics["by_interval"][0]["scope"] == "sampled_disk_proxy"


def test_r1_empty_geometry_is_different_from_missing_geometry(run_factory, profile_factory):
    profile = profile_factory("R1")
    truth = [sample(0, x=0, y=0)]
    empty = run_factory({"truth_pose": truth, "obstacles": [sample(0, static=True, shapes=[])]})
    missing = run_factory({"truth_pose": truth})
    assert result(evaluate(empty, profile), "R1").verdict == "PASS"
    assert result(evaluate(missing, profile), "R1").verdict == "INCONCLUSIVE"


@pytest.mark.parametrize(
    "truth_x,end,expected",
    [(0.2, 3 * NS, "PASS"), (2, 3 * NS, "FAIL"), (0.2, NS // 2, "INCONCLUSIVE")],
)
def test_r2_arrival_and_horizon(run_factory, profile_factory, truth_x, end, expected):
    run = run_factory(
        {
            "truth_pose": [sample(0, x=truth_x, y=0)],
            "estimated_pose": [sample(0, x=0.1, y=0)],
            "goal": [sample(0, x=0, y=0, revision="a")],
            "requested_command": [command(0)],
        },
        end=end,
    )
    profile = profile_factory("R2", parameters={"deadline_ns": 2 * NS})
    assert result(evaluate(run, profile), "R2").verdict == expected


def test_r2_dwell_does_not_cross_motion_or_goal_revisions(run_factory, profile_factory):
    run = run_factory(
        {
            "truth_pose": [sample(0, x=0, y=0)],
            "estimated_pose": [sample(0, x=0, y=0)],
            "goal": [sample(0, x=0, y=0, revision="a"), sample(NS // 2, x=0, y=0, revision="b")],
            "requested_command": [command(0), command(NS, 0.5), command(2 * NS)],
        }
    )
    checked = result(evaluate(run, profile_factory("R2", parameters={"deadline_ns": 2 * NS})), "R2")
    assert checked.verdict != "PASS"  # First revision never completes its dwell.
    assert checked.trigger_count == 2


@pytest.mark.parametrize("error,expected", [(0.5, "PASS"), (0.500001, "FAIL")])
def test_r6_error_threshold_and_truth_interpolation(run_factory, profile_factory, error, expected):
    run = run_factory(
        {
            "truth_pose": [sample(0, x=0, y=0), sample(2 * NS, x=2, y=0)],
            "estimated_pose": [sample(NS, x=1 + error, y=0)],
            "guard_state": [guard(0, True)],
            "uav_observation": [sample(NS)],
        },
        end=2 * NS,
    )
    assert result(evaluate(run, profile_factory("R6")), "R6").verdict == expected


def test_r6_truth_gap_and_ineligible_coverage(run_factory, profile_factory):
    run = run_factory(
        {
            "truth_pose": [sample(0, x=0, y=0), sample(3 * NS, x=0, y=0)],
            "estimated_pose": [sample(NS, x=0, y=0)],
            "guard_state": [guard(0, True)],
            "uav_observation": [sample(NS)],
        }
    )
    checked = result(evaluate(run, profile_factory("R6", gap=NS)), "R6")
    assert checked.verdict == "INCONCLUSIVE"
    assert checked.metrics["by_interval"][0]["conditional_rmse_m"] is None


def test_r6_no_fresh_uav_is_unexercised(run_factory, profile_factory):
    run = run_factory(
        {
            "truth_pose": [sample(0, x=0, y=0), sample(3 * NS, x=0, y=0)],
            "estimated_pose": [sample(NS, x=0, y=0)],
            "guard_state": [guard(0, True)],
            "uav_observation": [],
        }
    )
    checked = result(evaluate(run, profile_factory("R6")), "R6")
    assert checked.verdict == "INCONCLUSIVE"
    assert checked.trigger_count == 0


def test_r6_equal_time_guard_change_cannot_hide_a_possible_violation(run_factory, profile_factory):
    run = run_factory(
        {
            "truth_pose": [sample(0, x=0, y=0), sample(3 * NS, x=0, y=0)],
            "estimated_pose": [sample(NS, x=1, y=0)],
            "guard_state": [guard(0, True), guard(NS, False)],
            "uav_observation": [sample(NS)],
        }
    )
    assert result(evaluate(run, profile_factory("R6")), "R6").verdict == "INCONCLUSIVE"
