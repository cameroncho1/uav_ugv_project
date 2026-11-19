import pytest
from conftest import NS, command, event, guard, result, sample

from uav_ugv_verification import evaluate


@pytest.mark.parametrize("stop,expected", [(1_500_000_000, "PASS"), (1_500_000_001, "FAIL")])
def test_r3_exact_deadline(run_factory, profile_factory, stop, expected):
    run = run_factory(
        {
            "guard_state": [guard(0, True), guard(NS, False), guard(2 * NS, True)],
            "requested_command": [command(0, 0.5), command(stop)],
        }
    )
    checked = result(evaluate(run, profile_factory()), "R3")
    assert checked.verdict == expected
    assert checked.trigger_count == 1


@pytest.mark.parametrize(
    "commands,end,expected",
    [
        ([command(0, 0.5)], 1_400_000_000, "INCONCLUSIVE"),
        ([command(0, 0.5), command(1_100_000_000), command(1_300_000_000, 0.2)], 3 * NS, "FAIL"),
        ([command(0)], 3 * NS, "PASS"),
    ],
)
def test_r3_partial_trace_hold_and_already_quiet(
    run_factory, profile_factory, commands, end, expected
):
    states = [guard(0, True), guard(NS, False)]
    if end >= 2 * NS:
        states.append(guard(2 * NS, True))
    run = run_factory({"guard_state": states, "requested_command": commands}, end=end)
    assert result(evaluate(run, profile_factory()), "R3").verdict == expected


def test_r3_repeated_invalid_samples_never_reset_deadline(run_factory, profile_factory):
    run = run_factory(
        {
            "guard_state": [
                guard(0, True),
                guard(NS, False),
                guard(1_200_000_000, False),
                guard(1_400_000_000, False),
            ],
            "requested_command": [command(0, 0.5), command(1_600_000_000)],
        }
    )
    assert result(evaluate(run, profile_factory()), "R3").verdict == "FAIL"


def test_r3_no_trigger_and_command_gap_are_inconclusive(run_factory, profile_factory):
    run = run_factory({"guard_state": [guard(0, True)], "requested_command": [command(0)]})
    assert result(evaluate(run, profile_factory()), "R3").verdict == "INCONCLUSIVE"
    run = run_factory(
        {
            "guard_state": [guard(0, False), guard(NS, False), guard(2 * NS, True)],
            "requested_command": [command(0), command(2 * NS)],
        }
    )
    assert result(evaluate(run, profile_factory(gap=NS)), "R3").verdict == "INCONCLUSIVE"


def test_r3_recovery_does_not_cancel_pending_response(run_factory, profile_factory):
    run = run_factory(
        {
            "guard_state": [guard(0, True), guard(NS, False), guard(1_100_000_000, True)],
            "requested_command": [command(0, 0.5)],
        }
    )
    assert result(evaluate(run, profile_factory()), "R3").verdict == "FAIL"


@pytest.mark.parametrize(
    "stop,advancing,uncertainty,expected",
    [
        (300_000_000, True, 0, "PASS"),
        (500_000_000, True, 0, "FAIL"),
        (300_000_000, False, 0, "INCONCLUSIVE"),
        (500_000_000, True, 100_000_000, "FAIL"),
    ],
)
def test_r4_expiry_and_settling(
    run_factory, profile_factory, stop, advancing, uncertainty, expected
):
    run = run_factory(
        {
            "events": [event(100_000_000, "command_loss")],
            "actuator_input": [command(0, 0.5, accepted=True)],
            "actuator_output": [command(0, 0.5), command(stop)],
            "measured_motion": [
                command(0, 0.5, simulation_advancing=advancing),
                command(stop, simulation_advancing=advancing, uncertainty_ns=uncertainty),
            ],
        },
        clock="steady:watchdog",
        end=NS,
    )
    checked = result(evaluate(run, profile_factory("R4", clock="steady:watchdog")), "R4")
    assert checked.verdict == expected


def test_r4_missing_physical_horizon_keeps_expiry_metric(run_factory, profile_factory):
    run = run_factory(
        {
            "events": [event(100_000_000, "command_loss")],
            "actuator_input": [command(0, 0.5, accepted=True)],
            "actuator_output": [command(0, 0.5), command(300_000_000)],
            "measured_motion": [command(0, 0.5, simulation_advancing=True)],
        },
        end=400_000_000,
    )
    checked = result(evaluate(run, profile_factory("R4")), "R4")
    assert checked.verdict == "INCONCLUSIVE"
    assert checked.metrics["by_interval"][0]["actuator_output_maximum_latency_ns"] == 300_000_000


def hazard_run(run_factory, *, revision="a", route_time=1_100_000_000, stop=False):
    return run_factory(
        {
            "events": [
                event(
                    NS,
                    "hazard_observed",
                    "hazard",
                    goal_revision="a",
                    qualifying=True,
                    obstacle_id="box",
                )
            ],
            "obstacles": [
                sample(
                    0,
                    static=True,
                    shapes=[
                        {"id": "box", "kind": "box", "x": 0, "y": 0, "half_x": 0.5, "half_y": 0.5}
                    ],
                )
            ],
            "route": [
                sample(0, goal_revision="old", points=[[-2, 0], [2, 0]]),
                sample(route_time, goal_revision=revision, points=[[-2, 2], [2, 2]]),
            ],
            "requested_command": [command(0, 0 if stop else 0.5)],
        },
        end=4 * NS,
    )


@pytest.mark.parametrize("revision,expected", [("a", "PASS"), ("other", "FAIL")])
def test_r5_route_must_match_goal_and_clear_geometry(
    run_factory, profile_factory, revision, expected
):
    checked = result(
        evaluate(hazard_run(run_factory, revision=revision), profile_factory("R5")), "R5"
    )
    assert checked.verdict == expected


def test_r5_equal_time_cross_source_route_is_ambiguous(run_factory, profile_factory):
    checked = result(evaluate(hazard_run(run_factory, route_time=NS), profile_factory("R5")), "R5")
    assert checked.verdict == "INCONCLUSIVE"


def test_r5_already_stopped_satisfies_dwell(run_factory, profile_factory):
    checked = result(
        evaluate(hazard_run(run_factory, revision="other", stop=True), profile_factory("R5")), "R5"
    )
    assert checked.verdict == "PASS"


def test_r3_preserves_intermediate_same_time_command_violation(run_factory, profile_factory):
    run = run_factory(
        {
            "guard_state": [guard(0, False), guard(2 * NS, True)],
            "requested_command": [command(0, 0.5), command(NS), command(NS, 0.5), command(NS)],
        }
    )
    checked = result(evaluate(run, profile_factory(parameters={"response_ns": 2 * NS})), "R3")
    assert checked.verdict == "FAIL"


def test_r3_unknown_guard_recovery_cannot_establish_hold_violation(run_factory, profile_factory):
    run = run_factory(
        {
            "guard_state": [guard(0, False), guard(2 * NS, True)],
            "requested_command": [command(0), command(1_500_000_000, 0.5)],
        }
    )
    checked = result(evaluate(run, profile_factory(gap=NS)), "R3")
    assert checked.verdict == "INCONCLUSIVE"


@pytest.mark.parametrize("uncertainty,expected", [(0, "PASS"), (250_000_000, "INCONCLUSIVE")])
def test_r4_stop_dwell_may_confirm_after_stop_deadline(
    run_factory, profile_factory, uncertainty, expected
):
    run = run_factory(
        {
            "events": [event(100_000_000, "command_loss")],
            "actuator_input": [command(0, 0.5, accepted=True)],
            "actuator_output": [command(0, 0.5), command(300_000_000)],
            "measured_motion": [
                command(0, 0.5, simulation_advancing=True),
                command(600_000_000, simulation_advancing=True, uncertainty_ns=uncertainty),
            ],
        },
        end=NS,
    )
    checked = result(evaluate(run, profile_factory("R4")), "R4")
    assert checked.verdict == expected


def test_r4_lost_input_events_cannot_be_reported_as_watchdog_failure(run_factory, profile_factory):
    run = run_factory(
        {
            "events": [event(100_000_000, "command_loss")],
            "actuator_input": [
                command(0, 0.5, accepted=True),
                {**command(800_000_000, 0.5, accepted=False), "sequence": 2},
            ],
            "actuator_output": [command(0, 0.5)],
            "measured_motion": [command(0, 0.5, simulation_advancing=True)],
        },
        end=NS,
    )
    assert result(evaluate(run, profile_factory("R4")), "R4").verdict == "INCONCLUSIVE"
