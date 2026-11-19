"""Finite-state proof checks memory and rejects independent unsafe mutations."""

from pathlib import Path

import pytest

from uav_ugv_verification.supervisor import Age, Fault, Input, Mode, State, explore, policy, step

POLICY = Path(__file__).resolve().parents[1] / "supervisor/policy.json"


def test_reachable_graph_satisfies_properties_with_recovery_and_goal_memory():
    result, _ = explore(policy(POLICY))
    assert result["status"] == "PASS"
    assert result["controller_states"] > 5
    assert result["transitions"] > 160
    assert len(result["properties"]) == 11


@pytest.mark.parametrize(
    "mutant,property_name",
    [
        ("unsafe_navigation", "S1_navigation_authorization"),
        ("early_recovery", "S3_recovery_threshold"),
        ("forgot_completion", "S6_completion_memory"),
        ("spurious_hold", "S7_ready_progress"),
        ("permanent_fault", "S11_recovery_memory_integrity"),
        ("forgot_creation", "S10_completion_integrity"),
        ("spurious_completion", "S10_completion_integrity"),
        ("double_count", "S11_recovery_memory_integrity"),
        ("invented_maturity", "S11_recovery_memory_integrity"),
    ],
)
def test_independent_properties_find_shortest_mutation_counterexamples(mutant, property_name):
    result, _ = explore(policy(POLICY), mutant)
    assert result["status"] == "FAIL"
    assert property_name in result["properties_failed"]
    assert result["counterexample"]


def test_new_goal_does_not_erase_fault_recovery_or_release_early():
    rules = policy(POLICY)
    before = State(True, Mode.HOLD_FAULT, True, 9, Age.YOUNG, True)
    revised = step(before, Input(True, True, True, False, True, Fault.CLEAR), rules)
    assert revised.fault_latched and revised.clear_count == 10
    assert not revised.goal_completed
    assert revised.mode == Mode.HOLD_FAULT
    released = step(revised, Input(True, True, True, mature=True), rules)
    assert released.mode == Mode.NAVIGATE


def test_completed_goal_survives_guard_fault_and_clock_reset():
    rules = policy(POLICY)
    state = State(True, Mode.GOAL, goal_completed=True)
    for event in (
        Input(True, False),
        Input(True, True, True, fault=Fault.ALARM),
        Input(reset=True),
    ):
        state = step(state, event, rules)
        assert state.goal_completed and state.mode != Mode.NAVIGATE


def test_complete_model_check_reports_missing_cpp_without_claiming_conformance():
    from uav_ugv_verification.supervisor import check

    result = check(POLICY)
    assert result["status"] == "PASS"
    assert result["conformance"]["status"] == "NOT_RUN"
    assert len(result["implementation_sha256"]) == 64
    assert len(result["mutations"]) == 9


def test_progress_cannot_be_suppressed_by_a_spurious_post_state_fault_latch():
    from uav_ugv_verification.supervisor import violations

    rules = policy(POLICY)
    before = State(True, Mode.HOLD_FAULT, True, 10, Age.YOUNG)
    event = Input(True, True, True, mature=True)
    after = step(before, event, rules, "permanent_fault")
    assert "S7_ready_progress" in list(violations(before, event, after, False, rules))


def test_unsafe_policy_retains_a_model_counterexample_without_claiming_cpp_conformance(tmp_path):
    import json

    from uav_ugv_verification.supervisor import check

    rules = policy(POLICY)
    rules["decisions"][2]["mode"] = "NAVIGATE"
    path = tmp_path / "unsafe.json"
    path.write_text(json.dumps(rules))
    result = check(path)
    assert result["status"] == "FAIL"
    assert result["counterexample"]
    assert result["conformance"]["status"] == "NOT_RUN"
