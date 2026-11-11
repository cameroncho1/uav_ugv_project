import pytest
from conftest import NS, detector, fault, result

from uav_ugv_verification import evaluate
from uav_ugv_verification.reporting.summary import proportion


@pytest.mark.parametrize(
    "alarm_at,end,expected",
    [(2 * NS, 3 * NS, "PASS"), (None, 3 * NS, "FAIL"), (None, 1_500_000_000, "INCONCLUSIVE")],
)
def test_r7_detection_miss_and_truncated_horizon(
    run_factory, profile_factory, alarm_at, end, expected
):
    decisions = [detector(0)]
    if alarm_at:
        decisions.append(detector(alarm_at, True))
    run = run_factory({"events": [fault(NS)], "detector_decision": decisions}, end=end)
    profile = profile_factory("R7", parameters={"alarm_scope": "binary", "response_ns": NS})
    assert result(evaluate(run, profile), "R7").verdict == expected


def test_r7_preexisting_alarm_is_not_new_detection(run_factory, profile_factory):
    run = run_factory(
        {"events": [fault(NS)], "detector_decision": [detector(0, True), detector(2 * NS, True)]}
    )
    checked = result(
        evaluate(
            run, profile_factory("R7", parameters={"alarm_scope": "binary", "response_ns": NS})
        ),
        "R7",
    )
    assert checked.verdict == "FAIL"
    assert checked.metrics["by_interval"][0]["pre_existing_alarms"] == 1


def test_r7_binary_output_cannot_establish_class_detection(run_factory, profile_factory):
    run = run_factory(
        {"events": [fault(NS)], "detector_decision": [detector(0), detector(2 * NS, True)]}
    )
    assert result(evaluate(run, profile_factory("R7")), "R7").verdict == "INCONCLUSIVE"


@pytest.mark.parametrize("alarm,expected", [(False, "PASS"), (True, "FAIL")])
def test_r8_nominal_alarm(run_factory, profile_factory, alarm, expected):
    run = run_factory({"events": [], "detector_decision": [detector(0), detector(NS, alarm)]})
    checked = result(evaluate(run, profile_factory("R8")), "R8")
    assert checked.verdict == expected
    metrics = checked.metrics["by_interval"][0]
    assert metrics["false_alarm_events"] == int(alarm)
    assert metrics["nominal_exposure_ns"] == 3 * NS


def test_r8_fault_and_recovery_mask_is_not_nominal_exposure(run_factory, profile_factory):
    run = run_factory(
        {
            "events": [fault(NS), fault(2 * NS, "fault_end")],
            "detector_decision": [detector(0), detector(NS, True), detector(3 * NS)],
        },
        end=4 * NS,
    )
    checked = result(evaluate(run, profile_factory("R8", parameters={"recovery_ns": NS})), "R8")
    assert checked.verdict == "PASS"
    metrics = checked.metrics["by_interval"][0]
    assert metrics["nominal_exposure_ns"] == 2 * NS
    assert metrics["fault_free_run"] is False


def test_r8_required_channel_cannot_be_cleared_by_another_channel(run_factory, profile_factory):
    channel = {"sensor": "imu", "channel": "wz"}
    first = detector(0, True, **channel)
    first["data"]["scope"] = "channel"
    other = detector(NS, False, sensor="wheel", channel="vx")
    other["data"]["scope"] = "channel"
    run = run_factory({"events": [], "detector_decision": [first, other]})
    checked = result(
        evaluate(
            run,
            profile_factory(
                "R8", parameters={"decision_keys": ["channel:imu:wz", "channel:wheel:vx"]}
            ),
        ),
        "R8",
    )
    assert checked.verdict == "FAIL"
    assert checked.missing_data_count > 0


def test_exact_binomial_interval_has_correct_extremes():
    zero = proportion(0, 3)
    assert zero["ci95_two_sided"][0] == 0
    assert zero["ci95_two_sided"][1] == pytest.approx(0.707598226, rel=1e-6)
    assert proportion(3, 3)["ci95_two_sided"][1] == 1
    assert proportion(0, 0)["estimate"] is None


def test_r7_cross_source_alarm_onset_tie_is_ambiguous(run_factory, profile_factory):
    run = run_factory(
        {"events": [fault(NS)], "detector_decision": [detector(0), detector(NS, True)]}
    )
    profile = profile_factory("R7", parameters={"alarm_scope": "binary", "response_ns": NS})
    assert result(evaluate(run, profile), "R7").verdict == "INCONCLUSIVE"


def test_r8_lost_fault_events_do_not_manufacture_a_false_alarm(run_factory, profile_factory):
    end = fault(2 * NS, "fault_end")
    end["sequence"] = 2
    run = run_factory(
        {"events": [fault(NS), end], "detector_decision": [detector(0), detector(NS, True)]}
    )
    assert result(evaluate(run, profile_factory("R8")), "R8").verdict == "INCONCLUSIVE"


def test_r8_zero_nominal_exposure_is_not_a_pass(run_factory, profile_factory):
    run = run_factory({"events": [], "detector_decision": [detector(0)]})
    checked = result(evaluate(run, profile_factory("R8", parameters={"warmup_ns": 3 * NS})), "R8")
    assert checked.verdict == "INCONCLUSIVE"
    assert checked.eligible_duration_ns == 0
