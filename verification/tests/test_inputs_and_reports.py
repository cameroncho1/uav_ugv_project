import csv
import json
import subprocess
import sys
from pathlib import Path

import pytest
import yaml
from conftest import NS, command, detector, guard, result, sample

from uav_ugv_verification import (
    ValidationError,
    evaluate,
    evaluate_trace,
    inspect_run,
    load_profile,
)
from uav_ugv_verification.__main__ import main
from uav_ugv_verification.adapters import open_run
from uav_ugv_verification.reporting import report, write_evaluation


def test_empty_and_excluded_requirements_never_succeed(run_factory, profile_factory):
    run = run_factory({})
    assert evaluate(run, profile_factory(rules={})).reason == "no_applicable_requirements"
    rules = {
        "R3": {"applicable": False, "mandatory": True, "reason": "No controller in this setup"}
    }
    checked = evaluate(run, profile_factory(rules=rules))
    assert checked.exit_code == 2
    assert result(checked, "R3").verdict == "NOT_APPLICABLE"


def test_mandatory_failure_precedes_missing_optional_or_aborted_status(
    run_factory, profile_factory
):
    run = run_factory(
        {"guard_state": [guard(0, False)], "requested_command": [command(0, 0.5)]}, status="aborted"
    )
    rules = {
        "R3": {"applicable": True, "mandatory": True},
        "R1": {"applicable": True, "mandatory": False},
    }
    checked = evaluate(run, profile_factory(rules=rules))
    assert checked.exit_code == 1
    assert checked.execution_status == "aborted"
    assert result(checked, "R1").verdict == "INCONCLUSIVE"


def test_inspection_reports_missing_capabilities(run_factory, profile_factory):
    run = run_factory({"requested_command": [command(0)]})
    info = inspect_run(run, profile_factory())
    assert info["requirements"]["R3"]["missing_streams"] == ["guard_state"]
    assert len(info["source_sha256"]["requested_command.jsonl"]) == 64


def test_public_trace_reader_api_matches_file_evaluation(run_factory, profile_factory):
    run = run_factory({"guard_state": [guard(0, False)], "requested_command": [command(0)]})
    profile = load_profile(profile_factory())
    assert evaluate_trace(open_run(run), profile).to_dict() == evaluate(run, profile).to_dict()


@pytest.mark.parametrize(
    "mutation",
    ["unknown", "nan", "bool_time", "wrong_frame", "decreasing_time", "duplicate_sequence"],
)
def test_malformed_recordings_are_rejected(run_factory, profile_factory, mutation):
    rows = [guard(0, False), guard(NS, False)]
    if mutation == "unknown":
        rows[0]["typo"] = 1
    elif mutation == "nan":
        rows[0]["data"]["extra"] = float("nan")
    elif mutation == "bool_time":
        rows[0]["time_ns"] = True
    elif mutation == "wrong_frame":
        run = run_factory({"truth_pose": [{**sample(0, x=0, y=0), "frame": "odom"}]})
        with pytest.raises(ValidationError):
            evaluate(run, profile_factory("R1"))
        return
    elif mutation == "decreasing_time":
        rows.reverse()
    else:
        rows[1]["sequence"] = 0
    run = run_factory({"guard_state": rows, "requested_command": [command(0)]})
    with pytest.raises(ValidationError):
        evaluate(run, profile_factory())


@pytest.mark.parametrize(
    "replacement",
    [
        {"parameters": {"respnse_ns": 1}},
        {"parameters": {"response_ns": True}},
        {"max_gap_ns": 0},
        {"mandatory": "false"},
    ],
)
def test_invalid_profile_values_are_rejected(profile_factory, replacement):
    rule = {"applicable": True, "mandatory": True, **replacement}
    with pytest.raises(ValidationError):
        load_profile(profile_factory(rules={"R3": rule}))


def test_duplicate_yaml_keys_are_rejected(tmp_path):
    path = tmp_path / "duplicate.yaml"
    path.write_text("schema_version: 1\nschema_version: 1\n")
    with pytest.raises(ValidationError, match="duplicate"):
        load_profile(path)


def test_duplicate_json_fields_are_rejected(run_factory, profile_factory):
    run = run_factory({})
    (run / "manifest.json").write_text('{"schema_version":1,"schema_version":1}')
    with pytest.raises(ValidationError, match="duplicate"):
        evaluate(run, profile_factory())


def test_sequence_loss_is_inconclusive_but_retains_a_known_violation(run_factory, profile_factory):
    rows = [guard(0, True), {**guard(NS, True), "sequence": 2}]
    run = run_factory({"guard_state": rows, "requested_command": [command(0)]})
    assert result(evaluate(run, profile_factory()), "R3").verdict == "INCONCLUSIVE"
    rows = [guard(0, False), {**guard(2 * NS, True), "sequence": 2}]
    run = run_factory({"guard_state": rows, "requested_command": [command(0, 0.5)]})
    checked = result(evaluate(run, profile_factory()), "R3")
    assert checked.verdict == "INCONCLUSIVE"  # Cannot reconstruct missing guard transitions.


def test_epoch_reset_does_not_complete_an_old_obligation(run_factory, profile_factory):
    intervals = [
        {"clock": "sim", "epoch": 0, "start_ns": 0, "end_ns": 1_200_000_000},
        {"clock": "sim", "epoch": 1, "start_ns": 0, "end_ns": NS},
    ]
    run = run_factory(
        {
            "guard_state": [guard(0, True), guard(NS, False), {**guard(0, True), "epoch": 1}],
            "requested_command": [command(0, 0.5), {**command(0), "epoch": 1}],
        },
        intervals=intervals,
    )
    checked = result(evaluate(run, profile_factory()), "R3")
    assert checked.verdict == "INCONCLUSIVE"
    assert checked.completed_obligations == 0


def make_csv_dataset(root: Path):
    root.mkdir()
    (root / "metadata.json").write_text(
        json.dumps({"seed": 513, "duration_s": 3, "start_sim_time": 10})
    )
    files = {
        "imu": (
            ["stamp", "wz", "fault", "fault_type", "fault_column"],
            [
                [10, 0, 0, "none", ""],
                [11, 0.1, 1, "short", "wz"],
                [12, 0, 0, "none", ""],
                [13, 0, 0, "none", ""],
            ],
        ),
        "wheel_odom": (
            ["stamp", "vx", "fault", "fault_type", "fault_column"],
            [[10, 0, 0, "none", ""], [13, 0, 0, "none", ""]],
        ),
        "gps": (
            ["stamp", "latitude", "fault", "fault_type", "fault_column"],
            [[10, 47, 0, "none", ""], [13, 47, 0, "none", ""]],
        ),
        "truth": (["stamp", "x", "y"], [[10, 0, 0], [13, 0, 0]]),
        "cmd": (["stamp", "linear", "angular", "segment"], [[10, 0, 0, 0], [13, 0, 0, 0]]),
    }
    for name, (columns, rows) in files.items():
        with (root / f"{name}.csv").open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(columns)
            writer.writerows(rows)
    (root / "faults.json").write_text(json.dumps({"source_run": "drive_513", "faults": []}))
    return root


def test_existing_csv_reader_preserves_masks_and_excludes_navigation(tmp_path, profile_factory):
    run = make_csv_dataset(tmp_path / "dataset")
    reader = open_run(run)
    info = reader.inspection()
    assert info["source_run_id"] == "drive_513"
    records = list(reader.records("events", reader.manifest.intervals[0]))
    assert [(r.time_ns, r.data["kind"]) for r in records] == [
        (11 * NS, "fault_start"),
        (12 * NS, "fault_end"),
    ]
    checked = result(evaluate(run, profile_factory("R2")), "R2")
    assert checked.verdict == "INCONCLUSIVE"
    assert "estimated_pose" in checked.metrics["missing_streams"]


def test_csv_missing_metadata_does_not_claim_completed_run(tmp_path, profile_factory):
    run = make_csv_dataset(tmp_path / "dataset")
    (run / "metadata.json").unlink()
    with pytest.raises(ValidationError):
        evaluate(run, profile_factory("R8"))


def test_csv_completion_counts_detect_truncated_source(tmp_path):
    run = make_csv_dataset(tmp_path / "dataset")
    (run / "metadata.json").write_text(json.dumps({"counts": {"imu": 10}}))
    with pytest.raises(ValidationError, match="sample count"):
        open_run(run).inspection()


def test_csv_nonobject_metadata_is_rejected(tmp_path):
    run = make_csv_dataset(tmp_path / "dataset")
    (run / "faults.json").write_text("[]")
    with pytest.raises(ValidationError):
        open_run(run)


def test_saved_result_report_needs_no_recordings_and_deduplicates_source_runs(
    tmp_path, run_factory, profile_factory
):
    profile = profile_factory("R8")
    results_path = tmp_path / "results"
    for index in range(2):
        run = run_factory(
            {"events": [], "detector_decision": [detector(0)]}, source_id="same_source"
        )
        checked = evaluate(run, profile)
        write_evaluation(checked, results_path / str(index))
        # Remove source content; report generation must use saved results alone.
        for file in run.iterdir():
            file.unlink()
        run.rmdir()
    first = report(results_path, tmp_path / "report_a")
    second = report(results_path, tmp_path / "report_b")
    assert first == second
    assert first["unique_runs"] == 2
    stats = next(iter(first["cohorts"].values()))["R8_false_alarm"]
    assert stats["trials"] == 1  # Two variants are one independent source drive.
    assert stats["zero_alarm_upper95_one_sided"] == pytest.approx(0.95)
    assert (tmp_path / "report_a" / "summary.csv").read_bytes() == (
        tmp_path / "report_b" / "summary.csv"
    ).read_bytes()
    assert (tmp_path / "report_a" / "verdicts.svg").is_file()


def test_cli_exit_codes_and_structured_error(tmp_path, run_factory, profile_factory):
    profile = profile_factory()
    run = run_factory({"guard_state": [guard(0, False)], "requested_command": [command(0)]})
    args = [
        sys.executable,
        "-m",
        "uav_ugv_verification",
        "evaluate",
        str(run),
        "--profile",
        str(profile),
        "--output",
        str(tmp_path / "output"),
    ]
    process = subprocess.run(args, capture_output=True, text=True, timeout=60)
    assert process.returncode == 0, process.stderr
    first_result = (tmp_path / "output" / "result.json").read_bytes()
    repeated = subprocess.run(args, capture_output=True, text=True, timeout=60)
    assert repeated.returncode == 0
    assert first_result == (tmp_path / "output" / "result.json").read_bytes()
    (run / "manifest.json").write_text("not JSON")
    invalid = subprocess.run(args, capture_output=True, text=True, timeout=60)
    assert invalid.returncode == 2
    assert json.loads(invalid.stdout)["status"] == "invalid_input"
    assert not (tmp_path / "output" / "result.json").exists()
    assert (tmp_path / "output" / "error.json").is_file()


def test_output_io_error_is_not_a_safety_failure(tmp_path, run_factory, profile_factory, capsys):
    run = run_factory({"guard_state": [guard(0, False)], "requested_command": [command(0)]})
    output = tmp_path / "existing-file"
    output.write_text("retain")
    code = main(
        ["evaluate", str(run), "--profile", str(profile_factory()), "--output", str(output)]
    )
    assert code == 2
    assert json.loads(capsys.readouterr().out)["artifact_error"]
    assert output.read_text() == "retain"


def test_recorded_watchdog_profile_requires_a_steady_clock(profile_factory):
    path = profile_factory("R4")
    profile = yaml.safe_load(path.read_text())
    profile["calibration"] = "provisional"
    path.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValidationError, match="steady"):
        load_profile(path)
