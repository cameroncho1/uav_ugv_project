"""Portable exports of campaign accounting, saved verdicts, and figure inputs."""

from __future__ import annotations

import json
import shutil
from collections import Counter
from pathlib import Path

from .adapters.files import FileReader
from .reporting.files import atomic_text, write_json
from .validation import digest_file, read_json, require

FIELDS = {
    "truth_pose": ("x", "y"),
    "estimated_pose": ("x", "y"),
    "requested_command": ("linear", "angular"),
    "guard_state": ("guard_valid", "reason"),
    "actuator_input": ("linear", "angular", "accepted"),
    "actuator_output": ("linear", "angular", "expired"),
    "measured_motion": ("linear", "angular", "simulation_advancing", "uncertainty_ns"),
    "events": ("kind", "event_id", "sensor", "fault_class"),
    "obstacles": ("static", "shapes"),
}


def figure_series(root: Path, result: dict) -> dict:
    reader = FileReader(root)
    series = {}
    for stream, fields in FIELDS.items():
        if stream not in reader.streams:
            continue
        path = root / reader.manifest.streams[stream]
        require(
            digest_file(path) == result["source_sha256"][str(path.relative_to(root))],
            "normalized input changed after evaluation",
        )
        for interval in reader.manifest.intervals:
            count = sum(
                1 for r in reader.records(stream, interval) if r.time_ns >= interval.start_ns
            )
            stride = max(1, (count + 199) // 200)
            rows, before, index = [], None, 0
            for record in reader.records(stream, interval):
                if record.time_ns < interval.start_ns:
                    # Static geometry and latched goal-like data retain one prehistory state.
                    if stream == "obstacles":
                        rows = [
                            [
                                record.time_ns,
                                record.valid,
                                {k: record.data[k] for k in fields if k in record.data},
                            ]
                        ]
                    continue
                data = {k: record.data[k] for k in fields if k in record.data}
                state = data.get(
                    "guard_valid", data.get("expired", data.get("simulation_advancing"))
                )
                change = state is not None and state != before
                if index % stride == 0 or index == count - 1 or change or stream == "events":
                    rows.append([record.time_ns, record.valid, data])
                before = state
                index += 1
            if rows:
                series[f"{stream}/{interval.clock}/{interval.epoch}"] = dict(
                    stream=stream,
                    clock=interval.clock,
                    epoch=interval.epoch,
                    start_ns=interval.start_ns,
                    end_ns=interval.end_ns,
                    original_records=count,
                    stride=stride,
                    rows=rows,
                )
    return dict(
        schema_version=1,
        purpose="display only; verdicts use full-resolution traces",
        source_run_id=result["source_run_id"],
        streams=series,
    )


def export_campaign(source: Path, output: Path) -> dict:
    source = source.resolve()
    require(
        not output.resolve().is_relative_to(source / "attempts"),
        "export cannot replace raw attempts",
    )
    ledger = read_json(source / "ledger.json")
    output.mkdir(parents=True, exist_ok=True)
    if (output / "campaign.json").exists():
        require(
            read_json(output / "campaign.json")["source_sha256"] == ledger["source_sha256"],
            "export contains a different source cohort",
        )
    recordings, missing, diagnostics = [], [], []
    for attempt in ledger["attempts"]:
        readiness = (
            source / "attempts" / f"{attempt['id']:04d}-{attempt['case_id']}" / "run/readiness.json"
        )
        if readiness.is_file():
            relative = Path("diagnostics") / f"a{attempt['id']:04d}-readiness.json"
            destination = output / relative
            destination.parent.mkdir(exist_ok=True)
            shutil.copyfile(readiness, destination)
            diagnostics.append(
                dict(
                    attempt=attempt["id"], path=relative.as_posix(), sha256=digest_file(destination)
                )
            )
        for index, evidence in enumerate(attempt["detail"].get("evaluations", [])):
            result_path = Path(evidence["result_path"]).resolve()
            require(result_path.is_relative_to(source), "result path escapes campaign")
            if not result_path.is_file():
                missing.append(dict(attempt=attempt["id"], path=str(result_path)))
                continue
            result = read_json(result_path)
            relative = Path("recordings") / f"a{attempt['id']:04d}-r{index}"
            destination = output / relative
            destination.mkdir(parents=True, exist_ok=True)
            for name in ("result.json", "requirements.csv"):
                path = result_path.parent / name
                if path.is_file():
                    shutil.copyfile(path, destination / name)
            normalized = result_path.parent.parent / "normalized"
            if (normalized / "manifest.json").is_file():
                shutil.copyfile(normalized / "manifest.json", destination / "input_manifest.json")
                atomic_text(
                    destination / "series.json",
                    json.dumps(
                        figure_series(normalized, result), separators=(",", ":"), allow_nan=False
                    )
                    + "\n",
                )
            for name in ("evaluation_profile.yaml", "recording.json"):
                path = result_path.parent.parent / name
                if path.is_file():
                    shutil.copyfile(path, destination / name)
            fixture_root = result_path.parent.parent.parent
            for name in ("results.json", "result.json"):
                if (fixture_root / name).is_file():
                    shutil.copyfile(fixture_root / name, destination / "legacy_score.json")
                    break
            recordings.append(
                dict(
                    attempt=attempt["id"],
                    category=attempt["category"],
                    case_id=attempt["case_id"],
                    configuration=attempt["configuration"],
                    directory=relative.as_posix(),
                    result_sha256=digest_file(destination / "result.json"),
                    series_sha256=digest_file(destination / "series.json")
                    if (destination / "series.json").is_file()
                    else None,
                )
            )
    for name in (
        "freeze.json",
        "ledger.json",
        "source_archive.json",
        "diagnostic_R2_plan.json",
        "diagnostic_summary.json",
    ):
        if (source / name).is_file():
            shutil.copyfile(source / name, output / name)
    if (source / "search_worlds").is_dir():
        shutil.copytree(source / "search_worlds", output / "search_worlds", dirs_exist_ok=True)
    completed = {
        a["case_id"]
        for a in ledger["attempts"]
        if a["category"] == "matrix" and a["status"] not in ("running", "interrupted")
    }
    search = [a for a in ledger["attempts"] if a["category"] == "search"]
    summary = dict(
        schema_version=1,
        source_sha256=ledger["source_sha256"],
        plan=ledger["plan"],
        recordings=recordings,
        missing_artifacts=missing,
        diagnostics=diagnostics,
        attempt_counts=dict(Counter(a["status"] for a in ledger["attempts"])),
        matrix_cases_planned=len(ledger["plan"]["cases"]),
        matrix_cases_attempted=len(completed),
        search_attempts=len(search),
        search_wall_s=sum(a["detail"].get("duration_wall_s", 0) for a in search),
        raw_archive=str(source),
        raw_archive_committed=False,
    )
    write_json(output / "campaign.json", summary)
    return summary
