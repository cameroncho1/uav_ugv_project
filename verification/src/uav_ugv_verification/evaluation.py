"""Coordinate capabilities, independent clock epochs, and deterministic results."""

from __future__ import annotations

import hashlib
from dataclasses import asdict
from pathlib import Path

from .adapters import open_run
from .contracts import REQUIREMENTS, Evaluation, Profile, RequirementResult, TraceReader, Verdict
from .monitors import MONITORS
from .validation import load_profile


def inspect_run(run: str | Path, profile: Profile | str | Path) -> dict:
    profile = load_profile(profile) if not isinstance(profile, Profile) else profile
    reader = open_run(run)
    inspected = reader.inspection()
    inspected["profile_id"] = profile.profile_id
    inspected["requirements"] = {}
    for name in REQUIREMENTS:
        rule = profile.rules.get(name)
        inspected["requirements"][name] = {
            "applicable": bool(rule and rule.applicable),
            "mandatory": bool(rule and rule.mandatory),
            "missing_streams": sorted(set(MONITORS[name][1]) - reader.streams)
            if rule and rule.applicable
            else [],
            "reason": rule.reason if rule else "not_selected_by_profile",
        }
    return inspected


def _combine(name: str, rule, parts: list[RequirementResult]) -> RequirementResult:
    result = RequirementResult(
        name,
        parts[0].version,
        rule.mandatory,
        Verdict.PASS,
        "property_holds",
        rule.parameters.copy(),
    )
    for part in parts:
        result.intervals.extend(part.intervals)
        result.eligible_duration_ns += part.eligible_duration_ns
        result.trigger_count += part.trigger_count
        result.completed_obligations += part.completed_obligations
        result.missing_data_count += part.missing_data_count
        for witness in part.witnesses:
            if len(result.witnesses) < 32:
                result.witnesses.append({**part.intervals[0], **witness})
        if part.verdict == Verdict.FAIL:
            result.verdict = Verdict.FAIL
        elif part.verdict != Verdict.PASS and result.verdict != Verdict.FAIL:
            result.verdict = Verdict.INCONCLUSIVE
    # Per-epoch metrics retain their units and denominators; avoid averaging averages.
    result.metrics["by_interval"] = [{**p.intervals[0], **p.metrics} for p in parts]
    result.reason = {
        Verdict.PASS: "property_holds",
        Verdict.FAIL: "witnessed_violation",
        Verdict.INCONCLUSIVE: "incomplete_or_unexercised",
    }[result.verdict]
    return result


def evaluate(run: str | Path, profile: Profile | str | Path) -> Evaluation:
    return evaluate_trace(open_run(run), profile)


def evaluate_trace(reader: TraceReader, profile: Profile | str | Path) -> Evaluation:
    """Evaluate a validated adapter without requiring a filesystem-backed run."""
    profile = load_profile(profile) if not isinstance(profile, Profile) else profile
    inspected = reader.inspection()
    results = []
    for name in REQUIREMENTS:
        rule = profile.rules.get(name)
        if rule is None or not rule.applicable:
            results.append(
                RequirementResult(
                    name,
                    "1",
                    bool(rule and rule.mandatory),
                    Verdict.NOT_APPLICABLE,
                    rule.reason if rule else "not_selected_by_profile",
                )
            )
            continue
        monitor, required = MONITORS[name]
        intervals = [i for i in reader.manifest.intervals if i.clock == rule.clock]
        missing = set(required) - reader.streams
        if missing or not intervals:
            results.append(
                RequirementResult(
                    name,
                    "1",
                    rule.mandatory,
                    Verdict.INCONCLUSIVE,
                    "missing_capabilities",
                    rule.parameters.copy(),
                    [asdict(i) for i in intervals],
                    missing_data_count=1,
                    metrics={
                        "missing_streams": sorted(missing),
                        "missing_clock": not bool(intervals),
                    },
                )
            )
            continue
        parts = []
        for interval in intervals:
            # Missing state-changing events can change whether a violation exists at all.
            state_stream = {"R4": "actuator_input", "R8": "events"}.get(name)
            if state_stream and inspected["streams"][state_stream][
                "sequence_gaps_by_partition"
            ].get(f"{interval.clock}/{interval.epoch}", 0):
                parts.append(
                    RequirementResult(
                        name,
                        "1",
                        rule.mandatory,
                        Verdict.INCONCLUSIVE,
                        "state_event_sequence_gap",
                        rule.parameters.copy(),
                        [asdict(interval)],
                        missing_data_count=1,
                    )
                )
            else:
                parts.append(monitor(reader, rule, interval))
        result = _combine(name, rule, parts)
        if any(inspected["streams"][s]["sequence_gaps"] for s in required):
            result.missing_data_count += 1
            if result.verdict != Verdict.FAIL:
                result.verdict, result.reason = Verdict.INCONCLUSIVE, "source_sequence_gap"
        results.append(result)
    mandatory = [r for r in results if r.mandatory and r.verdict != Verdict.NOT_APPLICABLE]
    if any(r.verdict == Verdict.FAIL for r in mandatory):
        exit_code, reason = 1, "mandatory_requirement_failed"
    elif not mandatory:
        exit_code, reason = 2, "no_applicable_requirements"
    elif reader.manifest.status != "completed":
        exit_code, reason = 2, "run_not_completed"
    elif any(r.verdict != Verdict.PASS for r in mandatory):
        exit_code, reason = 2, "mandatory_evidence_incomplete"
    else:
        exit_code, reason = 0, "mandatory_requirements_passed"
    code_hash = hashlib.sha256()
    package = Path(__file__).parent
    for source in sorted(package.rglob("*.py")):
        code_hash.update(source.relative_to(package).as_posix().encode())
        code_hash.update(source.read_bytes().replace(b"\r\n", b"\n"))
    return Evaluation(
        reader.manifest.run_id,
        reader.manifest.source_run_id,
        profile.profile_id,
        profile.version,
        profile.sha256,
        profile.calibration,
        reader.manifest.status,
        reader.manifest.provenance,
        inspected["source_sha256"],
        results,
        exit_code,
        reason,
        evaluator_sha256=code_hash.hexdigest(),
    )
