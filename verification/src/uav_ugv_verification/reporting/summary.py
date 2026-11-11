"""Cohort-separated verdict summaries and source-run-clustered detector statistics."""

from __future__ import annotations

import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path

from scipy.stats import binomtest

from ..contracts import REQUIREMENTS, Verdict
from ..validation import integer, mapping, read_json, require, text
from .files import write_csv, write_json


def proportion(successes: int, trials: int) -> dict:
    if not trials:
        return {"successes": 0, "trials": 0, "estimate": None, "ci95_two_sided": None}
    interval = binomtest(successes, trials).proportion_ci(confidence_level=0.95, method="exact")
    return {
        "successes": successes,
        "trials": trials,
        "estimate": successes / trials,
        "ci95_two_sided": [float(interval.low), float(interval.high)],
    }


def _load(path: Path) -> dict:
    value = mapping(read_json(path), str(path))
    require(value.get("schema_version") == 1, f"{path}: unsupported result schema")
    for field in (
        "run_id",
        "source_run_id",
        "profile_id",
        "profile_version",
        "profile_sha256",
        "calibration",
        "execution_status",
        "evaluator_version",
        "evaluator_sha256",
    ):
        text(value.get(field), f"result.{field}")
    require(
        type(value.get("exit_code")) is int and value["exit_code"] in (0, 1, 2),
        f"{path}: invalid exit code",
    )
    mapping(value.get("provenance"), "result.provenance")
    mapping(value.get("source_sha256"), "result.source_sha256")
    results = value.get("results")
    require(isinstance(results, list), f"{path}: missing result list")
    require(
        [r.get("requirement") for r in results] == list(REQUIREMENTS),
        f"{path}: expected exactly R1-R8 in order",
    )
    for result in results:
        require(result.get("verdict") in {v.value for v in Verdict}, f"{path}: invalid verdict")
        for field in (
            "eligible_duration_ns",
            "trigger_count",
            "completed_obligations",
            "missing_data_count",
        ):
            integer(result.get(field), f"result.{field}")
        mapping(result.get("metrics"), "result.metrics")
        require(isinstance(result.get("witnesses"), list), "result: witnesses must be a list")
    return value


def _cohort(value: dict) -> str:
    context = {
        key: value[key]
        for key in (
            "profile_id",
            "profile_version",
            "profile_sha256",
            "calibration",
            "evaluator_version",
            "evaluator_sha256",
        )
    }
    context["code_commit"] = value["provenance"].get("code_commit", "unrecorded")
    context["dirty_patch_sha256"] = value["provenance"].get("dirty_patch_sha256", "unrecorded")
    return hashlib.sha256(json.dumps(context, sort_keys=True).encode()).hexdigest()[:16]


def _detector_statistics(runs: list[dict], requirement: str) -> dict:
    clusters = defaultdict(list)
    events, exposure = 0, 0
    for run in runs:
        result = next(r for r in run["results"] if r["requirement"] == requirement)
        if result["verdict"] == "NOT_APPLICABLE":
            continue
        metrics = result["metrics"].get("by_interval", [])
        if requirement == "R8":
            events += sum(m.get("false_alarm_events", 0) for m in metrics)
            exposure += sum(m.get("nominal_exposure_ns", 0) for m in metrics)
            if not metrics or not all(m.get("fault_free_run") for m in metrics):
                continue
        elif not result["trigger_count"]:
            continue
        clusters[run["source_run_id"]].append((run["execution_status"], result["verdict"]))
    outcomes = []
    excluded = 0
    for cluster in clusters.values():
        # Keep witnessed failures, but do not call an incomplete source a clean success.
        if any(verdict == "FAIL" for _, verdict in cluster):
            outcomes.append(requirement == "R8")
        elif all(status == "completed" and verdict == "PASS" for status, verdict in cluster):
            outcomes.append(requirement == "R7")
        else:
            excluded += 1
    result = proportion(sum(outcomes), len(outcomes))
    result.update(unit="independent_source_run", excluded_source_runs=excluded)
    if requirement == "R8":
        result.update(
            alarm_events=events,
            nominal_exposure_ns=exposure,
            events_per_hour=events * 3_600_000_000_000 / exposure if exposure else None,
            zero_alarm_upper95_one_sided=(1 - 0.05 ** (1 / len(outcomes)))
            if outcomes and not any(outcomes)
            else None,
        )
    return result


def _plot(rows: list[dict], output: Path) -> None:
    # Lazy import: numerical evaluation has no GUI or plotting import side effects.
    import matplotlib
    from matplotlib.figure import Figure

    matplotlib.use("Agg")
    colors = {
        "PASS": "#26734d",
        "FAIL": "#b83b36",
        "INCONCLUSIVE": "#bd8b26",
        "NOT_APPLICABLE": "#8a9299",
    }
    with matplotlib.rc_context({"svg.hashsalt": "uav-ugv-verification"}):
        figure = Figure(figsize=(9, 4), layout="constrained")
        axis = figure.subplots()
        left = [0] * len(REQUIREMENTS)
        for verdict, color in colors.items():
            counts = [
                sum(row[verdict] for row in rows if row["requirement"] == name)
                for name in REQUIREMENTS
            ]
            axis.barh(REQUIREMENTS, counts, left=left, label=verdict, color=color)
            left = [a + b for a, b in zip(left, counts, strict=True)]
        axis.set_xlabel("Recorded run verdicts (all cohorts; see summary.csv for separation)")
        axis.legend(fontsize=8)
        figure.savefig(output / "verdicts.svg", metadata={"Date": None})


def report(source: str | Path, output: str | Path) -> dict:
    source, output = Path(source), Path(output)
    paths = [source] if source.is_file() else sorted(source.rglob("result.json"))
    require(bool(paths), f"no saved result.json files under {source}")
    unique = {}
    for path in paths:
        value = _load(path)
        key = (value["run_id"], value["profile_sha256"], value["evaluator_sha256"])
        require(
            key not in unique or unique[key] == value,
            f"conflicting saved results for run/profile {key}",
        )
        unique[key] = value
    cohorts = defaultdict(list)
    for key in sorted(unique):
        value = unique[key]
        cohorts[_cohort(value)].append(value)
    rows, summaries = [], {}
    for cohort, runs in sorted(cohorts.items()):
        summaries[cohort] = {
            "runs": len(runs),
            "execution_status": dict(Counter(r["execution_status"] for r in runs)),
            "profile_id": runs[0]["profile_id"],
            "profile_sha256": runs[0]["profile_sha256"],
            "calibration": runs[0]["calibration"],
            "code_commit": runs[0]["provenance"].get("code_commit", "unrecorded"),
            "R7_detection": _detector_statistics(runs, "R7"),
            "R8_false_alarm": _detector_statistics(runs, "R8"),
        }
        for name in REQUIREMENTS:
            results = [next(r for r in run["results"] if r["requirement"] == name) for run in runs]
            counts = Counter(r["verdict"] for r in results)
            rows.append(
                {
                    "cohort": cohort,
                    "requirement": name,
                    "runs": len(runs),
                    **{v.value: counts[v.value] for v in Verdict},
                    "eligible_duration_ns": sum(r["eligible_duration_ns"] for r in results),
                    "triggers": sum(r["trigger_count"] for r in results),
                    "missing_data_count": sum(r["missing_data_count"] for r in results),
                }
            )
    summary = {
        "schema_version": 1,
        "unique_runs": len(unique),
        "cohorts": summaries,
        "run_index": [
            {
                "run_id": run["run_id"],
                "source_run_id": run["source_run_id"],
                "cohort": _cohort(run),
                "execution_status": run["execution_status"],
                "exit_code": run["exit_code"],
                "source_sha256": run["source_sha256"],
            }
            for _, run in sorted(unique.items())
        ],
    }
    output.mkdir(parents=True, exist_ok=True)
    write_json(output / "summary.json", summary)
    write_csv(output / "summary.csv", rows, list(rows[0]))
    _plot(rows, output)
    return summary
