"""Paper tables, static figures, and claims from a portable campaign export.

Saved verdicts are authoritative. Display series never feed requirement checks.
"""

from __future__ import annotations

import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path

from ..contracts import REQUIREMENTS
from ..validation import digest_file, read_json, require
from .files import atomic_text, write_csv, write_json
from .summary import _cohort, _load

CAPABILITIES = {
    "R1": "Sampled planar disk clearance against independent static boxes/cylinders.",
    "R2": "Acquisition-time arrival with physical/estimated tolerances and a quiet dwell.",
    "R3": "Consumer-observed invalid guard followed by a sustained zero command.",
    "R4": "Steady-time watchdog expiry and independently measured physical settling.",
    "R5": "Bounded response to an actual qualifying hazard observation.",
    "R6": "Acquisition-matched error conditional on valid pose and fresh UAV input.",
    "R7": "Actual detector decisions after onset, using the declared alarm scope.",
    "R8": "Nominal decision exposure and source-run-clustered false-alarm outcomes.",
}


def metric(requirement: dict, name: str):
    values = [row[name] for row in requirement["metrics"].get("by_interval", []) if name in row]
    if not values:
        return None
    return min(values) if name.startswith("minimum") else max(values)


def _read(root: Path):
    campaign = read_json(root / "campaign.json")
    ledger = read_json(root / "ledger.json")
    require(campaign["source_sha256"] == ledger["source_sha256"], "campaign/ledger source mismatch")
    require(
        campaign["matrix_cases_attempted"] == campaign["matrix_cases_planned"],
        "campaign matrix is incomplete; final publication requires all planned attempts",
    )
    if "plan" in ledger:
        planned = {c["case_id"] for c in ledger["plan"]["cases"]}
        terminal = {
            a["case_id"]
            for a in ledger["attempts"]
            if a["category"] == "matrix" and a["status"] not in ("running", "interrupted")
        }
        require(planned <= terminal, "durable matrix is incomplete")
        require(len(planned) == campaign["matrix_cases_planned"], "matrix size mismatch")
    counts = dict(Counter(a["status"] for a in ledger["attempts"]))
    require(counts == campaign["attempt_counts"], "attempt counts differ from durable ledger")
    recordings = []
    for entry in campaign["recordings"]:
        directory = (root / entry["directory"]).resolve()
        require(directory.is_relative_to(root.resolve()), "recording path escapes export")
        require(
            digest_file(directory / "result.json") == entry["result_sha256"],
            "saved verdict checksum mismatch",
        )
        value = _load(directory / "result.json")
        require(
            value["calibration"] != "fixture",
            "synthetic fixtures cannot support simulator paper claims",
        )
        series_path = directory / "series.json"
        if entry.get("series_sha256"):
            require(
                digest_file(series_path) == entry["series_sha256"], "figure input checksum mismatch"
            )
        series = (
            read_json(directory / "series.json")
            if (directory / "series.json").is_file()
            else {"streams": {}}
        )
        recordings.append({**entry, "result": value, "series": series})
    return campaign, ledger, recordings


def _comparisons(ledger, rows):
    """Pair actual matrix attempts by world/goal/outage/seed; retain absent evidence."""
    metrics = {r["attempt"]: r for r in rows}
    paired = defaultdict(dict)
    for attempt in ledger["attempts"]:
        config = attempt["configuration"]
        if attempt["category"] != "matrix" or config.get("kind") != "navigation":
            continue
        key = (config["world"], tuple(config["goal"]), config["outage"], config["seed"])
        paired[key][config["assistance"]] = attempt
    comparisons = []
    for (world, goal, outage, seed), pair in sorted(paired.items()):
        if set(pair) != {False, True}:
            continue
        row = dict(world=world, goal_x=goal[0], goal_y=goal[1], outage=outage, seed=seed)
        for assistance, name in ((False, "unassisted"), (True, "assisted")):
            attempt = pair[assistance]
            measured = metrics.get(attempt["id"], {})
            row.update(
                {
                    f"{name}_attempt": attempt["id"],
                    f"{name}_status": attempt["status"],
                    f"{name}_R2": measured.get("R2", "NO_RESULT"),
                    f"{name}_arrival_s": measured.get("arrival_s"),
                }
            )
        comparisons.append(row)
    return comparisons


def _rows(recordings):
    rows = []
    for entry in recordings:
        value = entry["result"]
        requirements = {r["requirement"]: r for r in value["results"]}
        row = dict(
            attempt=entry["attempt"],
            category=entry["category"],
            case_id=entry["case_id"],
            seed=entry["configuration"].get("seed"),
            run_id=value["run_id"],
            source_run_id=value["source_run_id"],
            cohort=_cohort(value),
            profile=value["profile_id"],
            profile_sha256=value["profile_sha256"],
            evaluator_sha256=value["evaluator_sha256"],
            exit_code=value["exit_code"],
            result=entry["directory"] + "/result.json",
        )
        row.update({name: requirements[name]["verdict"] for name in REQUIREMENTS})
        row.update(
            minimum_clearance_m=metric(requirements["R1"], "minimum_clearance_m"),
            arrival_s=_seconds(metric(requirements["R2"], "arrival_latency_ns")),
            conditional_rmse_m=metric(requirements["R6"], "conditional_rmse_m"),
            maximum_position_error_m=metric(requirements["R6"], "maximum_error_m"),
            guard_response_ms=_milliseconds(
                metric(requirements["R3"], "maximum_response_latency_ns")
            ),
            watchdog_expiry_ms=_milliseconds(
                metric(requirements["R4"], "actuator_output_maximum_latency_ns")
            ),
            physical_settling_ms=_milliseconds(
                metric(requirements["R4"], "measured_motion_maximum_latency_ns")
            ),
        )
        rows.append(row)
    return rows


def _seconds(value):
    return value / 1e9 if value is not None else None


def _milliseconds(value):
    return value / 1e6 if value is not None else None


def _number(value, places=3):
    return "unavailable" if value is None else f"{value:.{places}f}"


def _markdown(rows, columns):
    return "\n".join(
        [
            "| " + " | ".join(columns) + " |",
            "| " + " | ".join("---" for _ in columns) + " |",
            *(
                "| "
                + " | ".join(
                    str(row.get(key, "")) if row.get(key) is not None else "unavailable"
                    for key in columns
                )
                + " |"
                for row in rows
            ),
        ]
    )


def _streams(entry, stream, *, steady=False):
    return [
        s
        for s in entry["series"]["streams"].values()
        if s["stream"] == stream
        and (s["clock"].startswith("steady:") if steady else s["clock"] == "sim")
    ]


def _plots(output, ledger, recordings, rows):
    import textwrap

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, FancyArrowPatch, Rectangle
    from matplotlib.transforms import Affine2D

    figures = output / "figures"
    figures.mkdir(exist_ok=True)
    sources = {}

    def save(fig, name, evidence):
        if name != "architecture":
            selected = [r for r in recordings if name != "outcomes" or r["category"] == "matrix"]
            if "included_runs" in evidence:
                selected = [
                    r for r in recordings if r["result"]["run_id"] in evidence["included_runs"]
                ]
            elif "recording" in evidence:
                selected = [r for r in recordings if r["directory"] == evidence["recording"]]
            profiles = sorted(
                {
                    r["result"]["profile_id"] + "/v" + r["result"]["profile_version"]
                    for r in selected
                }
            )
            cohorts = sorted({_cohort(r["result"]) for r in selected})
            cohort_set = hashlib.sha256(json.dumps(cohorts).encode()).hexdigest()[:16]
            footer = (
                f"Profiles: {', '.join(profiles) or 'unavailable'}. "
                f"Cohort set: {cohort_set} ({len(cohorts)} cohorts, {len(selected)} recordings); "
                "full IDs and exclusions: figure_sources.json and saved results."
            )
            fig.text(
                0.01, 0.008, textwrap.fill(footer, 110), fontsize=6, color="#444444", va="bottom"
            )
            evidence = {
                **evidence,
                "profiles": profiles,
                "cohorts": cohorts,
                "cohort_set": cohort_set,
            }
            fig.tight_layout(rect=(0, 0.08, 1, 1))
        else:
            fig.tight_layout()
        for suffix in ("svg", "png", "pdf"):
            metadata = (
                {"Date": None}
                if suffix == "svg"
                else {"CreationDate": None, "ModDate": None}
                if suffix == "pdf"
                else None
            )
            fig.savefig(figures / f"{name}.{suffix}", dpi=180, metadata=metadata)
            if suffix == "svg":
                target = figures / f"{name}.svg"
                atomic_text(target, target.read_text(encoding="utf-8"))
        plt.close(fig)
        sources[name] = evidence

    with plt.rc_context(
        {
            "font.size": 9,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "svg.hashsalt": "uav-ugv-verification-publication-v1",
            "text.usetex": False,
        }
    ):
        fig, ax = plt.subplots(figsize=(10, 2.2))
        labels = [
            "ROS / Gazebo\nmeasurements + truth",
            "Record + actual events\nconsumer timestamps",
            "Setup adapter\nframes + geometry",
            "R1–R8 monitors\nfinite-trace verdicts",
            "Saved evidence\npaper + demo",
        ]
        for index, label in enumerate(labels):
            ax.add_patch(
                Rectangle((index * 2, 0.35), 1.8, 1.1, facecolor="#e7edf2", edgecolor="#365269")
            )
            ax.text(index * 2 + 0.9, 0.9, label, ha="center", va="center", fontsize=8)
            if index < 4:
                ax.add_patch(
                    FancyArrowPatch(
                        (index * 2 + 1.8, 0.9),
                        (index * 2 + 2, 0.9),
                        arrowstyle="->",
                        mutation_scale=10,
                    )
                )
        ax.set(xlim=(-0.1, 9.9), ylim=(0, 1.8))
        ax.axis("off")
        save(fig, "architecture", {"scope": "implemented capture/evaluate/report pipeline"})

        groups = defaultdict(Counter)
        for attempt in ledger["attempts"]:
            if attempt["category"] == "matrix":
                label = attempt["configuration"].get("id", attempt["case_id"].rsplit("-s", 1)[0])
                groups[label][attempt["status"]] += 1
        labels = list(groups)
        fig, ax = plt.subplots(figsize=(9, max(3, len(labels) * 0.29)))
        left = [0] * len(labels)
        for status, color in [
            ("passed", "#326b93"),
            ("failed", "#b34235"),
            ("inconclusive", "#d19b32"),
            ("startup_failed", "#7b8790"),
            ("timed_out", "#6e4b77"),
            ("interrupted", "#a6a6a6"),
            ("invalid", "#554c44"),
        ]:
            values = [groups[label][status] for label in labels]
            if any(values):
                ax.barh(labels, values, left=left, label=status, color=color)
                left = [a + b for a, b in zip(left, values, strict=True)]
        ax.set(
            xlabel="Attempt count (all terminal outcomes retained)", title="Fixed matrix outcomes"
        )
        ax.invert_yaxis()
        ax.legend(fontsize=7, loc="lower right")
        save(
            fig,
            "outcomes",
            {"source": "ledger.json", "denominator": "attempts, not only available recordings"},
        )

        for name, field, requirement, title, unit, parameter, scale in (
            (
                "arrival",
                "arrival_s",
                "R2",
                "Confirmed arrivals (R2 PASS only)",
                "Simulation seconds after first drive",
                "deadline_ns",
                1e-9,
            ),
            (
                "localization",
                "maximum_position_error_m",
                "R6",
                "Conditional maximum matched position error",
                "Position error (m)",
                "error_limit_m",
                1,
            ),
            (
                "clearance",
                "minimum_clearance_m",
                "R1",
                "Observed minimum disk clearance",
                "Clearance (m)",
                "contact_tolerance_m",
                -1,
            ),
        ):
            selected = [
                r
                for r in rows
                if r[field] is not None
                and r[requirement] in (("PASS",) if name == "arrival" else ("PASS", "FAIL"))
            ]
            labels = list(dict.fromkeys(r["case_id"].rsplit("-s", 1)[0] for r in selected))
            fig, ax = plt.subplots(figsize=(9, max(3, len(labels) * 0.3)))
            if selected:
                ax.scatter(
                    [r[field] for r in selected],
                    [labels.index(r["case_id"].rsplit("-s", 1)[0]) for r in selected],
                    c=["#b34235" if r[requirement] == "FAIL" else "#326b93" for r in selected],
                    s=16,
                )
                ax.set_yticks(range(len(labels)), labels, fontsize=8)
                limits = sorted(
                    {
                        r["parameters"][parameter] * scale
                        for entry in recordings
                        for r in entry["result"]["results"]
                        if r["requirement"] == requirement and parameter in r["parameters"]
                    }
                )
                for limit in limits:
                    ax.axvline(
                        limit,
                        color="#8b5542",
                        linestyle="--",
                        linewidth=1,
                        label=f"Profile bound {limit:g}",
                    )
                if limits:
                    ax.legend(fontsize=7)
                ax.invert_yaxis()
            else:
                ax.text(
                    0.5, 0.5, "No eligible measured values", transform=ax.transAxes, ha="center"
                )
            ax.set(xlabel=unit, title=f"{title} (n={len(selected)})")
            save(
                fig,
                name,
                {
                    "source": "metrics.csv",
                    "included_runs": [r["run_id"] for r in selected],
                    "omitted_records": len(rows) - len(selected),
                    "counts_of_all_outcomes": "outcomes.svg",
                },
            )

        navigation = next(
            (
                r
                for r in recordings
                if any(
                    check["requirement"] == "R2" and check["verdict"] == "FAIL"
                    for check in r["result"]["results"]
                )
                and _streams(r, "truth_pose")
            ),
            next((r for r in recordings if _streams(r, "truth_pose")), None),
        )
        arrival_failed = navigation is not None and any(
            r["requirement"] == "R2" and r["verdict"] == "FAIL"
            for r in navigation["result"]["results"]
        )
        fig, ax = plt.subplots(figsize=(6, 4))
        if navigation:
            for stream, color, label in (
                ("truth_pose", "#326b93", "truth"),
                ("estimated_pose", "#d08a28", "estimate"),
            ):
                values = _streams(navigation, stream)
                if values:
                    points = [row[2] for row in values[0]["rows"] if row[1]]
                    ax.plot(
                        [p["x"] for p in points],
                        [p["y"] for p in points],
                        color=color,
                        label=label,
                        linewidth=1.3,
                    )
            geometry = _streams(navigation, "obstacles")
            if geometry:
                for shape in geometry[0]["rows"][-1][2]["shapes"]:
                    if shape["kind"] == "cylinder":
                        ax.add_patch(
                            Circle(
                                (shape["x"], shape["y"]),
                                shape["radius"],
                                color="#777777",
                                alpha=0.5,
                            )
                        )
                    else:
                        patch = Rectangle(
                            (-shape["half_x"], -shape["half_y"]),
                            2 * shape["half_x"],
                            2 * shape["half_y"],
                            color="#777777",
                            alpha=0.5,
                        )
                        patch.set_transform(
                            Affine2D().rotate(shape.get("yaw", 0)).translate(shape["x"], shape["y"])
                            + ax.transData
                        )
                        ax.add_patch(patch)
            goal = navigation["configuration"].get("goal")
            if goal:
                ax.plot(*goal, marker="*", color="#9b3f32", markersize=10, label="goal")
            ax.legend(fontsize=8)
        else:
            ax.text(0.5, 0.5, "No navigation trace available", transform=ax.transAxes, ha="center")
        ax.set(
            xlabel="Map east (m)",
            ylabel="Map north (m)",
            title="Recorded failed-arrival trajectory" if arrival_failed else "Recorded trajectory",
        )
        ax.set_aspect("equal", adjustable="datalim")
        save(
            fig,
            "navigation",
            {
                "recording": navigation["directory"] if navigation else None,
                "display": "downsampled; no verdict recomputation",
            },
        )

        for kind, name in (("guard", "guard"), ("watchdog", "watchdog")):
            selected = next((r for r in recordings if r["configuration"].get("kind") == kind), None)
            fig, axes = plt.subplots(2, 1, figsize=(8, 4), sharex=True)
            if selected and any(
                s["clock"].startswith("steady:") for s in selected["series"]["streams"].values()
            ):
                event_series = _streams(selected, "events", steady=True)
                loss = next(
                    (
                        row[0]
                        for s in event_series
                        for row in s["rows"]
                        if row[2].get("kind") == "command_loss"
                    ),
                    None,
                )
                groups = (
                    (
                        ("guard_state", "guard_valid", axes[0], "guard valid"),
                        ("requested_command", "linear", axes[1], "linear command"),
                    )
                    if kind == "guard"
                    else (
                        ("actuator_input", "linear", axes[0], "accepted input"),
                        ("actuator_output", "linear", axes[0], "watchdog output"),
                        ("measured_motion", "linear", axes[1], "interval-average truth speed"),
                    )
                )
                origin = (
                    loss
                    if loss is not None
                    else min(
                        s["start_ns"]
                        for s in selected["series"]["streams"].values()
                        if s["clock"].startswith("steady:")
                    )
                )
                for stream, field, axis, label in groups:
                    series = _streams(selected, stream, steady=True)
                    if series:
                        values = [r for r in series[0]["rows"] if r[1] and field in r[2]]
                        axis.step(
                            [(r[0] - origin) / 1e9 for r in values],
                            [float(r[2][field]) for r in values],
                            where="post",
                            label=label,
                            linewidth=1.1,
                        )
                if loss is not None:
                    for axis in axes:
                        axis.axvline(
                            0, color="#b34235", linestyle="--", linewidth=1, label="command loss"
                        )
            else:
                axes[0].text(
                    0.5,
                    0.5,
                    "No measured trace available",
                    transform=axes[0].transAxes,
                    ha="center",
                )
            for axis in axes:
                if axis.get_legend_handles_labels()[0]:
                    axis.legend(fontsize=7)
                axis.grid(alpha=0.15)
            axes[0].set(
                ylabel="valid (0/1)" if kind == "guard" else "command (m/s)",
                title=f"Recorded {name} response",
            )
            axes[1].set(
                ylabel="command (m/s)" if kind == "guard" else "speed (m/s)",
                xlabel="Steady seconds relative to command loss"
                if kind == "watchdog"
                else "Steady seconds from recording start",
            )
            save(
                fig,
                name,
                {
                    "recording": selected["directory"] if selected else None,
                    "latencies": "authoritative values in metrics.csv",
                },
            )
    write_json(output / "figure_sources.json", sources)


def publish(source: Path, output: Path, *, plots=True) -> dict:
    campaign, ledger, recordings = _read(source)
    output.mkdir(parents=True, exist_ok=True)
    rows = _rows(recordings)
    if rows:
        write_csv(output / "metrics.csv", rows, list(rows[0]))
    comparisons = _comparisons(ledger, rows)
    write_csv(
        output / "assistance_comparisons.csv",
        comparisons,
        [
            "world",
            "goal_x",
            "goal_y",
            "outage",
            "seed",
            "unassisted_attempt",
            "unassisted_status",
            "unassisted_R2",
            "unassisted_arrival_s",
            "assisted_attempt",
            "assisted_status",
            "assisted_R2",
            "assisted_arrival_s",
        ],
    )
    ranges = []
    for field, requirement, unit in (
        ("minimum_clearance_m", "R1", "m"),
        ("arrival_s", "R2", "simulation s"),
        ("conditional_rmse_m", "R6", "m"),
        ("maximum_position_error_m", "R6", "m"),
        ("guard_response_ms", "R3", "steady ms"),
        ("watchdog_expiry_ms", "R4", "steady ms"),
        ("physical_settling_ms", "R4", "steady ms"),
    ):
        values = [
            r[field] for r in rows if r[field] is not None and r[requirement] in ("PASS", "FAIL")
        ]
        ranges.append(
            dict(
                metric=field,
                recordings=len(values),
                minimum=min(values) if values else None,
                maximum=max(values) if values else None,
                unit=unit,
            )
        )
    write_csv(output / "metric_ranges.csv", ranges, list(ranges[0]))
    attempts = [
        dict(
            attempt=a["id"],
            case_id=a["case_id"],
            category=a["category"],
            status=a["status"],
            duration_wall_s=a["detail"].get("duration_wall_s"),
            reason=a["detail"].get("error", ""),
        )
        for a in ledger["attempts"]
    ]
    if attempts:
        write_csv(output / "attempts.csv", attempts, list(attempts[0]))
    claims = []
    for requirement in REQUIREMENTS:
        selected = [
            r
            for entry in recordings
            for r in entry["result"]["results"]
            if r["requirement"] == requirement
        ]
        counts = Counter(r["verdict"] for r in selected)
        claims.append(
            dict(
                requirement=requirement,
                **{key: counts[key] for key in ("PASS", "FAIL", "INCONCLUSIVE", "NOT_APPLICABLE")},
                triggers=sum(r["trigger_count"] for r in selected),
                eligible_seconds=sum(r["eligible_duration_ns"] for r in selected) / 1e9,
                clock_domains=",".join(
                    sorted(
                        {
                            m["clock"].split(":", 1)[0]
                            for r in selected
                            for m in r["metrics"].get("by_interval", [])
                            if "clock" in m
                        }
                    )
                ),
                availability="measured" if counts["PASS"] or counts["FAIL"] else "unmeasured",
                claim=CAPABILITIES[requirement],
                evidence="; ".join(
                    entry["directory"] + "/result.json"
                    for entry in recordings
                    if any(
                        r["requirement"] == requirement and r["verdict"] in ("PASS", "FAIL")
                        for r in entry["result"]["results"]
                    )
                )
                or "No decided recorded evidence",
                limit="Unavailable or excluded in these profiles; see campaign exclusions."
                if not counts["PASS"] and not counts["FAIL"]
                else "Sampled recorded conditions; no continuous or discrete proof.",
            )
        )
    write_csv(output / "claims.csv", claims, list(claims[0]))
    category_claims = []
    for category in sorted({r["category"] for r in recordings}):
        for requirement in REQUIREMENTS:
            values = Counter(
                r["verdict"]
                for entry in recordings
                if entry["category"] == category
                for r in entry["result"]["results"]
                if r["requirement"] == requirement
            )
            category_claims.append(
                dict(
                    category=category,
                    requirement=requirement,
                    **{
                        key: values[key]
                        for key in ("PASS", "FAIL", "INCONCLUSIVE", "NOT_APPLICABLE")
                    },
                )
            )
    if category_claims:
        write_csv(output / "claims_by_category.csv", category_claims, list(category_claims[0]))
    violations = [
        dict(
            attempt=entry["attempt"],
            case_id=entry["case_id"],
            category=entry["category"],
            requirement=r["requirement"],
            reason=w["reason"],
            time_ns=w["time_ns"],
        )
        for entry in recordings
        for r in entry["result"]["results"]
        if r["verdict"] == "FAIL"
        for w in r["witnesses"]
    ]
    write_csv(
        output / "violations.csv",
        violations,
        ["attempt", "case_id", "category", "requirement", "reason", "time_ns"],
    )
    parameters = []
    for requirement in REQUIREMENTS:
        sets = sorted(
            {
                json.dumps(r["parameters"], sort_keys=True)
                for entry in recordings
                for r in entry["result"]["results"]
                if r["requirement"] == requirement and r["parameters"]
            }
        )
        parameters.extend(dict(requirement=requirement, parameters=value) for value in sets)
    write_csv(output / "bounds.csv", parameters, ["requirement", "parameters"])
    summary = dict(
        schema_version=1,
        campaign_source_sha256=campaign["source_sha256"],
        matrix_cases_planned=campaign["matrix_cases_planned"],
        matrix_cases_attempted=campaign["matrix_cases_attempted"],
        attempts=len(attempts),
        attempt_counts=campaign["attempt_counts"],
        recordings=len(recordings),
        cohorts=sorted({_cohort(r["result"]) for r in recordings}),
        claims=claims,
        metric_ranges=ranges,
        assistance_pairs=len(comparisons),
        parameter_sets=parameters,
        search_trials=campaign["search_attempts"],
        search_wall_s=campaign["search_wall_s"],
        replay_attempts=sum(a["category"] == "replay" for a in ledger["attempts"]),
        diagnostic_replay_attempts=sum(
            a["category"] == "diagnostic_replay" for a in ledger["attempts"]
        ),
        missing_artifacts=campaign["missing_artifacts"],
        exclusions=campaign.get("plan", {}).get("exclusions", {}),
        factors={
            "layouts": sorted(
                {
                    Path(
                        a["configuration"].get(
                            "world", a["configuration"].get("world_absolute", "")
                        )
                    ).stem
                    for a in ledger["attempts"]
                    if a["category"] == "matrix" and a["configuration"].get("kind") == "navigation"
                }
            ),
            "goals": [
                list(g)
                for g in sorted(
                    {
                        tuple(a["configuration"]["goal"])
                        for a in ledger["attempts"]
                        if a["category"] == "matrix" and "goal" in a["configuration"]
                    }
                )
            ],
            "seeds": sorted(
                {
                    a["configuration"]["seed"]
                    for a in ledger["attempts"]
                    if a["category"] == "matrix" and "seed" in a["configuration"]
                }
            ),
        },
        artifact_inputs={
            "campaign.json": digest_file(source / "campaign.json"),
            "ledger.json": digest_file(source / "ledger.json"),
        },
    )
    write_json(output / "summary.json", summary)
    if plots:
        _plots(output, ledger, recordings, rows)
    table = _markdown(
        claims, ["requirement", "PASS", "FAIL", "INCONCLUSIVE", "NOT_APPLICABLE", "triggers"]
    )
    pretty_ranges = [
        {**r, "minimum": _number(r["minimum"]), "maximum": _number(r["maximum"])} for r in ranges
    ]
    text = _template(
        "paper.md",
        summary,
        claims_table=table,
        metrics_table=_markdown(pretty_ranges, list(pretty_ranges[0])),
        violations_table=_markdown(violations[:10], ["case_id", "requirement", "reason"])
        if violations
        else "No witnessed requirement violations are recorded.",
    )
    atomic_text(output / "paper.md", text)
    atomic_text(output / "claims.md", _markdown(claims, list(claims[0])) + "\n")
    _latex(output, summary, claims)
    catalog(output)
    return summary


def catalog(output):
    write_json(
        output / "artifacts.json",
        {
            str(p.relative_to(output)): digest_file(p)
            for p in sorted(output.rglob("*"))
            if p.is_file()
            and p.suffix in (".md", ".tex", ".csv", ".json", ".png", ".svg", ".pdf")
            and p.name != "artifacts.json"
        },
    )


def _template(name, summary, **extra):
    from importlib.resources import files
    from string import Template

    context = {
        **summary,
        "cohort_count": len(summary["cohorts"]),
        "attempt_counts_text": json.dumps(summary["attempt_counts"], sort_keys=True),
        "search_wall_s_formatted": f"{summary['search_wall_s']:.3f}",
        "exclusions_text": json.dumps(summary.get("exclusions", {}), sort_keys=True),
        "factors_text": json.dumps(summary.get("factors", {}), sort_keys=True),
        "bounds_text": "; ".join(
            f"{p['requirement']}: {p['parameters']}" for p in summary["parameter_sets"]
        ),
        "unmeasured": ", ".join(
            r["requirement"] for r in summary["claims"] if r["availability"] == "unmeasured"
        )
        or "none",
        **extra,
    }
    substitutions = {
        "\\": r"\textbackslash{}",
        "_": r"\_",
        "%": r"\%",
        "&": r"\&",
        "#": r"\#",
        "{": r"\{",
        "}": r"\}",
        "$": r"\$",
    }
    context["exclusions_tex"] = "".join(substitutions.get(c, c) for c in context["exclusions_text"])
    context["factors_tex"] = "".join(substitutions.get(c, c) for c in context["factors_text"])
    return Template(
        files(__package__).joinpath("templates", name).read_text(encoding="utf-8")
    ).substitute(context)


def _latex(output, summary, claims):
    rows = "\n".join(
        f"{r['requirement']} & {r['PASS']} & {r['FAIL']} & {r['INCONCLUSIVE']} "
        f"& {r['NOT_APPLICABLE']} & {r['triggers']} " + r"\\"
        for r in claims
    )
    metrics = "\n".join(
        str(r["metric"]).replace("_", r"\_")
        + f" & {r['recordings']} & {_number(r['minimum'])} & {_number(r['maximum'])} & {r['unit']} "
        + r"\\"
        for r in summary["metric_ranges"]
    )
    atomic_text(
        output / "paper.tex", _template("paper.tex", summary, claim_rows=rows, metric_rows=metrics)
    )
