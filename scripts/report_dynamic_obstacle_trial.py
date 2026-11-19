#!/usr/bin/env python3
"""Export and plot saved dynamic-obstacle evidence without starting ROS or Gazebo."""

import argparse
import math
from pathlib import Path

from uav_ugv_verification.adapters.files import FileReader
from uav_ugv_verification.reporting.files import write_csv, write_json
from uav_ugv_verification.signals.geometry import segment_clearance
from uav_ugv_verification.validation import digest_file, read_json


def collect(source):
    summary = read_json(source / "summary.json")
    evaluation = read_json(source / "evaluation/result.json")
    actor = read_json(source / "run/hazards/scenario_result.json")
    observations = read_json(source / "run/hazards/observation_result.json")["observations"]
    reader = FileReader(source / "run/verification/normalized")
    intervals = [v for v in reader.manifest.intervals if v.clock == "sim"]
    truth, routes = [], []
    for interval in intervals:
        truth.extend(
            [v.time_ns, v.data["x"], v.data["y"]]
            for v in reader.records("truth_pose", interval)
            if v.valid
        )
        routes.extend(v for v in reader.records("route", interval) if v.valid)
    chosen = []
    radius = next(
        v["parameters"]["radius_m"] for v in evaluation["results"] if v["requirement"] == "R5"
    )
    for observation in observations:
        shape = next(
            v["obstacle"]
            for v in actor["history"]
            if v["kind"] == "obstacle_applied" and v["operation"]["id"] == observation["event_id"]
        )
        before = next(
            (v for v in reversed(routes) if v.time_ns < observation["requested_ns"]), None
        )
        after = next(
            (
                v
                for v in routes
                if v.time_ns > observation["grid_stamp_ns"]
                and v.data["goal_revision"] == observation["goal_revision"]
                and all(
                    segment_clearance(a, b, shape, radius) >= 0
                    for a, b in zip(v.data["points"], v.data["points"][1:], strict=False)
                )
            ),
            None,
        )
        chosen.append(
            dict(
                event_id=observation["event_id"],
                shape=shape,
                before=before.data["points"] if before else [],
                after=after.data["points"] if after else [],
                after_route_ns=after.time_ns if after else None,
            )
        )
    return dict(
        schema_version=1,
        summary=summary,
        evaluation=evaluation,
        observations=observations,
        paths=chosen,
        truth=truth[:: max(1, math.ceil(len(truth) / 4000))],
        display_sampling="At most 4,000 truth samples; verdicts use the full normalized trace.",
        sources={
            name: digest_file(source / name)
            for name in (
                "freeze.json",
                "summary.json",
                "evaluation/result.json",
                "run/hazards/observation_result.json",
                "run/hazards/scenario_result.json",
            )
        },
    )


def publish(value, output):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon

    output.mkdir(parents=True, exist_ok=False)
    write_json(output / "evidence.json", value)
    rows = [
        dict(
            operation=v["event_id"],
            qualifying=v["qualifying"],
            requested_s=v["requested_ns"] / 1e9,
            confirmed_s=v["confirmed_ns"] / 1e9,
            observed_s=v["grid_stamp_ns"] / 1e9,
            distance_lower_bound_m=v["distance_lower_bound_m"],
            minimum_distance_m=v["minimum_distance_m"],
        )
        for v in value["observations"]
    ]
    if rows:
        write_csv(output / "timeline.csv", rows, list(rows[0]))
    verdicts = value["summary"].get("verdicts", {})
    figure, axes = plt.subplots(figsize=(8, 4.6))
    if value["truth"]:
        axes.plot(
            [p[1] for p in value["truth"]],
            [p[2] for p in value["truth"]],
            color="#237350",
            linewidth=2,
            label="Measured UGV trajectory",
        )
    for index, path in enumerate(value["paths"]):
        shape = path["shape"]
        c, s = math.cos(shape["yaw"]), math.sin(shape["yaw"])
        corners = [
            (shape["x"] + c * x - s * y, shape["y"] + s * x + c * y)
            for x, y in (
                (-shape["half_x"], -shape["half_y"]),
                (shape["half_x"], -shape["half_y"]),
                (shape["half_x"], shape["half_y"]),
                (-shape["half_x"], shape["half_y"]),
            )
        ]
        axes.add_patch(
            Polygon(
                corners,
                facecolor="#9a4d3b",
                alpha=0.6,
                label="Appearing obstacle" if index == 0 else None,
            )
        )
        for key, style, color, label in (
            ("before", "--", "#737373", "Route before creation"),
            ("after", "-", "#326b93", "Route after obstacle observation"),
        ):
            points = path[key]
            if points:
                axes.plot(
                    [p[0] for p in points],
                    [p[1] for p in points],
                    style,
                    color=color,
                    label=label if index == 0 else None,
                )
    axes.set(
        xlabel="Map east (m)",
        ylabel="Map north (m)",
        title="Camera navigation with an appearing obstacle",
    )
    axes.set_aspect("equal", adjustable="datalim")
    axes.grid(alpha=0.2)
    axes.legend(loc="best", fontsize=8)
    figure.text(
        0.02,
        0.02,
        "Saved measured evidence; "
        + ", ".join(f"{k}: {v}" for k, v in verdicts.items() if k in ("R1", "R2", "R5", "R6")),
        fontsize=8,
    )
    figure.tight_layout(rect=(0, 0.06, 1, 1))
    for suffix in ("png", "svg", "pdf"):
        metadata = (
            {"Date": None}
            if suffix == "svg"
            else ({"CreationDate": None, "ModDate": None} if suffix == "pdf" else None)
        )
        figure.savefig(output / ("navigation." + suffix), dpi=160, metadata=metadata)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="Live-run directory or a saved report export")
    parser.add_argument("output", type=Path, help="Fresh report directory")
    args = parser.parse_args()
    value = (
        read_json(args.source / "evidence.json")
        if (args.source / "evidence.json").is_file()
        else collect(args.source)
    )
    publish(value, args.output)


if __name__ == "__main__":
    main()
