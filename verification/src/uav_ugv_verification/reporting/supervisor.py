"""Paper material from a source-bound supervisor proof and controlled ROS evidence."""

from __future__ import annotations

import json
from importlib.resources import files
from pathlib import Path
from string import Template

from ..validation import digest_file, read_json, require
from .files import atomic_text, write_csv, write_json
from .publication import catalog


def _figures(output, model, result, trace):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

    figures = output / "figures"
    figures.mkdir(exist_ok=True)
    sources = {}

    def save(fig, name, source):
        footer = (
            f"Policy {model['policy_sha256'][:16]}; implementation "
            f"{model['implementation_sha256'][:16]}. "
            "Discrete decisions / controlled ROS inputs; no physical-safety proof."
        )
        fig.text(0.02, 0.015, footer, fontsize=6, wrap=True)
        fig.tight_layout(rect=(0, 0.08, 1, 1))
        for suffix in ("png", "svg", "pdf"):
            metadata = (
                {"Date": None}
                if suffix == "svg"
                else ({"CreationDate": None, "ModDate": None} if suffix == "pdf" else None)
            )
            path = figures / (name + "." + suffix)
            fig.savefig(path, dpi=180, metadata=metadata)
            if suffix == "svg":
                atomic_text(path, path.read_text(encoding="utf-8"))
        plt.close(fig)
        sources[name] = source

    with plt.rc_context(
        {"font.size": 9, "svg.hashsalt": "navigation-supervisor-v1", "text.usetex": False}
    ):
        fig, ax = plt.subplots(figsize=(7, 3.2))
        labels = [
            "No active goal",
            "Fault latch",
            "Invalid pose",
            "Goal completed",
            "Invalid route",
            "Otherwise",
        ]
        modes = ["INIT", "HOLD_FAULT", "HOLD_GUARD", "GOAL", "HOLD_GUARD", "NAVIGATE"]
        for index, (label, mode) in enumerate(zip(labels, modes, strict=True)):
            y = 5 - index
            ax.add_patch(
                FancyBboxPatch(
                    (0, y),
                    3,
                    0.65,
                    boxstyle="round,pad=.03",
                    facecolor="#e6edf2",
                    edgecolor="#456376",
                )
            )
            ax.text(1.5, y + 0.33, label, ha="center", va="center")
            ax.text(
                5.1,
                y + 0.33,
                mode,
                ha="center",
                va="center",
                color="#326b93" if mode == "NAVIGATE" else "#333333",
            )
            ax.add_patch(
                FancyArrowPatch((3.05, y + 0.33), (4, y + 0.33), arrowstyle="->", mutation_scale=10)
            )
            if index < 5:
                ax.add_patch(
                    FancyArrowPatch(
                        (1.5, y - 0.03), (1.5, y - 0.32), arrowstyle="->", mutation_scale=9
                    )
                )
        ax.set(xlim=(-0.1, 6.5), ylim=(-0.1, 6.5), title="Mode precedence after memory updates")
        ax.axis("off")
        save(fig, "supervisor", {"source": "versioned policy decision table"})

        ten = next(
            c
            for c in result["cases"]
            if c["case"] == "initial_recovery_ten_windows_before_five_seconds"
        )
        origin = ten["first_clear_ns"]
        epoch = ten["observation"]["epoch"]
        selected = [
            v
            for v in trace
            if v["epoch"] == epoch and origin - 500_000_000 <= v["sim_ns"] <= origin + 5_250_000_000
        ]
        fig, axes = plt.subplots(2, 1, figsize=(8, 3.6), sharex=True)
        seconds = [(v["sim_ns"] - origin) / 1e9 for v in selected]
        axes[0].step(seconds, [v["clear_windows"] for v in selected], where="post")
        axes[0].axhline(model["recovery"]["clear_windows"], linestyle="--", color="#8b5542")
        axes[0].set(ylabel="Recovery count", title="Observed recovery count and permission")
        axes[1].step(seconds, [v["supervisor_state"] == "NAVIGATE" for v in selected], where="post")
        axes[1].set(
            ylabel="Permission (0/1)", xlabel="Simulation seconds from first clear decision"
        )
        for ax in axes:
            ax.axvline(model["recovery"]["minimum_clear_ns"] / 1e9, linestyle="--", color="#8b5542")
            ax.grid(alpha=0.15)
        save(
            fig,
            "recovery",
            {
                "source": "control.jsonl",
                "epoch": epoch,
                "origin_ns": origin,
                "records": len(selected),
            },
        )

        beginning = next(c for c in result["cases"] if c["case"] == "estimated_goal_latches")
        ending = next(
            c for c in result["cases"] if c["case"] == "new_revision_authorizes_new_mission"
        )
        origin = beginning["observation"]["sim_ns"]
        epoch = beginning["observation"]["epoch"]
        selected = [
            v
            for v in trace
            if v["epoch"] == epoch
            and origin - 300_000_000 <= v["sim_ns"] <= ending["observation"]["sim_ns"] + 300_000_000
        ]
        fig, axes = plt.subplots(2, 1, figsize=(8, 3.6), sharex=True)
        seconds = [(v["sim_ns"] - origin) / 1e9 for v in selected]
        axes[0].step(
            seconds, [v["goal_completed"] for v in selected], where="post", label="completion latch"
        )
        axes[0].step(
            seconds,
            [v["goal_reached"] for v in selected],
            where="post",
            label="inside estimated tolerance",
        )
        axes[0].set(
            ylabel="Predicate (0/1)", title="Completed goal survives drift and intervening holds"
        )
        axes[0].legend(fontsize=8)
        axes[1].step(seconds, [v["linear"] for v in selected], where="post")
        axes[1].set(ylabel="Command (m/s)", xlabel="Simulation seconds from completion observation")
        axes[1].axvline(
            (ending["observation"]["sim_ns"] - origin) / 1e9,
            linestyle="--",
            color="#8b5542",
            label="new revision / permission observed",
        )
        axes[1].legend(fontsize=7)
        for ax in axes:
            ax.grid(alpha=0.15)
        save(
            fig,
            "goal_memory",
            {
                "source": "control.jsonl",
                "epoch": epoch,
                "origin_ns": origin,
                "records": len(selected),
            },
        )
    write_json(output / "figure_sources.json", sources)


def publish(model_path: Path, controlled: Path, output: Path) -> dict:
    model, result = read_json(model_path), read_json(controlled / "result.json")
    require(
        model["status"] == model["conformance"]["status"] == "PASS",
        "passing compiled proof required",
    )
    require(
        result["status"] == "completed" and all(c.get("passed") is True for c in result["cases"]),
        "controlled ROS evidence is incomplete",
    )
    required = {
        "initial_recovery_ten_windows_before_five_seconds",
        "estimated_goal_latches",
        "new_revision_authorizes_new_mission",
    }
    names = [c["case"] for c in result["cases"]]
    require(
        result["kind"] == "controlled_ROS_supervisor_fixture"
        and required <= set(names)
        and len(set(names)) == len(names),
        "controlled supervisor cases are missing or duplicated",
    )
    recording = read_json(controlled / "verification/recording.json")
    require(recording["status"] == "completed", "controlled recording did not complete")
    require(
        recording["source_sha256"].get("verification/supervisor/policy.json")
        == model["policy_sha256"],
        "recorded policy source differs from checked policy",
    )
    for name, sha in model["implementation_sources"].items():
        require(
            recording["source_sha256"].get(name) == sha,
            "controlled ROS evidence uses different implementation sources",
        )
    trace = [json.loads(line) for line in (controlled / "control.jsonl").read_text().splitlines()]
    require(
        trace and all(v["supervisor_policy"] == model["policy_sha256"] for v in trace),
        "runtime policy differs from the checked model",
    )
    output.mkdir(parents=True, exist_ok=False)
    properties = [
        dict(property=key, formula=value, verdict="PASS")
        for key, value in model["properties"].items()
    ]
    mutations = [
        dict(
            mutation=key,
            verdict=value["status"],
            detected_by=";".join(value["properties_failed"]),
            witness_transitions=len(value["counterexample"]),
        )
        for key, value in model["mutations"].items()
    ]
    cases = [
        dict(
            case=c["case"],
            passed=c["passed"],
            observed_mode=c.get("observation", {}).get("supervisor_state", ""),
        )
        for c in result["cases"]
    ]
    write_csv(output / "properties.csv", properties, list(properties[0]))
    write_csv(output / "mutations.csv", mutations, list(mutations[0]))
    write_csv(output / "cases.csv", cases, list(cases[0]))
    summary = dict(
        controller_states=model["controller_states"],
        product_states=model["product_states"],
        transitions=model["transitions"],
        properties=len(properties),
        detected_mutations=len(mutations),
        controlled_cases=len(cases),
        policy_sha256=model["policy_sha256"],
        implementation_sha256=model["implementation_sha256"],
        input_sha256={
            "model.json": digest_file(model_path),
            "controlled_result.json": digest_file(controlled / "result.json"),
            "control.jsonl": digest_file(controlled / "control.jsonl"),
        },
        scope="Finite decision logic and controlled ROS gating; excludes physical safety.",
    )
    write_json(output / "summary.json", summary)
    _figures(output, model, result, trace)
    for name in ("supervisor.md", "supervisor.tex"):
        text = files(__package__).joinpath("templates", name).read_text(encoding="utf-8")
        atomic_text(
            output / ("paper_section" + Path(name).suffix), Template(text).substitute(summary)
        )
    catalog(output)
    return summary
