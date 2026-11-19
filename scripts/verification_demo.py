#!/usr/bin/env python3
"""Run or replay a verification demonstration without opening desktop windows."""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from pathlib import Path

import yaml
from uav_ugv_verification.campaign import export_campaign
from uav_ugv_verification.reporting.files import write_json
from uav_ugv_verification.reporting.publication import publish

ROOT = Path(__file__).resolve().parents[1]


def verdict(campaign, source):
    try:
        codes = [
            json.loads((source / r["directory"] / "result.json").read_text())[
                "exit_code"
            ]
            for r in campaign["recordings"]
        ]
    except (OSError, ValueError, KeyError):
        return 2
    if 1 in codes:
        return 1
    if (
        campaign["matrix_cases_attempted"] != campaign["matrix_cases_planned"]
        or campaign["missing_artifacts"]
    ):
        return 2
    if campaign["attempt_counts"].get("failed", 0):
        return 1
    expected = sum(
        2 if c["kind"] == "watchdog" else 1 for c in campaign["plan"]["cases"]
    )
    if not codes or len(codes) != expected or any(code != 0 for code in codes):
        return 2
    return 0 if set(campaign["attempt_counts"]) == {"passed"} else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--mode", choices=("live", "replay"), default="live")
    parser.add_argument(
        "--campaign", type=Path, help="Portable export for recorded replay"
    )
    parser.add_argument("--seed", type=int, default=641)
    args = parser.parse_args()
    args.output = args.output.resolve()
    if args.mode == "live" and args.output.exists():
        parser.error(
            "Live output already exists; choose a new directory to retain prior attempts"
        )
    signal.signal(signal.SIGINT, signal.default_int_handler)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    try:
        return run(args)
    except KeyboardInterrupt:
        result = {
            "mode": args.mode,
            "exit_code": 130,
            "status": "interrupted",
            "new_simulation": args.mode == "live",
        }
    except (OSError, ValueError, KeyError, RuntimeError) as exc:
        result = {
            "mode": args.mode,
            "exit_code": 2,
            "status": "invalid",
            "error": str(exc),
            "new_simulation": args.mode == "live",
        }
    write_json(args.output / "demo_summary.json", result)
    print(json.dumps(result, sort_keys=True))
    return result["exit_code"]


def run(args):
    started = time.monotonic_ns()
    if args.mode == "replay":
        if args.campaign is None:
            raise ValueError("--campaign is required for recorded replay")
        source = args.campaign.resolve()
        campaign = json.loads((source / "campaign.json").read_text())
        publish(source, args.output / "report")
        result = {
            "mode": "recorded_replay",
            "new_simulation": False,
            "original_campaign_sha256": campaign["source_sha256"],
            "exit_code": verdict(campaign, source),
            "attempt_counts": campaign["attempt_counts"],
            "duration_wall_s": (time.monotonic_ns() - started) / 1e9,
        }
    else:
        if sys.platform != "linux":
            raise RuntimeError(
                "Live demonstration requires Linux with ROS/PX4/Gazebo sourced"
            )
        # Import POSIX orchestration only for live runs; replay works without ROS.
        from verification_experiments import run_process

        args.output.mkdir(parents=True, exist_ok=False)
        plan = {
            "schema_version": 1,
            "id": "verification-live-demo-v1",
            "seeds": [args.seed],
            "parameters": {
                "simulation_timeout_s": 100,
                "wall_timeout_s": 160,
                "attempt_timeout_s": 240,
                "search_seed": 0,
                "search_trials": 0,
                "search_wall_budget_s": 0,
                "replay_limit": 0,
            },
            "exclusions": {
                "R5": "No actual hazard producer.",
                "R7_R8": "No online detector producer.",
            },
            "rows": [
                {
                    "id": "assisted-arrival",
                    "kind": "navigation",
                    "world": "simulation/worlds/simple_goal_demo.sdf",
                    "goal": [4.0, 0.0],
                    "outage": False,
                    "assistance": True,
                },
                {"id": "guard-response", "kind": "guard"},
                {"id": "watchdog-response", "kind": "watchdog"},
            ],
        }
        plan_path = args.output / "demo_plan.yaml"
        plan_path.write_text(yaml.safe_dump(plan))
        import os

        process = run_process(
            [
                sys.executable,
                str(ROOT / "scripts/verification_experiments.py"),
                str(args.output / "raw"),
                "--plan",
                str(plan_path),
                "--stage",
                "matrix",
            ],
            os.environ.copy(),
            args.output,
            720,
        )
        portable = args.output / "evidence"
        campaign = export_campaign(args.output / "raw", portable)
        code = verdict(campaign, portable)
        if process["returncode"] != 0 and code == 0:
            code = 2
        if code == 0:
            publish(portable, args.output / "report")
        startup = None
        for recording in campaign["recordings"]:
            if recording["configuration"]["kind"] == "navigation":
                manifest = json.loads(
                    (
                        portable / recording["directory"] / "input_manifest.json"
                    ).read_text()
                )
                steady = next(
                    (
                        i
                        for i in manifest["intervals"]
                        if i["clock"].startswith("steady:")
                    ),
                    None,
                )
                if steady:
                    startup = (steady["start_ns"] - started) / 1e9
                break
        result = {
            "mode": "live",
            "new_simulation": True,
            "exit_code": code,
            "attempt_counts": campaign["attempt_counts"],
            "startup_to_first_drive_wall_s": startup,
            "duration_wall_s": (time.monotonic_ns() - started) / 1e9,
            "process": process,
        }
    write_json(args.output / "demo_summary.json", result)
    print(json.dumps(result, sort_keys=True))
    return result["exit_code"]


if __name__ == "__main__":
    raise SystemExit(main())
