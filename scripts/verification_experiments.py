#!/usr/bin/env python3
"""Run frozen verification campaigns serially in Linux without GUI windows."""

from __future__ import annotations

import argparse
import ctypes
import fcntl
import json
import os
import random
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
from uav_ugv_verification import evaluate
from uav_ugv_verification.campaign import export_campaign
from uav_ugv_verification.adapters.profile import bind_profile
from uav_ugv_verification.adapters.rosbag import import_bag
from uav_ugv_verification.experiments import (
    Ledger,
    load_plan,
    objective,
    source_snapshot,
)
from uav_ugv_verification.reporting import write_evaluation
from uav_ugv_verification.validation import digest_file, require

ROOT = Path(__file__).resolve().parents[1]


def parent_death_signal():
    if ctypes.CDLL(None).prctl(1, signal.SIGINT) != 0 or os.getppid() == 1:
        raise RuntimeError("could not establish parent-death cleanup")


def interrupt(_signum, _frame):
    raise KeyboardInterrupt


def process_groups(root_pid):
    rows = [
        tuple(map(int, line.split()))
        for line in subprocess.check_output(
            ["ps", "-eo", "pid=,ppid=,pgid="], text=True
        ).splitlines()
    ]
    owned = {root_pid}
    while True:
        added = {pid for pid, parent, _ in rows if parent in owned} - owned
        if not added:
            break
        owned |= added
    return {group for pid, _, group in rows if pid in owned and group != os.getpgrp()}


def stop(process, groups):
    groups |= process_groups(process.pid)
    # Let runner traps finish the recorder and terminate their own descendants.
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=35)
        except subprocess.TimeoutExpired:
            pass
    for sig in (signal.SIGTERM, signal.SIGKILL):
        for group in groups:
            try:
                os.killpg(group, sig)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            live = [
                line.split()
                for line in subprocess.check_output(
                    ["ps", "-eo", "pgid=,stat="], text=True
                ).splitlines()
            ]
            if not any(int(group) in groups and not state.startswith("Z") for group, state in live):
                break
            time.sleep(0.05)
    process.wait(timeout=3)


def run_process(command, environment, directory, timeout):
    groups = set()
    started = time.monotonic()
    with (directory / "runner.log").open("w") as log:
        process = subprocess.Popen(
            command,
            cwd=ROOT,
            env=environment,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            preexec_fn=parent_death_signal,
        )
        timed_out = False
        try:
            while process.poll() is None:
                groups |= process_groups(process.pid)
                if time.monotonic() - started >= timeout:
                    timed_out = True
                    break
                time.sleep(0.25)
        finally:
            stop(process, groups)
        return dict(
            returncode=process.returncode,
            timed_out=timed_out,
            duration_wall_s=time.monotonic() - started,
        )


def evaluate_recording(root, case, attempt_dir):
    camera = case["kind"] == "navigation"
    template = (
        ROOT
        / "verification/profiles"
        / (
            "m2_camera.yaml"
            if camera
            else "m2_guard.yaml"
            if case["kind"] == "guard"
            else "m2_watchdog.yaml"
        )
    )
    if camera and not case["assistance"]:
        data = yaml.safe_load(template.read_text())
        data["id"] = "m3-camera-unassisted"
        data["requirements"]["R6"] = dict(
            applicable=False, mandatory=False, reason="UAV assistance disabled"
        )
        template = attempt_dir / "unassisted_template.yaml"
        template.write_text(yaml.safe_dump(data))
    normalized = root / "normalized"
    if not normalized.is_dir() and (root / "bag/metadata.yaml").is_file():
        status = json.loads((root / "recording.json").read_text())["status"]
        import_bag(
            root / "bag",
            ROOT
            / "verification/setups"
            / ("camera_navigation.yaml" if camera else "controlled_navigation.yaml"),
            normalized,
            status=status,
            world=Path(case["world_absolute"]) if camera else None,
            events=root / "events.jsonl",
        )
    if not normalized.is_dir():
        return None
    profile = root / "evaluation_profile.yaml"
    bind_profile(normalized, template, profile)
    result = evaluate(normalized, profile)
    write_evaluation(result, root / "evaluation")
    return dict(
        result_path=str(root / "evaluation/result.json"),
        exit_code=result.exit_code,
        execution_status=result.execution_status,
        objective_r1_m=objective(result.to_dict()),
        verdicts={r.requirement: r.verdict.value for r in result.results},
    )


def execute(ledger, case, parameters, *, category="matrix"):
    attempt = ledger.start(case, category)
    directory = Path(attempt["directory"])
    env = os.environ.copy()
    env.update(
        ROS_DOMAIN_ID=str(90 + attempt["id"] % 100),
        GZ_PARTITION=f"verification_{os.getpid()}_{attempt['id']}",
        SEED=str(case["seed"]),
    )
    detail = dict(command=[], environment={}, evaluations=[])
    status = "invalid"
    try:
        if case["kind"] == "navigation":
            world = (
                Path(case["world_absolute"]) if "world_absolute" in case else ROOT / case["world"]
            )
            require(digest_file(world) == case["world_sha256"], "world changed after freeze")
            case = {**case, "world_absolute": str(world)}
            env.update(
                WORLD_FILE=str(world),
                GOAL_X=str(case["goal"][0]),
                GOAL_Y=str(case["goal"][1]),
                UAV_ASSISTANCE=str(case["assistance"]).lower(),
                INITIAL_GPS_ENABLED="true",
                GPS_OUTAGE_AFTER_MOTION="3" if case["outage"] else "-1",
                TRIAL_TIMEOUT=str(parameters["simulation_timeout_s"]),
                WALL_TIMEOUT=str(parameters["wall_timeout_s"]),
            )
            command = [
                "bash",
                str(ROOT / "scripts/run_goal_navigation_trial.sh"),
                str(directory / "run"),
            ]
            roots = [directory / "run/verification"]
        elif case["kind"] == "guard":
            command = [
                sys.executable,
                str(ROOT / "scripts/test_navigation_guards.py"),
                str(directory / "run/result.json"),
            ]
            roots = [directory / "run/verification"]
        else:
            command = [
                sys.executable,
                str(ROOT / "scripts/test_actuator_watchdog.py"),
                str(directory / "run"),
            ]
            roots = [
                directory / "run" / name / "verification"
                for name in ("controller_death", "command_transport_loss")
            ]
        detail.update(
            command=command,
            environment={
                k: env[k]
                for k in (
                    "ROS_DOMAIN_ID",
                    "GZ_PARTITION",
                    "SEED",
                    "WORLD_FILE",
                    "GOAL_X",
                    "GOAL_Y",
                    "UAV_ASSISTANCE",
                    "GPS_OUTAGE_AFTER_MOTION",
                    "TRIAL_TIMEOUT",
                    "WALL_TIMEOUT",
                )
                if k in env
            },
        )
        (directory / "invocation.json").write_text(json.dumps(detail, indent=2) + "\n")
        process = run_process(command, env, directory, parameters["attempt_timeout_s"])
        detail.update(process)
        for root in roots:
            try:
                evaluation = evaluate_recording(root, case, directory)
                if evaluation:
                    detail["evaluations"].append(evaluation)
            except (ValueError, OSError, KeyError) as exc:
                detail.setdefault("evaluation_errors", []).append(str(exc))
        codes = [e["exit_code"] for e in detail["evaluations"]]
        if process["timed_out"]:
            status = "timed_out"
        elif not codes:
            status = "startup_failed"
        elif 1 in codes:
            status = "failed"
        elif process["returncode"] != 0 or len(codes) != len(roots) or 2 in codes:
            status = "inconclusive"
        else:
            status = "passed"
    except KeyboardInterrupt:
        status = "interrupted"
        raise
    except Exception as exc:
        detail["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        ledger.finish(attempt["id"], status, detail)
        print(
            json.dumps(dict(attempt=attempt["id"], case=case["case_id"], status=status)), flush=True
        )
    return ledger.rows()[-1]


def search_cases(parameters, root):
    generator = random.Random(parameters["search_seed"])
    for index in range(parameters["search_trials"]):
        layout = ("sparse", "dense", "corridor", "narrow")[index % 4]
        tree = ET.parse(ROOT / f"simulation/worlds/simple_goal_demo_{layout}.sdf")
        offsets = {}
        for model in tree.findall(".//world/model"):
            if model.get("name", "").startswith("obstacle_"):
                pose = model.find("pose")
                values = [float(v) for v in pose.text.split()]
                dx, dy = generator.uniform(-0.25, 0.25), generator.uniform(-0.25, 0.25)
                values[0] += dx
                values[1] += dy
                pose.text = " ".join(str(v) for v in values)
                offsets[model.get("name")] = [dx, dy]
        world = root / f"search-{index:02d}.sdf"
        tree.write(world, encoding="utf-8", xml_declaration=True)
        yield dict(
            case_id=f"search-{index:02d}",
            kind="navigation",
            seed=700 + index,
            world_absolute=str(world),
            world_sha256=digest_file(world),
            layout=layout,
            goal=[4.0, 0.0],
            assistance=True,
            outage=False,
            offsets=offsets,
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--plan", type=Path, default=ROOT / "verification/experiments/matrix.yaml")
    parser.add_argument(
        "--stage", choices=("matrix", "search", "replay", "all", "export"), default="all"
    )
    parser.add_argument("--export-output", type=Path)
    args = parser.parse_args()
    args.output = args.output.resolve()
    if args.stage == "export":
        require(args.export_output is not None, "--export-output is required")
        summary = export_campaign(args.output, args.export_output)
        print(
            json.dumps(
                {
                    "recordings": len(summary["recordings"]),
                    "attempt_counts": summary["attempt_counts"],
                }
            )
        )
        return
    args.output.mkdir(parents=True, exist_ok=True)
    with (
        (args.output / "runner.lock").open("w") as lock,
        open("/tmp/uav-verification-px4.lock", "w") as port_lock,
    ):
        for handle in (lock, port_lock):
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        plan = load_plan(args.plan, ROOT)
        snapshot = source_snapshot(ROOT)
        ledger = Ledger(args.output, plan, snapshot["sha256"])
        (args.output / "freeze.json").write_text(
            json.dumps(dict(plan=plan, source=snapshot), indent=2) + "\n"
        )
        ledger.recover()
        signal.signal(signal.SIGTERM, interrupt)
        try:
            if args.stage in ("all", "matrix"):
                for case in plan["cases"]:
                    if not ledger.attempted(case["case_id"]):
                        require(
                            source_snapshot(ROOT)["sha256"] == snapshot["sha256"],
                            "source changed while campaign running",
                        )
                        execute(ledger, case, plan["parameters"])
            if args.stage in ("all", "search"):
                search_dir = args.output / "search_worlds"
                search_dir.mkdir(exist_ok=True)
                spent = sum(
                    r["detail"].get("duration_wall_s", 0)
                    for r in ledger.rows()
                    if r["category"] == "search"
                )
                for case in search_cases(plan["parameters"], search_dir):
                    if spent + 45 >= plan["parameters"]["search_wall_budget_s"]:
                        break
                    if not ledger.attempted(case["case_id"], "search"):
                        remaining = plan["parameters"]["search_wall_budget_s"] - spent
                        params = {
                            **plan["parameters"],
                            "attempt_timeout_s": min(
                                remaining - 40, plan["parameters"]["attempt_timeout_s"]
                            ),
                        }
                        row = execute(ledger, case, params, category="search")
                        spent += row["detail"].get("duration_wall_s", 0)
            if args.stage in ("all", "replay"):
                candidates = [
                    r
                    for r in ledger.rows()
                    if r["category"] != "replay"
                    and any(
                        e["verdicts"].get("R1") == "FAIL"
                        for e in r["detail"].get("evaluations", [])
                    )
                ]
                candidates.sort(
                    key=lambda r: min(
                        (
                            e["objective_r1_m"]
                            for e in r["detail"]["evaluations"]
                            if e["objective_r1_m"] is not None
                        ),
                        default=float("inf"),
                    )
                )
                for row in candidates[: plan["parameters"]["replay_limit"]]:
                    case = {
                        **row["configuration"],
                        "case_id": "replay-" + row["case_id"],
                        "parent_attempt": row["id"],
                    }
                    if not ledger.attempted(case["case_id"], "replay"):
                        execute(ledger, case, plan["parameters"], category="replay")
        finally:
            ledger.close()


if __name__ == "__main__":
    main()
