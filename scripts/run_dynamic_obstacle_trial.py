#!/usr/bin/env python3
"""Run one headless camera-navigation scenario and evaluate its saved R1/R2/R5/R6 evidence."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
from uav_ugv_verification import evaluate, load_profile
from uav_ugv_verification.adapters.files import FileReader
from uav_ugv_verification.adapters.profile import bind_profile
from uav_ugv_verification.experiments import source_snapshot
from uav_ugv_verification.reporting import write_evaluation
from uav_ugv_verification.reporting.files import write_json
from uav_ugv_verification.validation import digest_file, require

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENARIO = ROOT / "ros2_ws/src/simulation_support/config/scenarios/single_box.yaml"


def acceptance(expected, process, actor, observations, evaluation, recorded):
    """Require every planned spawn in observer state and recorded R5 evidence."""
    exercised = {v["event_id"] for v in observations["observations"] if v["qualifying"]}
    r5 = next(v for v in evaluation.results if v.requirement == "R5")
    complete = (
        actor["status"] == "completed"
        and observations["error"] is None
        and expected == exercised == set(recorded)
        and len(recorded) == len(expected)
        and r5.trigger_count == len(expected)
        and not process["timed_out"]
        and process["returncode"] == 0
    )
    code = evaluation.exit_code if complete or evaluation.exit_code == 1 else 2
    return dict(
        status="completed" if complete else "incomplete",
        exit_code=code,
        expected_hazards=sorted(expected),
        exercised_hazards=sorted(exercised),
        recorded_hazards=sorted(recorded),
    )


def main():
    import fcntl

    from simulation_support.obstacle_scenarios import load_scenario
    from verification_experiments import interrupt, run_process

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="Fresh output directory")
    parser.add_argument("--scenario", type=Path, default=DEFAULT_SCENARIO)
    parser.add_argument(
        "--world", type=Path, default=ROOT / "simulation/worlds/simple_goal_demo.sdf"
    )
    parser.add_argument("--goal", type=float, nargs=2, default=(4.0, 0.0), metavar=("X", "Y"))
    parser.add_argument("--seed", type=int, default=741)
    parser.add_argument(
        "--profile", type=Path, default=ROOT / "verification/profiles/dynamic_obstacles.yaml"
    )
    args = parser.parse_args()
    output = args.output.resolve()
    require(not output.exists(), "choose a fresh output directory")
    scenario = load_scenario(args.scenario)
    # The reusable scenario adapter supports any declared world; the supplied camera
    # launcher currently supports the named simple_goal_demo world and its variants.
    require(
        scenario.world == ET.parse(args.world).find("world").get("name"),
        "scenario/world names differ",
    )
    require(scenario.world == "simple_goal_demo", "this camera launcher requires simple_goal_demo")
    require(args.seed >= 0, "nonnegative seed required")
    profile = load_profile(args.profile)
    navigation = yaml.safe_load(
        (ROOT / "ros2_ws/src/mission_manager/config/navigation.yaml").read_text()
    )
    planner = navigation["local_astar_planner"]["ros__parameters"]
    controller = navigation["ugv_goal_controller"]["ros__parameters"]
    require(
        profile.rules["R1"].parameters["radius_m"]
        == planner["robot_radius"]
        == profile.rules["R5"].parameters["radius_m"],
        "evaluation and planner footprint radii differ",
    )
    output.mkdir(parents=True)
    summary = dict(status="aborted", scenario_id=scenario.id, seed=args.seed, exit_code=2)
    with open("/tmp/uav-verification-px4.lock", "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, interrupt)
        frozen_scenario, frozen_profile = output / "scenario.yaml", output / "profile_template.yaml"
        shutil.copyfile(args.scenario, frozen_scenario)
        shutil.copyfile(args.profile, frozen_profile)
        snapshot = source_snapshot(ROOT)
        write_json(
            output / "freeze.json",
            dict(
                source=snapshot,
                scenario_sha256=digest_file(frozen_scenario),
                profile_sha256=digest_file(frozen_profile),
                world_sha256=digest_file(args.world),
                scenario=str(args.scenario.resolve()),
                world=str(args.world.resolve()),
                goal=list(args.goal),
                seed=args.seed,
            ),
        )
        observer_params = output / "observer_parameters.yaml"
        observer_params.write_text(
            yaml.safe_dump(
                {
                    "hazard_observer": {
                        "ros__parameters": {
                            "maximum_speed": float(controller["max_linear_speed"]),
                            "robot_radius": float(planner["robot_radius"]),
                            "obstacle_clearance": float(planner["obstacle_clearance"]),
                            "response_bound": profile.rules["R5"].parameters["response_ns"] / 1e9,
                        }
                    }
                }
            )
        )
        environment = dict(
            os.environ,
            OBSTACLE_SCENARIO=str(frozen_scenario),
            RECORDING_SETUP="camera_dynamic_obstacles",
            HAZARD_OBSERVER_PARAMS=str(observer_params),
            WORLD_FILE=str(args.world.resolve()),
            GOAL_X=str(args.goal[0]),
            GOAL_Y=str(args.goal[1]),
            SEED=str(args.seed),
            INITIAL_GPS_ENABLED="true",
            UAV_ASSISTANCE="true",
            TRIAL_TIMEOUT=str(profile.rules["R2"].parameters["deadline_ns"] / 1e9),
            WALL_TIMEOUT="200",
            ROS_DOMAIN_ID=str(30 + args.seed % 40),
            GZ_PARTITION=f"dynamic_obstacles_{os.getpid()}",
        )
        try:
            process = run_process(
                ["bash", str(ROOT / "scripts/run_goal_navigation_trial.sh"), str(output / "run")],
                environment,
                output,
                300,
            )
            summary["process"] = process
            recording = output / "run/verification"
            bound = output / "evaluation_profile.yaml"
            bind_profile(
                recording / "normalized",
                frozen_profile,
                bound,
            )
            result = evaluate(recording / "normalized", bound)
            write_evaluation(result, output / "evaluation")
            if result.exit_code == 1:
                summary["exit_code"] = 1
            actor = json.loads((output / "run/hazards/scenario_result.json").read_text())
            observations = json.loads((output / "run/hazards/observation_result.json").read_text())
            expected = {op.id for op in scenario.operations if op.action == "spawn"}
            reader = FileReader(recording / "normalized")
            recorded = [
                v.data["event_id"]
                for interval in reader.manifest.intervals
                if interval.clock == "sim"
                for v in reader.records("events", interval)
                if v.valid
                and interval.start_ns <= v.time_ns <= interval.end_ns
                and v.data["kind"] == "hazard_observed"
                and v.data["qualifying"]
            ]
            summary.update(acceptance(expected, process, actor, observations, result, recorded))
            summary.update(
                verdicts={r.requirement: r.verdict.value for r in result.results},
                source_unchanged=source_snapshot(ROOT)["sha256"] == snapshot["sha256"],
            )
            if not summary["source_unchanged"]:
                summary.update(status="invalid", exit_code=2)
        except KeyboardInterrupt:
            summary.update(status="interrupted", exit_code=130)
        except (OSError, ValueError, KeyError) as exc:
            summary.update(error=str(exc))
        finally:
            write_json(output / "summary.json", summary)
    print(json.dumps(summary))
    return summary["exit_code"]


if __name__ == "__main__":
    raise SystemExit(main())
