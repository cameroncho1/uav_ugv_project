#!/usr/bin/env python3
"""Execute the live ML chapter against scripts/run_ml_demo.sh in Linux."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
from urllib.error import URLError
from urllib.parse import urlencode

from rehearse_demo import request


def run(args):
    args.output.mkdir(parents=True, exist_ok=False)
    started_wall = time.monotonic()
    deadline = started_wall + args.timeout
    phase, phase_at = "startup", 0.0
    counts = dict(truth=0, fused=0, baseline=0, series=0, ml=0, layers=-1)
    result = dict(recorded_utc=datetime.now(timezone.utc).isoformat(), success=False,
                  configuration=dict(seed=513, model="paired time-domain BiLSTM", threshold=0.5,
                                     persistence=3, wheel_speed_scale=1.6, fault_hold=True,
                                     check_uav_timeout=args.check_uav_timeout),
                  actions=[], highlights={}, checks={}, model_series=[])

    def capture(label, state):
        result["highlights"][label] = {k: state[k] for k in (
            "t", "mode", "reason", "guard_valid", "flags", "ml", "truth", "fused", "controls")}

    def control(name, value, state):
        ack = request(args.url, "/control", dict(name=name, value=value))
        if not ack.get("ok"):
            raise RuntimeError(str(ack))
        result["actions"].append(dict(t=state["t"], name=name, value=value, ack=ack))
        print(f"{state['t']:.3f}: {name} = {value}", flush=True)

    try:
        with (args.output / "samples.jsonl").open("w") as output:
            while time.monotonic() < deadline:
                try:
                    state = request(args.url, "/state?" + urlencode(counts))
                except (URLError, TimeoutError):
                    time.sleep(0.2)
                    continue
                for k in ("truth", "fused", "baseline"):
                    counts[k] = state["trail_count"][k]
                counts.update(series=state["series_count"], layers=state["layers_seq"], ml=state["ml_count"])
                result["model_series"].extend(state["ml_series"])
                row = {k: state[k] for k in ("t", "mode", "reason", "guard_valid", "flags", "ml", "controls")}
                output.write(json.dumps(dict(phase=phase, **row)) + "\n")
                output.flush()
                t, flags = state["t"], state["flags"]
                wheel, policy = state["ml"]["wheel"], state["ml"]["policy"]
                if phase == "startup" and state["started"]:
                    if not state["detector_hold"]:
                        raise RuntimeError("ML chapter requires detector-driven holds")
                    phase, phase_at = "clean", t
                elif phase == "clean" and t - phase_at >= 3 and state["mode"] == "NAVIGATE":
                    if not wheel or not policy or not policy["valid"] or wheel["fault_probability"] >= 0.5:
                        continue
                    capture("clean", state)
                    control("wheel_speed_scale", 1.6, state)
                    phase, phase_at = "fault", t
                elif phase == "fault":
                    count = int(policy["wheel_count"]) if policy else 0
                    if count in (1, 2, 3) and f"persistence_{count}" not in result["highlights"]:
                        capture(f"persistence_{count}", state)
                    if flags.get("wheels") and state["mode"] == "HOLD_FAULT":
                        if not state["guard_valid"]:
                            raise RuntimeError("Pose guard failed during the learned-detector hold")
                        capture("detector_hold", state)
                        phase, phase_at = "confirmed", t
                    elif t - phase_at > 20 or state["mode"] == "GOAL":
                        raise RuntimeError("No learned wheel hold before the fault deadline/arrival")
                elif phase == "confirmed" and t - phase_at >= 2:
                    control("wheel_speed_scale", 1.0, state)
                    phase, phase_at = "recovery", t
                elif phase == "recovery" and state["mode"] == "NAVIGATE" and not flags.get("wheels"):
                    capture("navigation_resumes", state)
                    phase, phase_at = "arrival", t
                elif phase == "arrival" and state["mode"] == "GOAL":
                    capture("goal", state)
                    if math.dist(state["truth"][:2], [4.0, 0.0]) > 0.85:
                        raise RuntimeError("Goal truth distance exceeds 0.85 m")
                    if not args.check_uav_timeout:
                        break
                    control("uav_measurements", False, state)
                    phase, phase_at = "uav_outage", t
                elif phase == "uav_outage" and flags.get("uav"):
                    capture("uav_timeout", state)
                    control("uav_measurements", True, state)
                    phase, phase_at = "uav_restore", t
                elif phase == "uav_restore" and not flags.get("uav"):
                    capture("uav_measurements_return", state)
                    break
                time.sleep(0.2)
            else:
                raise RuntimeError(f"ML rehearsal deadline in {phase}")
        h = result["highlights"]
        result["timeline"] = request(args.url, "/events")
        result["checks"] = dict(clean_score_below_threshold=h["clean"]["ml"]["wheel"]["fault_probability"] < 0.5,
            three_windows=all(f"persistence_{i}" in h for i in (1, 2, 3)),
            learned_hold=h["detector_hold"]["flags"]["wheels"] and h["detector_hold"]["mode"] == "HOLD_FAULT",
            pose_guard_valid=h["detector_hold"]["guard_valid"],
            navigation_recovers=h["navigation_resumes"]["mode"] == "NAVIGATE",
            goal_reached=h["goal"]["mode"] == "GOAL")
        if args.check_uav_timeout:
            result["checks"].update(
                f4_timeout=h["uav_timeout"]["flags"]["uav"],
                f4_restored=not h["uav_measurements_return"]["flags"]["uav"])
        result["duration"] = dict(wall_s=round(time.monotonic() - started_wall, 3), simulation_s=state["t"])
        result["success"] = all(result["checks"].values())
        if not result["success"]:
            raise RuntimeError("ML rehearsal check failed")
    except Exception as error:
        result["failure"] = str(error)
        raise
    finally:
        (args.output / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result["checks"], indent=2), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8765")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=180)
    parser.add_argument("--check-uav-timeout", action="store_true",
                        help="Append UAV outage/restoration checks after goal arrival.")
    run(parser.parse_args())
