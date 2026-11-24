#!/usr/bin/env python3
"""Rehearse the presentation against an already running demo dashboard in Linux.

Only existing HTTP controls are used. No measurements, actuator commands, or
simulator truth are published. Results include acknowledged actions and the
dashboard's navigation/flag event trace.
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import Request, urlopen


# Each action is followed by an observation interval. The same sequence is in docs/demo.md.
STEPS = (
    ("noise", "/control", dict(name="noise_scale", value=3.0), 12),
    ("nominal", "/control", dict(name="noise_scale", value=1.0), 3),
    ("gps_off", "/control", dict(name="gps_enabled", value=False), 8),
    ("corrections_off", "/control", dict(name="uav_assistance", value=False), 8),
    ("corrections_on", "/control", dict(name="uav_assistance", value=True), 8),
    ("wheel_error", "/control", dict(name="wheel_speed_scale", value=1.2), 6),
    ("combined_faults", "/control", dict(name="imu_yaw_bias_rate", value=math.radians(2)), 30),
    ("sensors_restored", "/reset", {}, 8),
)


def request(base, path, body=None):
    req = Request(base + path, data=None if body is None else json.dumps(body).encode(),
                  headers={"Content-Type": "application/json"})
    with urlopen(req, timeout=12) as response:
        return json.load(response)


def run(args):
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    started_wall = time.monotonic()
    deadline = started_wall + args.timeout
    counts = dict(truth=0, fused=0, baseline=0, series=0, layers=-1)
    state, samples, actions, phases = {}, [], [], []
    phase, phase_at, step = "startup", 0.0, -1
    block_added, block_attempt = False, -100.0
    result = dict(recorded_utc=datetime.now(timezone.utc).isoformat(),
                  configuration=dict(seed=513, measurement_profile="nominal", fault_hold=False,
                                     world="simple_goal_demo_complex"),
                  success=False, checks={}, actions=actions, phases=phases)

    def act(path, body):
        response = request(args.url, path, body)
        actions.append(dict(t=state["t"], phase=phase, path=path, body=body, response=response))
        if not response.get("ok"):
            raise RuntimeError(f"Action rejected: {path}: {response}")
        print(f"{state['t']:.2f} {phase}: {path} {body}", flush=True)

    try:
        with (output / "samples.jsonl").open("w") as stream:
            while time.monotonic() < deadline:
                try:
                    state = request(args.url, "/state?" + urlencode(counts))
                except (URLError, TimeoutError):
                    time.sleep(0.5)
                    continue
                t = state["t"]
                for name in ("truth", "fused", "baseline"):
                    counts[name] = state["trail_count"][name]
                counts.update(series=state["series_count"], layers=state["layers_seq"])
                sample = {name: state.get(name) for name in (
                    "t", "mode", "reason", "truth", "fused", "baseline", "goal", "flags", "controls", "series")}
                sample.update(phase=phase, wall_elapsed_s=round(time.monotonic() - started_wall, 3))
                stream.write(json.dumps(sample) + "\n")
                stream.flush()
                samples.append(sample)
                if phase == "startup" and state["started"]:
                    phase = "course"
                    phase_at = t
                    phases.append(dict(name=phase, t=t))
                if phase == "course":
                    if not block_added and t - phase_at >= 10 and t - block_attempt >= 4:
                        block_attempt = t
                        try:
                            act("/block", {})
                            block_added = True
                        except HTTPError as error:
                            print(f"Block placement: {error.read().decode()}", flush=True)
                    if state["mode"] == "GOAL":
                        distance = math.dist(state["truth"][:2], [9, 3])
                        if distance > 0.85:
                            raise RuntimeError(f"First goal truth error {distance:.3f} m exceeds tolerance")
                        result["checks"]["first_goal"] = dict(t=t, truth_distance_m=distance,
                            truth=state["truth"], hazards=state["hazards"])
                        result["checks"]["live_block_added"] = block_added
                        return_goal = request(args.url, "/world")["return_goal"]
                        result["configuration"]["return_goal"] = return_goal
                        act("/goal", dict(x=return_goal[0], y=return_goal[1]))
                        step = 0
                        phase, path, body, _ = STEPS[step]
                        phase_at = t
                        phases.append(dict(name=phase, t=t))
                        act(path, body)
                elif step >= 0 and t - phase_at >= STEPS[step][3]:
                    step += 1
                    if step == len(STEPS):
                        break
                    phase, path, body, _ = STEPS[step]
                    phase_at = t
                    phases.append(dict(name=phase, t=t))
                    act(path, body)
                time.sleep(0.5)
            else:
                raise RuntimeError("Rehearsal deadline reached")

        events = request(args.url, "/events")
        result["timeline"] = events
        checks = result["checks"]
        navigation = [e for e in events if e["kind"] == "navigation"]
        first_nav = next(e["t"] for e in navigation if e["mode"] == "NAVIGATE")
        first_goal = next(e["t"] for e in navigation if e["mode"] == "GOAL")
        checks["first_goal"]["navigation_elapsed_sim_s"] = round(first_goal - first_nav, 3)
        checks["timed_hazard_present"] = any("slalom_box" in h["id"] for h in checks["first_goal"]["hazards"])
        checks["controls_restored"] = state["controls"] == dict(gps_enabled=True, uav_assistance=True,
            noise_scale=1.0, gps_horizontal_stddev=1.5, wheel_speed_scale=1.0, imu_yaw_bias_rate=0.0)
        checks["live_blocks_removed"] = not any("live_block" in h["id"] for h in state["hazards"])
        checks["return_goal_observed"] = state["goal"] == result["configuration"]["return_goal"]
        result["errors"] = {}
        for item in phases:
            name = item["name"]
            subset = [s for s in samples if s["phase"] == name]
            rows = [row for s in subset for row in s["series"]]
            stats = {}
            for index, label in ((1, "cooperative"), (2, "baseline")):
                values = [row[index] for row in rows if row[index] is not None]
                if values:
                    stats[label] = dict(count=len(values), mean_m=statistics.mean(values),
                                        max_m=max(values), stddev_m=statistics.pstdev(values),
                                        first_m=values[0], last_m=values[-1])
            sigma = [s["fused"][2] for s in subset if s.get("fused") and s["fused"][2] is not None]
            if sigma:
                stats["cooperative_sigma"] = dict(first_m=sigma[0], max_m=max(sigma), last_m=sigma[-1])
            result["errors"][name] = stats
        off = result["errors"]["corrections_off"]["cooperative_sigma"]
        on = result["errors"]["corrections_on"]["cooperative_sigma"]
        checks["uncertainty_grows_without_corrections"] = off["max_m"] > off["first_m"] * 1.5
        checks["uncertainty_recovers_with_corrections"] = on["last_m"] < off["max_m"] * 0.7
        limit_t = next(p["t"] for p in phases if p["name"] == "combined_faults")
        restore_t = next(p["t"] for p in phases if p["name"] == "sensors_restored")
        checks["combined_fault_guard_hold"] = any(e["mode"] == "HOLD_GUARD" and
            "uncertainty" in e["reason"].lower() and limit_t <= e["t"] < restore_t for e in navigation)
        return_start = next(e["t"] for e in events if e["kind"] == "goal" and
                            e["goal"] == result["configuration"]["return_goal"])
        checks["fault_hold_precedes_return_arrival"] = not any(
            e["mode"] == "GOAL" and return_start <= e["t"] < restore_t for e in navigation)
        held = next((s for s in samples if s["phase"] == "combined_faults" and
                     s["mode"] == "HOLD_GUARD" and "uncertainty" in s["reason"].lower()), None)
        if held:
            result["combined_hold"] = dict(t=held["t"], reason=held["reason"], truth=held["truth"],
                distance_to_return_goal_m=math.dist(held["truth"][:2], result["configuration"]["return_goal"]))
        result["duration"] = dict(wall_s=round(time.monotonic() - started_wall, 2), simulation_s=state["t"])
        result["success"] = all(bool(value) for value in checks.values())
        if not result["success"]:
            raise RuntimeError("A recorded check failed; inspect summary.json")
    except Exception as error:
        result["failure"] = str(error)
        raise
    finally:
        (output / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result.get("checks", {}), indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:8765")
    parser.add_argument("--output", type=Path, required=True, help="Fresh evidence directory")
    parser.add_argument("--timeout", type=float, default=450)
    run(parser.parse_args())


if __name__ == "__main__":
    main()
