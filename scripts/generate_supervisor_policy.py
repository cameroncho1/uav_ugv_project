#!/usr/bin/env python3
"""Compile the shared navigation policy into a dependency-free C++ header."""

import argparse
import hashlib
import json
from pathlib import Path

CONDITIONS = {
    "no_goal": "NoGoal",
    "fault_latched": "FaultLatched",
    "invalid_pose": "InvalidPose",
    "goal_completed": "GoalCompleted",
    "invalid_route": "InvalidRoute",
    "always": "Always",
}
MODES = {
    "INIT": "Init",
    "NAVIGATE": "Navigate",
    "HOLD_GUARD": "HoldGuard",
    "HOLD_FAULT": "HoldFault",
    "GOAL": "Goal",
}


def generate(policy: Path, output: Path):
    raw = policy.read_bytes()
    data = json.loads(raw)
    if data["schema_version"] != 1 or not data["id"]:
        raise ValueError("unsupported supervisor policy")
    rows = data["decisions"]
    if len(rows) != len(CONDITIONS) or {r["when"] for r in rows} != set(CONDITIONS):
        raise ValueError("each decision condition must occur exactly once")
    if rows[-1]["when"] != "always":
        raise ValueError("the unconditional decision must be last")
    recovery = data["recovery"]
    for key, value in recovery.items():
        if type(value) is not int or value <= 0 or value > 10**12:
            raise ValueError(f"invalid recovery parameter: {key}")
    if recovery["clear_windows"] > 100:
        raise ValueError("clear-window state space exceeds the supported bound")
    rules = ",\n".join(
        "  {Condition::" + CONDITIONS[r["when"]] + ", Mode::" + MODES[r["mode"]] + "}"
        for r in rows
    )
    repository = Path(__file__).resolve().parents[1]
    names = [
        "include/ugv_control/supervisor.hpp",
        "include/ugv_control/supervisor_inputs.hpp",
        "src/supervisor_transition_probe.cpp",
        "src/goal_controller.cpp",
    ]
    hashes = {
        "ros2_ws/src/ugv_control/" + name: hashlib.sha256(
            (repository / "ros2_ws/src/ugv_control" / name).read_bytes()
        ).hexdigest()
        for name in names
    }
    hashes["scripts/generate_supervisor_policy.py"] = hashlib.sha256(
        Path(__file__).read_bytes()
    ).hexdigest()
    sources_hash = hashlib.sha256(
        json.dumps(hashes, sort_keys=True).encode()
    ).hexdigest()
    text = f'''// Generated from verification/supervisor/policy.json. Do not edit.
#ifndef UGV_CONTROL__SUPERVISOR_POLICY_HPP_
#define UGV_CONTROL__SUPERVISOR_POLICY_HPP_
namespace ugv_control
{{
inline constexpr char kSupervisorPolicyId[] = "{data["id"]}";
inline constexpr char kSupervisorPolicySha256[] = "{hashlib.sha256(raw).hexdigest()}";
inline constexpr char kSupervisorSourcesSha256[] = "{sources_hash}";
inline constexpr unsigned kClearWindows = {recovery["clear_windows"]};
inline constexpr int64_t kMinimumClearNs = {recovery["minimum_clear_ns"]}LL;
inline constexpr int64_t kMaximumDecisionAgeNs =
  {recovery["maximum_decision_age_ns"]}LL;
inline constexpr int64_t kMaximumDecisionGapNs =
  {recovery["maximum_decision_gap_ns"]}LL;
inline constexpr Rule kSupervisorRules[] = {{
{rules}
}};
}}  // namespace ugv_control
#endif
'''
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text, encoding="utf-8", newline="\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    generate(args.policy, args.output)


if __name__ == "__main__":
    main()
