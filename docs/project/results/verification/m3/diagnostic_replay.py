#!/usr/bin/env python3
"""Supplemental post-hoc replay of matrix failures using unchanged frozen code."""

import argparse
import fcntl
import json
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("campaign", type=Path)
    parser.add_argument("--repository", type=Path, required=True)
    parser.add_argument("--requirement", choices=[f"R{i}" for i in range(1, 9)], default="R2")
    parser.add_argument("--limit", type=int, default=2)
    args = parser.parse_args()
    if args.limit <= 0:
        parser.error("--limit must be positive")
    sys.path.insert(0, str(args.repository.resolve() / "scripts"))
    from uav_ugv_verification.experiments import Ledger, load_plan, source_snapshot
    from uav_ugv_verification.validation import require
    from verification_experiments import execute

    root = args.campaign.resolve()
    with (
        (root / "runner.lock").open("w") as lock,
        open("/tmp/uav-verification-px4.lock", "w") as port_lock,
    ):
        for handle in (lock, port_lock):
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        plan = load_plan(args.repository / "verification/experiments/matrix.yaml", args.repository)
        ledger = Ledger(root, plan, source_snapshot(args.repository)["sha256"])
        try:
            require(
                all(ledger.attempted(c["case_id"]) for c in plan["cases"]),
                "finish the fixed matrix before diagnostics",
            )
            ledger.recover()
            candidates = []
            for attempt in ledger.rows():
                if attempt["category"] != "matrix":
                    continue
                if any(
                    e["verdicts"].get(args.requirement) == "FAIL"
                    for e in attempt["detail"].get("evaluations", [])
                ):
                    candidates.append(attempt)
            selected = candidates[: args.limit]
            selection = dict(
                role="post_hoc_diagnostic",
                requirement=args.requirement,
                selection="earliest witnessed matrix failures by attempt ID",
                parent_attempts=[a["id"] for a in selected],
                held_out_matrix=False,
                search_budget=False,
            )
            (root / f"diagnostic_{args.requirement}_plan.json").write_text(
                json.dumps(selection, indent=2) + "\n"
            )
            for original in selected:
                case = {
                    **original["configuration"],
                    "case_id": f"diagnostic-{args.requirement}-a{original['id']}",
                    "parent_attempt": original["id"],
                    "replay_requirement": args.requirement,
                    "selection_role": "post_hoc_diagnostic",
                }
                if not ledger.attempted(case["case_id"], "diagnostic_replay"):
                    execute(ledger, case, plan["parameters"], category="diagnostic_replay")
            summary = []
            for attempt in ledger.rows():
                if attempt["category"] != "diagnostic_replay":
                    continue
                values = [
                    e["verdicts"].get(args.requirement)
                    for e in attempt["detail"].get("evaluations", [])
                ]
                outcome = (
                    "reproduced"
                    if "FAIL" in values
                    else "not_reproduced"
                    if values and all(v == "PASS" for v in values)
                    else "incomplete"
                )
                summary.append(
                    dict(
                        attempt=attempt["id"],
                        parent_attempt=attempt["configuration"]["parent_attempt"],
                        requirement=args.requirement,
                        outcome=outcome,
                    )
                )
            (root / "diagnostic_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
            print(json.dumps(summary))
        finally:
            ledger.close()


if __name__ == "__main__":
    main()
