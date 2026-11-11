#!/usr/bin/env python3
"""Check reachable supervisor states, temporal safety, and compiled C++ conformance."""

import argparse
import json
import subprocess
from pathlib import Path

from uav_ugv_verification.reporting.files import write_json
from uav_ugv_verification.supervisor import check

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--policy", type=Path, default=ROOT / "verification/supervisor/policy.json"
    )
    parser.add_argument(
        "--probe", type=Path, help="Compiled supervisor_transition_probe executable"
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        result = check(args.policy, args.probe, repository=ROOT)
    except (OSError, ValueError, KeyError, subprocess.SubprocessError) as exc:
        result = {"status": "INVALID", "error": str(exc)}
    write_json(args.output, result)
    print(
        json.dumps(
            {
                k: result[k]
                for k in (
                    "status",
                    "controller_states",
                    "product_states",
                    "transitions",
                    "conformance",
                    "error",
                )
                if k in result
            }
        )
    )
    return 0 if result["status"] == "PASS" else 1 if result["status"] == "FAIL" else 2


if __name__ == "__main__":
    raise SystemExit(main())
