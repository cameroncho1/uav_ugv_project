#!/usr/bin/env python3
"""Interrupt a headless live demo after recording starts and verify owned cleanup."""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

from uav_ugv_verification.reporting.files import write_json
from uav_ugv_verification.validation import require
from verification_experiments import parent_death_signal, process_groups, stop

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="Fresh output directory")
    args = parser.parse_args()
    output = args.output.resolve()
    require(not output.exists(), "choose a new interruption-test directory")
    output.mkdir(parents=True)
    command = [
        sys.executable,
        str(ROOT / "scripts/verification_demo.py"),
        str(output / "demo"),
    ]
    groups = set()
    started = time.monotonic()
    with (output / "demo.log").open("w") as log:
        process = subprocess.Popen(
            command,
            cwd=ROOT,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            preexec_fn=parent_death_signal,  # noqa: PLW1509 - single-threaded Linux launcher
        )
        try:
            deadline = started + 150
            while not list(
                (output / "demo/raw/attempts").glob("*/run/verification/bag/*.db3")
            ):
                groups |= process_groups(process.pid)
                require(process.poll() is None, "demo exited before recording started")
                require(
                    time.monotonic() < deadline,
                    "recording startup exceeded 150 wall seconds",
                )
                time.sleep(0.1)
            groups |= process_groups(process.pid)
            interrupted_at = time.monotonic()
            os.kill(process.pid, signal.SIGINT)
            process.wait(timeout=65)
            cleanup_wall_s = time.monotonic() - interrupted_at
            live = [
                line.split(maxsplit=2)
                for line in subprocess.check_output(
                    ["ps", "-eo", "pgid=,stat=,args="],
                    text=True,
                ).splitlines()
            ]
            remaining = [
                row
                for row in live
                if int(row[0]) in groups and not row[1].startswith("Z")
            ]
            ledger = json.loads((output / "demo/raw/ledger.json").read_text())
            statuses = [a["status"] for a in ledger["attempts"]]
            summary = json.loads((output / "demo/demo_summary.json").read_text())
            recordings = list(
                (output / "demo/raw/attempts").glob("*/run/verification/recording.json")
            )
            result = {
                "command": command,
                "returncode": process.returncode,
                "interruption": "SIGINT to demo coordinator after bag creation",
                "owned_process_groups": sorted(groups),
                "remaining_owned_processes": remaining,
                "attempt_statuses": statuses,
                "finalized_recordings": len(recordings),
                "demo_summary": summary,
                "cleanup_wall_s": cleanup_wall_s,
                "duration_wall_s": time.monotonic() - started,
            }
            write_json(output / "interruption_result.json", result)
            require(
                process.returncode == 130, "demo did not retain cancellation status"
            )
            require(
                not remaining, "owned simulation processes remain after cancellation"
            )
            require(
                "interrupted" in statuses and "running" not in statuses,
                "durable ledger did not retain interrupted attempt",
            )
            require(bool(recordings), "recorder did not finalize its metadata")
            print(json.dumps(result, sort_keys=True))
        finally:
            stop(process, groups)


if __name__ == "__main__":
    main()
