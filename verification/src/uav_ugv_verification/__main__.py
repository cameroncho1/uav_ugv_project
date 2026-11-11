"""Inspect recordings, evaluate requirements, and report saved results."""

from __future__ import annotations

import argparse
import json
import sqlite3
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from .evaluation import evaluate, inspect_run
from .reporting import report, write_evaluation
from .reporting.files import write_json
from .validation import ValidationError


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subcommands = parser.add_subparsers(dest="command", required=True)
    for name in ("inspect", "evaluate"):
        command = subcommands.add_parser(name)
        command.add_argument("run", type=Path)
        command.add_argument("--profile", type=Path, required=True)
        if name == "evaluate":
            command.add_argument("--output", type=Path, required=True)
    command = subcommands.add_parser("report")
    command.add_argument("results", type=Path)
    command.add_argument("--output", type=Path, required=True)
    command = subcommands.add_parser("import-bag")
    command.add_argument("bag", type=Path)
    command.add_argument("--setup", type=Path, required=True)
    command.add_argument("--output", type=Path, required=True)
    command.add_argument("--status", choices=("completed", "aborted", "failed"), required=True)
    command.add_argument("--world", type=Path)
    command.add_argument("--events", type=Path)
    command.add_argument("--start-ns", type=int)
    command.add_argument("--end-ns", type=int)
    command = subcommands.add_parser("bind-profile")
    command.add_argument("run", type=Path)
    command.add_argument("--template", type=Path, required=True)
    command.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == "bind-profile":
            from .adapters.profile import bind_profile

            value = bind_profile(args.run, args.template, args.output)
            exit_code = 0
        elif args.command == "import-bag":
            from .adapters.rosbag import import_bag

            manifest = import_bag(
                args.bag,
                args.setup,
                args.output,
                status=args.status,
                world=args.world,
                events=args.events,
                start_ns=args.start_ns,
                end_ns=args.end_ns,
            )
            value = {"run_id": manifest["run_id"], "streams": sorted(manifest["streams"])}
            exit_code = 0
        elif args.command == "inspect":
            value = inspect_run(args.run, args.profile)
            exit_code = 0
        elif args.command == "evaluate":
            result = evaluate(args.run, args.profile)
            write_evaluation(result, args.output)
            value = {
                "run_id": result.run_id,
                "exit_code": result.exit_code,
                "reason": result.reason,
                "verdicts": {r.requirement: r.verdict.value for r in result.results},
            }
            exit_code = result.exit_code
        else:
            value = report(args.results, args.output)
            exit_code = 0
    except (
        ValidationError,
        OSError,
        UnicodeError,
        ValueError,
        KeyError,
        sqlite3.DatabaseError,
        ET.ParseError,
        yaml.YAMLError,
    ) as exc:
        value = {"schema_version": 1, "status": "invalid_input", "exit_code": 2, "reason": str(exc)}
        if args.command == "evaluate":
            try:
                write_json(args.output / "error.json", value)
                # A previous result must not masquerade as this failed evaluation.
                (args.output / "result.json").unlink(missing_ok=True)
                (args.output / "requirements.csv").unlink(missing_ok=True)
            except OSError as artifact_error:
                value["artifact_error"] = str(artifact_error)
        exit_code = 2
    print(json.dumps(value, sort_keys=True, allow_nan=False))
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
