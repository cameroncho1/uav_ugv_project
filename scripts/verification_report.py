#!/usr/bin/env python3
"""Regenerate paper tables and figures from a portable campaign export."""

import argparse
import json
import re
import subprocess
from pathlib import Path

from uav_ugv_verification.reporting.publication import catalog, publish


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("campaign", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--compile-pdf", action="store_true")
    args = parser.parse_args()
    try:
        generate(args)
    except (
        OSError,
        ValueError,
        KeyError,
        RuntimeError,
        subprocess.SubprocessError,
    ) as exc:
        print(json.dumps({"exit_code": 2, "error": str(exc)}))
        return 2
    return 0


def generate(args):
    summary = publish(args.campaign.resolve(), args.output.resolve())
    if args.compile_pdf:
        command = [
            "pdflatex",
            "-no-shell-escape",
            "-interaction=nonstopmode",
            "-halt-on-error",
        ]
        version = subprocess.check_output(["pdflatex", "--version"], text=True)
        if "MiKTeX" in version:
            command.append("--disable-installer")
        with (args.output / "latex.log").open("w") as log:
            subprocess.run(
                [*command, "paper.tex"],
                cwd=args.output,
                stdout=log,
                stderr=subprocess.STDOUT,
                check=True,
                timeout=120,
            )
        pages = re.search(
            r"Output written on paper.pdf \((\d+) pages?",
            (args.output / "paper.log").read_text(errors="replace"),
        )
        if pages and int(pages.group(1)) > 10:
            raise RuntimeError(
                "Generated verification report exceeds the ten-page limit"
            )
        catalog(args.output)
    print(
        json.dumps(
            {
                key: summary[key]
                for key in (
                    "attempts",
                    "recordings",
                    "attempt_counts",
                    "search_trials",
                    "replay_attempts",
                )
            }
        )
    )


if __name__ == "__main__":
    raise SystemExit(main())
