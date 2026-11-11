#!/usr/bin/env python3
"""Generate source-bound supervisor paper material from saved proof/ROS evidence."""

import argparse
import json
import subprocess
from pathlib import Path

from uav_ugv_verification.reporting.files import write_json
from uav_ugv_verification.reporting.latex import compile_pdf
from uav_ugv_verification.reporting.publication import catalog
from uav_ugv_verification.reporting.supervisor import publish


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", type=Path)
    parser.add_argument("controlled", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--compile-pdf", action="store_true")
    args = parser.parse_args()
    try:
        result = publish(args.model, args.controlled, args.output)
        if args.compile_pdf:
            result["pdf_pages"] = compile_pdf(
                args.output, "paper_section", maximum_pages=2
            )
            write_json(args.output / "summary.json", result)
            catalog(args.output)
        print(json.dumps(result))
    except (OSError, ValueError, KeyError, subprocess.SubprocessError) as exc:
        print(json.dumps(dict(status="INVALID", error=str(exc))))
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
