"""Atomic, finite, deterministic JSON and CSV artifacts."""

from __future__ import annotations

import csv
import io
import json
import os
import tempfile
from pathlib import Path

from ..contracts import Evaluation
from ..validation import finite_tree


def atomic_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            newline="",
            dir=path.parent,
            prefix=f".{path.name}.",
            delete=False,
        ) as handle:
            temporary = Path(handle.name)
            handle.write(content)
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def write_json(path: Path, value) -> None:
    finite_tree(value)
    atomic_text(path, json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n")


def write_csv(path: Path, rows: list[dict], fields: list[str]) -> None:
    buffer = io.StringIO(newline="")
    writer = csv.DictWriter(buffer, fieldnames=fields, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    atomic_text(path, buffer.getvalue())


def write_evaluation(result: Evaluation, output: str | Path) -> None:
    output = Path(output)
    write_json(output / "result.json", result.to_dict())
    rows = [
        {
            "run_id": result.run_id,
            "requirement": r.requirement,
            "version": r.version,
            "mandatory": r.mandatory,
            "verdict": r.verdict.value,
            "reason": r.reason,
            "eligible_duration_ns": r.eligible_duration_ns,
            "triggers": r.trigger_count,
            "completed_obligations": r.completed_obligations,
            "missing_data_count": r.missing_data_count,
        }
        for r in result.results
    ]
    write_csv(output / "requirements.csv", rows, list(rows[0]))
    (output / "error.json").unlink(missing_ok=True)
