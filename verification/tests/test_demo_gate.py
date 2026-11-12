"""A process-level success cannot override failed or incomplete verification."""

import importlib.util
import json
from pathlib import Path

import pytest

SPEC = importlib.util.spec_from_file_location(
    "verification_demo", Path(__file__).resolve().parents[2] / "scripts/verification_demo.py"
)
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


@pytest.mark.parametrize("code", [0, 1, 2])
def test_saved_requirement_exit_code_gates_demo_despite_passed_attempt(tmp_path, code):
    result = tmp_path / "recording/result.json"
    result.parent.mkdir()
    result.write_text(json.dumps({"exit_code": code}))
    campaign = dict(
        matrix_cases_attempted=1,
        matrix_cases_planned=1,
        missing_artifacts=[],
        attempt_counts={"passed": 1},
        recordings=[{"directory": "recording"}],
        plan={"cases": [{"kind": "navigation"}]},
    )
    assert module.verdict(campaign, tmp_path) == code
