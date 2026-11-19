"""Recorded hazard coverage must agree with the planned and observed operations."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

SCRIPT = Path(__file__).resolve().parents[2] / "scripts/run_dynamic_obstacle_trial.py"
SPEC = importlib.util.spec_from_file_location("dynamic_trial", SCRIPT)
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


def outcome(recorded, *, failed=False, actor_status="completed"):
    return module.acceptance(
        {"a", "b"},
        {"returncode": 0, "timed_out": False},
        {"status": actor_status},
        {"error": None, "observations": [dict(event_id=v, qualifying=True) for v in ("a", "b")]},
        SimpleNamespace(
            exit_code=1 if failed else 0,
            results=[SimpleNamespace(requirement="R5", trigger_count=len(recorded))],
        ),
        recorded,
    )


def test_every_scheduled_observation_must_exist_in_recorded_evidence():
    assert outcome(["a", "b"])["exit_code"] == 0
    assert outcome(["b"])["exit_code"] == 2  # Losing the first event has no preceding sequence gap.
    assert outcome(["a", "a"])["exit_code"] == 2


def test_known_requirement_failure_keeps_precedence_over_incomplete_execution():
    assert outcome(["b"], failed=True, actor_status="failed")["exit_code"] == 1
