"""Durable attempts, frozen inputs, and invalid-search evidence boundaries."""

import json
from pathlib import Path

import pytest

from uav_ugv_verification.experiments import Ledger, load_plan, objective
from uav_ugv_verification.validation import ValidationError


def test_matrix_has_explicit_pairs_and_disjoint_calibration_seeds():
    root = Path(__file__).resolve().parents[2]
    plan = load_plan(root / "verification/experiments/matrix.yaml", root)
    assert len(plan["cases"]) == 45
    navigation = [c for c in plan["cases"] if c["kind"] == "navigation"]
    assert len(navigation) == 39
    assert {c["seed"] for c in navigation} == {631, 632, 633}
    assert len({c["world"] for c in navigation}) == 4
    for outage in (c for c in navigation if c["outage"]):
        assert any(
            c["world"] == outage["world"]
            and c["goal"] == outage["goal"]
            and c["seed"] == outage["seed"]
            and not c["outage"]
            and c["assistance"]
            for c in navigation
        )


def test_interrupted_attempt_is_retained_and_resume_gets_new_identity(tmp_path):
    ledger = Ledger(tmp_path, {"cases": []}, "frozen-source")
    first = ledger.start({"case_id": "case-a"})
    ledger.close()
    resumed = Ledger(tmp_path, {"cases": []}, "frozen-source")
    resumed.recover()
    second = resumed.start({"case_id": "case-a"})
    resumed.finish(second["id"], "failed", {"reason": "witnessed R1 violation"})
    assert [r["status"] for r in resumed.rows()] == ["interrupted", "failed"]
    assert first["directory"] != second["directory"]
    assert resumed.attempted("case-a")
    with pytest.raises(ValidationError, match="already terminal"):
        resumed.finish(second["id"], "passed", {})
    assert (
        json.loads((tmp_path / "ledger.json").read_text())["attempts"][0]["status"] == "interrupted"
    )
    resumed.close()


@pytest.mark.parametrize("plan,source", [({"changed": True}, "old"), ({}, "changed")])
def test_changed_cohort_cannot_resume_existing_campaign(tmp_path, plan, source):
    Ledger(tmp_path, {}, "old").close()
    with pytest.raises(ValidationError, match="frozen inputs changed"):
        Ledger(tmp_path, plan, source)


@pytest.mark.parametrize(
    "verdict,missing,expected",
    [("PASS", 0, 0.25), ("FAIL", 0, -0.1), ("INCONCLUSIVE", 0, None), ("FAIL", 1, None)],
)
def test_search_never_rewards_missing_evidence(verdict, missing, expected):
    result = {
        "results": [
            {
                "requirement": "R1",
                "verdict": verdict,
                "missing_data_count": missing,
                "metrics": {
                    "by_interval": [{"minimum_clearance_m": -0.1 if verdict == "FAIL" else 0.25}]
                },
            }
        ]
    }
    assert objective(result) == expected


def test_publication_templates_are_frozen_but_generated_outputs_are_not(tmp_path):
    from uav_ugv_verification.experiments import source_snapshot

    (tmp_path / "verification/src/reporting/templates").mkdir(parents=True)
    for name in ("pyproject.toml", "requirements-tested.txt"):
        (tmp_path / "verification" / name).write_text("test")
    template = tmp_path / "verification/src/reporting/templates/paper.tex"
    template.write_text("first")
    before = source_snapshot(tmp_path)
    (tmp_path / "verification/outputs").mkdir()
    (tmp_path / "verification/outputs/paper.tex").write_text("generated")
    assert source_snapshot(tmp_path)["sha256"] == before["sha256"]
    template.write_text("changed")
    assert source_snapshot(tmp_path)["sha256"] != before["sha256"]


@pytest.mark.parametrize(
    "package,executable",
    [
        ("traversability_mapping", "traversability_mapper"),
        ("simulation_support", "obstacle_scenario"),
    ],
)
def test_extensionless_runtime_changes_invalidate_frozen_source(tmp_path, package, executable):
    from uav_ugv_verification.experiments import source_snapshot

    (tmp_path / "verification").mkdir()
    for name in ("pyproject.toml", "requirements-tested.txt"):
        (tmp_path / "verification" / name).write_text("test")
    script = tmp_path / "ros2_ws/src" / package / "scripts" / executable
    script.parent.mkdir(parents=True)
    script.write_text("#!/usr/bin/env python3\nprint('first')\n")
    before = source_snapshot(tmp_path)
    assert str(script.relative_to(tmp_path)) in before["files"]
    script.write_text("#!/usr/bin/env python3\nprint('changed')\n")
    assert source_snapshot(tmp_path)["sha256"] != before["sha256"]
