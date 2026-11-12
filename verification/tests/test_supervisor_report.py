"""Saved supervisor evidence reproduces tables and rejects mixed source revisions."""

import json
import shutil
from pathlib import Path

import pytest

from uav_ugv_verification.reporting.supervisor import publish
from uav_ugv_verification.validation import ValidationError

EVIDENCE = Path(__file__).resolve().parents[2] / "docs/project/results/supervisor"


def test_saved_supervisor_evidence_reproduces_paper_tables(tmp_path):
    output = tmp_path / "paper"
    summary = publish(EVIDENCE / "model.json", EVIDENCE / "controlled", output)
    assert (summary["transitions"], summary["properties"], summary["detected_mutations"]) == (
        7120,
        11,
        9,
    )
    assert summary["controlled_cases"] == 29
    for name in ("properties.csv", "mutations.csv", "cases.csv", "paper_section.md"):
        assert (output / name).read_bytes() == (EVIDENCE / "paper" / name).read_bytes()
    assert len(list((output / "figures").glob("*.svg"))) == 3


def test_supervisor_report_rejects_mixed_implementation_evidence(tmp_path):
    controlled = tmp_path / "controlled"
    shutil.copytree(EVIDENCE / "controlled", controlled)
    path = controlled / "verification/recording.json"
    recording = json.loads(path.read_text())
    source = "ros2_ws/src/ugv_control/src/goal_controller.cpp"
    recording["source_sha256"][source] = "0" * 64
    path.write_text(json.dumps(recording))
    with pytest.raises(ValidationError, match="different implementation sources"):
        publish(EVIDENCE / "model.json", controlled, tmp_path / "invalid")
    assert not (tmp_path / "invalid").exists()
