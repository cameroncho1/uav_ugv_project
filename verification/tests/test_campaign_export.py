"""Campaign exports keep unsuccessful attempts and reject escaped evidence paths."""

import json

import pytest

from uav_ugv_verification.campaign import export_campaign
from uav_ugv_verification.experiments import Ledger
from uav_ugv_verification.validation import ValidationError


def test_export_retains_failed_startup_and_missing_artifact(tmp_path):
    source = tmp_path / "raw"
    plan = {"cases": [{"case_id": "a"}, {"case_id": "b"}]}
    ledger = Ledger(source, plan, "frozen")
    first = ledger.start({"case_id": "a"})
    ledger.finish(first["id"], "startup_failed", {"returncode": 2})
    second = ledger.start({"case_id": "b"})
    ledger.finish(
        second["id"],
        "failed",
        {"evaluations": [{"result_path": str(source / "attempts/missing/result.json")}]},
    )
    (source / "freeze.json").write_text(json.dumps({"source": "frozen"}))
    summary = export_campaign(source, tmp_path / "portable")
    assert summary["attempt_counts"] == {"startup_failed": 1, "failed": 1}
    assert summary["matrix_cases_attempted"] == 2
    assert len(summary["missing_artifacts"]) == 1
    assert not summary["recordings"]
    ledger.close()


def test_export_rejects_evidence_outside_campaign(tmp_path):
    source = tmp_path / "raw"
    ledger = Ledger(source, {"cases": []}, "frozen")
    first = ledger.start({"case_id": "escape"})
    ledger.finish(
        first["id"],
        "failed",
        {"evaluations": [{"result_path": str(tmp_path / "other-project/result.json")}]},
    )
    (source / "freeze.json").write_text("{}")
    with pytest.raises(ValidationError, match="escapes campaign"):
        export_campaign(source, tmp_path / "portable")
    ledger.close()
