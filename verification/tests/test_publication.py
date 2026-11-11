"""Portable publication uses saved verdicts and retains unsuccessful denominators."""

import json
import shutil
from pathlib import Path

import pytest

from uav_ugv_verification.reporting.publication import publish
from uav_ugv_verification.validation import ValidationError, digest_file


@pytest.fixture
def campaign(tmp_path):
    root = tmp_path / "campaign"
    directory = root / "recordings/guard"
    directory.mkdir(parents=True)
    actual = (
        Path(__file__).resolve().parents[2]
        / "docs/project/results/verification/m2/guard/result.json"
    )
    shutil.copyfile(actual, directory / "result.json")
    attempts = [
        dict(
            id=1,
            case_id="guard",
            category="matrix",
            status="passed",
            configuration={"kind": "guard"},
            detail={},
        ),
        dict(
            id=2,
            case_id="startup",
            category="matrix",
            status="startup_failed",
            configuration={},
            detail={},
        ),
    ]
    ledger = dict(source_sha256="test-source", attempts=attempts)
    metadata = dict(
        source_sha256="test-source",
        matrix_cases_attempted=2,
        matrix_cases_planned=2,
        attempt_counts={"passed": 1, "startup_failed": 1},
        recordings=[
            dict(
                attempt=1,
                category="matrix",
                case_id="guard",
                configuration={"kind": "guard"},
                directory="recordings/guard",
                result_sha256=digest_file(directory / "result.json"),
            )
        ],
        search_attempts=0,
        search_wall_s=0,
        missing_artifacts=[],
    )
    (root / "campaign.json").write_text(json.dumps(metadata))
    (root / "ledger.json").write_text(json.dumps(ledger))
    return root


def test_saved_verdicts_report_without_raw_bags_and_include_failed_startup(campaign, tmp_path):
    first = tmp_path / "report-a"
    second = tmp_path / "report-b"
    summary = publish(campaign, first, plots=False)
    publish(campaign, second, plots=False)
    assert summary["attempts"] == 2 and summary["recordings"] == 1
    assert summary["attempt_counts"]["startup_failed"] == 1
    assert next(r for r in summary["claims"] if r["requirement"] == "R3")["PASS"] == 1
    assert (first / "metrics.csv").read_bytes() == (second / "metrics.csv").read_bytes()
    assert b"\r" not in (first / "metrics.csv").read_bytes()
    assert (first / "paper.tex").read_bytes() == (second / "paper.tex").read_bytes()


def test_publication_rejects_changed_saved_verdict(campaign, tmp_path):
    with (campaign / "recordings/guard/result.json").open("a") as handle:
        handle.write(" ")
    with pytest.raises(ValidationError, match="checksum mismatch"):
        publish(campaign, tmp_path / "report", plots=False)


def test_publication_rejects_changed_display_series(campaign, tmp_path):
    directory = campaign / "recordings/guard"
    series = directory / "series.json"
    series.write_text('{"streams": {}}')
    path = campaign / "campaign.json"
    value = json.loads(path.read_text())
    value["recordings"][0]["series_sha256"] = digest_file(series)
    path.write_text(json.dumps(value))
    series.write_text('{"streams": {}, "changed": true}')
    with pytest.raises(ValidationError, match="figure input checksum mismatch"):
        publish(campaign, tmp_path / "report", plots=False)


def test_assistance_comparison_retains_missing_startup_evidence():
    from uav_ugv_verification.reporting.publication import _comparisons

    config = dict(kind="navigation", world="empty.sdf", goal=[4.0, 0.0], outage=False, seed=1)
    attempts = [
        dict(
            id=1,
            category="matrix",
            status="startup_failed",
            configuration={**config, "assistance": True},
        ),
        dict(
            id=2, category="matrix", status="passed", configuration={**config, "assistance": False}
        ),
    ]
    rows = [dict(attempt=2, R2="PASS", arrival_s=10.0)]
    comparison = _comparisons({"attempts": attempts}, rows)[0]
    assert comparison["assisted_R2"] == "NO_RESULT"
    assert comparison["assisted_arrival_s"] is None
    assert comparison["unassisted_R2"] == "PASS"
    assert comparison["unassisted_arrival_s"] == 10.0


def test_partial_campaign_figures_export_without_a_display(campaign, tmp_path):
    output = tmp_path / "report"
    publish(campaign, output)
    assert len(list((output / "figures").glob("*.pdf"))) == 8
    assert len(list((output / "figures").glob("*.svg"))) == 8
    assert (output / "bounds.csv").is_file()
    assert "R5, R6, R7, R8" in (output / "paper.md").read_text(encoding="utf-8")
