"""Dirty source snapshots cannot be pooled as the same simulator implementation."""

from copy import deepcopy

from conftest import command, guard

from uav_ugv_verification import evaluate
from uav_ugv_verification.reporting.summary import _cohort


def test_different_recorded_source_snapshots_separate_cohorts(run_factory, profile_factory):
    run = run_factory({"guard_state": [guard(0, False)], "requested_command": [command(0)]})
    first = evaluate(run, profile_factory()).to_dict()
    first["provenance"]["source_snapshot_sha256"] = "source-a"
    second = deepcopy(first)
    second["provenance"]["source_snapshot_sha256"] = "source-b"
    assert _cohort(first) != _cohort(second)
    assert _cohort(first) == _cohort(deepcopy(first))
