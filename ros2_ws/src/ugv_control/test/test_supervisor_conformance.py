"""Registered Linux check compares every reachable transition with compiled C++."""

import os
from pathlib import Path

from uav_ugv_verification.supervisor import check


def test_compiled_supervisor_conforms_over_reachable_graph():
    root = Path(__file__).resolve().parents[4]
    result = check(root / "verification/supervisor/policy.json", Path(os.environ["SUPERVISOR_PROBE"]))
    assert result["status"] == result["conformance"]["status"] == "PASS"
    assert result["conformance"]["transitions"] == result["transitions"]
