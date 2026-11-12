"""Regressions for assistance ablations and slow/frozen acquisition clocks."""

import importlib.util
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[2] / "scripts/evaluation_math.py"
SPEC = importlib.util.spec_from_file_location("evaluation_math", SCRIPT)
math = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(math)


def test_unassisted_arrival_does_not_require_unused_uav_inputs():
    data = dict(
        wheel_count=200,
        relative_count=0,
        correction_count=0,
        gps_count=50,
        initial_gps_enabled=True,
        outage_applied=True,
    )
    assert all(math.navigation_evidence(assisted=False, **data).values())
    assert not all(math.navigation_evidence(assisted=True, **data).values())


def test_arrival_dwell_requires_advancing_simulation_time():
    dwell = math.ArrivalDwell(1.0)
    assert not dwell.update(10.0, True)
    for _ in range(100):
        assert not dwell.update(10.0, True)
    assert not dwell.update(10.99, True)
    assert dwell.update(11.0, True)


def test_outage_service_ack_is_not_evidence_of_application():
    checks = math.navigation_evidence(
        assisted=False,
        wheel_count=200,
        relative_count=0,
        correction_count=0,
        gps_count=50,
        initial_gps_enabled=True,
    )
    assert checks["gps"] and not checks["outage_applied"]
