"""Profiles bind only to an unambiguous recording clock, without changing bounds."""

import pytest
from conftest import command

from uav_ugv_verification.adapters.profile import bind_profile
from uav_ugv_verification.validation import ValidationError, load_profile


def test_bind_preserves_timing_thresholds(run_factory, profile_factory, tmp_path):
    run = run_factory(
        {"requested_command": [command(0), command(1_000_000_000)]}, clock="steady:boot-123"
    )
    template = profile_factory("R4", clock="steady:recording", parameters={"expiry_ns": 450000000})
    output = tmp_path / "bound.yaml"
    bind_profile(run, template, output)
    rule = load_profile(output).rules["R4"]
    assert rule.clock == "steady:boot-123"
    assert rule.parameters["expiry_ns"] == 450000000
    with pytest.raises(ValidationError, match="already exists"):
        bind_profile(run, template, output)


def test_sim_only_trace_cannot_bind_actuator_clock(run_factory, profile_factory, tmp_path):
    run = run_factory({"requested_command": [command(0)]})
    template = profile_factory("R4", clock="steady:recording")
    with pytest.raises(ValidationError, match="one verified steady"):
        bind_profile(run, template, tmp_path / "bound.yaml")
