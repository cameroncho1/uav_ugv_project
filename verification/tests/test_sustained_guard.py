"""A completed response deadline does not end the required zero-command hold."""

import pytest
from conftest import NS, command, guard

from uav_ugv_verification import evaluate


@pytest.mark.parametrize("resume,verdict", [(False, "PASS"), (True, "FAIL")])
def test_repeated_zero_commands_after_response_deadline(
    run_factory, profile_factory, resume, verdict
):
    ticks = range(0, 3 * NS + 1, NS // 10)
    run = run_factory(
        {
            "guard_state": [guard(t, False) for t in ticks],
            "requested_command": [
                command(t, linear=0.2 if resume and t == 2 * NS else 0.0) for t in ticks
            ],
        }
    )
    result = evaluate(run, profile_factory("R3", gap=NS // 4))
    assert result.results[2].verdict.value == verdict


def test_observed_response_needs_no_command_before_first_emission(run_factory, profile_factory):
    ticks = range(0, 3 * NS + 1, NS // 10)
    run = run_factory(
        {
            "guard_state": [guard(t, False) for t in ticks],
            "requested_command": [command(t) for t in ticks if t > 0],
        }
    )
    result = evaluate(run, profile_factory("R3", gap=NS // 4))
    assert result.results[2].verdict.value == "PASS"
