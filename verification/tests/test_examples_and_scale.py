import json
import tracemalloc
from pathlib import Path

import pytest

from uav_ugv_verification import evaluate, load_profile
from uav_ugv_verification.contracts import Interval, Record
from uav_ugv_verification.signals.timeline import align, ordered_after


def test_committed_example_exercises_all_requirements():
    examples = Path(__file__).parents[1] / "examples"
    checked = evaluate(examples / "full_run", examples / "full_run.yaml")
    expected = json.loads((examples / "expected.json").read_text())
    assert checked.exit_code == expected["exit_code"]
    assert {r.requirement: r.verdict.value for r in checked.results} == expected["verdicts"]
    assert len(checked.evaluator_sha256) == 64
    assert checked.provenance["kind"] == "synthetic"


def test_all_standard_profiles_are_valid():
    for path in (Path(__file__).parents[1] / "profiles").glob("*.yaml"):
        assert load_profile(path).calibration == "provisional"


@pytest.mark.parametrize("start", [0, 9000])
def test_stream_alignment_does_not_materialize_long_traces(start):
    class GeneratedReader:
        def records(self, stream, interval):
            for index in range(10001):
                yield Record(stream, index, "sim", 0, stream, index, {"value": index})

    tracemalloc.start()
    try:
        snapshots, duration = 0, 0
        for snapshot in align(
            GeneratedReader(), ("raw.a", "raw.b"), Interval("sim", 0, start, 10000), 10
        ):
            snapshots += 1
            duration += snapshot.duration_ns
        _, peak = tracemalloc.get_traced_memory()
    finally:
        tracemalloc.stop()
    assert snapshots == 10001 - start
    assert duration == 10000 - start
    assert peak < 2_000_000


def test_stream_counters_do_not_order_cross_stream_ties():
    trigger = Record("events", 10, "sim", 0, "source", 2, {})
    response = Record("route", 10, "sim", 0, "source", 100, {})
    assert ordered_after(response, trigger) is None


def test_explicit_source_counter_resolves_equal_time_order():
    trigger = Record("events", 10, "sim", 0, "source", 2, {}, source_sequence=5)
    response = Record("route", 10, "sim", 0, "source", 0, {}, source_sequence=6)
    assert ordered_after(response, trigger) is True
    assert ordered_after(trigger, response) is False
