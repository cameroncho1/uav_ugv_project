"""Arrival completion and log coverage rules reproduce the audited failures."""

import sys
from pathlib import Path
import subprocess
import shlex
import time
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "scripts"))
from evaluation_math import ArrivalDwell, matched_heading, cross_track  # noqa: E402


def test_deadline_during_arrival_dwell_cannot_pass():
    dwell = ArrivalDwell()
    assert not dwell.update(.2, True)
    assert not dwell.update(.3, True)
    assert dwell.update(1.3, True)
    assert not dwell.update(1.4, False)
    assert not dwell.update(1.5, True)


def test_heading_endpoints_are_not_extrapolated_and_gaps_are_rejected():
    good, _, _, skipped = matched_heading([.9, 1., 1.05, 1.5, 2., 2.1],
                                          [0.] * 6, [1., 1.1, 2.], [0.] * 3)
    np.testing.assert_array_equal(good, [False, True, True, False, True, False])
    assert skipped == {"outside_estimate_coverage": 2, "estimate_gap": 1}


@pytest.mark.parametrize("times", [[], [1.], [1., 1.], [2., 1.], [1., float("nan")]])
def test_bad_heading_logs_rejected(times):
    with pytest.raises(ValueError):
        matched_heading([1.], [0.], times, [0.] * len(times))


def test_polyline_tracking_uses_segments_and_bends():
    assert cross_track([1., .5], [[0., 0.], [2., 0.]]) == .5
    assert cross_track([2.5, 1.], [[0., 0.], [2., 0.], [2., 2.]]) == .5


def test_cleanup_terminates_an_interrupt_resistant_process_group():
    script = ROOT / "scripts/process_cleanup.sh"
    command = "source \"" + str(script) + "\"; setsid bash -c 'trap \"\" INT TERM; while :; do sleep 1; done' & child=$!; sleep .2; stop_process_groups \"$child\""
    started = time.monotonic()
    subprocess.run(["bash", "-c", command], check=True, timeout=9)
    assert time.monotonic() - started < 8


def test_cleanup_includes_descendant_with_its_own_session(tmp_path):
    pid_file = tmp_path / "descendant.pid"
    resistant = 'trap "" INT TERM; while :; do sleep 1; done'
    parent = ('trap "" INT TERM; setsid bash -c ' + shlex.quote(resistant)
              + ' & echo $! > ' + shlex.quote(str(pid_file)) + '; wait')
    command = ('source ' + shlex.quote(str(ROOT / "scripts/process_cleanup.sh"))
               + '; setsid bash -c ' + shlex.quote(parent)
               + ' & child=$!; sleep .3; stop_process_groups "$child"')
    subprocess.run(["bash", "-c", command], check=True, timeout=9)
    status = Path("/proc") / pid_file.read_text().strip() / "status"
    assert not status.exists() or "State:\tZ" in status.read_text()
