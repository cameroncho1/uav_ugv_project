"""Linux regression: timeout cleanup includes children that create new sessions."""

import importlib.util
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
SPEC = importlib.util.spec_from_file_location(
    "verification_experiments", ROOT / "scripts/verification_experiments.py"
)
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


def test_timeout_stops_owned_child_sessions(tmp_path):
    script = (
        "import subprocess,sys,time; "
        "child=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)'], start_new_session=True); "
        "print(child.pid,flush=True); time.sleep(60)"
    )
    result = module.run_process([sys.executable, "-c", script], os.environ.copy(), tmp_path, 1.0)
    assert result["timed_out"]
    assert result["duration_wall_s"] < 15
    child = int((tmp_path / "runner.log").read_text().splitlines()[0])
    status = Path(f"/proc/{child}/stat")
    assert not status.exists() or status.read_text().split()[2] == "Z"
