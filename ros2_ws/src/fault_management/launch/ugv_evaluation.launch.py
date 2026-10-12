"""Run explicitly owned simulation evaluation components."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from evaluation_launch import description  # noqa: E402


def generate_launch_description():
    return description(False)
