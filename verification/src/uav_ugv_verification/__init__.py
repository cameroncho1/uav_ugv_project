"""Numerical verification without ROS or simulator imports."""

from .contracts import Evaluation, Profile, Record, RequirementResult, TraceReader, Verdict
from .evaluation import evaluate, evaluate_trace, inspect_run
from .validation import ValidationError, load_profile

__all__ = [
    "Evaluation",
    "RequirementResult",
    "Profile",
    "Record",
    "TraceReader",
    "ValidationError",
    "Verdict",
    "evaluate",
    "evaluate_trace",
    "inspect_run",
    "load_profile",
]
__version__ = "0.1.0"
