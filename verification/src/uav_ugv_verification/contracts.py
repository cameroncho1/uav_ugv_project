"""Stable input and result types. All evaluation times are integer nanoseconds."""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any, Protocol

SCHEMA_VERSION = 1
EVALUATOR_VERSION = "0.1.0"
REQUIREMENTS = tuple(f"R{i}" for i in range(1, 9))


class Verdict(str, Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    INCONCLUSIVE = "INCONCLUSIVE"
    NOT_APPLICABLE = "NOT_APPLICABLE"


@dataclass(frozen=True)
class Interval:
    clock: str
    epoch: int
    start_ns: int
    end_ns: int


@dataclass(frozen=True)
class Record:
    stream: str
    time_ns: int
    clock: str
    epoch: int
    source: str
    sequence: int
    data: dict[str, Any]
    valid: bool = True
    frame: str | None = None
    source_sequence: int | None = None


@dataclass(frozen=True)
class Manifest:
    run_id: str
    source_run_id: str
    adapter: str
    status: str
    intervals: tuple[Interval, ...]
    streams: dict[str, str]
    provenance: dict[str, Any]
    frame: str = "map"
    reference_point: str = "model_origin"


@dataclass(frozen=True)
class Rule:
    requirement: str
    applicable: bool
    mandatory: bool
    clock: str
    max_gap_ns: int
    parameters: dict[str, Any]
    reason: str = ""


@dataclass(frozen=True)
class Profile:
    profile_id: str
    version: str
    calibration: str
    rules: dict[str, Rule]
    sha256: str


class TraceReader(Protocol):
    manifest: Manifest

    @property
    def streams(self) -> frozenset[str]: ...

    def records(self, stream: str, interval: Interval) -> Iterator[Record]:
        """Yield ordered records in the interval's clock/epoch, including prehistory."""
        ...

    def inspection(self) -> dict[str, Any]: ...


@dataclass
class RequirementResult:
    requirement: str
    version: str
    mandatory: bool
    verdict: Verdict
    reason: str
    parameters: dict[str, Any] = field(default_factory=dict)
    intervals: list[dict[str, Any]] = field(default_factory=list)
    eligible_duration_ns: int = 0
    trigger_count: int = 0
    completed_obligations: int = 0
    missing_data_count: int = 0
    witnesses: list[dict[str, Any]] = field(default_factory=list)
    metrics: dict[str, Any] = field(default_factory=dict)


@dataclass
class Evaluation:
    run_id: str
    source_run_id: str
    profile_id: str
    profile_version: str
    profile_sha256: str
    calibration: str
    execution_status: str
    provenance: dict[str, Any]
    source_sha256: dict[str, str]
    results: list[RequirementResult]
    exit_code: int
    reason: str
    schema_version: int = SCHEMA_VERSION
    evaluator_version: str = EVALUATOR_VERSION
    evaluator_sha256: str = ""

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)
