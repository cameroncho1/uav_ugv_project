"""Strict file/schema validation; errors never become passing evaluations."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .contracts import REQUIREMENTS, SCHEMA_VERSION, Interval, Manifest, Profile, Record, Rule


class ValidationError(ValueError):
    """Invalid configuration or recording, with a source-specific explanation."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValidationError(message)


def finite_tree(value: Any, context: str = "document") -> None:
    if isinstance(value, float):
        require(math.isfinite(value), f"{context}: nonfinite number")
    elif isinstance(value, dict):
        for key, child in value.items():
            require(isinstance(key, str), f"{context}: object keys must be strings")
            finite_tree(child, f"{context}.{key}")
    elif isinstance(value, list):
        for child in value:
            finite_tree(child, context)
    else:
        require(
            value is None or isinstance(value, (str, int, bool)),
            f"{context}: unsupported value type",
        )


def mapping(value: Any, context: str) -> dict:
    require(isinstance(value, dict), f"{context}: expected an object")
    return value


def keys(value: dict, allowed: set[str], required: set[str], context: str) -> None:
    require(not (set(value) - allowed), f"{context}: unknown fields {sorted(set(value) - allowed)}")
    require(required <= set(value), f"{context}: missing fields {sorted(required - set(value))}")


def integer(value: Any, context: str, minimum: int = 0) -> int:
    require(type(value) is int and value >= minimum, f"{context}: expected integer >= {minimum}")
    return value


def number(value: Any, context: str, minimum: float | None = None) -> float:
    require(
        type(value) in (int, float) and math.isfinite(value), f"{context}: expected finite number"
    )
    require(minimum is None or value >= minimum, f"{context}: must be >= {minimum}")
    return float(value)


def text(value: Any, context: str) -> str:
    require(isinstance(value, str) and bool(value.strip()), f"{context}: expected nonempty string")
    return value


def boolean(value: Any, context: str) -> bool:
    require(type(value) is bool, f"{context}: expected boolean")
    return value


def digest_file(path: Path) -> str:
    result = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def read_json(path: Path) -> Any:
    try:
        value = parse_json(path.read_text(encoding="utf-8-sig"))
    except (OSError, ValueError) as exc:
        raise ValidationError(f"{path}: {exc}") from exc
    return value


def parse_json(content: str) -> Any:
    def unique(pairs):
        value = {}
        for key, child in pairs:
            require(key not in value, f"JSON: duplicate key {key}")
            value[key] = child
        return value

    value = json.loads(content, object_pairs_hook=unique)
    finite_tree(value)
    return value


class UniqueLoader(yaml.SafeLoader):
    """Reject duplicate profile keys instead of silently replacing parameters."""


def _unique_mapping(loader, node):
    value = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node)
        require(isinstance(key, str) and key not in value, "YAML: invalid or duplicate key")
        value[key] = loader.construct_object(value_node)
    return value


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _unique_mapping)


@dataclass(frozen=True)
class Parameter:
    default: Any
    kind: type
    minimum: float | None = None
    choices: tuple[str, ...] = ()

    def validate(self, value: Any, context: str) -> Any:
        if self.kind is int:
            return integer(value, context, int(self.minimum or 0))
        if self.kind is float:
            return number(value, context, self.minimum)
        if self.kind is bool:
            return boolean(value, context)
        if self.kind is list:
            require(isinstance(value, list) and value, f"{context}: nonempty string list required")
            require(all(isinstance(v, str) and v for v in value), f"{context}: invalid key")
            require(len(set(value)) == len(value), f"{context}: duplicate key")
            return list(value)
        value = text(value, context)
        require(not self.choices or value in self.choices, f"{context}: expected {self.choices}")
        return value


P = Parameter
QUIET = {"linear_epsilon_mps": P(0.001, float, 0), "angular_epsilon_rps": P(0.001, float, 0)}
PARAMETERS = {
    "R1": {"radius_m": P(0.5, float, 0), "contact_tolerance_m": P(0.0, float, 0)},
    "R2": {
        **QUIET,
        "goal_tolerance_m": P(0.5, float, 0),
        "physical_tolerance_m": P(0.85, float, 0),
        "dwell_ns": P(1_000_000_000, int, 1),
        "deadline_ns": P(60_000_000_000, int, 1),
    },
    "R3": {**QUIET, "response_ns": P(500_000_000, int, 1)},
    "R4": {
        **QUIET,
        "expiry_ns": P(400_000_000, int, 1),
        "settling_ns": P(750_000_000, int, 1),
        "dwell_ns": P(200_000_000, int, 1),
        "speed_epsilon_mps": P(0.02, float, 0),
        "yaw_rate_epsilon_rps": P(0.02, float, 0),
    },
    "R5": {
        **QUIET,
        "response_ns": P(2_000_000_000, int, 1),
        "stop_dwell_ns": P(200_000_000, int, 1),
        "radius_m": P(0.5, float, 0),
    },
    "R6": {"error_limit_m": P(0.5, float, 0), "uav_max_age_ns": P(500_000_000, int, 1)},
    "R7": {
        "response_ns": P(5_000_000_000, int, 1),
        "alarm_scope": P("class", str, choices=("class", "channel", "binary")),
    },
    "R8": {
        "warmup_ns": P(0, int, 0),
        "recovery_ns": P(0, int, 0),
        "merge_gap_ns": P(0, int, 0),
        "decision_keys": P(["binary"], list),
    },
}


def load_profile(path: str | Path) -> Profile:
    path = Path(path)
    try:
        raw = yaml.load(path.read_text(encoding="utf-8-sig"), Loader=UniqueLoader)
    except (OSError, yaml.YAMLError) as exc:
        raise ValidationError(f"{path}: {exc}") from exc
    raw = mapping(raw, str(path))
    finite_tree(raw, str(path))
    keys(
        raw,
        {"schema_version", "id", "version", "calibration", "max_gap_ns", "requirements"},
        {"schema_version", "id", "version", "calibration", "max_gap_ns", "requirements"},
        "profile",
    )
    require(
        type(raw["schema_version"]) is int and raw["schema_version"] == SCHEMA_VERSION,
        "profile: unsupported schema_version",
    )
    calibration = text(raw["calibration"], "profile.calibration")
    require(calibration in ("fixture", "provisional", "calibrated"), "profile: invalid calibration")
    gap = integer(raw["max_gap_ns"], "profile.max_gap_ns", 1)
    configs = mapping(raw["requirements"], "profile.requirements")
    require(set(configs) <= set(REQUIREMENTS), "profile: unknown requirement ID")
    rules = {}
    for name, value in configs.items():
        config = mapping(value, name)
        keys(
            config,
            {"applicable", "mandatory", "clock", "max_gap_ns", "parameters", "reason"},
            {"applicable", "mandatory"},
            name,
        )
        applicable = boolean(config["applicable"], f"{name}.applicable")
        reason = config.get("reason", "")
        require(isinstance(reason, str), f"{name}.reason: expected string")
        if not applicable:
            text(reason, f"{name}.reason")
        params = mapping(config.get("parameters", {}), f"{name}.parameters")
        require(set(params) <= set(PARAMETERS[name]), f"{name}: unknown parameters")
        normalized = {
            key: spec.validate(params.get(key, spec.default), f"{name}.{key}")
            for key, spec in PARAMETERS[name].items()
        }
        clock = text(config.get("clock", "sim"), f"{name}.clock")
        require(
            name != "R4"
            or not applicable
            or calibration == "fixture"
            or clock.startswith("steady:"),
            "R4: use a steady: clock for recorded trials",
        )
        rules[name] = Rule(
            name,
            applicable,
            boolean(config["mandatory"], f"{name}.mandatory"),
            clock,
            integer(config.get("max_gap_ns", gap), f"{name}.max_gap_ns", 1),
            normalized,
            reason,
        )
    return Profile(
        text(raw["id"], "profile.id"),
        text(raw["version"], "profile.version"),
        calibration,
        rules,
        digest_file(path),
    )


def parse_manifest(raw: Any) -> Manifest:
    raw = mapping(raw, "manifest")
    keys(
        raw,
        {
            "schema_version",
            "run_id",
            "source_run_id",
            "adapter",
            "status",
            "intervals",
            "streams",
            "provenance",
            "frame",
            "reference_point",
        },
        {
            "schema_version",
            "run_id",
            "source_run_id",
            "adapter",
            "status",
            "intervals",
            "streams",
            "provenance",
        },
        "manifest",
    )
    require(
        type(raw["schema_version"]) is int and raw["schema_version"] == SCHEMA_VERSION,
        "manifest: unsupported schema_version",
    )
    require(raw["adapter"] in ("normalized", "dataset_csv"), "manifest: unsupported adapter")
    require(raw["status"] in ("completed", "aborted", "failed"), "manifest: invalid status")
    require(isinstance(raw["intervals"], list) and raw["intervals"], "manifest: intervals required")
    intervals = []
    seen = set()
    for value in raw["intervals"]:
        value = mapping(value, "interval")
        keys(
            value,
            {"clock", "epoch", "start_ns", "end_ns"},
            {"clock", "epoch", "start_ns", "end_ns"},
            "interval",
        )
        interval = Interval(
            text(value["clock"], "interval.clock"),
            integer(value["epoch"], "interval.epoch"),
            integer(value["start_ns"], "interval.start_ns"),
            integer(value["end_ns"], "interval.end_ns"),
        )
        require(interval.end_ns > interval.start_ns, "interval: end must exceed start")
        require((interval.clock, interval.epoch) not in seen, "interval: duplicate clock/epoch")
        seen.add((interval.clock, interval.epoch))
        intervals.append(interval)
    streams = mapping(raw["streams"], "manifest.streams")
    for name, path in streams.items():
        text(name, "stream name")
        text(path, f"stream path {name}")
    return Manifest(
        text(raw["run_id"], "manifest.run_id"),
        text(raw["source_run_id"], "manifest.source_run_id"),
        raw["adapter"],
        raw["status"],
        tuple(intervals),
        streams,
        mapping(raw["provenance"], "manifest.provenance"),
        text(raw.get("frame", "map"), "manifest.frame"),
        text(raw.get("reference_point", "model_origin"), "manifest.reference_point"),
    )


def parse_record(raw: Any, stream: str) -> Record:
    raw = mapping(raw, "record")
    fields = {
        "time_ns",
        "clock",
        "epoch",
        "source",
        "sequence",
        "valid",
        "frame",
        "data",
        "source_sequence",
    }
    keys(raw, fields, fields - {"valid", "frame", "source_sequence"}, f"record {stream}")
    data = mapping(raw["data"], f"{stream}.data")
    valid = boolean(raw.get("valid", True), f"{stream}.valid")
    frame = raw.get("frame")
    require(frame is None or isinstance(frame, str), f"{stream}.frame: expected string or null")
    record = Record(
        stream,
        integer(raw["time_ns"], "record.time_ns"),
        text(raw["clock"], "record.clock"),
        integer(raw["epoch"], "record.epoch"),
        text(raw["source"], "record.source"),
        integer(raw["sequence"], "record.sequence"),
        data,
        valid,
        frame,
        integer(raw["source_sequence"], "record.source_sequence")
        if "source_sequence" in raw
        else None,
    )
    validate_payload(record)
    return record


def validate_payload(record: Record) -> None:
    """Validate observed payloads; explicitly invalid records may omit their payload."""
    require(record.stream != "events" or record.valid, "event log contains an invalid event")
    if not record.valid:
        return
    data, stream = record.data, record.stream

    def numeric(*names):
        for name in names:
            require(name in data, f"{stream}: missing {name}")
            number(data[name], f"{stream}.{name}")

    if stream in ("truth_pose", "estimated_pose"):
        numeric("x", "y")
        require(record.frame == "map", f"{stream}: normalized pose must use map")
    elif stream == "goal":
        numeric("x", "y")
        text(data.get("revision"), "goal.revision")
        require(record.frame == "map", "goal: frame must use map")
    elif stream in ("requested_command", "actuator_input", "actuator_output"):
        numeric("linear", "angular")
        if stream == "actuator_input":
            boolean(data.get("accepted"), "actuator_input.accepted")
    elif stream == "measured_motion":
        numeric("linear", "angular")
        boolean(data.get("simulation_advancing"), "measured_motion.simulation_advancing")
        integer(data.get("uncertainty_ns", 0), "measured_motion.uncertainty_ns")
    elif stream == "guard_state":
        boolean(data.get("guard_valid"), "guard_state.guard_valid")
        text(data.get("reason"), "guard_state.reason")
    elif stream == "detector_decision":
        boolean(data.get("alarm"), "detector_decision.alarm")
        text(data.get("scope"), "detector_decision.scope")
        require(data["scope"] in ("class", "channel", "binary"), "detector: invalid scope")
        if data["scope"] in ("class", "channel"):
            text(data.get("sensor"), "detector.sensor")
            text(data.get("channel"), "detector.channel")
        if data["scope"] == "class":
            text(data.get("fault_class"), "detector.fault_class")
    elif stream == "obstacles":
        if "static" in data:
            boolean(data["static"], "obstacles.static")
        shapes = data.get("shapes")
        require(isinstance(shapes, list), "obstacles: explicit shapes list required")
        ids = set()
        for shape in shapes:
            mapping(shape, "shape")
            if "id" in shape:
                identifier = text(shape["id"], "shape.id")
                require(identifier not in ids, "duplicate obstacle ID")
                ids.add(identifier)
            require(shape.get("kind") in ("box", "cylinder"), "obstacles: unsupported shape")
            for key in ("x", "y"):
                number(shape.get(key), f"shape.{key}")
            if shape["kind"] == "box":
                for key in ("half_x", "half_y"):
                    require(number(shape.get(key), f"shape.{key}") > 0, "box: invalid extent")
                number(shape.get("yaw", 0), "shape.yaw")
            else:
                require(number(shape.get("radius"), "shape.radius") > 0, "cylinder: invalid radius")
    elif stream == "route":
        text(data.get("goal_revision"), "route.goal_revision")
        points = data.get("points")
        require(
            isinstance(points, list) and len(points) >= 2, "route: at least two points required"
        )
        for point in points:
            require(isinstance(point, list) and len(point) == 2, "route: expected [x,y]")
            for value in point:
                number(value, "route coordinate")
        require(record.frame == "map", "route: frame must use map")
    elif stream == "events":
        text(data.get("kind"), "event.kind")
        text(data.get("event_id"), "event.event_id")
        if data["kind"] in ("fault_start", "fault_end"):
            for key in ("sensor", "channel", "fault_class"):
                text(data.get(key), f"event.{key}")
        if data["kind"] == "hazard_observed":
            text(data.get("goal_revision"), "event.goal_revision")
            text(data.get("obstacle_id"), "event.obstacle_id")
            boolean(data.get("qualifying"), "event.qualifying")
    elif stream in ("uav_observation", "grid"):
        pass  # Presence/validity and acquisition time are used; payload is preserved.
    elif not stream.startswith("raw."):
        raise ValidationError(f"unsupported canonical stream {stream}")
