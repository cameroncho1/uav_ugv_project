"""Validated obstacle schedules and geometry, independent of ROS and Gazebo."""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from decimal import Decimal
from pathlib import Path

import yaml

SECOND = 1_000_000_000


def require(condition, message):
    if not condition:
        raise ValueError(message)


def fields(value, names, context):
    require(isinstance(value, dict) and set(value) == set(names), f"{context}: expected {names}")


def identifier(value):
    require(
        isinstance(value, str) and re.fullmatch(r"[A-Za-z][A-Za-z0-9_\-]{0,95}", value),
        "IDs must contain letters, numbers, underscores, or hyphens",
    )
    return value


def finite(value):
    require(type(value) in (int, float) and math.isfinite(value), "finite numbers required")
    return float(value)


class UniqueLoader(yaml.SafeLoader):
    """Reject duplicate keys before a scenario can mutate a world."""


def unique_mapping(loader, node):
    result = {}
    for key, value in node.value:
        key = loader.construct_object(key)
        require(
            isinstance(key, str) and key not in result,
            f"invalid or duplicate scenario field: {key}",
        )
        result[key] = loader.construct_object(value)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)


@dataclass(frozen=True)
class Box:
    id: str
    size: tuple[float, float, float]
    pose: tuple[float, float, float, float]  # map x, y, z, yaw

    @property
    def model_name(self):
        return "scenario_" + self.id

    def shape(self):
        return dict(
            id=self.model_name,
            kind="box",
            x=self.pose[0],
            y=self.pose[1],
            half_x=self.size[0] / 2,
            half_y=self.size[1] / 2,
            yaw=self.pose[3],
        )

    def sdf(self):
        root = ET.Element("sdf", version="1.9")
        model = ET.SubElement(root, "model", name=self.model_name)
        ET.SubElement(model, "static").text = "true"
        x, y, z, yaw = self.pose
        ET.SubElement(model, "pose").text = f"{x} {y} {z} 0 0 {yaw}"
        link = ET.SubElement(model, "link", name="body")
        for kind in ("collision", "visual"):
            element = ET.SubElement(link, kind, name=kind)
            geometry = ET.SubElement(element, "geometry")
            ET.SubElement(ET.SubElement(geometry, "box"), "size").text = " ".join(
                map(str, self.size)
            )
            if kind == "visual":
                material = ET.SubElement(element, "material")
                for name in ("ambient", "diffuse"):
                    ET.SubElement(material, name).text = "0.025 0.025 0.025 1"
        return ET.tostring(root, encoding="unicode")


@dataclass(frozen=True)
class Operation:
    id: str
    at_ns: int
    action: str
    obstacle: str


@dataclass(frozen=True)
class Scenario:
    id: str
    world: str
    boxes: dict[str, Box]
    operations: tuple[Operation, ...]


def load_scenario(path: Path) -> Scenario:
    value = yaml.load(path.read_text(encoding="utf-8"), Loader=UniqueLoader)
    fields(
        value,
        ("schema_version", "id", "world", "frame", "trigger", "obstacles", "operations"),
        "scenario",
    )
    require(
        type(value["schema_version"]) is int and value["schema_version"] == 1,
        "unsupported scenario version",
    )
    require(value["frame"] == "map", "only ENU map coordinates are supported")
    require(value["trigger"] == "first_drive_command", "unsupported scenario trigger")
    require(isinstance(value["obstacles"], list) and value["obstacles"], "obstacles required")
    require(isinstance(value["operations"], list) and value["operations"], "operations required")
    boxes = {}
    for raw in value["obstacles"]:
        fields(raw, ("id", "shape", "size", "pose"), "obstacle")
        name = identifier(raw["id"])
        require(name not in boxes and raw["shape"] == "box", "duplicate ID or unsupported shape")
        require(isinstance(raw["size"], list) and len(raw["size"]) == 3, "box size requires xyz")
        require(isinstance(raw["pose"], list) and len(raw["pose"]) == 4, "pose requires xyz/yaw")
        size, pose = tuple(map(finite, raw["size"])), tuple(map(finite, raw["pose"]))
        require(all(v > 0 for v in size), "box dimensions must be positive")
        require(abs(pose[2] - size[2] / 2) <= 1e-8, "boxes must rest on the z=0 ground plane")
        boxes[name] = Box(name, size, pose)
    operations, ids, active = [], set(), set()
    previous = -1
    for raw in value["operations"]:
        fields(raw, ("id", "at_s", "action", "obstacle"), "operation")
        name = identifier(raw["id"])
        seconds = finite(raw["at_s"])
        exact = Decimal(str(raw["at_s"])) * SECOND
        require(seconds >= 0 and exact == exact.to_integral_value(), "nanosecond schedule required")
        at_ns = int(exact)
        require(name not in ids and at_ns >= previous, "unique IDs and ordered times required")
        target, action = identifier(raw["obstacle"]), raw["action"]
        require(target in boxes and action in ("spawn", "remove"), "unknown obstacle/action")
        require(
            (target not in active) if action == "spawn" else (target in active),
            "invalid obstacle lifecycle",
        )
        active.add(target) if action == "spawn" else active.remove(target)
        operations.append(Operation(name, at_ns, action, target))
        ids.add(name)
        previous = at_ns
    return Scenario(identifier(value["id"]), identifier(value["world"]), boxes, tuple(operations))


class Schedule:
    """Serial, exactly-once dispatch; clock rewind aborts rather than replays mutations."""

    def __init__(self, scenario):
        self.scenario = scenario
        self.started_ns = None
        self.last_ns = None
        self.index = 0
        self.pending = None

    def observe(self, now_ns, driving=False):
        require(type(now_ns) is int and now_ns >= 0, "invalid simulation time")
        require(self.last_ns is None or now_ns >= self.last_ns, "scenario clock rewound")
        self.last_ns = now_ns
        if driving and now_ns > 0 and self.started_ns is None:
            self.started_ns = now_ns

    def next_due(self, now_ns):
        self.observe(now_ns)
        if self.pending is not None or self.started_ns is None or self.complete:
            return None
        operation = self.scenario.operations[self.index]
        if now_ns - self.started_ns < operation.at_ns:
            return None
        self.pending = operation
        return operation

    def confirm(self, operation_id):
        require(
            self.pending is not None and self.pending.id == operation_id,
            "confirmation does not match pending operation",
        )
        self.index += 1
        self.pending = None

    @property
    def complete(self):
        return self.index == len(self.scenario.operations)


def grid_region(grid, shape):
    """Return cell centers inside an oriented box footprint, without resampling its grid."""
    width, height, resolution = grid["width"], grid["height"], grid["resolution"]
    ox, oy = grid["origin"]
    require(
        width > 0 and height > 0 and resolution > 0 and len(grid["data"]) == width * height,
        "invalid occupancy-grid geometry",
    )
    c, s = math.cos(shape.get("yaw", 0)), math.sin(shape.get("yaw", 0))
    hx, hy = shape["half_x"], shape["half_y"]
    rx, ry = abs(c) * hx + abs(s) * hy, abs(s) * hx + abs(c) * hy
    # Full coverage is required: clipping an obstacle at the map boundary is not observation.
    if (
        shape["x"] - rx < ox
        or shape["y"] - ry < oy
        or shape["x"] + rx >= ox + width * resolution
        or shape["y"] + ry >= oy + height * resolution
    ):
        return []
    result = []
    for row in range(
        max(0, int((shape["y"] - ry - oy) // resolution)),
        min(height, int((shape["y"] + ry - oy) // resolution) + 1),
    ):
        for col in range(
            max(0, int((shape["x"] - rx - ox) // resolution)),
            min(width, int((shape["x"] + rx - ox) // resolution) + 1),
        ):
            x, y = (
                ox + (col + 0.5) * resolution - shape["x"],
                oy + (row + 0.5) * resolution - shape["y"],
            )
            if abs(c * x + s * y) <= hx and abs(-s * x + c * y) <= hy:
                result.append(grid["data"][row * width + col])
    return result
