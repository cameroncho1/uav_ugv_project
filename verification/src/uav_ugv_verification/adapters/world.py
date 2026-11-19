"""Extract independent planar collision geometry from explicitly supported SDF worlds."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

from ..validation import require


def collision_geometry(path: Path, ignored_models: list[str]) -> list[dict]:
    root = ET.parse(path).getroot()
    world = root.find("world")
    require(world is not None, "geometry: SDF world required")
    for include in world.findall("include"):
        name = include.findtext("name") or include.findtext("uri", "").removeprefix("model://")
        require(name in ignored_models, "geometry: resolve obstacle includes before importing")

    def pose(element):
        node = element.find("pose")
        require(node is None or not node.attrib, "geometry: resolve named/relative pose frames")
        values = [float(v) for v in (node.text if node is not None else "0 0 0 0 0 0").split()]
        require(len(values) == 6 and all(math.isfinite(v) for v in values), "invalid SDF pose")
        require(
            abs(values[3]) < 1e-9 and abs(values[4]) < 1e-9, "geometry: upright shapes required"
        )
        return values[0], values[1], values[5]

    def compose(a, b):
        c, s = math.cos(a[2]), math.sin(a[2])
        return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2]

    shapes = []
    for model in world.findall("model"):
        if model.get("name") in ignored_models:
            continue
        if model.findall(".//collision"):
            require(
                model.findtext("static") == "true",
                "geometry: moving collisions need a recorded geometry stream",
            )
        require(
            not model.findall("model") and not model.findall("include"),
            "geometry: nested models unsupported",
        )
        for link in model.findall("link"):
            for collision in link.findall("collision"):
                x, y, yaw = compose(compose(pose(model), pose(link)), pose(collision))
                geometry = collision.find("geometry")
                require(geometry is not None, "collision geometry missing")
                shape = {
                    "id": "/".join(e.get("name", "") for e in (model, link, collision)),
                    "x": x,
                    "y": y,
                }
                box, cylinder = geometry.find("box"), geometry.find("cylinder")
                if box is not None:
                    size = [float(v) for v in box.findtext("size", "").split()]
                    require(
                        len(size) == 3 and all(math.isfinite(v) and v > 0 for v in size),
                        "invalid box",
                    )
                    shape.update(kind="box", half_x=size[0] / 2, half_y=size[1] / 2, yaw=yaw)
                else:
                    require(
                        cylinder is not None, "geometry: only box/cylinder collisions supported"
                    )
                    radius = float(cylinder.findtext("radius", "nan"))
                    require(math.isfinite(radius) and radius > 0, "invalid cylinder")
                    shape.update(kind="cylinder", radius=radius)
                shapes.append(shape)
    return shapes
