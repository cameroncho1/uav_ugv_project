"""Scenario lifecycle and map-observation boundaries are numerical contracts."""

import copy
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml
from simulation_support.obstacle_scenarios import Schedule, grid_region, load_scenario

EXAMPLE = Path(__file__).resolve().parents[1] / "config/scenarios/single_box.yaml"


def test_pause_and_repeated_ticks_never_dispatch_twice():
    schedule = Schedule(load_scenario(EXAMPLE))
    assert schedule.next_due(100) is None
    schedule.observe(1_000_000_000, driving=True)
    assert schedule.next_due(1_499_999_999) is None
    operation = schedule.next_due(1_500_000_000)
    assert operation.id == "add_route_box"
    for _ in range(10):
        assert schedule.next_due(1_500_000_000) is None
    with pytest.raises(ValueError, match="does not match"):
        schedule.confirm("other")
    schedule.confirm(operation.id)
    assert schedule.complete and schedule.next_due(10_000_000_000) is None
    with pytest.raises(ValueError, match="rewound"):
        schedule.next_due(1)


def test_reusable_multiple_obstacles_and_remove_then_respawn(tmp_path):
    raw = yaml.safe_load(EXAMPLE.read_text())
    second = copy.deepcopy(raw["obstacles"][0])
    second.update(id="second", pose=[10.0, 2.0, 0.2, 1.2])
    raw["obstacles"].append(second)
    raw["operations"] += [
        dict(id="second_box", at_s=2.0, action="spawn", obstacle="second"),
        dict(id="remove_first", at_s=3.0, action="remove", obstacle="route_box"),
        dict(id="respawn_first", at_s=4.0, action="spawn", obstacle="route_box"),
    ]
    path = tmp_path / "scenario.yaml"
    path.write_text(yaml.safe_dump(raw))
    schedule = Schedule(load_scenario(path))
    schedule.observe(1, driving=True)
    seen = []
    for _ in range(4):
        operation = schedule.next_due(5_000_000_001)
        seen.append(operation.id)
        schedule.confirm(operation.id)
    assert seen == [v["id"] for v in raw["operations"]]


@pytest.mark.parametrize(
    "mutate",
    [
        lambda v: v.update(frame="odom"),
        lambda v: v.update(unknown="typo"),
        lambda v: v["obstacles"][0].update(shape="mesh"),
        lambda v: v["obstacles"][0].update(size=[-1, 1, 1]),
        lambda v: v["obstacles"][0].update(pose=[0, 0, float("nan"), 0]),
        lambda v: v["operations"][0].update(action="remove"),
        lambda v: v["operations"][0].update(at_s=-1),
        lambda v: v["operations"][0].update(at_s=0.0000000001),
    ],
)
def test_invalid_scenarios_are_rejected_before_mutation(tmp_path, mutate):
    raw = yaml.safe_load(EXAMPLE.read_text())
    mutate(raw)
    path = tmp_path / "invalid.yaml"
    path.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError):
        load_scenario(path)


def test_sdf_collision_and_visible_geometry_match_and_have_stable_names():
    box = load_scenario(EXAMPLE).boxes["route_box"]
    model = ET.fromstring(box.sdf()).find("model")
    assert model.get("name") == box.model_name
    assert model.findtext("static") == "true"
    assert model.findtext("link/collision/geometry/box/size") == model.findtext(
        "link/visual/geometry/box/size"
    )


def test_grid_observation_uses_actual_resolution_origin_and_orientation():
    grid = dict(width=10, height=6, resolution=0.5, origin=[-2.5, -1.5], data=[0] * 60)
    shape = dict(kind="box", x=0.0, y=0.0, half_x=1.0, half_y=0.5, yaw=0.0)
    assert len(grid_region(grid, shape)) == 8
    shape["yaw"] = 1.5707963267948966
    assert len(grid_region(grid, shape)) == 8
    shape["x"] = 10.0
    assert grid_region(grid, shape) == []
