"""A spawn request, an old map, and an unrelated route cannot establish hazard visibility."""

from copy import deepcopy

import pytest
from hazard_assessment import assess_hazard


def inputs():
    box = dict(id="box", kind="box", x=0.0, y=0.0, half_x=0.5, half_y=0.5, yaw=0.0)
    applied = dict(
        operation=dict(id="spawn_box"),
        obstacle=box,
        requested_ns=1_000_000_000,
        confirmed_ns=1_100_000_000,
    )
    before = dict(
        time_ns=900_000_000, width=16, height=8, resolution=0.5, origin=[-4.0, -2.0], data=[0] * 128
    )
    after = deepcopy(before)
    after["time_ns"] = 2_000_000_000
    for row, col in ((3, 7), (3, 8), (4, 7), (4, 8)):
        after["data"][row * 16 + col] = 100
    route = dict(time_ns=900_000_000, goal_revision="goal_a", points=[[-4.0, 0.0], [4.0, 0.0]])
    return applied, before, route, after, (1_900_000_000, -4.0, 0.0), "goal_a"


def test_actual_new_map_occupancy_on_fresh_route_with_sufficient_distance_qualifies():
    observation = assess_hazard(*inputs())
    assert observation["qualifying"]
    assert observation["occupied_cells"] == 4
    assert observation["distance_lower_bound_m"] == pytest.approx(3.45)
    assert observation["minimum_distance_m"] == pytest.approx(2.05)


@pytest.mark.parametrize("change", ["old_grid", "free_grid", "one_dark_cell"])
def test_absent_new_map_evidence_creates_no_observation(change):
    values = list(inputs())
    grid = values[3]
    if change == "old_grid":
        grid["time_ns"] = values[0]["confirmed_ns"]
    else:
        grid["data"] = [0] * 128
        if change == "one_dark_cell":
            grid["data"][3 * 16 + 7] = 100
    assert assess_hazard(*values) is None


@pytest.mark.parametrize(
    "change",
    ["unknown_baseline", "stale_route", "other_goal", "off_route", "stale_truth", "too_close"],
)
def test_observation_outside_declared_envelope_is_not_a_qualifying_trigger(change):
    values = list(inputs())
    if change == "unknown_baseline":
        values[1]["data"] = [-1] * 128
    elif change == "stale_route":
        values[2]["time_ns"] = -1
    elif change == "other_goal":
        values[5] = "goal_b"
    elif change == "off_route":
        values[2]["points"] = [[-4.0, 3.0], [4.0, 3.0]]
    elif change == "stale_truth":
        values[4] = (1_000_000_000, -4.0, 0.0)
    else:
        values[4] = (1_900_000_000, -1.0, 0.0)
    assert not assess_hazard(*values)["qualifying"]
