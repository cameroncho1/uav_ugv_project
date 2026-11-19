"""Numerical eligibility of a newly observed hazard; no ROS or simulator imports."""

from simulation_support.obstacle_scenarios import grid_region
from uav_ugv_verification.signals.geometry import clearance, segment_clearance


def assess_hazard(
    applied,
    baseline,
    route,
    grid,
    truth,
    current_revision,
    *,
    radius=0.5,
    clearance_margin=0.05,
    maximum_speed=0.5,
    response_s=2.0,
):
    shape, confirmed = applied["obstacle"], applied["confirmed_ns"]
    if grid["time_ns"] <= confirmed:
        return None
    occupied = sum(value >= 50 for value in grid_region(grid, shape))
    if occupied < 3:
        return None
    baseline_fresh = (
        baseline is not None and 0 <= applied["requested_ns"] - baseline["time_ns"] <= 1_000_000_000
    )
    before = grid_region(baseline, shape) if baseline_fresh else []
    new_observation = bool(before) and all(0 <= value < 50 for value in before)
    route_fresh = (
        route is not None and 0 <= applied["requested_ns"] - route["time_ns"] <= 1_000_000_000
    )
    blocked_route = route_fresh and any(
        segment_clearance(a, b, shape, radius) < 0
        for a, b in zip(route["points"], route["points"][1:], strict=False)
    )
    same_goal = route is not None and current_revision == route["goal_revision"]
    truth_fresh = truth is not None and 0 <= grid["time_ns"] - truth[0] <= 250_000_000
    distance = None
    if truth_fresh:
        # Bound movement between the previous truth sample and image acquisition.
        distance = (
            clearance((truth[1], truth[2]), shape, 0)
            - maximum_speed * (grid["time_ns"] - truth[0]) / 1e9
        )
    observation_s = (grid["time_ns"] - applied["requested_ns"]) / 1e9
    minimum_distance = maximum_speed * (observation_s + response_s) + radius + clearance_margin
    qualifying = bool(
        new_observation
        and blocked_route
        and same_goal
        and truth_fresh
        and distance >= minimum_distance
    )
    return dict(
        kind="hazard_observed",
        event_id=applied["operation"]["id"],
        obstacle_id=shape["id"],
        goal_revision=route["goal_revision"] if route else "unknown",
        qualifying=qualifying,
        occupied_cells=occupied,
        baseline_cells=len(before),
        baseline_free=new_observation,
        preappearance_route_blocked=bool(blocked_route),
        same_goal=same_goal,
        truth_fresh=truth_fresh,
        distance_lower_bound_m=distance,
        minimum_distance_m=minimum_distance,
        request_to_observation_upper_bound_s=observation_s,
        requested_ns=applied["requested_ns"],
        confirmed_ns=confirmed,
        grid_stamp_ns=grid["time_ns"],
    )
