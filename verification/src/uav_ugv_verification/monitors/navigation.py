"""Clearance, goal arrival, and acquisition-matched localization checks."""

from __future__ import annotations

import math

from ..contracts import Interval, Rule, TraceReader
from ..signals.geometry import clearance
from ..signals.timeline import Cursor, align, quiet
from .common import Evidence


def clearance_check(reader: TraceReader, rule: Rule, interval: Interval):
    evidence = Evidence(rule, interval)
    evidence.result.metrics["scope"] = "sampled_disk_proxy"
    for snapshot in align(reader, ("truth_pose", "obstacles"), interval, rule.max_gap_ns):
        truth, obstacles = snapshot.values["truth_pose"], snapshot.values["obstacles"]
        if truth is None or obstacles is None:
            evidence.missing(snapshot.time_ns, "truth_or_geometry_gap")
            continue
        evidence.result.eligible_duration_ns += snapshot.duration_ns
        # Score actual truth samples, not artificial sample-and-hold points.
        for sample in snapshot.updates["truth_pose"]:
            if not sample.valid or sample.time_ns < interval.start_ns:
                continue
            shapes = obstacles.data["shapes"]
            margin = min(
                (
                    clearance(
                        (sample.data["x"], sample.data["y"]), shape, rule.parameters["radius_m"]
                    )
                    for shape in shapes
                ),
                default=None,
            )
            evidence.checks += 1
            if margin is not None:
                evidence.minimum("minimum_clearance_m", margin)
                if margin < -rule.parameters["contact_tolerance_m"]:
                    evidence.fail(sample.time_ns, "disk_proxy_overlap", clearance_m=margin)
    return evidence.finish()


def arrival(reader: TraceReader, rule: Rule, interval: Interval):
    evidence = Evidence(rule, interval)
    p = rule.parameters
    revision, goal_started, dwell_started, arrived = None, None, None, False
    uncertain = False

    def finish_goal(at_ns):
        if revision is None:
            return
        if arrived:
            evidence.result.completed_obligations += 1
        elif uncertain or at_ns < goal_started + p["deadline_ns"]:
            evidence.missing(at_ns, "arrival_deadline_or_data_incomplete")
        else:
            evidence.fail(
                goal_started + p["deadline_ns"], "goal_deadline_missed", revision=revision
            )

    streams = ("truth_pose", "estimated_pose", "requested_command", "goal")
    for snapshot in align(reader, streams, interval, rule.max_gap_ns):
        goal = snapshot.values["goal"]
        if goal is None:
            if snapshot.duration_ns:
                evidence.missing(snapshot.time_ns, "goal_gap")
                uncertain = True
            continue
        if goal.data["revision"] != revision:
            finish_goal(snapshot.time_ns)
            revision = goal.data["revision"]
            goal_started = max(goal.time_ns, interval.start_ns)
            dwell_started, arrived, uncertain = None, False, False
            evidence.result.trigger_count += 1
            evidence.checks += 1
        if arrived:
            continue
        if any(snapshot.values[s] is None for s in streams):
            uncertain = True
            dwell_started = None
            if snapshot.duration_ns:
                evidence.missing(snapshot.time_ns, "arrival_input_gap")
            continue
        distances = {}
        for stream in ("truth_pose", "estimated_pose"):
            pose = snapshot.values[stream].data
            distances[stream] = math.hypot(pose["x"] - goal.data["x"], pose["y"] - goal.data["y"])
        evidence.minimum("physical_min_distance_m", distances["truth_pose"])
        evidence.minimum("estimated_min_distance_m", distances["estimated_pose"])
        evidence.result.eligible_duration_ns += snapshot.duration_ns
        eligible = (
            distances["truth_pose"] <= p["physical_tolerance_m"]
            and distances["estimated_pose"] <= p["goal_tolerance_m"]
            and quiet(snapshot.values["requested_command"], p)
        )
        if not eligible:
            dwell_started = None
            continue
        dwell_started = snapshot.time_ns if dwell_started is None else dwell_started
        completion = dwell_started + p["dwell_ns"]
        if completion <= snapshot.end_ns and completion <= goal_started + p["deadline_ns"]:
            arrived = True
            evidence.maximum("arrival_latency_ns", completion - goal_started)
    finish_goal(interval.end_ns)
    return evidence.finish()


def localization(reader: TraceReader, rule: Rule, interval: Interval):
    evidence = Evidence(rule, interval)
    truth = Cursor(reader, "truth_pose", interval, rule.max_gap_ns)
    guard = Cursor(reader, "guard_state", interval, rule.max_gap_ns)
    uav = Cursor(reader, "uav_observation", interval, rule.parameters["uav_max_age_ns"])
    squared_sum, samples = 0.0, 0
    full_sum, full_samples = 0.0, 0
    for estimate in reader.records("estimated_pose", interval):
        if estimate.time_ns < interval.start_ns:
            continue
        t = estimate.time_ns
        truth.advance(t)
        guard_updates = guard.advance(t)
        previous_guard = guard.before
        uav.advance(t)
        before, after = truth.current, truth.next
        point = None
        if before and before.valid:
            if before.time_ns == t:
                point = (before.data["x"], before.data["y"])
            elif (
                after
                and after.valid
                and after.time_ns - before.time_ns <= rule.max_gap_ns
                and after.sequence == before.sequence + 1
            ):
                fraction = (t - before.time_ns) / (after.time_ns - before.time_ns)
                point = tuple(
                    before.data[k] + fraction * (after.data[k] - before.data[k]) for k in ("x", "y")
                )
        if point is None or not estimate.valid:
            evidence.missing(t, "estimate_or_truth_matching_gap")
            continue
        error = math.hypot(estimate.data["x"] - point[0], estimate.data["y"] - point[1])
        full_sum += error * error
        full_samples += 1
        tied_guards = [r for r in guard_updates if r.time_ns == t and r.source != estimate.source]
        if tied_guards and t > interval.start_ns:
            flags = {r.data.get("guard_valid") if r.valid else None for r in tied_guards}
            flags.add(
                previous_guard.data.get("guard_valid")
                if previous_guard
                and previous_guard.valid
                and t - previous_guard.time_ns < rule.max_gap_ns
                else None
            )
            if len(flags) > 1:
                if error > rule.parameters["error_limit_m"]:
                    evidence.missing(t, "guard_estimate_order_ambiguous")
                continue
        state = guard.value(t)
        if state is None:
            evidence.missing(t, "pose_validity_unknown")
            continue
        if not state.data["guard_valid"] or uav.value(t) is None:
            continue
        evidence.checks += 1
        evidence.result.trigger_count += 1
        samples += 1
        squared_sum += error * error
        evidence.maximum("maximum_error_m", error)
        evidence.minimum("minimum_margin_m", rule.parameters["error_limit_m"] - error)
        if error > rule.parameters["error_limit_m"]:
            evidence.fail(t, "position_error_limit", error_m=error)
    # Duration requires joint signal coverage and bounded truth interpolation.
    duration_truth = Cursor(reader, "truth_pose", interval, rule.max_gap_ns)
    for snapshot in align(
        reader,
        ("estimated_pose", "guard_state", "uav_observation", "truth_pose"),
        interval,
        rule.max_gap_ns,
        gaps={"uav_observation": rule.parameters["uav_max_age_ns"]},
    ):
        duration_truth.advance(snapshot.time_ns)
        before, after = duration_truth.current, duration_truth.next
        if (
            all(snapshot.values.values())
            and snapshot.values["guard_state"].data["guard_valid"]
            and before
            and after
            and before.valid
            and after.valid
            and after.sequence == before.sequence + 1
            and after.time_ns - before.time_ns <= rule.max_gap_ns
        ):
            evidence.result.eligible_duration_ns += max(
                0, min(snapshot.end_ns, after.time_ns) - snapshot.time_ns
            )
    evidence.result.metrics.update(
        eligible_samples=samples,
        full_run_matched_samples=full_samples,
        conditional_rmse_m=math.sqrt(squared_sum / samples) if samples else None,
        full_run_rmse_m=math.sqrt(full_sum / full_samples) if full_samples else None,
    )
    return evidence.finish()
