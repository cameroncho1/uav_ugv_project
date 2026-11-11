"""Bounded command, watchdog, and hazard responses with explicit incomplete horizons."""

from __future__ import annotations

from dataclasses import dataclass

from ..contracts import Interval, Record, Rule, TraceReader
from ..signals.geometry import segment_clearance
from ..signals.timeline import align, as_of, events, ordered_after, quiet
from .common import Evidence


@dataclass
class Obligation:
    start_ns: int
    recovered_ns: int | None = None
    responded_ns: int | None = None
    unknown: bool = False
    violated: bool = False
    guard_gap: bool = False
    response_gap: bool = False


def guard_response(reader: TraceReader, rule: Rule, interval: Interval):
    evidence = Evidence(rule, interval)
    pending: list[Obligation] = []
    active = None
    before_valid = True
    p = rule.parameters
    for snapshot in align(reader, ("guard_state", "requested_command"), interval, rule.max_gap_ns):
        t = snapshot.time_ns
        state, command = snapshot.values["guard_state"], snapshot.values["requested_command"]
        if state is None:
            if snapshot.duration_ns:
                evidence.missing(t, "guard_coverage_gap")
        else:
            valid = state.data["guard_valid"]
            if not valid and before_valid:
                active = Obligation(t)
                pending.append(active)
                evidence.result.trigger_count += 1
                evidence.checks += 1
            elif valid and not before_valid and active:
                active.recovered_ns = t
                active = None
            before_valid = valid
        for obligation in pending:
            deadline = obligation.start_ns + p["response_ns"]
            if command is None or state is None:
                if snapshot.duration_ns and (t <= deadline or obligation.recovered_ns is None):
                    obligation.response_gap = True
                    # A witnessed bounded response needs no command prehistory.
                    # After that response, the sustained hold needs full coverage.
                    obligation.unknown |= state is None or obligation.responded_ns is not None
                    obligation.guard_gap |= state is None
            if obligation.responded_ns is None and t <= deadline and command and quiet(command, p):
                obligation.responded_ns = t
                evidence.minimum("minimum_response_slack_ns", deadline - t)
                evidence.maximum("maximum_response_latency_ns", t - obligation.start_ns)
            # Inspect every same-source emission, not just the last value at a timestamp.
            stopped_in_batch = obligation.responded_ns is not None and obligation.responded_ns < t
            for emission in snapshot.updates["requested_command"]:
                if not emission.valid or emission.time_ns < obligation.start_ns:
                    continue
                if quiet(emission, p) and emission.time_ns <= deadline:
                    stopped_in_batch = True
                    if obligation.responded_ns is None:
                        obligation.responded_ns = emission.time_ns
                elif (
                    not quiet(emission, p)
                    and stopped_in_batch
                    and obligation.recovered_ns is None
                    and not obligation.guard_gap
                ):
                    if t == obligation.start_ns:
                        obligation.unknown = True  # Guard/command cross-source tie.
                    elif not obligation.violated:
                        evidence.fail(t, "intermediate_command_resumed_during_hold")
                        obligation.violated = True
            if (
                obligation.responded_ns is not None
                and obligation.recovered_ns is None
                and not obligation.guard_gap
                and state is not None
            ):
                updates = snapshot.updates["requested_command"]
                # Include intermediate same-time command values after a prior successful stop.
                nonzero = command is not None and not quiet(command, p)
                if obligation.responded_ns < t:
                    nonzero |= any(r.valid and not quiet(r, p) for r in updates)
                if nonzero and not obligation.violated:
                    evidence.fail(t, "command_resumed_before_guard_recovery")
                    obligation.violated = True
            if obligation.responded_ns is None and t >= deadline:
                if obligation.unknown or obligation.response_gap:
                    evidence.missing(t, "guard_response_window_incomplete")
                else:
                    evidence.fail(deadline, "guard_response_deadline_missed")
                obligation.violated = True
        remaining = []
        for obligation in pending:
            done = obligation.violated or (
                obligation.responded_ns is not None
                and (obligation.recovered_ns is not None or t == interval.end_ns)
            )
            if done:
                if obligation.unknown:
                    evidence.missing(t, "guard_hold_coverage_incomplete")
                elif not obligation.violated:
                    evidence.result.completed_obligations += 1
            else:
                remaining.append(obligation)
        pending = remaining
        if state and command:
            evidence.result.eligible_duration_ns += snapshot.duration_ns
    for _ in pending:
        evidence.missing(interval.end_ns, "guard_deadline_beyond_recording")
    return evidence.finish()


def _window(interval: Interval, start: int, end: int) -> Interval:
    return Interval(
        interval.clock, interval.epoch, max(interval.start_ns, start), min(interval.end_ns, end)
    )


def _first_quiet(reader, stream, interval, gap, parameters, dwell_ns=0):
    """Return onset of a confirmed quiet dwell and its timing uncertainty."""
    since, unknown = None, False
    for snapshot in align(reader, (stream,), interval, gap):
        record = snapshot.values[stream]
        if record is None:
            unknown |= snapshot.duration_ns > 0
            since = None
            continue
        if stream == "measured_motion":
            if not record.data["simulation_advancing"]:
                unknown = True
                since = None
                continue
            stopped = (
                abs(record.data["linear"]) <= parameters["speed_epsilon_mps"]
                and abs(record.data["angular"]) <= parameters["yaw_rate_epsilon_rps"]
            )
        else:
            stopped = quiet(record, parameters)
        if not stopped:
            since = None
            continue
        since = snapshot.time_ns if since is None else since
        completion = since + dwell_ns
        if completion <= snapshot.end_ns:
            return since, unknown, record.data.get("uncertainty_ns", 0)
    return None, unknown, 0


def watchdog_response(reader: TraceReader, rule: Rule, interval: Interval):
    evidence = Evidence(rule, interval)
    p = rule.parameters
    for event in events(reader, interval, "command_loss"):
        evidence.checks += 1
        evidence.result.trigger_count += 1
        accepted = None
        for record in reader.records("actuator_input", interval):
            if record.time_ns > event.time_ns:
                break
            if record.valid and record.data["accepted"]:
                accepted = record
        output = as_of(reader, "actuator_output", interval, event.time_ns, rule.max_gap_ns)
        if accepted is None or output is None or quiet(accepted, p) or quiet(output, p):
            evidence.missing(event.time_ns, "nonzero_pre_loss_input_output_not_established")
            continue
        start = accepted.time_ns
        final_deadline = start + max(p["expiry_ns"], p["settling_ns"] + p["dwell_ns"])
        resumed = any(
            r.valid and r.data["accepted"] and start < r.time_ns <= final_deadline
            for r in reader.records("actuator_input", interval)
        )
        if resumed:
            evidence.missing(event.time_ns, "accepted_input_resumed_inside_loss_window")
            continue
        complete = True
        for stream, bound, dwell in (
            ("actuator_output", p["expiry_ns"], 0),
            ("measured_motion", p["settling_ns"], p["dwell_ns"]),
        ):
            window = _window(interval, event.time_ns, start + bound + dwell)
            if window.end_ns < window.start_ns:
                evidence.missing(event.time_ns, f"{stream}_loss_event_after_deadline")
                complete = False
                continue
            response, unknown, uncertainty = _first_quiet(
                reader, stream, window, rule.max_gap_ns, p, dwell
            )
            if response is not None:
                if response - uncertainty > start + bound:
                    evidence.fail(response, f"{stream}_deadline_missed")
                    complete = False
                elif response + uncertainty > start + bound:
                    evidence.missing(response, f"{stream}_deadline_overlaps_sampling_uncertainty")
                    complete = False
                else:
                    evidence.maximum(f"{stream}_maximum_latency_ns", response - start)
                    evidence.minimum(f"{stream}_minimum_slack_ns", start + bound - response)
                    evidence.result.eligible_duration_ns += max(0, response - event.time_ns)
            elif unknown or interval.end_ns < start + bound + dwell:
                evidence.missing(window.end_ns, f"{stream}_response_incomplete")
                complete = False
            else:
                evidence.fail(start + bound, f"{stream}_deadline_missed")
                complete = False
        if complete:
            evidence.result.completed_obligations += 1
    return evidence.finish()


def _route_clear(route: Record, obstacles: Record, event: Record, radius: float) -> bool | None:
    if route.data["goal_revision"] != event.data["goal_revision"]:
        return False
    if not any(shape.get("id") == event.data["obstacle_id"] for shape in obstacles.data["shapes"]):
        return None
    if not ordered_after(route, event):
        return None if ordered_after(route, event) is None else False
    points = route.data["points"]
    return all(
        segment_clearance(a, b, shape, radius) >= 0
        for a, b in zip(points, points[1:], strict=False)
        for shape in obstacles.data["shapes"]
    )


def hazard_response(reader: TraceReader, rule: Rule, interval: Interval):
    evidence = Evidence(rule, interval)
    p = rule.parameters
    for event in events(reader, interval, "hazard_observed"):
        if not event.data["qualifying"]:
            continue
        evidence.checks += 1
        evidence.result.trigger_count += 1
        deadline = event.time_ns + p["response_ns"]
        window = _window(interval, event.time_ns, deadline)
        unknown, response, stopped_since = False, None, None
        for snapshot in align(
            reader, ("route", "obstacles", "requested_command"), window, rule.max_gap_ns
        ):
            route, obstacles, command = (
                snapshot.values[s] for s in ("route", "obstacles", "requested_command")
            )
            if command and quiet(command, p):
                stopped_since = snapshot.time_ns if stopped_since is None else stopped_since
                if stopped_since + p["stop_dwell_ns"] <= snapshot.end_ns:
                    response = stopped_since + p["stop_dwell_ns"]
                    break
            else:
                stopped_since = None
            if route and obstacles:
                clear = _route_clear(route, obstacles, event, p["radius_m"])
                if clear:
                    response = snapshot.time_ns
                    break
                unknown |= clear is None
            unknown |= command is None or route is None or obstacles is None
        if response is not None:
            evidence.result.completed_obligations += 1
            evidence.maximum("maximum_response_latency_ns", response - event.time_ns)
            evidence.minimum("minimum_response_slack_ns", deadline - response)
        elif unknown or interval.end_ns < deadline:
            evidence.missing(window.end_ns, "hazard_response_or_geometry_incomplete")
        else:
            evidence.fail(deadline, "hazard_response_deadline_missed")
    return evidence.finish()
