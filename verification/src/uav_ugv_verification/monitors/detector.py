"""Event detection and exposure-based false alarms; labels are not predictions."""

from __future__ import annotations

from ..contracts import Interval, Rule, TraceReader
from ..signals.timeline import events, ordered_after
from .common import Evidence, decision_key, matching_decision


def detection(reader: TraceReader, rule: Rule, interval: Interval):
    p = rule.parameters
    evidence = Evidence(rule, interval, version=f"1.{p['alarm_scope']}")
    evidence.result.metrics["pre_existing_alarms"] = 0
    for event in events(reader, interval, "fault_start"):
        evidence.checks += 1
        evidence.result.trigger_count += 1
        deadline = event.time_ns + p["response_ns"]
        previous, response, scope_seen = None, None, False
        last_stamp, unknown = None, False
        for decision in reader.records("detector_decision", interval):
            if not decision.valid:
                unknown |= event.time_ns <= decision.time_ns <= deadline
                previous = None
                continue
            if not matching_decision(decision, event, p["alarm_scope"]):
                continue
            scope_seen = True
            if decision.time_ns > min(deadline, interval.end_ns):
                break
            alarm = decision.data["alarm"]
            if decision.time_ns < event.time_ns:
                previous = alarm
                last_stamp = decision.time_ns
                continue
            if last_stamp is None or decision.time_ns - last_stamp >= rule.max_gap_ns:
                unknown = True
            if alarm and previous is False:
                if ordered_after(decision, event) is True:
                    response = decision.time_ns
                    break
                unknown = True
            if previous is None and alarm:
                unknown = True  # No recorded clear state establishes a new alarm.
            previous, last_stamp = alarm, decision.time_ns
        if scope_seen:
            before = None
            for decision in reader.records("detector_decision", interval):
                if decision.time_ns >= event.time_ns:
                    break
                if decision.valid and matching_decision(decision, event, p["alarm_scope"]):
                    before = decision
            if before and before.data["alarm"]:
                evidence.result.metrics["pre_existing_alarms"] += 1
        if response is not None:
            evidence.result.completed_obligations += 1
            evidence.maximum("maximum_detection_latency_ns", response - event.time_ns)
            evidence.minimum("minimum_detection_slack_ns", deadline - response)
        elif (
            not scope_seen
            or unknown
            or interval.end_ns < deadline
            or last_stamp is None
            or deadline - last_stamp >= rule.max_gap_ns
        ):
            evidence.missing(
                min(deadline, interval.end_ns), "detection_scope_or_horizon_incomplete"
            )
        else:
            evidence.fail(deadline, "new_matching_alarm_not_detected")
    return evidence.finish()


def _fault_windows(reader, interval, recovery_ns):
    active, windows = {}, []
    for event in reader.records("events", interval):
        if not event.valid:
            continue
        key, kind = event.data["event_id"], event.data["kind"]
        if kind == "fault_start":
            active[key] = event.time_ns
        elif kind == "fault_end":
            windows.append((active.pop(key, interval.start_ns), event.time_ns + recovery_ns))
    windows.extend((start, interval.end_ns + 1) for start in active.values())
    return windows


def false_alarms(reader: TraceReader, rule: Rule, interval: Interval):
    evidence = Evidence(rule, interval)
    p = rule.parameters
    windows = _fault_windows(reader, interval, p["recovery_ns"])
    required = set(p["decision_keys"])
    records = iter(reader.records("detector_decision", interval))
    upcoming = next(records, None)
    current, alarm_active, last_alarm_end = {}, {}, {}
    excluded_boundaries = {value for window in windows for value in window}
    excluded_boundaries.add(interval.start_ns + p["warmup_ns"])
    time_ns = interval.start_ns
    alarm_events = 0
    observed_alarm = False
    while time_ns <= interval.end_ns:
        while upcoming is not None and upcoming.time_ns <= time_ns:
            if upcoming.valid:
                current[decision_key(upcoming)] = upcoming
            else:
                current.clear()
            upcoming = next(records, None)
        boundaries = [interval.end_ns]
        if upcoming:
            boundaries.append(upcoming.time_ns)
        boundaries.extend(t for t in excluded_boundaries if time_ns < t <= interval.end_ns)
        boundaries.extend(
            r.time_ns + rule.max_gap_ns
            for r in current.values()
            if time_ns < r.time_ns + rule.max_gap_ns <= interval.end_ns
        )
        end_ns = min(t for t in boundaries if t >= time_ns)
        nominal = (
            end_ns > time_ns
            and time_ns >= interval.start_ns + p["warmup_ns"]
            and not any(start <= time_ns < end for start, end in windows)
        )
        if nominal:
            available = {key: current.get(key) for key in required}
            available = {
                key: r
                for key, r in available.items()
                if r is not None and r.valid and time_ns < r.time_ns + rule.max_gap_ns
            }
            if len(available) != len(required):
                if end_ns > time_ns:
                    evidence.missing(time_ns, "nominal_detector_coverage_gap")
            else:
                evidence.checks += 1
                evidence.result.eligible_duration_ns += end_ns - time_ns
            for key, record in available.items():
                alarm = record.data["alarm"]
                if alarm:
                    observed_alarm = True
                    if not alarm_active.get(key, False):
                        if (
                            key not in last_alarm_end
                            or time_ns - last_alarm_end[key] > p["merge_gap_ns"]
                        ):
                            alarm_events += 1
                        evidence.fail(time_ns, "alarm_during_nominal_exposure", decision_key=key)
                    last_alarm_end[key] = end_ns
                alarm_active[key] = alarm
        else:
            alarm_active = {}
        if time_ns == interval.end_ns:
            break
        time_ns = end_ns
    duration = evidence.result.eligible_duration_ns
    evidence.result.metrics.update(
        false_alarm_events=alarm_events,
        nominal_exposure_ns=duration,
        false_alarm_events_per_hour=alarm_events * 3_600_000_000_000 / duration
        if duration
        else None,
        fault_free_run=not bool(windows),
        observed_nominal_alarm=observed_alarm,
    )
    return evidence.finish()
