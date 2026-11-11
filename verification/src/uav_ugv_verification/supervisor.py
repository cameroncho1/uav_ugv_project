"""Explicit-state safety checking and C++ conformance for the navigation supervisor.

The graph includes fault-recovery memory and an independent completed-goal monitor.
Exact timestamp validation and physical motion are outside this finite abstraction.
"""

from __future__ import annotations

import hashlib
import itertools
import json
import subprocess
from collections import deque
from dataclasses import asdict, dataclass
from enum import IntEnum
from pathlib import Path

from .validation import digest_file, read_json, require


class Mode(IntEnum):
    INIT = 0
    NAVIGATE = 1
    HOLD_GUARD = 2
    HOLD_FAULT = 3
    GOAL = 4


class Fault(IntEnum):
    NONE = 0
    CLEAR = 1
    ALARM = 2
    UNAVAILABLE = 3


class Age(IntEnum):
    NONE = 0
    YOUNG = 1
    MATURE = 2


@dataclass(frozen=True)
class State:
    enabled: bool
    mode: Mode = Mode.INIT
    fault_latched: bool = False
    clear_count: int = 0
    age: Age = Age.NONE
    goal_completed: bool = False


@dataclass(frozen=True)
class Input:
    have_goal: bool = False
    pose_valid: bool = False
    route_valid: bool = False
    goal_reached: bool = False
    new_goal: bool = False
    fault: Fault = Fault.NONE
    mature: bool = False
    reset: bool = False


PROPERTIES = {
    "S1_navigation_authorization": "G(permit -> goal & pose_valid & route_valid & !fault & !done)",
    "S2_alarm_hold": "G(enabled & (alarm | unavailable) -> X fault_latched)",
    "S3_recovery_threshold": "G(fault_latched & !release_ready -> X fault_latched)",
    "S4_distinct_windows": "G(no_new_clear & still_latched -> X count <= count)",
    "S5_completed_goal_hold": "G(completed -> (!permit W new_valid_goal))",
    "S6_completion_memory": "G(done & !new_valid_goal -> X done)",
    "S7_ready_progress": "G(ready & !done -> X NAVIGATE)",
    "S8_reset_hold": "G(reset -> X(INIT & count=0 & detector_unknown))",
    "S11_recovery_memory_integrity": "G(recovery_memory_next = specified_update(memory,input))",
    "S10_completion_integrity": "G(done_next <-> ((done & !new_goal) | eligible_completion))",
    "S9_memory_bounds": "G(count in [0,N] & (count=0 <-> age=NONE))",
}


def policy(path: Path) -> dict:
    value = read_json(path)
    require(
        value["schema_version"] == 1 and isinstance(value.get("id"), str) and bool(value["id"]),
        "unsupported supervisor policy",
    )
    require(len(value["decisions"]) == 6, "each decision condition must occur once")
    require(
        all(r["mode"] in Mode.__members__ for r in value["decisions"]), "unknown supervisor mode"
    )
    for name in ("minimum_clear_ns", "maximum_decision_age_ns", "maximum_decision_gap_ns"):
        v = value["recovery"][name]
        require(type(v) is int and 0 < v <= 10**12, "invalid recovery timing parameter")
    require(
        {r["when"] for r in value["decisions"]}
        == {
            "no_goal",
            "fault_latched",
            "invalid_pose",
            "goal_completed",
            "invalid_route",
            "always",
        },
        "incomplete decision table",
    )
    require(value["decisions"][-1]["when"] == "always", "unconditional rule must be last")
    count = value["recovery"]["clear_windows"]
    require(type(count) is int and 0 < count <= 100, "invalid clear-window bound")
    return value


def step(before: State, event: Input, rules: dict, mutant: str | None = None) -> State:
    """Reference semantics; C++ executes its own numerical transition function."""
    if event.reset:
        return State(
            before.enabled, fault_latched=before.enabled, goal_completed=before.goal_completed
        )
    count, age, latched = before.clear_count, before.age, before.fault_latched
    if before.enabled:
        if event.fault in (Fault.ALARM, Fault.UNAVAILABLE):
            count, age, latched = 0, Age.NONE, True
        elif latched:
            if event.fault == Fault.CLEAR:
                age = Age.YOUNG if count == 0 else age
                count = min(
                    rules["recovery"]["clear_windows"],
                    count + (2 if mutant == "double_count" else 1),
                )
            if count and (event.mature or mutant == "invented_maturity"):
                age = Age.MATURE
            if (
                mutant != "permanent_fault"
                and count == rules["recovery"]["clear_windows"]
                and (age == Age.MATURE or mutant == "early_recovery")
            ):
                count, age, latched = 0, Age.NONE, False
    done = before.goal_completed and not event.new_goal
    done |= event.have_goal and event.pose_valid and event.goal_reached and not latched
    if mutant == "forgot_completion" and not event.pose_valid:
        done = False
    if mutant == "forgot_creation":
        done = before.goal_completed and not event.new_goal
    if mutant == "spurious_completion" and event.have_goal:
        done = True
    facts = {
        "no_goal": not event.have_goal,
        "fault_latched": latched,
        "invalid_pose": not event.pose_valid,
        "goal_completed": done,
        "invalid_route": not event.route_valid,
        "always": True,
    }
    mode = next(Mode[r["mode"]] for r in rules["decisions"] if facts[r["when"]])
    if mutant == "unsafe_navigation" and event.have_goal and not event.pose_valid:
        mode = Mode.NAVIGATE
    if mutant == "spurious_hold" and mode == Mode.NAVIGATE:
        mode = Mode.HOLD_GUARD
    return State(before.enabled, mode, latched, count, age, done)


def inputs(state: State):
    # Quotient of admissible validator outputs. Impossible goal/timer combinations
    # are excluded explicitly; irrelevant fault fields are canonical when disabled.
    for goal, pose, route, reached, revised in itertools.product((False, True), repeat=5):
        if (revised or route) and not goal or reached and not (goal and pose):
            continue
        for fault in Fault if state.enabled else (Fault.NONE,):
            ages = (
                (False, True)
                if state.clear_count and fault in (Fault.NONE, Fault.CLEAR)
                else (False,)
            )
            for mature in ages:
                yield Input(goal, pose, route, reached, revised, fault, mature)
    yield Input(reset=True)


def violations(before: State, event: Input, after: State, remembered_done: bool, rules: dict):
    """Property predicates are independent of the transition table/interpreter."""
    permitted = after.mode == Mode.NAVIGATE
    # The obligation is derived from pre-state/input, never from DUT post-state.
    alarm = before.enabled and event.fault in (Fault.ALARM, Fault.UNAVAILABLE)
    release = (
        before.clear_count + (event.fault == Fault.CLEAR) >= rules["recovery"]["clear_windows"]
        and (before.age == Age.MATURE or event.mature)
        and not alarm
    )
    expected_fault = before.enabled and (alarm or before.fault_latched and not release)
    expected_count, expected_age = before.clear_count, before.age
    if event.reset:
        expected_fault, expected_count, expected_age = before.enabled, 0, Age.NONE
    elif not before.enabled or not expected_fault or alarm:
        expected_count, expected_age = 0, Age.NONE
    else:
        expected_count = min(
            rules["recovery"]["clear_windows"], before.clear_count + (event.fault == Fault.CLEAR)
        )
        if expected_count == 0:
            expected_age = Age.NONE
        elif event.mature or before.age == Age.MATURE:
            expected_age = Age.MATURE
        else:
            expected_age = Age.YOUNG
    if (after.fault_latched, after.clear_count, after.age) != (
        expected_fault,
        expected_count,
        expected_age,
    ):
        yield "S11_recovery_memory_integrity"
    expected_done = remembered_done and not event.new_goal
    if not event.reset:
        expected_done |= (
            event.have_goal and event.pose_valid and event.goal_reached and not expected_fault
        )
    if after.goal_completed != expected_done:
        yield "S10_completion_integrity"
    if event.reset:
        if (
            after.mode != Mode.INIT
            or after.clear_count
            or after.age != Age.NONE
            or after.fault_latched != before.enabled
        ):
            yield "S8_reset_hold"
    else:
        if permitted and not (
            event.have_goal
            and event.pose_valid
            and event.route_valid
            and not after.fault_latched
            and not after.goal_completed
        ):
            yield "S1_navigation_authorization"
        if (
            before.enabled
            and event.fault in (Fault.ALARM, Fault.UNAVAILABLE)
            and not after.fault_latched
        ):
            yield "S2_alarm_hold"
        if before.fault_latched and not after.fault_latched:
            windows = before.clear_count + (event.fault == Fault.CLEAR)
            if (
                windows < rules["recovery"]["clear_windows"]
                or not (before.age == Age.MATURE or event.mature)
                or event.fault in (Fault.ALARM, Fault.UNAVAILABLE)
            ):
                yield "S3_recovery_threshold"
        if (
            before.fault_latched
            and after.fault_latched
            and event.fault != Fault.CLEAR
            and after.clear_count > before.clear_count
        ):
            yield "S4_distinct_windows"
        ready = (
            event.have_goal
            and event.pose_valid
            and event.route_valid
            and not expected_fault
            and not expected_done
        )
        if ready and after.mode != Mode.NAVIGATE:
            yield "S7_ready_progress"
    if remembered_done and not event.new_goal and permitted:
        yield "S5_completed_goal_hold"
    if before.goal_completed and not event.new_goal and not after.goal_completed:
        yield "S6_completion_memory"
    if not 0 <= after.clear_count <= rules["recovery"]["clear_windows"] or (
        after.clear_count == 0
    ) != (after.age == Age.NONE):
        yield "S9_memory_bounds"


def values(value):
    return tuple(int(v) for v in asdict(value).values())


def named(value):
    return {k: v.name if isinstance(v, IntEnum) else v for k, v in asdict(value).items()}


def explore(rules: dict, mutant: str | None = None, *, collect=False) -> tuple[dict, list]:
    roots = [(State(enabled, fault_latched=enabled), False) for enabled in (False, True)]
    parents = dict.fromkeys(roots)
    pending = deque(roots)
    transitions, cases = 0, []
    digest = hashlib.sha256()
    while pending:
        node = pending.popleft()
        before, remembered = node
        for event in inputs(before):
            after = step(before, event, rules, mutant)
            transitions += 1
            serialized = " ".join(map(str, (*values(before), *values(event)))) + "\n"
            digest.update(serialized.encode())
            errors = list(violations(before, event, after, remembered, rules))
            if errors:
                path = [dict(before=named(before), input=named(event), after=named(after))]
                cursor = node
                while parents[cursor] is not None:
                    previous, incoming = parents[cursor]
                    path.append(
                        dict(
                            before=named(previous[0]), input=named(incoming), after=named(cursor[0])
                        )
                    )
                    cursor = previous
                return dict(
                    status="FAIL",
                    properties_failed=errors,
                    counterexample=list(reversed(path)),
                    states=len(parents),
                    transitions=transitions,
                ), cases
            if collect:
                cases.append((serialized, values(after) + (int(after.mode == Mode.NAVIGATE),)))
            observed_completion = (
                event.have_goal
                and event.pose_valid
                and event.goal_reached
                and not after.fault_latched
                and not event.reset
            )
            monitor = (remembered and not event.new_goal) or observed_completion
            following = (after, monitor)
            if following not in parents:
                parents[following] = (node, event)
                pending.append(following)
    return dict(
        status="PASS",
        product_states=len(parents),
        controller_states=len({state for state, _ in parents}),
        transitions=transitions,
        input_corpus_sha256=digest.hexdigest(),
        properties=PROPERTIES,
        roots=2,
        exploration="breadth-first reachable-state/product-monitor safety check",
    ), cases


def check(policy_path: Path, probe: Path | None = None, *, repository: Path | None = None) -> dict:
    rules = policy(policy_path)
    result, cases = explore(rules, collect=probe is not None)
    if result["status"] != "PASS":
        result.update(
            schema_version=1,
            policy_id=rules["id"],
            policy_sha256=digest_file(policy_path),
            conformance=dict(status="NOT_RUN", reason="model safety violation"),
        )
        return result
    mutants = {}
    for name in (
        "unsafe_navigation",
        "early_recovery",
        "forgot_completion",
        "spurious_hold",
        "permanent_fault",
        "forgot_creation",
        "spurious_completion",
        "double_count",
        "invented_maturity",
    ):
        failure, _ = explore(rules, name)
        require(failure["status"] == "FAIL", f"mutation escaped checking: {name}")
        mutants[name] = failure
    result.update(
        schema_version=1,
        policy_id=rules["id"],
        policy_sha256=digest_file(policy_path),
        recovery=rules["recovery"],
        mutations=mutants,
        checker_sha256=digest_file(Path(__file__)),
    )
    repository = repository or policy_path.resolve().parents[2]
    names = [
        "include/ugv_control/supervisor.hpp",
        "include/ugv_control/supervisor_inputs.hpp",
        "src/supervisor_transition_probe.cpp",
        "src/goal_controller.cpp",
    ]
    hashes = {
        "ros2_ws/src/ugv_control/" + name: digest_file(
            repository / "ros2_ws/src/ugv_control" / name
        )
        for name in names
    }
    hashes["scripts/generate_supervisor_policy.py"] = digest_file(
        repository / "scripts/generate_supervisor_policy.py"
    )
    result["implementation_sources"] = hashes
    result["implementation_sha256"] = hashlib.sha256(
        json.dumps(hashes, sort_keys=True).encode()
    ).hexdigest()
    if probe is not None:
        identity = subprocess.check_output([str(probe), "--identity"], text=True).split()
        require(
            identity == [result["policy_sha256"], result["implementation_sha256"]],
            "C++ probe policy/source digest differs; rebuild ugv_control",
        )
        corpus = "".join(line for line, _ in cases)
        process = subprocess.run(
            [str(probe)], input=corpus, capture_output=True, text=True, check=True, timeout=60
        )
        lines = process.stdout.splitlines()
        require(len(lines) == len(cases), "C++ conformance output is incomplete")
        for index, (line, (_, expected)) in enumerate(zip(lines, cases, strict=True)):
            actual = tuple(map(int, line.split()))
            require(
                actual == expected,
                f"C++ conformance mismatch at transition {index}: {actual} != {expected}",
            )
        result["conformance"] = dict(
            status="PASS",
            transitions=len(cases),
            probe_sha256=digest_file(probe),
            output_sha256=hashlib.sha256(process.stdout.encode()).hexdigest(),
        )
    else:
        result["conformance"] = dict(status="NOT_RUN", reason="compiled C++ probe required")
    return result
