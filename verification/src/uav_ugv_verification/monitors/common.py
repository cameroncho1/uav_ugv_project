"""Evidence accounting shared by checks, including bounded witness storage."""

from __future__ import annotations

from dataclasses import asdict

from ..contracts import Interval, RequirementResult, Rule, Verdict


class Evidence:
    def __init__(self, rule: Rule, interval: Interval, *, version: str = "1"):
        self.result = RequirementResult(
            rule.requirement,
            version,
            rule.mandatory,
            Verdict.INCONCLUSIVE,
            "unexercised",
            rule.parameters.copy(),
            [asdict(interval)],
        )
        self.failures = 0
        self.checks = 0
        self.omitted = 0

    def fail(self, time_ns: int, reason: str, **details) -> None:
        self.failures += 1
        self._witness(time_ns, reason, **details)

    def missing(self, time_ns: int, reason: str) -> None:
        self.result.missing_data_count += 1
        self._witness(time_ns, reason)

    def _witness(self, time_ns: int, reason: str, **details) -> None:
        if len(self.result.witnesses) < 32:
            self.result.witnesses.append({"time_ns": time_ns, "reason": reason, **details})
        else:
            self.omitted += 1

    def minimum(self, name: str, value: float | int) -> None:
        self.result.metrics[name] = min(self.result.metrics.get(name, value), value)

    def maximum(self, name: str, value: float | int) -> None:
        self.result.metrics[name] = max(self.result.metrics.get(name, value), value)

    def finish(self) -> RequirementResult:
        if self.failures:
            self.result.verdict, self.result.reason = Verdict.FAIL, "witnessed_violation"
        elif self.result.missing_data_count:
            self.result.verdict, self.result.reason = Verdict.INCONCLUSIVE, "incomplete_evidence"
        elif self.checks:
            self.result.verdict, self.result.reason = Verdict.PASS, "property_holds"
        self.result.metrics["checks"] = self.checks
        self.result.metrics["violations"] = self.failures
        if self.omitted:
            self.result.metrics["omitted_witnesses"] = self.omitted
        return self.result


def matching_decision(record, event, scope: str) -> bool:
    data = record.data
    if data["scope"] != scope:
        return False
    if scope == "binary":
        return True
    if (data["sensor"], data["channel"]) != (event.data["sensor"], event.data["channel"]):
        return False
    return scope != "class" or data["fault_class"] == event.data["fault_class"]


def decision_key(record) -> str:
    data = record.data
    if data["scope"] == "binary":
        return "binary"
    key = f"{data['scope']}:{data['sensor']}:{data['channel']}"
    return f"{key}:{data['fault_class']}" if data["scope"] == "class" else key
