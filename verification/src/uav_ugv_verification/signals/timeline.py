"""Merge right-continuous streams with explicit validity and sequence gaps."""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass

from ..contracts import Interval, Record, TraceReader


@dataclass(frozen=True)
class Snapshot:
    time_ns: int
    end_ns: int
    values: dict[str, Record | None]
    updates: dict[str, tuple[Record, ...]]

    @property
    def duration_ns(self) -> int:
        return self.end_ns - self.time_ns


class Cursor:
    def __init__(self, reader: TraceReader, stream: str, interval: Interval, gap: int):
        self.records = iter(reader.records(stream, interval))
        self.next = next(self.records, None)
        self.current = None
        self.gap = gap
        self.stream = stream
        self.before = None
        self._last_time = None
        self._updates = ()

    @property
    def expires_ns(self) -> int | None:
        if self.current is None or self.stream == "goal":
            return None
        if self.stream == "obstacles" and self.current.data.get("static", False):
            return None
        return self.current.time_ns + self.gap

    def advance(self, time_ns: int) -> tuple[Record, ...]:
        if self._last_time == time_ns:
            return self._updates
        if self._last_time is not None and time_ns < self._last_time:
            raise ValueError("cursor cannot move backward inside an epoch")
        updates = []
        self.before = self.current
        while self.next is not None and self.next.time_ns <= time_ns:
            self.current = self.next
            if self.current.time_ns == time_ns:
                updates.append(self.current)
            else:
                self.before = self.current
            self.next = next(self.records, None)
        self._last_time = time_ns
        self._updates = tuple(updates)
        return self._updates

    def value(self, time_ns: int) -> Record | None:
        if (
            self.current is None
            or not self.current.valid
            or (self.expires_ns is not None and time_ns >= self.expires_ns)
            or (self.next is not None and self.next.sequence != self.current.sequence + 1)
        ):
            return None
        return self.current


def align(
    reader: TraceReader,
    streams: tuple[str, ...],
    interval: Interval,
    gap_ns: int,
    *,
    gaps: dict[str, int] | None = None,
) -> Iterator[Snapshot]:
    """Memory is bounded by the number of streams and equal-time update batches."""
    cursors = {
        name: Cursor(reader, name, interval, (gaps or {}).get(name, gap_ns)) for name in streams
    }
    time_ns = interval.start_ns
    while time_ns <= interval.end_ns:
        updates = {name: cursor.advance(time_ns) for name, cursor in cursors.items()}
        values = {name: cursor.value(time_ns) for name, cursor in cursors.items()}
        candidates = [interval.end_ns]
        for cursor in cursors.values():
            if cursor.next is not None and cursor.next.time_ns > time_ns:
                candidates.append(cursor.next.time_ns)
            if cursor.expires_ns is not None and cursor.expires_ns > time_ns:
                candidates.append(cursor.expires_ns)
        end_ns = min(t for t in candidates if t >= time_ns)
        yield Snapshot(time_ns, end_ns, values, updates)
        if time_ns == interval.end_ns:
            break
        time_ns = end_ns


def as_of(
    reader: TraceReader, stream: str, interval: Interval, at_ns: int, gap_ns: int
) -> Record | None:
    cursor = Cursor(reader, stream, interval, gap_ns)
    cursor.advance(at_ns)
    return cursor.value(at_ns)


def ordered_after(response: Record, trigger: Record) -> bool | None:
    if response.time_ns != trigger.time_ns:
        return response.time_ns > trigger.time_ns
    if (
        response.source == trigger.source
        and response.source_sequence is not None
        and trigger.source_sequence is not None
        and response.source_sequence != trigger.source_sequence
    ):
        return response.source_sequence > trigger.source_sequence
    return None


def quiet(record: Record, parameters: dict) -> bool:
    return (
        abs(record.data["linear"]) <= parameters["linear_epsilon_mps"]
        and abs(record.data["angular"]) <= parameters["angular_epsilon_rps"]
    )


def events(reader: TraceReader, interval: Interval, kind: str) -> Iterator[Record]:
    for record in reader.records("events", interval):
        if (
            record.valid
            and interval.start_ns <= record.time_ns <= interval.end_ns
            and record.data["kind"] == kind
        ):
            yield record
