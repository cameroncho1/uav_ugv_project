"""Incremental normalized-JSONL and existing sensor-dataset CSV readers."""

from __future__ import annotations

import csv
from collections import defaultdict
from collections.abc import Iterator
from dataclasses import asdict, replace
from decimal import Decimal, InvalidOperation
from pathlib import Path

from ..contracts import Interval, Manifest, Record
from ..validation import (
    ValidationError,
    digest_file,
    finite_tree,
    mapping,
    parse_json,
    parse_manifest,
    parse_record,
    read_json,
    require,
    text,
    validate_payload,
)


def seconds_to_ns(value: str) -> int:
    try:
        seconds = Decimal(value)
        require(seconds.is_finite() and seconds >= 0, "CSV stamp must be finite and nonnegative")
        return int((seconds * 1_000_000_000).to_integral_value())
    except (InvalidOperation, ValueError) as exc:
        raise ValidationError(f"invalid CSV timestamp {value!r}") from exc


class FileReader:
    def __init__(self, root: Path):
        self.root = root.resolve()
        self.manifest = parse_manifest(read_json(self.root / "manifest.json"))
        require(self.manifest.adapter == "normalized", "manifest: use normalized adapter")
        self.files = {name: self._path(path) for name, path in self.manifest.streams.items()}

    def _path(self, value: str) -> Path:
        path = (self.root / value).resolve()
        require(path.is_relative_to(self.root), f"source path escapes run directory: {value}")
        require(path.is_file(), f"missing source file: {path}")
        return path

    @property
    def streams(self) -> frozenset[str]:
        return frozenset(self.files)

    def _all_records(self, stream: str) -> Iterator[Record]:
        path = self.files[stream]
        with path.open(encoding="utf-8-sig") as handle:
            for line_number, line in enumerate(handle, 1):
                try:
                    raw = parse_json(line)
                    yield parse_record(raw, stream)
                except (ValueError, TypeError) as exc:
                    raise ValidationError(f"{path}:{line_number}: {exc}") from exc

    def records(self, stream: str, interval: Interval) -> Iterator[Record]:
        if stream not in self.streams:
            return
        for record in self._all_records(stream):
            if (record.clock, record.epoch) == (interval.clock, interval.epoch):
                if record.time_ns <= interval.end_ns:
                    yield record

    def inspection(self) -> dict:
        """Validate every record before evaluation; keep only counts and hashes in memory."""
        partitions = {(i.clock, i.epoch) for i in self.manifest.intervals}
        summaries = {}
        for stream in sorted(self.streams):
            counts = defaultdict(int)
            previous = {}
            source = {}
            gaps = 0
            partition_gaps = defaultdict(int)
            goal_revisions = {}
            for record in self._all_records(stream):
                key = (record.clock, record.epoch)
                require(key in partitions, f"{stream}: undeclared clock/epoch {key}")
                if key in previous:
                    before = previous[key]
                    require(record.time_ns >= before.time_ns, f"{stream}: out-of-order timestamp")
                    require(record.sequence > before.sequence, f"{stream}: nonincreasing sequence")
                    require(record.source == source[key], f"{stream}: source changed in epoch")
                    if record.source_sequence is not None and before.source_sequence is not None:
                        require(
                            record.source_sequence > before.source_sequence,
                            f"{stream}: nonincreasing source-wide sequence",
                        )
                    lost = max(0, record.sequence - before.sequence - 1)
                    gaps += lost
                    partition_gaps[f"{record.clock}/{record.epoch}"] += lost
                source[key] = record.source
                previous[key] = record
                counts[f"{record.clock}/{record.epoch}"] += 1
                if record.frame is not None:
                    require(self.manifest.frame == "map", "manifest: normalized frame must be map")
                if stream in ("truth_pose", "estimated_pose") and record.valid:
                    require(
                        record.data.get("reference_point", self.manifest.reference_point)
                        == self.manifest.reference_point,
                        f"{stream}: reference point mismatch",
                    )
                if stream == "goal" and record.valid:
                    revision = (key, record.data["revision"])
                    coordinates = (record.data["x"], record.data["y"])
                    require(
                        revision not in goal_revisions or goal_revisions[revision] == coordinates,
                        "goal: coordinates changed without a revision",
                    )
                    goal_revisions[revision] = coordinates
            summaries[stream] = {
                "records": sum(counts.values()),
                "partitions": dict(counts),
                "sequence_gaps": gaps,
                "sequence_gaps_by_partition": dict(partition_gaps),
            }
        return {
            "run_id": self.manifest.run_id,
            "source_run_id": self.manifest.source_run_id,
            "adapter": self.manifest.adapter,
            "execution_status": self.manifest.status,
            "intervals": [asdict(i) for i in self.manifest.intervals],
            "streams": summaries,
            "source_sha256": {
                str(p.relative_to(self.root)): digest_file(p)
                for p in sorted(
                    set(self.files.values())
                    | {
                        self.root
                        / (
                            "manifest.json"
                            if self.manifest.adapter == "normalized"
                            else "metadata.json"
                        )
                    }
                )
            },
        }


class DatasetReader(FileReader):
    """Read collect_ugv_dataset output without manufacturing navigation capabilities."""

    SENSOR_FILES = {"imu": "raw.imu", "wheel_odom": "raw.wheel_odom", "gps": "raw.gps"}

    def __init__(self, root: Path):
        self.root = root.resolve()
        metadata = mapping(read_json(self.root / "metadata.json"), "dataset metadata")
        self.files = {
            stream: self._path(f"{name}.csv") for name, stream in self.SENSOR_FILES.items()
        }
        self.files.update(
            truth_pose=self._path("truth.csv"), requested_command=self._path("cmd.csv")
        )
        self.fault_metadata = None
        if (self.root / "faults.json").is_file():
            self.fault_metadata = mapping(read_json(self.root / "faults.json"), "fault metadata")
            self.files["events"] = self.root / "faults.json"
        first, last = None, None
        for record in self._all_records("truth_pose"):
            first = record.time_ns if first is None else first
            last = record.time_ns
        require(first is not None and last > first, "dataset: truth needs a nonzero time interval")
        source_id = text(
            (self.fault_metadata or {}).get("source_run", self.root.name), "dataset source_run"
        )
        self.manifest = Manifest(
            self.root.name,
            source_id,
            "dataset_csv",
            "completed",
            (Interval("sim", 0, first, last),),
            {s: p.name for s, p in self.files.items()},
            {
                "kind": "dataset_csv",
                "metadata": metadata,
                "faults": self.fault_metadata,
                "timestamp_conversion": "decimal seconds rounded to nanoseconds",
            },
        )

    def _rows(self, path: Path):
        with path.open(newline="", encoding="utf-8-sig") as handle:
            reader = csv.DictReader(handle)
            require(
                reader.fieldnames is not None and "stamp" in reader.fieldnames,
                f"{path}: missing stamp column",
            )
            for index, row in enumerate(reader):
                require(
                    None not in row and all(v is not None for v in row.values()),
                    f"{path}:{index + 2}: malformed CSV row",
                )
                yield index, row

    def _all_records(self, stream: str) -> Iterator[Record]:
        if stream == "events":
            # Merge at most one event per sensor into chronological order.
            import heapq

            merged = heapq.merge(
                *(
                    self._sensor_events(name, canonical)
                    for name, canonical in self.SENSOR_FILES.items()
                ),
                key=lambda r: (r.time_ns, r.data["sensor"], r.sequence),
            )
            for sequence, record in enumerate(merged):
                yield replace(record, sequence=sequence)
            return
        for sequence, row in self._rows(self.files[stream]):
            data = {}
            for name, value in row.items():
                if name == "stamp":
                    continue
                if name in ("fault_type", "fault_column"):
                    data[name] = value
                else:
                    try:
                        data[name] = float(value)
                    except ValueError as exc:
                        raise ValidationError(f"{self.files[stream]}: invalid {name}") from exc
            if stream == "truth_pose":
                data["reference_point"] = "model_origin"
            record = Record(
                stream,
                seconds_to_ns(row["stamp"]),
                "sim",
                0,
                stream,
                sequence,
                data,
                frame="map" if stream == "truth_pose" else None,
            )
            finite_tree(data, stream)
            validate_payload(record)
            yield record

    def _sensor_events(self, name: str, canonical: str) -> Iterator[Record]:
        active = None
        start_index = 0
        sequence = 0
        for index, row in self._rows(self.files[canonical]):
            require(
                {"fault", "fault_type", "fault_column"} <= row.keys(),
                f"{name}: faults.json requires per-sample label columns",
            )
            require(row["fault"] in ("0", "1", "0.0", "1.0"), f"{name}: invalid fault label")
            key = (row["fault_column"], row["fault_type"]) if float(row["fault"]) else None
            if key == active:
                continue
            stamp = seconds_to_ns(row["stamp"])
            if active:
                yield Record(
                    "events",
                    stamp,
                    "sim",
                    0,
                    "dataset_events",
                    sequence,
                    {
                        "kind": "fault_end",
                        "event_id": f"{name}:{start_index}",
                        "sensor": name,
                        "channel": active[0],
                        "fault_class": active[1],
                    },
                )
                sequence += 1
            active = key
            if active:
                require(all(active), f"{name}: missing active fault column/type")
                start_index = index
                yield Record(
                    "events",
                    stamp,
                    "sim",
                    0,
                    "dataset_events",
                    sequence,
                    {
                        "kind": "fault_start",
                        "event_id": f"{name}:{index}",
                        "sensor": name,
                        "channel": active[0],
                        "fault_class": active[1],
                    },
                )
                sequence += 1

    def inspection(self) -> dict:
        # Event streams are merged from independent sensors: assign one global sequence.
        result = super().inspection()
        for stream in self.streams - {"events"}:
            require(result["streams"][stream]["records"] > 0, f"dataset: empty stream {stream}")
        counts = self.manifest.provenance["metadata"].get("counts", {})
        mapping(counts, "dataset sample counts")
        names = {**self.SENSOR_FILES, "truth": "truth_pose", "cmd": "requested_command"}
        for name, expected in counts.items():
            require(
                name in names and type(expected) is int and expected >= 0,
                f"dataset: invalid count {name}",
            )
            require(
                result["streams"][names[name]]["records"] == expected,
                f"dataset: sample count differs from metadata for {name}",
            )
        extra = {self.root / "metadata.json"}
        result["source_sha256"].update({p.name: digest_file(p) for p in extra})
        return result


def open_run(path: str | Path) -> FileReader:
    path = Path(path)
    require(path.is_dir(), f"run directory not found: {path}")
    if (path / "manifest.json").is_file():
        return FileReader(path)
    return DatasetReader(path)
