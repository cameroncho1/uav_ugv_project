"""Humble sqlite3 bag import. ROS deserialization is loaded only during import.

Storage insertion order preserves clock resets. A disk-backed staging table sorts
acquisition stamps without retaining images or an entire recording in memory.
"""

from __future__ import annotations

import hashlib
import json
import sqlite3
import tempfile
from contextlib import closing
from dataclasses import asdict
from pathlib import Path

import yaml

from ..contracts import Record
from ..validation import (
    UniqueLoader,
    digest_file,
    keys,
    mapping,
    parse_json,
    require,
    text,
    validate_payload,
)
from .files import FileReader
from .world import collision_geometry

ADAPTER_VERSION = "humble-sqlite3-v1"
MESSAGE_TYPES = {
    "control": "std_msgs/msg/String",
    "watchdog": "std_msgs/msg/String",
    "motion": "std_msgs/msg/String",
    "fault": "std_msgs/msg/String",
    "canonical": "std_msgs/msg/String",
    "truth_pose": "geometry_msgs/msg/PoseStamped",
    "estimated_pose": "geometry_msgs/msg/PoseWithCovarianceStamped",
    "goal": "geometry_msgs/msg/PoseStamped",
    "route": "uav_ugv_msgs/msg/PlannedRoute",
    "grid": "nav_msgs/msg/OccupancyGrid",
    "uav_observation": "geometry_msgs/msg/PoseWithCovarianceStamped",
}


def stamp_ns(stamp) -> int:
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def translate(
    kind: str, message, *, epoch: int, receipt_sim_ns: int, sequence: int, source: str
) -> list[Record]:
    """Translate one selected numerical message; no simulator/topic branches in monitors."""

    def record(
        stream, data, *, time=None, clock="sim", part=None, frame=None, valid=True, seq=None
    ):
        return Record(
            stream,
            receipt_sim_ns if time is None else time,
            clock,
            epoch if part is None else part,
            source,
            sequence if seq is None else seq,
            data,
            valid,
            frame,
        )

    if kind in ("control", "watchdog", "motion", "fault", "canonical"):
        value = parse_json(message.data)
        require(value.get("schema_version") == 1, "unsupported telemetry version")
        if kind == "canonical":
            # Explicit canonical event/detector messages preserve their own clock/stamp.
            from ..validation import parse_record

            stream = value.pop("stream")
            value.pop("schema_version")
            require(stream in ("events", "detector_decision"), "unsupported canonical topic stream")
            return [parse_record(value, stream)]
        host = value.get("host_id")
        require(
            isinstance(host, str) and host and host != "unverified",
            "telemetry needs a verified boot ID",
        )
        steady = "steady:" + host
        seq = value["sequence"]
        now = value["steady_ns"]
        part = value.get("epoch", epoch)
        if kind == "control":
            result = []
            for clock, tick, e in (("sim", value["sim_ns"], part), (steady, now, 0)):
                for stream, data in (
                    (
                        "guard_state",
                        {
                            "guard_valid": value["guard_valid"],
                            "reason": value["guard_reason"],
                            "pose_stamp_ns": value["pose_stamp_ns"],
                            "pose_receipt_ns": value["pose_receipt_ns"],
                            "command_nonzero": abs(value["linear"]) > 0.001
                            or abs(value["angular"]) > 0.001,
                        },
                    ),
                    (
                        "requested_command",
                        {
                            "linear": value["linear"],
                            "angular": value["angular"],
                            "hold_reason": value["hold_reason"],
                        },
                    ),
                ):
                    emission = stream == "requested_command" and clock == steady
                    result.append(
                        record(
                            stream,
                            data,
                            clock=clock,
                            time=value.get("command_steady_ns", tick) if emission else tick,
                            part=e,
                            seq=seq,
                            valid=not emission or "command_steady_ns" in value,
                        )
                    )
            return result
        if kind == "watchdog":
            stream = {"watchdog_input": "actuator_input", "watchdog_output": "actuator_output"}.get(
                value["kind"]
            )
            require(stream is not None, "unknown watchdog telemetry")
            data = {
                k: v
                for k, v in value.items()
                if k not in ("schema_version", "kind", "host_id", "sequence", "steady_ns")
            }
            return [
                record(
                    stream,
                    data,
                    time=now,
                    clock=steady,
                    part=0,
                    seq=seq,
                    valid=bool(value.get("queued", True)),
                )
            ]
        if kind == "motion":
            return [
                record(
                    "measured_motion",
                    {
                        k: value[k]
                        for k in ("linear", "angular", "simulation_advancing", "uncertainty_ns")
                    },
                    time=now,
                    clock=steady,
                    part=0,
                    seq=seq,
                )
            ]
        return [record("events", value["event"], time=value["sim_ns"], part=epoch, seq=seq)]

    stamp = stamp_ns(message.header.stamp)
    frame = message.header.frame_id
    if kind in ("truth_pose", "estimated_pose", "goal"):
        pose = message.pose.pose if kind == "estimated_pose" else message.pose
        data = {"x": pose.position.x, "y": pose.position.y}
        if kind == "goal":
            data["revision"] = str(stamp)
        else:
            data["reference_point"] = "model_origin"
        return [record(kind, data, time=stamp, frame=frame, valid=frame == "map" and stamp > 0)]
    if kind == "route":
        points = [[p.pose.position.x, p.pose.position.y] for p in message.path.poses]
        return [
            record(
                "route",
                {
                    "goal_revision": str(stamp_ns(message.goal.header.stamp)),
                    "points": points,
                    "partial": message.partial,
                },
                time=stamp,
                frame=frame,
                valid=frame == "map" and len(points) >= 2 and stamp > 0,
            )
        ]
    require(kind in ("grid", "uav_observation"), f"unsupported setup mapping {kind}")
    return [
        record(
            kind,
            {"frame_id": frame},
            time=stamp,
            valid=stamp > 0 and (kind != "grid" or frame == "map"),
        )
    ]


def import_bag(
    bag: Path,
    setup_path: Path,
    output: Path,
    *,
    status: str,
    world: Path | None = None,
    events: Path | None = None,
    start_ns: int | None = None,
    end_ns: int | None = None,
) -> dict:
    """Create an ordinary normalized run. Refuse overwrite or unsupported storage."""
    require(not output.exists(), "import output already exists; use a new directory")
    setup = mapping(
        yaml.load(setup_path.read_text(encoding="utf-8-sig"), Loader=UniqueLoader), "setup"
    )
    keys(
        setup,
        {"schema_version", "id", "topics", "ignored_models", "mission_start"},
        {"schema_version", "id", "topics"},
        "setup",
    )
    text(setup["id"], "setup.id")
    mapping(setup["topics"], "setup.topics")
    require(
        setup.get("mission_start", "recording") in ("recording", "first_drive"),
        "unsupported mission_start",
    )
    ignored = setup.get("ignored_models", [])
    require(
        isinstance(ignored, list) and all(isinstance(v, str) and v for v in ignored),
        "invalid ignored_models",
    )
    for topic, kind in setup["topics"].items():
        require(
            topic.startswith("/") and topic != "/clock" and kind in MESSAGE_TYPES,
            "invalid topic/kind mapping",
        )
    require(setup.get("schema_version") == 1, "unsupported setup version")
    metadata = yaml.safe_load((bag / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    require(metadata["storage_identifier"] == "sqlite3", "only Humble sqlite3 bags are supported")
    try:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        from ..validation import ValidationError

        raise ValidationError(
            "import-bag requires a sourced ROS 2 Humble Linux environment"
        ) from exc

    topics = setup["topics"]
    hashes = {"setup": digest_file(setup_path), "metadata.yaml": digest_file(bag / "metadata.yaml")}
    recording_path = bag.parent / "recording.json"
    recording = None
    if recording_path.is_file():
        recording = parse_json(recording_path.read_text())
        hashes["recording.json"] = digest_file(recording_path)
        require(
            recording["status"] == status, "recording terminal status differs from import status"
        )
    output.parent.mkdir(parents=True, exist_ok=True)
    with (
        tempfile.TemporaryDirectory(prefix="verification-import-", dir=output.parent) as staging,
        closing(sqlite3.connect(str(Path(staging) / "records.sqlite3"))) as store,
    ):
        store.execute(
            "CREATE TABLE records(stream, clock, epoch INTEGER, stamp INTEGER, "
            "sequence INTEGER, payload)"
        )
        partitions, counters, observed, boot_ids = {}, {}, set(), set()
        event_sequences = {}
        epoch, sim, received = 0, 0, 0

        def insert(record):
            validate_payload(record)
            if record.stream == "events":
                source_key = record.source, record.clock, record.epoch
                before = event_sequences.get(source_key)
                if before is not None:
                    require(record.sequence > before, "event producer sequence must increase")
                    record.data["producer_sequence_gap"] = record.sequence - before - 1
                event_sequences[source_key] = record.sequence
            require(record.time_ns >= 0, "negative acquisition/receipt timestamp")
            key = record.clock, record.epoch
            bounds = partitions.setdefault(key, [record.time_ns, record.time_ns])
            bounds[0], bounds[1] = min(bounds[0], record.time_ns), max(bounds[1], record.time_ns)
            value = asdict(record)
            value.pop("stream")
            value.pop("source_sequence")
            store.execute(
                "INSERT INTO records VALUES(?,?,?,?,?,?)",
                (
                    record.stream,
                    record.clock,
                    record.epoch,
                    record.time_ns,
                    record.sequence,
                    json.dumps(value, allow_nan=False),
                ),
            )

        for name in metadata["relative_file_paths"]:
            path = (bag / name).resolve()
            require(path.is_relative_to(bag.resolve()), "bag path escapes recording directory")
            hashes[name] = digest_file(path)
            with sqlite3.connect(path.as_uri() + "?mode=ro", uri=True) as connection:
                types = {
                    row[0]: (row[1], row[2])
                    for row in connection.execute("SELECT id,name,type FROM topics")
                }
                messages = {}
                for ident, (topic, type_name) in types.items():
                    if topic in topics or topic == "/clock":
                        expected = (
                            "rosgraph_msgs/msg/Clock"
                            if topic == "/clock"
                            else MESSAGE_TYPES[topics[topic]]
                        )
                        require(
                            type_name == expected, f"{topic}: expected {expected}, got {type_name}"
                        )
                        messages[ident] = get_message(type_name)
                selected = ",".join("?" for _ in messages)
                query = (
                    f"SELECT topic_id,data FROM messages WHERE topic_id IN ({selected}) ORDER BY id"
                )
                for ident, blob in connection.execute(query, tuple(messages)):
                    received += 1
                    if ident not in messages:
                        continue  # Never deserialize images or unrelated topics.
                    topic = types[ident][0]
                    message = deserialize_message(blob, messages[ident])
                    if topic == "/clock":
                        tick = stamp_ns(message.clock)
                        if tick < sim:
                            epoch += 1
                        sim = tick
                        continue
                    observed.add(topic)
                    counters[topic] = counters.get(topic, 0) + 1
                    for record in translate(
                        topics[topic],
                        message,
                        epoch=epoch,
                        receipt_sim_ns=sim,
                        sequence=counters[topic],
                        source=topic,
                    ):
                        if record.clock.startswith("steady:"):
                            boot_ids.add(record.clock)
                        insert(record)
        require(len(boot_ids) <= 1, "multiple Linux boot IDs: steady clocks cannot be joined")
        if events:
            hashes["events"] = digest_file(events)
            from ..validation import parse_record

            with events.open(encoding="utf-8") as handle:
                for line in handle:
                    record = parse_record(parse_json(line), "events")
                    require(
                        not record.clock.startswith("steady:") or record.clock in boot_ids,
                        "event boot ID differs from telemetry",
                    )
                    insert(record)
        require(partitions, "no supported numerical messages found")
        if world:
            hashes["world"] = digest_file(world)
            shapes = collision_geometry(world, setup.get("ignored_models", []))
            for (clock, part), (start, _) in list(partitions.items()):
                if clock == "sim":
                    insert(
                        Record(
                            "obstacles",
                            start,
                            clock,
                            part,
                            "world_collision_geometry",
                            0,
                            {"static": True, "shapes": shapes},
                            frame="map",
                        )
                    )
        store.commit()
        store.execute("CREATE INDEX ordered_records ON records(stream,clock,epoch,stamp,sequence)")
        streams = [
            row[0] for row in store.execute("SELECT DISTINCT stream FROM records ORDER BY stream")
        ]
        target = Path(staging) / "run"
        target.mkdir()
        for stream in streams:
            with (target / (stream + ".jsonl")).open("w", encoding="utf-8", newline="\n") as handle:
                normalized_sequences = {}
                for (payload,) in store.execute(
                    "SELECT payload FROM records WHERE stream=? "
                    "ORDER BY clock,epoch,stamp,sequence",
                    (stream,),
                ):
                    value = json.loads(payload)
                    # Header-only ROS messages have no publisher sequence. Assign
                    # canonical sequence after acquisition sorting, preserving bag order.
                    if stream in (
                        "truth_pose",
                        "estimated_pose",
                        "goal",
                        "route",
                        "grid",
                        "uav_observation",
                        "events",
                    ):
                        key = value["clock"], value["epoch"]
                        value["data"]["source_receipt_sequence"] = value["sequence"]
                        value["data"]["producer"] = value["source"]
                        value["sequence"] = normalized_sequences.get(key, 0) + (
                            value["data"].get("producer_sequence_gap", 0)
                            if stream == "events"
                            else 0
                        )
                        normalized_sequences[key] = value["sequence"] + 1
                        if stream == "events":
                            value["source"] = "observed_events"
                    handle.write(json.dumps(value, allow_nan=False) + "\n")
        intervals = []
        missing_mission_start = []
        for (clock, part), (start, end) in sorted(partitions.items()):
            if setup.get("mission_start") == "first_drive" and (clock != "sim" or start_ns is None):
                drive = None
                for (payload,) in store.execute(
                    "SELECT payload FROM records WHERE stream='guard_state' "
                    "AND clock=? AND epoch=? ORDER BY stamp,sequence",
                    (clock, part),
                ):
                    value = json.loads(payload)
                    if value["valid"] and value["data"].get("command_nonzero", False):
                        drive = value["time_ns"]
                        break
                if drive is None:
                    missing_mission_start.append({"clock": clock, "epoch": part})
                else:
                    start = drive
            if clock == "sim":
                start = start if start_ns is None else start_ns
                end = end if end_ns is None else end_ns
            require(end > start, "mission interval needs positive duration")
            intervals.append(dict(clock=clock, epoch=part, start_ns=start, end_ns=end))
        bag_digest = hashlib.sha256(
            json.dumps(
                {k: v for k, v in hashes.items() if k.endswith((".db3", "metadata.yaml"))},
                sort_keys=True,
            ).encode()
        ).hexdigest()
        source_snapshot = (recording or {}).get("source_sha256")
        manifest = dict(
            schema_version=1,
            run_id=output.name + "-" + bag_digest[:12],
            source_run_id="rosbag2:" + bag_digest,
            adapter="normalized",
            status=status,
            frame="map",
            reference_point="model_origin",
            intervals=intervals,
            streams={s: s + ".jsonl" for s in streams},
            provenance=dict(
                kind="rosbag2",
                adapter_version=ADAPTER_VERSION,
                adapter_sha256={
                    name: digest_file(Path(__file__).with_name(name))
                    for name in ("rosbag.py", "world.py")
                },
                setup_id=setup["id"],
                mission_scope={
                    "start": setup.get("mission_start", "recording"),
                    "explicit_start_ns": start_ns,
                    "explicit_end_ns": end_ns,
                    "prehistory_retained": True,
                    "missing_first_drive": missing_mission_start,
                },
                recording=recording,
                code_commit=(recording or {}).get("git_commit", "unrecorded"),
                source_snapshot_sha256=hashlib.sha256(
                    json.dumps(source_snapshot, sort_keys=True).encode()
                ).hexdigest()
                if source_snapshot
                else "unrecorded",
                source_sha256=hashes,
                observed_topics=sorted(observed),
                missing_topics=sorted(set(topics) - observed),
                storage_order="sqlite message ID",
                timing="header acquisition; native steady consumer telemetry; "
                "no bag receipt inference",
            ),
        )
        (target / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
        FileReader(target).inspection()
        target.rename(output)
    return manifest
