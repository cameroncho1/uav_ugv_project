"""Adapter semantics without ROS; real CDR coverage is registered with colcon."""

import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from uav_ugv_verification.adapters.rosbag import translate
from uav_ugv_verification.adapters.world import collision_geometry
from uav_ugv_verification.validation import ValidationError

ROOT = Path(__file__).resolve().parents[2]


def test_obstacle_scenario_preserves_geometry_acquisition_and_native_sequence():
    value = dict(
        schema_version=1,
        sim_ns=100,
        sequence=7,
        error=None,
        envelope=[dict(id="box", kind="box", x=1.0, y=2.0, half_x=0.3, half_y=0.3)],
    )
    records = telemetry("obstacle_scenario", value)
    assert len(records) == 1
    geometry = records[0]
    assert (geometry.stream, geometry.time_ns, geometry.sequence, geometry.frame) == (
        "obstacles",
        100,
        7,
        "map",
    )
    assert not geometry.data["static"] and geometry.data["shapes"] == value["envelope"]
    assert geometry.valid
    value["error"] = "scene confirmation failed"
    assert not telemetry("obstacle_scenario", value)[0].valid


def telemetry(kind, value):
    return translate(
        kind,
        NS(data=json.dumps(value)),
        epoch=0,
        receipt_sim_ns=123,
        sequence=999,
        source="telemetry",
    )


def test_control_receipt_and_frozen_clock_are_separate():
    base = dict(
        schema_version=1,
        host_id="linux-boot",
        sequence=7,
        steady_ns=18000000000000001,
        command_steady_ns=18000000000000011,
        sim_ns=10_000_000_000,
        epoch=0,
        pose_stamp_ns=9_900_000_000,
        pose_receipt_ns=18000000000000000,
        guard_valid=False,
        guard_reason="stale",
        linear=0.0,
        angular=0.0,
        hold_reason="stale",
    )
    first = telemetry("control", base)
    second = telemetry(
        "control",
        dict(
            base,
            sequence=9,
            steady_ns=base["steady_ns"] + 200_000_000,
            command_steady_ns=base["command_steady_ns"] + 200_000_000,
        ),
    )
    assert first[0].time_ns == second[0].time_ns == base["sim_ns"]
    assert second[2].time_ns - first[2].time_ns == 200_000_000
    assert second[2].clock == "steady:linux-boot"
    assert second[2].sequence - first[2].sequence == 2  # Do not hide dropped telemetry.
    assert first[0].data["pose_receipt_ns"] == 18000000000000000
    assert first[3].time_ns - first[2].time_ns == 10
    assert first[3].valid


def test_applied_fault_stamp_not_bag_or_request_time():
    event = dict(
        kind="fault_start",
        event_id="gps-1",
        sensor="gps",
        channel="position",
        fault_class="gps_outage",
    )
    records = telemetry(
        "fault",
        dict(
            schema_version=1,
            host_id="linux-boot",
            sequence=2,
            sim_ns=42_000_000_001,
            steady_ns=888,
            event=event,
        ),
    )
    assert records[0].time_ns == 42_000_000_001
    assert records[0].data == event


def test_unverified_steady_origin_is_rejected():
    with pytest.raises(ValidationError, match="verified boot"):
        telemetry("watchdog", dict(schema_version=1, host_id="unverified"))


def test_watchdog_reports_failed_publish_as_invalid():
    record = telemetry(
        "watchdog",
        dict(
            schema_version=1,
            host_id="boot",
            sequence=3,
            steady_ns=100,
            kind="watchdog_output",
            queued=False,
            linear=0.0,
            angular=0.0,
        ),
    )[0]
    assert record.stream == "actuator_output" and not record.valid


@pytest.mark.parametrize(
    "layout,count",
    [("simple_goal_demo", 0), ("simple_goal_demo_sparse", 3), ("simple_goal_demo_dense", 7)],
)
def test_actual_world_geometry(layout, count):
    shapes = collision_geometry(
        ROOT / f"simulation/worlds/{layout}.sdf", ["ground_plane", "simple_ugv"]
    )
    assert len(shapes) == count
    if count:
        assert {s["kind"] for s in shapes} == {"box", "cylinder"}


def test_composed_collision_pose_and_unsupported_frame(tmp_path):
    path = tmp_path / "world.sdf"
    path.write_text(
        '<sdf><world><model name="m"><static>true</static>'
        "<pose>1 2 0 0 0 1.5707963267948966</pose>"
        '<link name="l"><pose>2 0 0 0 0 0</pose><collision name="c">'
        "<geometry><box><size>2 4 1</size></box></geometry>"
        "</collision></link></model></world></sdf>"
    )
    shape = collision_geometry(path, [])[0]
    assert shape["x"] == pytest.approx(1)
    assert shape["y"] == pytest.approx(4)
    assert (shape["half_x"], shape["half_y"]) == (1, 2)
    path.write_text(path.read_text().replace("<pose>2", '<pose relative_to="unknown">2'))
    with pytest.raises(ValidationError, match="named/relative"):
        collision_geometry(path, [])
