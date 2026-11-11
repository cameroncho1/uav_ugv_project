"""Required Humble CDR/storage regression; executed by colcon in Linux."""

import json

import rosbag2_py
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.serialization import serialize_message
from rosgraph_msgs.msg import Clock
from uav_ugv_verification import evaluate
from uav_ugv_verification.adapters.files import FileReader
from uav_ugv_verification.adapters.rosbag import import_bag


def test_real_cdr_clock_rewind_and_image_exclusion(tmp_path):
    bag = tmp_path / "bag"
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    for name, kind in (
        ("/clock", "rosgraph_msgs/msg/Clock"),
        ("/truth", "geometry_msgs/msg/PoseStamped"),
        ("/image", "sensor_msgs/msg/Image"),
    ):
        writer.create_topic(
            rosbag2_py.TopicMetadata(name=name, type=kind, serialization_format="cdr")
        )
    for index, seconds in enumerate((10, 11, 2, 3)):
        clock = Clock()
        clock.clock.sec = seconds
        pose = PoseStamped()
        pose.header.stamp = clock.clock
        pose.header.frame_id = "map"
        pose.pose.position.x = float(index)
        # Deliberately descending bag storage timestamps across the rewind.
        writer.write("/clock", serialize_message(clock), seconds * 10**9)
        writer.write("/truth", serialize_message(pose), seconds * 10**9)
    writer.write("/image", b"not valid CDR: must never deserialize", 1)
    del writer
    setup = tmp_path / "setup.yaml"
    setup.write_text(
        yaml.safe_dump(
            dict(
                schema_version=1,
                id="cdr-fixture",
                mission_start="first_drive",
                topics={"/truth": "truth_pose"},
            )
        )
    )
    output = tmp_path / "run"
    import_bag(bag, setup, output, status="aborted")
    reader = FileReader(output)
    assert reader.inspection()["streams"]["truth_pose"]["records"] == 4
    assert [(i.epoch, i.start_ns, i.end_ns) for i in reader.manifest.intervals] == [
        (0, 10 * 10**9, 11 * 10**9),
        (1, 2 * 10**9, 3 * 10**9),
    ]
    assert json.loads((output / "manifest.json").read_text())["status"] == "aborted"
    assert len(reader.manifest.provenance["mission_scope"]["missing_first_drive"]) == 2
    profile = tmp_path / "guard.yaml"
    profile.write_text(
        yaml.safe_dump(
            dict(
                schema_version=1,
                id="historical",
                version="1",
                calibration="provisional",
                max_gap_ns=500000000,
                requirements={"R3": {"applicable": True, "mandatory": True}},
            )
        )
    )
    result = evaluate(output, profile)
    assert result.exit_code == 2
    assert result.results[2].verdict.value == "INCONCLUSIVE"


def test_native_sequence_loss_and_declared_mission_scope(tmp_path):
    from std_msgs.msg import String

    bag = tmp_path / "bag"
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name="/control", type="std_msgs/msg/String", serialization_format="cdr"
        )
    )
    for sequence, tick, linear in [(2, 10, 0.0), (4, 11, 0.2), (5, 12, 0.2)]:
        value = dict(
            schema_version=1,
            host_id="boot",
            sequence=sequence,
            steady_ns=tick * 10**9,
            command_steady_ns=tick * 10**9 + 1,
            sim_ns=tick * 10**9,
            epoch=0,
            pose_stamp_ns=tick * 10**9,
            pose_receipt_ns=tick * 10**9,
            guard_valid=True,
            guard_reason="ready",
            hold_reason="drive",
            linear=linear,
            angular=0.0,
        )
        writer.write("/control", serialize_message(String(data=json.dumps(value))), tick * 10**9)
    del writer
    setup = tmp_path / "setup.yaml"
    setup.write_text(
        yaml.safe_dump(
            dict(
                schema_version=1,
                id="native",
                mission_start="first_drive",
                topics={"/control": "control"},
            )
        )
    )
    manifest = import_bag(bag, setup, tmp_path / "run", status="completed")
    assert {i["start_ns"] for i in manifest["intervals"]} == {11 * 10**9}
    inspection = FileReader(tmp_path / "run").inspection()
    assert inspection["streams"]["guard_state"]["sequence_gaps"] == 2  # Sim and steady partitions.
    again = import_bag(bag, setup, tmp_path / "again", status="completed")
    assert again["source_run_id"] == manifest["source_run_id"]
