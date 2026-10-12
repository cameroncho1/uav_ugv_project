"""Reject wire and magnetic-field mismatches before simulation starts."""

import importlib.util
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[4] / "scripts/check_sim_dependencies.py"
SPEC = importlib.util.spec_from_file_location("sim_dependencies", SCRIPT)
dependencies = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(dependencies)


def test_message_constants_and_comments_do_not_change_wire_fields(tmp_path):
    px4 = tmp_path / "px4.msg"
    ros = tmp_path / "ros.msg"
    px4.write_text("uint32 MESSAGE_VERSION = 4\nuint64 timestamp # acquisition\n"
                   "uint8 ARMED = 2\nfloat32[3] position\n")
    ros.write_text("uint32 MESSAGE_VERSION=4\nuint8 OTHER=9\n"
                   "uint64 timestamp\nfloat32[3] position\n")
    dependencies.check_message(px4, ros)


@pytest.mark.parametrize("schema", [
    "uint32 MESSAGE_VERSION=1\nuint64 timestamp\nfloat32[3] position\n",
    "uint32 MESSAGE_VERSION=4\nfloat32[3] position\nuint64 timestamp\n",
    "uint32 MESSAGE_VERSION=4\nuint64 timestamp\nfloat32 position\n",
])
def test_version_field_order_and_array_shape_must_match(tmp_path, schema):
    px4 = tmp_path / "px4.msg"
    ros = tmp_path / "ros.msg"
    px4.write_text("uint32 MESSAGE_VERSION=4\nuint64 timestamp\nfloat32[3] position\n")
    ros.write_text(schema)
    with pytest.raises(ValueError):
        dependencies.check_message(px4, ros)


@pytest.mark.parametrize("flags", ["", "<use_units_gauss>true</use_units_gauss>",
                                   "<use_units_gauss>false</use_units_gauss>"
                                   "<use_earth_frame_ned>true</use_earth_frame_ned>"])
def test_legacy_or_missing_magnetic_flags_are_rejected(tmp_path, flags):
    world = tmp_path / "world.sdf"
    world.write_text('<sdf><world><plugin name="gz::sim::systems::Magnetometer">'
                     + flags + '</plugin></world></sdf>')
    with pytest.raises(ValueError):
        dependencies.check_world(world)
