#!/usr/bin/env python3
"""Check firmware, ROS wire definitions, and Gazebo magnetic-field settings."""

import argparse
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET


MESSAGES = (
    "VehicleOdometry", "VehicleStatus", "VehicleLandDetected",
    "OffboardControlMode", "TrajectorySetpoint", "VehicleCommand",
)


def command(*arguments):
    return subprocess.check_output(arguments, text=True, stderr=subprocess.STDOUT).strip()


def version_tuple(value):
    return tuple(int(part) for part in value.split(".")[:3])


def message_schema(path):
    """Keep ordered wire fields and the version; constants do not occupy the wire."""
    fields = []
    version = 0
    for line in path.read_text().splitlines():
        line = " ".join(line.split("#", 1)[0].split())
        if "MESSAGE_VERSION" in line:
            version = int(line.split("=", 1)[1])
        elif line and "=" not in line:
            fields.append(line)
    return fields, version


def check_message(px4_message, ros_message):
    if message_schema(px4_message) != message_schema(ros_message):
        raise ValueError(f"ROS wire definition differs from PX4: {ros_message.name}")


def check_world(world):
    plugin = ET.parse(world).find(".//plugin[@name='gz::sim::systems::Magnetometer']")
    if plugin is None or any(plugin.findtext(flag) != "false" for flag in
                             ("use_units_gauss", "use_earth_frame_ned")):
        raise ValueError(f"{world.name} must use the ENU magnetic field in tesla")


def check(root):
    manifest = json.loads((root / "simulation/dependencies.json").read_text())
    for relative_path, expected in manifest["revisions"].items():
        actual = command("git", "-C", str(root / relative_path), "rev-parse", "HEAD")
        if actual != expected:
            raise ValueError(f"{relative_path}: expected {expected}, found {actual}")
    installed = command("pkg-config", "--modversion", "gz-sim8")
    if version_tuple(installed) < version_tuple(manifest["minimum_gazebo_version"]):
        raise ValueError("Gazebo Sim 8.6 or newer is required for the magnetometer settings")
    px4 = root / "external/PX4-Autopilot"
    for name in MESSAGES:
        definition = next((px4 / "msg").rglob(name + ".msg"))
        check_message(definition, root / "ros2_ws/src/px4_msgs/msg" / (name + ".msg"))
    for name in ("uav_ugv", "simple_goal_demo"):
        check_world(root / "simulation/worlds" / (name + ".sdf"))
    model = ET.parse(px4 / "Tools/simulation/gz/models/x500_base/model.sdf")
    noise = model.findall(".//magnetometer/*/noise/stddev")
    if len(noise) != 3 or any(not 0 < float(value.text) < 1e-5 for value in noise):
        raise ValueError("X500 magnetometer noise must be specified in tesla")
    print(f"Simulation dependencies match; Gazebo {installed}, ROS wire definitions, "
          "and ENU/tesla magnetometer settings verified.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    try:
        check(args.root)
    except (OSError, ValueError, StopIteration, subprocess.CalledProcessError) as error:
        print(f"Simulation dependency check failed: {error}\n"
              "Run git submodule update --init --recursive, install Gazebo development "
              "packages, and rebuild PX4 and the ROS workspace.", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
