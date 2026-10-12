"""Expose PX4 state through the independent map-anchor contract."""

from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    value = lambda key: LaunchConfiguration(key).perform(context)
    origin = yaml.safe_load(value("map_origin"))
    if not isinstance(origin, list) or len(origin) != 3:
        raise ValueError("map_origin requires a YAML list of three map coordinates")
    share = Path(get_package_share_directory("uav_control"))
    return [Node(package="uav_control", executable="uav_pose.py",
                 parameters=[str(share / "config/uav_pose.yaml"),
                             {"map_origin": [float(v) for v in origin],
                              "use_sim_time": value("use_sim_time").lower() == "true",
                              "require_airborne": value("require_airborne").lower() == "true",
                              "initial_settle_time": float(value("initial_settle_time"))}],
                 remappings=[("/fmu/out/vehicle_odometry", value("odometry_topic"))],
                 output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("map_origin", default_value="[0.0, 0.0, 0.0]"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("odometry_topic", default_value="/fmu/out/vehicle_odometry"),
        DeclareLaunchArgument("require_airborne", default_value="false"),
        DeclareLaunchArgument("initial_settle_time", default_value="0.0"),
        OpaqueFunction(function=start),
    ])
