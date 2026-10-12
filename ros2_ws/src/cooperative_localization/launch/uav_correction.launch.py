"""Consume an independent map anchor and a sensing-independent observation."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("cooperative_localization"))
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="cooperative_localization", executable="uav_correction.py",
             parameters=[str(share / "config/uav_correction.yaml"),
                         {"use_sim_time": ParameterValue(
                             LaunchConfiguration("use_sim_time"), value_type=bool)}]),
    ])
