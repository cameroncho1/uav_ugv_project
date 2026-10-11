"""Add a truth reader and test command bridge to the operational localization chain."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("cooperative_localization"))
    world = LaunchConfiguration("world")
    seed = LaunchConfiguration("seed")
    initial_gps_enabled = LaunchConfiguration("initial_gps_enabled")
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="uav_ugv"),
        DeclareLaunchArgument("seed", default_value="513"),
        DeclareLaunchArgument("initial_gps_enabled", default_value="true"),
        DeclareLaunchArgument("measurement_profile", default_value="nominal"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch/ugv_localization.launch.py")),
            launch_arguments={"world": world, "seed": seed,
                              "measurement_profile": LaunchConfiguration("measurement_profile"),
                              "initial_gps_enabled": initial_gps_enabled}.items(),
        ),
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            name="ugv_evaluation_command_bridge",
            parameters=[{"config_file": str(share / "config/ugv_evaluation_bridges.yaml")}],
            additional_env={"GZ_PARTITION": EnvironmentVariable(
                "GZ_PARTITION", default_value="uav_ugv_sim"),
                "GZ_IP": "127.0.0.1"},
        ),
        Node(
            package="cooperative_localization", executable="ugv_truth",
            parameters=[{"world": world}],
            additional_env={"GZ_PARTITION": EnvironmentVariable(
                "GZ_PARTITION", default_value="uav_ugv_sim"),
                "GZ_IP": "127.0.0.1"},
        ),
    ])
