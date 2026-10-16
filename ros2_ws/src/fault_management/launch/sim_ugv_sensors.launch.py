"""Apply stated sensor noise/faults at the explicit simulation boundary."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("fault_management"))
    sim = Path(get_package_share_directory("simulation_support"))
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="uav_ugv",
                              choices=["uav_ugv", "simple_goal_demo"]),
        DeclareLaunchArgument("seed", default_value="513"),
        DeclareLaunchArgument("measurement_config",
                              default_value=str(share / "config/measurement_model.yaml"),
                              description="UGV noise YAML path relative to the launch directory."),
        DeclareLaunchArgument("initial_gps_enabled", default_value="true"),
        DeclareLaunchArgument("shared_position_stddev", default_value="0.0"),
        DeclareLaunchArgument("measurement_profile", default_value="nominal",
                              choices=["nominal", "drift"]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(sim / "launch/ugv_sensors.launch.py"))),
        Node(package="fault_management", executable="sim_ugv_sensors.py",
             parameters=[LaunchConfiguration("measurement_config"),
                         PathJoinSubstitution([str(share), "config",
                             ["measurement_", LaunchConfiguration("measurement_profile"), ".yaml"]]),
                         PathJoinSubstitution([str(sim), "config",
                                               [LaunchConfiguration("world"), ".yaml"]]),
                         {"random_seed": ParameterValue(LaunchConfiguration("seed"), value_type=int),
                          "initial_gps_enabled": ParameterValue(
                              LaunchConfiguration("initial_gps_enabled"), value_type=bool),
                          "shared_position_stddev": ParameterValue(
                              LaunchConfiguration("shared_position_stddev"), value_type=float)}],
             output="screen"),
    ])
