"""UAV pose and relative-observation adapters; accepts simulated or real inputs."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("cooperative_localization"))
    config = str(share / "config/uav_measurement_model.yaml")
    world = LaunchConfiguration("world")
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="uav_ugv"),
        DeclareLaunchArgument("uav_odometry_topic", default_value="/fmu/out/vehicle_odometry"),
        DeclareLaunchArgument("uav_require_airborne", default_value="false"),
        DeclareLaunchArgument("uav_initial_settle_time", default_value="0.0"),
        Node(package="cooperative_localization", executable="uav_pose.py",
             parameters=[config, PathJoinSubstitution([str(share), "config", [world, ".yaml"]]),
                         {"require_airborne": ParameterValue(
                             LaunchConfiguration("uav_require_airborne"), value_type=bool),
                          "initial_settle_time": ParameterValue(
                             LaunchConfiguration("uav_initial_settle_time"), value_type=float)}],
             remappings=[("/fmu/out/vehicle_odometry", LaunchConfiguration("uav_odometry_topic"))],
             output="screen"),
        Node(package="cooperative_localization", executable="uav_correction.py",
             parameters=[config], output="screen"),
    ])
