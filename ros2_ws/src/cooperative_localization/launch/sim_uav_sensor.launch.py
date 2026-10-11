"""Generate noisy relative observations at the Gazebo simulator boundary."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("cooperative_localization"))
    world = LaunchConfiguration("world")
    sensor_parameters = {
        "random_seed": ParameterValue(LaunchConfiguration("seed"), value_type=int),
        "publish_anchor": ParameterValue(LaunchConfiguration("publish_anchor"), value_type=bool),
        "outlier_every": ParameterValue(LaunchConfiguration("outlier_every"), value_type=int),
        "dropout_start": ParameterValue(LaunchConfiguration("dropout_start"), value_type=float),
        "dropout_end": ParameterValue(LaunchConfiguration("dropout_end"), value_type=float),
    }
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="uav_ugv"),
        DeclareLaunchArgument("seed", default_value="513"),
        DeclareLaunchArgument("uav_model", default_value=""),
        DeclareLaunchArgument("publish_anchor", default_value="true"),
        DeclareLaunchArgument("outlier_every", default_value="0"),
        DeclareLaunchArgument("dropout_start", default_value="-1.0"),
        DeclareLaunchArgument("dropout_end", default_value="-1.0"),
        Node(package="cooperative_localization", executable="sim_vehicle_poses",
             parameters=[{"world": world, "uav_model": LaunchConfiguration("uav_model")}],
             additional_env={"GZ_PARTITION": EnvironmentVariable(
                 "GZ_PARTITION", default_value="uav_ugv_sim"),
                 "GZ_IP": "127.0.0.1"}),
        Node(package="cooperative_localization", executable="sim_uav_sensor.py",
             parameters=[str(share / "config/uav_measurement_model.yaml"),
                         PathJoinSubstitution([str(share), "config", [world, ".yaml"]]),
                         sensor_parameters],
             output="screen"),
    ])
