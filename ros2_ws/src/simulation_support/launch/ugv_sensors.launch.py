"""Integration owns the Gazebo clock and raw sensor transport boundary."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("simulation_support"))
    return LaunchDescription([Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="ugv_sensor_bridge",
        parameters=[{"config_file": str(share / "config/ugv_sensor_bridges.yaml")}],
        additional_env={"GZ_PARTITION": EnvironmentVariable(
            "GZ_PARTITION", default_value="uav_ugv_sim"), "GZ_IP": "127.0.0.1"},
        output="screen")])
