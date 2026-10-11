from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("cooperative_localization"))
    config = package_share / "config" / "ugv_sensor_bridges.yaml"
    world = LaunchConfiguration("world")

    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value="uav_ugv", choices=["uav_ugv", "simple_goal_demo"],
            description="Gazebo world whose UGV spawn and GPS origin are used.",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ugv_sensor_bridge",
            parameters=[{"config_file": str(config)}],
            additional_env={"GZ_PARTITION": "uav_ugv_sim", "GZ_IP": "127.0.0.1"},
            output="screen",
        ),
        Node(
            package="cooperative_localization",
            executable="ugv_measurements.py",
            name="ugv_measurements",
            parameters=[
                str(package_share / "config" / "measurement_model.yaml"),
                PathJoinSubstitution([str(package_share), "config", [world, ".yaml"]]),
            ],
            output="screen",
        ),
    ])
