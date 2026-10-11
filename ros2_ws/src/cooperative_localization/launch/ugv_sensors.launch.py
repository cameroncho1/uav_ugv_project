from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("cooperative_localization"))
    config = package_share / "config" / "ugv_sensor_bridges.yaml"
    world = LaunchConfiguration("world")
    seed = LaunchConfiguration("seed")
    initial_gps_enabled = LaunchConfiguration("initial_gps_enabled")

    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value="uav_ugv", choices=["uav_ugv", "simple_goal_demo"],
            description="Gazebo world whose UGV spawn and GPS origin are used.",
        ),
        DeclareLaunchArgument("seed", default_value="513"),
        DeclareLaunchArgument("initial_gps_enabled", default_value="true"),
        DeclareLaunchArgument("measurement_profile", default_value="nominal",
                              choices=["nominal", "drift"]),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ugv_sensor_bridge",
            parameters=[{"config_file": str(config)}],
            additional_env={"GZ_PARTITION": EnvironmentVariable(
                "GZ_PARTITION", default_value="uav_ugv_sim"),
                "GZ_IP": "127.0.0.1"},
            output="screen",
        ),
        Node(
            package="cooperative_localization",
            executable="ugv_measurements.py",
            name="ugv_measurements",
            parameters=[
                str(package_share / "config" / "measurement_model.yaml"),
                PathJoinSubstitution([
                    str(package_share), "config",
                    ["measurement_", LaunchConfiguration("measurement_profile"), ".yaml"],
                ]),
                PathJoinSubstitution([str(package_share), "config", [world, ".yaml"]]),
                {"random_seed": seed, "initial_gps_enabled": initial_gps_enabled},
            ],
            output="screen",
        ),
    ])
