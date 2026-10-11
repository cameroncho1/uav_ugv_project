from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = (
        Path(get_package_share_directory("cooperative_localization"))
        / "config"
        / "ugv_sensor_bridges.yaml"
    )

    return LaunchDescription([
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ugv_sensor_bridge",
            parameters=[{"config_file": str(config)}],
            additional_env={"GZ_PARTITION": "uav_ugv_sim", "GZ_IP": "127.0.0.1"},
            output="screen",
        ),
    ])
