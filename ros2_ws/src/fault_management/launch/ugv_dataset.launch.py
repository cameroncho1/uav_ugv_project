"""Record clean raw UGV sensors and truth while driving seeded open-loop maneuvers."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = Path(get_package_share_directory("simulation_support"))
    env = {"GZ_PARTITION": EnvironmentVariable("GZ_PARTITION", default_value="uav_ugv_sim"),
           "GZ_IP": "127.0.0.1"}
    recorder = Node(
        package="fault_management", executable="ugv_dataset_recorder.py",
        arguments=["--seed", LaunchConfiguration("seed"),
                   "--duration", LaunchConfiguration("duration"),
                   "--world", LaunchConfiguration("world"),
                   "--output", LaunchConfiguration("output")],
        parameters=[{"use_sim_time": True}], output="screen")
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="ugv_dataset",
                              choices=["ugv_dataset", "simple_goal_demo", "uav_ugv"]),
        DeclareLaunchArgument("seed", default_value="513"),
        DeclareLaunchArgument("duration", default_value="120.0"),
        DeclareLaunchArgument("output"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(sim / "launch/ugv_sensors.launch.py"))),
        Node(package="simulation_support", executable="ugv_truth",
             parameters=[{"world": LaunchConfiguration("world"), "use_sim_time": True}],
             additional_env=env),
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             name="ugv_dataset_command_bridge",
             parameters=[{"config_file": str(sim / "config/ugv_evaluation_bridges.yaml")}],
             additional_env=env),
        recorder,
        RegisterEventHandler(OnProcessExit(
            target_action=recorder, on_exit=[EmitEvent(event=Shutdown())])),
    ])
