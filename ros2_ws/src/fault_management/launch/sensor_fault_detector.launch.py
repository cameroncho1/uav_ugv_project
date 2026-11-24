"""Flag UGV sensor-channel faults live from the prepared measurements."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("command_topic", default_value="/ugv/cmd_vel",
                              description="UGV drive command used as the speed reference."),
        DeclareLaunchArgument("use_autoencoder", default_value="false",
                              description="Also flag windows above the autoencoder threshold."),
        DeclareLaunchArgument("monitor_uav", default_value="true",
                              description="Flag uav_loss when UAV relative measurements stop."),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(package="fault_management", executable="sensor_fault_detector.py",
             parameters=[{
                 "command_topic": LaunchConfiguration("command_topic"),
                 "use_autoencoder": ParameterValue(
                     LaunchConfiguration("use_autoencoder"), value_type=bool),
                 "monitor_uav": ParameterValue(
                     LaunchConfiguration("monitor_uav"), value_type=bool),
                 "use_sim_time": ParameterValue(
                     LaunchConfiguration("use_sim_time"), value_type=bool)}],
             output="screen"),
    ])
