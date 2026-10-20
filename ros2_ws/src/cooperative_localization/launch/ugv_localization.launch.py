"""Estimator only: validated measurements, optional corrections and fused pose."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("cooperative_localization"))
    clock = {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("origin_latitude", default_value="47.397971057728974"),
        DeclareLaunchArgument("origin_longitude", default_value="8.546163739800146"),
        DeclareLaunchArgument("origin_altitude", default_value="0.0"),
        DeclareLaunchArgument("uav_assistance", default_value="false"),
        DeclareLaunchArgument("uav_heading", default_value="false",
                              description="Use measured relative orientation when available."),
        DeclareLaunchArgument("systematic_position_stddev", default_value="0.0"),
        Node(package="cooperative_localization", executable="ugv_measurements.py",
             parameters=[str(share / "config/measurements.yaml"), clock,
                         {key: ParameterValue(LaunchConfiguration(key), value_type=float)
                          for key in ("origin_latitude", "origin_longitude", "origin_altitude")}],
             output="screen"),
        Node(package="tf2_ros", executable="static_transform_publisher", name="ugv_imu_mount",
             arguments=["--frame-id", "simple_ugv/chassis", "--child-frame-id",
                        "simple_ugv/chassis/imu"], parameters=[clock]),
        Node(package="cooperative_localization", executable="uav_correction.py",
             parameters=[str(share / "config/uav_correction.yaml"), clock,
                         {"heading_enabled": ParameterValue(
                             LaunchConfiguration("uav_heading"), value_type=bool)}],
             condition=IfCondition(LaunchConfiguration("uav_assistance")), output="screen"),
        Node(package="robot_localization", executable="ekf_node", name="ugv_ekf",
             parameters=[str(share / "config/ugv_ekf.yaml"), clock],
             condition=UnlessCondition(LaunchConfiguration("uav_assistance")),
             remappings=[("odometry/filtered", "/ugv/localization/odometry")]),
        Node(package="robot_localization", executable="ekf_node", name="ugv_ekf",
             parameters=[str(share / "config/ugv_ekf.yaml"),
                         str(share / "config/ugv_uav_ekf.yaml"), clock],
             condition=IfCondition(LaunchConfiguration("uav_assistance")),
             remappings=[("odometry/filtered", "/ugv/localization/odometry")]),
        Node(package="cooperative_localization", executable="ugv_fused_pose.py",
             parameters=[clock, {"systematic_position_stddev": ParameterValue(
                 LaunchConfiguration("systematic_position_stddev"), value_type=float)}]),
    ])
