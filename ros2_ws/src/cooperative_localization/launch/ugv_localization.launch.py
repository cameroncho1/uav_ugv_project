"""Run the UGV sensor adapter and its map-frame EKF."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("cooperative_localization"))
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
        DeclareLaunchArgument("measurement_profile", default_value="nominal"),
        DeclareLaunchArgument("uav_assistance", default_value="false"),
        DeclareLaunchArgument("uav_odometry_topic", default_value="/fmu/out/vehicle_odometry"),
        DeclareLaunchArgument("uav_require_airborne", default_value="false"),
        DeclareLaunchArgument("uav_initial_settle_time", default_value="0.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package_share / "launch/uav_correction.launch.py")),
            launch_arguments={
                "world": world,
                "uav_odometry_topic": LaunchConfiguration("uav_odometry_topic"),
                "uav_require_airborne": LaunchConfiguration("uav_require_airborne"),
                "uav_initial_settle_time": LaunchConfiguration("uav_initial_settle_time"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("uav_assistance")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package_share / "launch/ugv_sensors.launch.py")),
            launch_arguments={"world": world, "seed": seed,
                              "measurement_profile": LaunchConfiguration("measurement_profile"),
                              "initial_gps_enabled": initial_gps_enabled}.items(),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="ugv_imu_mount",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "simple_ugv/chassis",
                "--child-frame-id", "simple_ugv/chassis/imu",
            ],
            output="screen",
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ugv_ekf",
            parameters=[str(package_share / "config/ugv_ekf.yaml")],
            condition=UnlessCondition(LaunchConfiguration("uav_assistance")),
            remappings=[("odometry/filtered", "/ugv/localization/odometry")],
            output="screen",
        ),
        Node(
            package="robot_localization", executable="ekf_node", name="ugv_ekf",
            parameters=[str(package_share / "config/ugv_ekf.yaml"),
                        str(package_share / "config/ugv_uav_ekf.yaml")],
            condition=IfCondition(LaunchConfiguration("uav_assistance")),
            remappings=[("odometry/filtered", "/ugv/localization/odometry")], output="screen",
        ),
        Node(
            package="cooperative_localization",
            executable="ugv_fused_pose.py",
            name="ugv_fused_pose",
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
    ])
