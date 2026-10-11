"""Compare independent baseline and assisted filters on identical sensor streams."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    share = Path(get_package_share_directory("cooperative_localization"))
    baseline_parameters = yaml.safe_load(
        (share / "config/ugv_ekf.yaml").read_text())["ugv_ekf"]["ros__parameters"]

    def include(file, arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / file)),
            launch_arguments=arguments.items(),
        )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="uav_ugv"),
        DeclareLaunchArgument("seed", default_value="513"),
        DeclareLaunchArgument("initial_gps_enabled", default_value="true"),
        DeclareLaunchArgument("measurement_profile", default_value="nominal"),
        DeclareLaunchArgument("outlier_every", default_value="0"),
        DeclareLaunchArgument("dropout_start", default_value="-1.0"),
        DeclareLaunchArgument("dropout_end", default_value="-1.0"),
        DeclareLaunchArgument("uav_model", default_value=""),
        DeclareLaunchArgument("publish_anchor", default_value="true"),
        DeclareLaunchArgument("uav_odometry_topic", default_value="/simulation/uav_odometry"),
        DeclareLaunchArgument("uav_require_airborne", default_value="false"),
        DeclareLaunchArgument("uav_initial_settle_time", default_value="0.0"),
        include("ugv_localization.launch.py", {
            "world": LaunchConfiguration("world"), "seed": LaunchConfiguration("seed"),
            "initial_gps_enabled": LaunchConfiguration("initial_gps_enabled"),
            "measurement_profile": LaunchConfiguration("measurement_profile"),
            "uav_assistance": "true",
            "uav_odometry_topic": LaunchConfiguration("uav_odometry_topic"),
            "uav_require_airborne": LaunchConfiguration("uav_require_airborne"),
            "uav_initial_settle_time": LaunchConfiguration("uav_initial_settle_time"),
        }),
        include("sim_uav_sensor.launch.py", {
            key: LaunchConfiguration(key) for key in
            ("world", "seed", "outlier_every", "dropout_start",
             "dropout_end", "uav_model", "publish_anchor")
        }),
        # This EKF receives the same wheel, IMU, and GPS inputs, without UAV readings.
        Node(package="robot_localization", executable="ekf_node", name="ugv_ekf_baseline",
             parameters=[baseline_parameters],
             remappings=[("odometry/filtered", "/evaluation/baseline_odometry")], output="screen"),
        Node(package="cooperative_localization", executable="ugv_fused_pose.py",
             name="baseline_pose", parameters=[{"use_sim_time": True}],
             remappings=[("/ugv/localization/odometry", "/evaluation/baseline_odometry"),
                         ("/ugv/localization/pose", "/evaluation/baseline_pose")]),
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             name="ugv_evaluation_command_bridge",
             parameters=[{"config_file": str(share / "config/ugv_evaluation_bridges.yaml")}],
             additional_env={"GZ_PARTITION": EnvironmentVariable(
                 "GZ_PARTITION", default_value="uav_ugv_sim"),
                 "GZ_IP": "127.0.0.1"}),
        Node(package="cooperative_localization", executable="ugv_truth",
             parameters=[{"world": LaunchConfiguration("world")}],
             additional_env={"GZ_PARTITION": EnvironmentVariable(
                 "GZ_PARTITION", default_value="uav_ugv_sim"),
                 "GZ_IP": "127.0.0.1"}),
    ])
