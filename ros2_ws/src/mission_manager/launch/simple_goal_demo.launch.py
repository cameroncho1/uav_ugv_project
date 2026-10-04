from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    project_root = Path(get_package_prefix("mission_manager")).parents[2]
    package_share = Path(get_package_share_directory("mission_manager"))
    start_simulation = LaunchConfiguration("start_simulation")
    start_rviz = LaunchConfiguration("start_rviz")

    world = ExecuteProcess(
        cmd=["bash", str(project_root / "scripts/world_goal_demo.sh")],
        cwd=str(project_root),
        output="screen",
        name="gazebo_world",
        condition=IfCondition(start_simulation),
    )
    agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        output="screen",
        name="micro_xrce_agent",
        condition=IfCondition(start_simulation),
    )
    px4 = ExecuteProcess(
        cmd=["bash", str(project_root / "scripts/px4_goal_demo.sh")],
        cwd=str(project_root),
        output="screen",
        name="px4_sitl",
        condition=IfCondition(start_simulation),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_simulation", default_value="true",
            description="Start Gazebo, PX4 SITL, and Micro XRCE-DDS Agent.",
        ),
        DeclareLaunchArgument(
            "start_rviz", default_value="true",
            description="Start RViz with the simple goal demo configuration.",
        ),
        DeclareLaunchArgument("goal_x", default_value="4.0", description="Goal east coordinate in meters."),
        DeclareLaunchArgument("goal_y", default_value="0.0", description="Goal north coordinate in meters."),
        DeclareLaunchArgument(
            "ugv_odom_to_map_x", default_value="-4.0",
            description="X offset from Gazebo UGV odometry to the map frame.",
        ),
        DeclareLaunchArgument(
            "ugv_odom_to_map_y", default_value="0.0",
            description="Y offset from Gazebo UGV odometry to the map frame.",
        ),
        DeclareLaunchArgument("uav_altitude", default_value="4.0", description="UAV target altitude in meters."),
        DeclareLaunchArgument(
            "grid_resolution", default_value="0.25",
            description="Camera occupancy-grid cell size in meters.",
        ),
        DeclareLaunchArgument(
            "obstacle_luminance_threshold", default_value="32.0",
            description="Dark-pixel threshold used by the initial camera occupancy heuristic.",
        ),
        SetEnvironmentVariable("GZ_PARTITION", "uav_ugv_sim"),
        SetEnvironmentVariable("GZ_IP", "127.0.0.1"),
        world,
        agent,
        Node(
            package="mission_manager",
            executable="goal_manager",
            name="goal_manager",
            parameters=[{
                "goal_x": ParameterValue(LaunchConfiguration("goal_x"), value_type=float),
                "goal_y": ParameterValue(LaunchConfiguration("goal_y"), value_type=float),
            }],
            output="screen",
        ),
        Node(
            package="uav_control",
            executable="offboard_goal_controller",
            name="uav_goal_controller",
            parameters=[{
                "uav_altitude": ParameterValue(LaunchConfiguration("uav_altitude"), value_type=float),
            }],
            output="screen",
        ),
        Node(
            package="ugv_control",
            executable="goal_controller",
            name="ugv_goal_controller",
            parameters=[{
                "odom_to_map_x": ParameterValue(
                    LaunchConfiguration("ugv_odom_to_map_x"), value_type=float),
                "odom_to_map_y": ParameterValue(
                    LaunchConfiguration("ugv_odom_to_map_y"), value_type=float),
            }],
            output="screen",
        ),
        Node(
            package="traversability_mapping",
            executable="traversability_mapper",
            name="traversability_mapper",
            parameters=[{
                "grid_resolution": ParameterValue(LaunchConfiguration("grid_resolution"), value_type=float),
                "obstacle_luminance_threshold": ParameterValue(
                    LaunchConfiguration("obstacle_luminance_threshold"), value_type=float),
            }],
            output="screen",
        ),
        Node(
            package="route_planning",
            executable="local_astar_planner",
            name="local_astar_planner",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(package_share / "config/simple_goal_demo.rviz")],
            output="screen",
            condition=IfCondition(start_rviz),
        ),
        TimerAction(period=2.0, actions=[px4]),
    ])