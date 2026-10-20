"""Configure mutually compatible simulation/camera evaluation components."""

from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def resolve_mode(source, world="", model="", publish="", topic=""):
    if source not in ("camera", "simulated"):
        raise ValueError("relative_source must be camera or simulated")
    camera = source == "camera"
    world = world or ("simple_goal_demo" if camera else "uav_ugv")
    if world not in ("uav_ugv", "simple_goal_demo"):
        raise ValueError("Evaluation requires a configured integration world")
    model = model or ("cooperative_camera_uav_0" if camera else "")
    publish = publish.lower() if publish else ("false" if camera else "true")
    if publish not in ("true", "false"):
        raise ValueError("publish_anchor must be true or false")
    topic = topic or ("/simulation/uav_odometry" if publish == "true"
                      else "/fmu/out/vehicle_odometry")
    if camera and (publish == "true" or topic == "/simulation/uav_odometry"):
        raise ValueError("Camera mode requires an independently supplied PX4 anchor")
    return world, model, publish, topic


def description(assisted):
    def start(context):
        value = lambda key: LaunchConfiguration(key).perform(context)
        share = Path(get_package_share_directory("fault_management"))
        sim = Path(get_package_share_directory("simulation_support"))
        loc = Path(get_package_share_directory("cooperative_localization"))
        source = value("relative_source")
        world, model, publish, topic = resolve_mode(
            source, value("world"), value("uav_model"), value("publish_anchor"),
            value("uav_odometry_topic"))

        def include(pkg, file, arguments):
            path = Path(get_package_share_directory(pkg)) / "launch" / file
            return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                             launch_arguments=arguments.items())

        geometry = yaml.safe_load((sim / "config" / (world + ".yaml")).read_text())
        datum = geometry["sim_ugv_sensors"]["ros__parameters"]
        fault_args = {key: value(key) for key in ("seed", "initial_gps_enabled",
                                                 "shared_position_stddev", "measurement_profile")}
        fault_args["world"] = world
        nodes = [include("fault_management", "sim_ugv_sensors.launch.py", fault_args),
                 include("cooperative_localization", "ugv_localization.launch.py", {
                     "use_sim_time": "true", "uav_assistance": str(assisted).lower(),
                     "uav_heading": value("uav_heading") or str(source == "camera").lower(),
                     "systematic_position_stddev": value("systematic_position_stddev"),
                     **{key: str(datum[key]) for key in
                        ("origin_latitude", "origin_longitude", "origin_altitude")}})]
        env = {"GZ_PARTITION": EnvironmentVariable("GZ_PARTITION", default_value="uav_ugv_sim"),
               "GZ_IP": "127.0.0.1"}
        nodes += [Node(package="simulation_support", executable="ugv_truth",
                       parameters=[{"world": world, "use_sim_time": True}], additional_env=env),
                  Node(package="ros_gz_bridge", executable="parameter_bridge",
                       name="ugv_evaluation_command_bridge",
                       parameters=[{"config_file": str(sim / "config/ugv_evaluation_bridges.yaml")}],
                       additional_env=env)]
        if not assisted:
            return nodes
        live = topic != "/simulation/uav_odometry"
        nodes += [include("uav_control", "uav_pose.launch.py", {
            "use_sim_time": "true", "odometry_topic": topic,
            "map_origin": str(geometry["uav_pose"]["ros__parameters"]["map_origin"]),
            "require_airborne": value("uav_require_airborne") or str(live).lower(),
            "initial_settle_time": value("uav_initial_settle_time") or ("10.0" if live else "0.0")})]
        if source == "camera":
            nodes += [include("uav_sensing", "uav_camera_sensor.launch.py", {
                "world": world, "uav_model": model})]
        else:
            args = {key: value(key) for key in ("seed", "shared_position_stddev", "outlier_every",
                                               "dropout_start", "dropout_end")}
            args.update(world=world, uav_model=model, publish_anchor=publish)
            nodes += [include("fault_management", "sim_uav_sensor.launch.py", args)]
        baseline = yaml.safe_load((loc / "config/ugv_ekf.yaml").read_text())["ugv_ekf"]["ros__parameters"]
        baseline["use_sim_time"] = True
        nodes += [Node(package="robot_localization", executable="ekf_node", name="ugv_ekf_baseline",
                       parameters=[baseline], remappings=[("odometry/filtered", "/evaluation/baseline_odometry")]),
                  Node(package="cooperative_localization", executable="ugv_fused_pose.py",
                       name="baseline_pose", parameters=[{"use_sim_time": True,
                           "systematic_position_stddev": float(value("systematic_position_stddev"))}],
                       remappings=[("/ugv/localization/odometry", "/evaluation/baseline_odometry"),
                                   ("/ugv/localization/pose", "/evaluation/baseline_pose")])]
        return nodes
    return LaunchDescription([
        DeclareLaunchArgument("relative_source", default_value="simulated",
                              choices=["simulated", "camera"]),
        DeclareLaunchArgument("uav_heading", default_value="",
                              description="Camera heading fusion; defaults on for camera inputs."),
        DeclareLaunchArgument("world", default_value=""),
        DeclareLaunchArgument("seed", default_value="513"),
        DeclareLaunchArgument("initial_gps_enabled", default_value="true"),
        DeclareLaunchArgument("measurement_profile", default_value="nominal"),
        DeclareLaunchArgument("shared_position_stddev", default_value="0.0"),
        DeclareLaunchArgument("systematic_position_stddev",
                              default_value=LaunchConfiguration("shared_position_stddev")),
        DeclareLaunchArgument("uav_model", default_value=""),
        DeclareLaunchArgument("publish_anchor", default_value=""),
        DeclareLaunchArgument("uav_odometry_topic", default_value=""),
        DeclareLaunchArgument("uav_require_airborne", default_value=""),
        DeclareLaunchArgument("uav_initial_settle_time", default_value=""),
        DeclareLaunchArgument("outlier_every", default_value="0"),
        DeclareLaunchArgument("dropout_start", default_value="-1.0"),
        DeclareLaunchArgument("dropout_end", default_value="-1.0"),
        OpaqueFunction(function=start),
    ])
