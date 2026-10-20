# Simple UAV–UGV Goal Demo

[Documentation index](docs/README.md) · [System diagram](docs/system_design.html) · [Localization guide](docs/cooperative_localization/README.md)

The goal demo keeps the UAV horizontally stationary while it climbs to a camera altitude. A local occupancy grid is derived from the downward camera image, A* plans through the grid from the UGV pose to the fixed goal, and the UGV follows that local path. The flat world uses a solid matte ground color and simple geometric vehicle and goal visuals, with shadows disabled to reduce rendering load. It has no physical obstacles. The image classifier is an initial dark-pixel heuristic, not semantic obstacle detection. There is no global path or global replanning yet.

The default local goal is `(4, 0)` in the ROS `map` frame. The UGV starts at `(-4, 0)` and the UAV holds its initial horizontal position at a 4.5 m altitude setpoint above its PX4 local origin so its camera footprint covers the route. The integration-owned world configuration supplies the UGV spawn and independent UAV map origin. Goal and altitude can be changed with launch arguments; if changing the goal coordinates, also update the bullseye pose in the world SDF.

## Build

Install Gazebo Harmonic development libraries, `MicroXRCEAgent`, the PX4 Gazebo target, a Harmonic build of `ros_gz_bridge`, `robot_localization`, and OpenCV through the [dependency setup](docs/simulation_setup.md#dependency-metadata-and-harmonic-overlay). The setup includes an amd64 binary bridge path and an ARM64 source-build path. From the repository root:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-up-to mission_manager uav_control ugv_control traversability_mapping route_planning cooperative_localization --symlink-install
source install/setup.bash
```

## Run

Open a terminal in the Ubuntu desktop session so Gazebo and RViz inherit `DISPLAY`. From the repository root, source ROS and the workspace, then launch the complete demo:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch mission_manager simple_goal_demo.launch.py
```

The launch starts the Gazebo world, PX4 SITL, Micro XRCE-DDS Agent, the goal manager, stationary-XY UAV controller, camera grid mapper, local A-star planner, UGV path follower, and RViz. The UAV controller holds its initial horizontal position, streams the altitude setpoint, waits for PX4 preflight checks, then retries Offboard and arm requests until PX4 confirms the state. Keep the launch terminal open; Ctrl+C shuts down the launched processes.

The launch also starts UGV localization. An EKF combines wheel speed, IMU heading, GPS map position, and UAV-derived UGV position on `/ugv/localization/pose` with uncertainty. The UAV adapter converts the live PX4 anchor from NED/FRD to ENU/FLU after takeoff and a ten-second reference-settling interval. A calibrated AprilTag detector produces UGV-relative observations from camera pixels; a timestamp-matched adapter rotates them into map and includes UAV position and attitude uncertainty. `uav_assistance:=false` selects the GPS/wheel/IMU baseline. [The localization interfaces](docs/cooperative_localization/interfaces.md) define topics, gates, and evaluation commands. The planner and controller consume `/ugv/localization/pose` and stop on stale or uncertain estimates. Motion also requires a goal-associated `/ugv/navigation/route` with a footprint-clearance grid; `/local_path` is visualization/evaluation only. A steady-clock actuator watchdog expires commands if the controller or bridge disappears. [Navigation](docs/navigation.md) lists the configured limits and checks.

ROS nodes use `/clock`, and `px4_goal_demo.sh` sets `UXRCE_DDS_SYNCT=0` so PX4 samples use the same time source. When starting PX4 separately for this demo, use that script. UAV relative observations come from a known 0.4 m roof tag, CameraInfo intrinsics, and body-camera extrinsics. The detector and occupancy classifier use the same rendered images through separate interfaces. [Perception](docs/cooperative_localization/perception.md) gives calibration assumptions, visibility gates, and tests.

Change the local goal, camera altitude, grid resolution, or image threshold with launch arguments:

```bash
ros2 launch mission_manager simple_goal_demo.launch.py goal_x:=4.0 goal_y:=0.0 uav_altitude:=4.5 grid_resolution:=0.25 obstacle_luminance_threshold:=32.0
```

If the matching `simple_goal_demo` world, camera UAV, PX4, and DDS agent are already running, use `start_simulation:=false`. The basic `start_sim.sh` world and X500 are not that camera setup. To omit RViz, use `start_rviz:=false`. The camera still requires `DISPLAY`.

In the Gazebo window, add its **Image Display** plugin and select `/world/simple_goal_demo/model/cooperative_camera_uav_0/link/camera_link/sensor/camera/image`. To list camera topics, run `gz topic -l | grep -Ei 'camera|image'`.

RViz's **Camera heading** display shows `/uav/camera/heading_image`, with a green arrow pointing toward the UGV front as measured from its roof tag. Heading fusion is enabled in the demo; `uav_heading:=false` keeps camera position assistance while omitting camera heading from the filter. Planning and control read the fused pose from `/ugv/localization/pose` in both modes. [Camera heading](docs/cooperative_localization/camera_heading.md) documents the measurement, mounting calibration, uncertainty, and validation commands.

## Grid Scope

RViz displays `/local_grid`, `/local_path`, and the `/global_goal` marker. Each ground cell uses five projected pixel samples; any sample below the threshold marks it occupied. Cells outside the complete footprint remain unknown. Grids retain image acquisition time and require a matched independent map anchor. A matched bounded-uncertainty UGV pose masks its own visible body. This is only a prototype visual heuristic: shadows and dark texture may become false obstacles, while obstacles similar in brightness to grass can be missed. The grid is not a semantic or safety-rated obstacle map. The camera feed and image-derived grid are real simulation outputs; A* runs only within the current camera footprint.

## Starting just Gazebo

Run each command group in its own Ubuntu terminal. Source ROS and the workspace in each terminal. Use the same `ROS_DOMAIN_ID` and `GZ_PARTITION` throughout. Start Gazebo before PX4, then start the ROS launch.

```bash
cd ~/uav_ugv_project
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
./scripts/world_goal_demo.sh
```

## Starting DDS agent

```bash
source /opt/ros/humble/setup.bash
MicroXRCEAgent udp4 -p 8888
```

## Starting all ROS2 nodes without Gazebo and Rviz2

```bash
cd ~/uav_ugv_project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mission_manager simple_goal_demo.launch.py start_simulation:=false start_rviz:=false
```

## Starting PX4 (once Gazebo is fully running)

```bash
cd ~/uav_ugv_project
./scripts/px4_goal_demo.sh
```

## Starting Rviz2 for grid visualization

```bash
cd ~/uav_ugv_project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
rviz2 -d install/mission_manager/share/mission_manager/config/simple_goal_demo.rviz
```

The demo uses actual PX4 anchors and calibrated camera-derived relative positions. [Simulation setup](docs/simulation_setup.md) defines the matching firmware, ROS messages, UAV models, and ENU/tesla magnetometer settings.
