# Simple UAV–UGV Goal Demo

This milestone keeps the UAV horizontally stationary while it climbs to a camera altitude. A local occupancy grid is derived from the downward camera image, A* plans through the grid from the UGV pose to the fixed goal, and the UGV follows that local path. The flat world uses a solid matte ground color and simple geometric vehicle and goal visuals, with shadows disabled to reduce rendering load. It has no physical obstacles. The image classifier is an initial dark-pixel heuristic, not semantic obstacle detection. There is no global path or global replanning yet.

The default local goal is `(4, 0)` in the ROS `map` frame. The UGV starts at `(-4, 0)` and the UAV holds its initial horizontal position at 4 m altitude so its camera footprint covers the route. Gazebo UGV odometry is offset into the map frame with `ugv_odom_to_map_x` and `ugv_odom_to_map_y` (defaults `-4` and `0`); adjust them if you change the UGV spawn pose. Goal and altitude can be changed with launch arguments; if changing the goal coordinates, also update the bullseye pose in the world SDF.

## Build

Gazebo Harmonic transport development libraries (`gz-transport13` and `gz-msgs10`), `MicroXRCEAgent`, and a built PX4 Gazebo target must be available. From the repository root:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-up-to mission_manager uav_control ugv_control traversability_mapping route_planning --symlink-install
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

Change the local goal, camera altitude, grid resolution, or image threshold with launch arguments:

```bash
ros2 launch mission_manager simple_goal_demo.launch.py goal_x:=4.0 goal_y:=0.0 uav_altitude:=4.0 grid_resolution:=0.25 obstacle_luminance_threshold:=32.0
```

If Gazebo, PX4, and the DDS agent are already running in other terminals, skip starting them again with `start_simulation:=false`. To omit RViz, use `start_rviz:=false`.

In the Gazebo window, add its **Image Display** plugin and select `/world/simple_goal_demo/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image`. To list camera topics, run `gz topic -l | grep -Ei 'camera|image'`.

## Grid Scope

RViz displays `/local_grid`, `/local_path`, and the `/global_goal` marker. Each image cell is classified occupied when its average luminance falls below the configured threshold; other cells are free. This is only a prototype visual heuristic: shadows and dark texture may become false obstacles, while obstacles similar in brightness to grass can be missed. The grid is not a semantic or safety-rated obstacle map. The camera feed and image-derived grid are real simulation outputs; A* runs only within the current camera footprint.

## Starting just Gazebo
cd ~/uav_ugv_project
./scripts/world_goal_demo.sh

## Starting DDS agent
MicroXRCEAgent udp4 -p 8888

## Starting all ROS2 nodes without Gazebo and Rviz2
cd ~/uav_ugv_project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mission_manager simple_goal_demo.launch.py start_simulation:=false start_rviz:=false

## Starting PX4 (once Gazebo is fully running)
cd ~/uav_ugv_project
./scripts/px4_goal_demo.sh

## Starting Rviz2 for grid visualization
cd ~/uav_ugv_project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
rviz2 -d install/mission_manager/share/mission_manager/config/simple_goal_demo.rviz
