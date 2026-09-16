
# UAV–UGV PX4 / ROS 2 Simulation

This repository contains a UAV–UGV simulation using PX4 SITL, Gazebo Sim, ROS 2, and Micro XRCE-DDS.

The current milestone supports:

- Custom Gazebo world with a simple UGV
- PX4 X500 UAV in the same world
- PX4 takeoff without requiring QGroundControl
- PX4 ↔ ROS 2 communication through Micro XRCE-DDS
- Live PX4 odometry received by a custom C++ ROS 2 node
- ROS 2 package structure for future autonomy development

---

## Tested Environment

| Component | Version / Setting |
|---|---|
| Ubuntu | 22.04 LTS |
| Architecture | ARM64 / aarch64 |
| ROS 2 | Humble |
| Gazebo Sim | 8.14.0 |
| PX4-Autopilot | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| px4_msgs | `86d8239e962f6939e05c3737784f60c02fa884db` |
| UAV model | PX4 `gz_x500` |
| DDS transport | UDP |
| DDS port | `8888` |
| Gazebo world | `uav_ugv` |
| Gazebo partition | `uav_ugv_sim` |
| Gazebo IP | `127.0.0.1` |
| Gazebo GUI renderer | Ogre 1 (`--render-engine-gui ogre`) |
| GCS data-link requirement | Disabled with `NAV_DLL_ACT=0` |
| Micro XRCE-DDS Agent | Required for PX4 ↔ ROS 2 |

PX4 and `px4_msgs` are pinned using Git submodules.

---

# Repository Layout

```text
uav_ugv_project/
├── external/                       #External dependencies such as PX4.
│   └── PX4-Autopilot/              # pinned PX4 submodule
│
├── ros2_ws/
│   └── src/
│       ├── px4_msgs/               # pinned PX4 ROS 2 message definitions
│       ├── uav_control/
│       ├── ugv_control/
│       ├── traversability_mapping/
│       ├── route_planning/
│       ├── cooperative_localization/
│       ├── fault_management/
│       ├── mission_manager/
│       └── uav_ugv_msgs/
│
├── simulation/                    # Simulation worlds, models, sensors, and configuration.
│   └── worlds/
│       └── uav_ugv.sdf
│
├── scripts/
└── README.md
```

## ROS 2 Package Responsibilities

### `uav_control`

PX4/UAV interface.

Current example:

```text
odometry_listener.cpp
```

Future code may include:

```text
offboard_controller.cpp
scout_controller.cpp
```

This package communicates with PX4 topics such as:

```text
/fmu/out/vehicle_odometry
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
```

### `ugv_control`

UGV state and motion control.

Example interfaces:

```text
/ugv/odom
/ugv/cmd_vel
```

### `traversability_mapping`

UAV camera/perception data → terrain or hazard map.

Future examples:

- road / grass / mud / water classification
- semantic segmentation
- obstacle detection
- traversability cost map

### `route_planning`

UGV path planning and replanning.

Future examples:

- A*
- Dijkstra
- terrain-risk-aware planning
- energy-aware planning

### `cooperative_localization`

UAV-assisted UGV localization, especially for GPS-denied scenarios.

### `fault_management`

Sensor, communication, and vehicle health monitoring.

### `mission_manager`

Coordinates high-level UAV/UGV mission state and behaviors.

### `uav_ugv_msgs`

Custom messages shared between project packages.

---

# 1. Clone

```bash
git clone --recurse-submodules https://github.com/cameroncho1/uav_ugv_project.git
cd uav_ugv_project
```

If already cloned without submodules:

```bash
git submodule update --init --recursive
```

---

# 2. Install Dependencies

## PX4 dependencies

On Ubuntu 22.04:

```bash
bash external/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx
```

Reboot if requested.

## ROS 2 Humble

Verify ROS 2 is installed:

```bash
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO
```

Expected:

```text
humble
```

---

# 3. Build PX4

From the repository root:

```bash
./scripts/build_px4.sh
```

Manual equivalent:

```bash
cd external/PX4-Autopilot
make px4_sitl
```

---

# 4. Build the ROS 2 Workspace

## First full build

From a fresh clone:

```bash
cd ~/uav_ugv_project/ros2_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash
```

The first build can take a long time because `px4_msgs` generates and compiles many ROS 2 message types.

`--symlink-install` is recommended for development. It symlinks installable non-compiled files where possible instead of copying them.

C++ files still require recompilation after changes.

### Important

Do not source this before the first clean build:

```bash
source install/setup.bash
```

Source it after the build finishes.

If the workspace was previously built without `--symlink-install` and CMake reports a directory/symbolic-link conflict, do a one-time clean rebuild:

```bash
cd ~/uav_ugv_project/ros2_ws

rm -rf build install log

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Normal incremental development

You do NOT need to rebuild `px4_msgs` every time you modify your code.

For example, after changing code in `uav_control`:

```bash
cd ~/uav_ugv_project/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build \
  --packages-select uav_control \
  --symlink-install

source install/setup.bash
```

If you create another package, build only that package:

```bash
colcon build \
  --packages-select route_planning \
  --symlink-install
```

Use a full:

```bash
colcon build --symlink-install
```

mainly for fresh setup or when you intentionally want to rebuild the entire workspace.

---

# 5. Start the Simulation

The current setup uses separate terminals.

## Terminal 1 — Gazebo Server

```bash
cd ~/uav_ugv_project
./scripts/world.sh
```

## Terminal 2 — PX4 SITL + X500

```bash
cd ~/uav_ugv_project
./scripts/px4.sh
```

PX4 should eventually report:

```text
Ready for takeoff!
```

If the script does not set the GCS data-link behavior automatically:

```text
pxh> param set NAV_DLL_ACT 0
pxh> commander check
```

Expected:

```text
Preflight check: OK
```

## Terminal 3 — Micro XRCE-DDS Agent

```bash
cd ~/uav_ugv_project
./scripts/dds.sh
```

Manual equivalent:

```bash
MicroXRCEAgent udp4 -p 8888
```

Leave this running.

The Agent connects PX4's lightweight XRCE-DDS client to the normal ROS 2 DDS network.

## Terminal 4 — ROS 2

Source the environment:

```bash
source /opt/ros/humble/setup.bash
source ~/uav_ugv_project/ros2_ws/install/setup.bash
```

Run the current odometry node:

```bash
ros2 run uav_control odometry_listener
```

If your `scripts/ros.sh` launches the odometry node directly:

```bash
cd ~/uav_ugv_project
./scripts/ros.sh
```

## Terminal 5 — Gazebo GUI

Run from the Ubuntu graphical desktop terminal:

```bash
cd ~/uav_ugv_project
./scripts/gui.sh
```

The current VM uses Ogre 1 because Ogre 2 has crashed in this environment.

The Gazebo GUI is optional.

---

# 6. Test PX4 ↔ ROS 2

Make sure these are running:

```text
Gazebo server
PX4
Micro XRCE-DDS Agent
```

Then in a sourced ROS 2 terminal:

```bash
ros2 topic list | grep /fmu
```

Example output includes:

```text
/fmu/out/vehicle_odometry
/fmu/out/vehicle_global_position
/fmu/out/vehicle_local_position_v1
/fmu/out/vehicle_status_v1

/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
```

Some topics are versioned (`_v1`) in this PX4/px4_msgs version. Always use the exact topic names returned by:

```bash
ros2 topic list
```

Test PX4 → ROS 2:

```bash
ros2 topic echo /fmu/out/vehicle_odometry --once
```

Inspect the topic:

```bash
ros2 topic info /fmu/out/vehicle_odometry
```

Expected:

```text
Type: px4_msgs/msg/VehicleOdometry
Publisher count: 1
```

The communication path is:

```text
Gazebo
   ↕
PX4 SITL
   ↕
PX4 Micro XRCE-DDS Client
   ↕ UDP 8888
Micro XRCE-DDS Agent
   ↕
ROS 2
   ↕
Project C++ nodes
```

---

# 7. Test the C++ ROS 2 Odometry Node

Build only `uav_control`:

```bash
cd ~/uav_ugv_project/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build \
  --packages-select uav_control \
  --symlink-install

source install/setup.bash
```

Run:

```bash
ros2 run uav_control odometry_listener
```

Expected output is similar to:

```text
Listening to /fmu/out/vehicle_odometry
PX4 position (NED): x=..., y=..., z=... m
```

PX4 uses NED coordinates:

```text
+x = North
+y = East
+z = Down
```

Therefore a UAV approximately 2.3 m above the local origin may report:

```text
z ≈ -2.3 m
```

---

# 8. Test UAV Takeoff

In the PX4 terminal:

```text
pxh> commander check
```

Expected:

```text
Preflight check: OK
```

Then:

```text
pxh> commander takeoff
```

The X500 should take off vertically in Gazebo.

---

# 9. Test UGV Motion

Direct Gazebo command:

```bash
gz topic -t /simple_ugv/cmd_vel \
  -m gz.msgs.Twist \
  -p 'linear: {x: 0.5}'
```

If the helper script exists:

```bash
./scripts/drive_ugv.sh 0.5 0
```

Stop:

```bash
./scripts/drive_ugv.sh 0 0
```

---

# Normal Startup Summary

After dependencies and builds are already complete:

```text
Terminal 1: ./scripts/world.sh
Terminal 2: ./scripts/px4.sh
Terminal 3: ./scripts/dds.sh
Terminal 4: ROS 2 node / ./scripts/ros.sh
Terminal 5: ./scripts/gui.sh
```

Without the Gazebo GUI, four terminals are sufficient.

---

# Adding New ROS 2 C++ Code

Adding a `.cpp` file to a package does not automatically create a ROS 2 executable.

Add the target to that package's `CMakeLists.txt`.

Example:

```cmake
add_executable(offboard_controller
  src/offboard_controller.cpp
)

ament_target_dependencies(
  offboard_controller
  rclcpp
  px4_msgs
)

install(
  TARGETS
    odometry_listener
    offboard_controller
  DESTINATION lib/${PROJECT_NAME}
)
```

Then build only the package:

```bash
colcon build \
  --packages-select uav_control \
  --symlink-install
```

Run:

```bash
ros2 run uav_control offboard_controller
```

ROS 2 build roles:

```text
package.xml     = package metadata and dependencies
CMakeLists.txt  = how the C++ code is compiled
ament_cmake     = ROS-aware CMake helpers
colcon          = builds packages/workspaces in dependency order
ros2 run        = finds and launches installed ROS executables
```

---

# Useful Version Checks

```bash
lsb_release -a
uname -m
printenv ROS_DISTRO
gz sim --versions
MicroXRCEAgent --version
```

Verify pinned repositories:

```bash
git -C external/PX4-Autopilot rev-parse HEAD
git -C ros2_ws/src/px4_msgs rev-parse HEAD
```

Expected:

```text
PX4:
d6f12ad1c4f70ad3230afd7d86e971421e02fef4

px4_msgs:
86d8239e962f6939e05c3737784f60c02fa884db
```

---

# Notes

- Do not use `commander arm -f` for normal testing.
- `NAV_DLL_ACT=0` disables the GCS/data-link-loss requirement for this headless ROS 2 simulation.
- ROS 2 Offboard control has its own failsafe behavior and should be configured separately later.
- A one-time transient simulated sensor timeout can occur in a VM. If PX4 reports `Ready for takeoff!` and `commander check` returns `OK`, the vehicle recovered.
- Repeated sensor timeouts indicate a simulation/performance problem.
- `px4_msgs` is a dependency/message-definition package, not the drone itself.
- Micro XRCE-DDS transports data between PX4 and the ROS 2 DDS network.
