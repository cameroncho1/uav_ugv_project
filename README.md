# UAV–UGV PX4 / ROS 2 Simulation

This repository contains a UAV–UGV simulation using PX4 SITL, Gazebo Sim, ROS 2, and Micro XRCE-DDS.

Current milestone:
- Custom Gazebo world with a simple UGV
- PX4 X500 UAV in the same world
- PX4 takeoff without requiring QGroundControl
- PX4 ↔ ROS 2 communication through Micro XRCE-DDS
- Live PX4 odometry received by a custom C++ ROS 2 node
- ROS 2 package structure for future autonomy development

---

## Tested Environment

| Component | Tested Version / Setting |
|---|---|
| Host | Apple Silicon Mac |
| VM | UTM |
| Ubuntu | 22.04 LTS |
| Architecture | ARM64 / aarch64 |
| ROS 2 | Humble |
| Gazebo Sim | Harmonic / gz-sim8 (tested with 8.14.0) |
| PX4-Autopilot | `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` |
| px4_msgs | `86d8239e962f6939e05c3737784f60c02fa884db` |
| UAV model | PX4 `gz_x500` |
| Micro XRCE-DDS Agent | 2.4.2 recommended for ROS 2 Humble |
| DDS transport | UDP |
| DDS port | `8888` |
| Gazebo world | `uav_ugv` |
| Gazebo partition | `uav_ugv_sim` |
| Gazebo IP | `127.0.0.1` |
| Gazebo GUI renderer | Ogre 1 (`--render-engine-gui ogre`) |
| GCS data-link requirement | Disabled with `NAV_DLL_ACT=0` |

PX4 and `px4_msgs` are pinned using Git submodules.

---

# Repository Layout

```text
uav_ugv_project/
├── external/
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
├── simulation/
│   └── worlds/
│       └── uav_ugv.sdf
│
├── scripts/
└── README.md
```

## ROS 2 Package Responsibilities (Just an outline for now, subject to change)

- `uav_control`: PX4/UAV interface. Current example: `odometry_listener.cpp`. Future examples: offboard control and scout control.
- `ugv_control`: UGV state and motion control.
- `traversability_mapping`: UAV perception data to terrain/hazard/traversability map.
- `route_planning`: UGV path planning and dynamic replanning.
- `cooperative_localization`: UAV-assisted UGV localization for GPS-denied scenarios.
- `fault_management`: Sensor, communication, and vehicle-health monitoring.
- `mission_manager`: High-level UAV/UGV mission coordination.
- `uav_ugv_msgs`: Custom ROS 2 messages shared between project packages.

---

# Fresh Setup: Mac + UTM + Ubuntu 22.04

Skip this section if Ubuntu 22.04 ARM64 is already installed.

## 1. Install UTM on the Mac

Install UTM for macOS:

https://mac.getutm.app/

Download an Ubuntu 22.04 LTS **ARM64 Ubuntu Server ISO** from Ubuntu.

Official UTM Ubuntu guide:

https://docs.getutm.app/guides/ubuntu/

## 2. Create the Ubuntu VM

In UTM:

1. Click `+`.
2. Select **Virtualize**.
3. Select **Linux**.
4. Select the Ubuntu 22.04 ARM64 Server ISO.
5. Assign CPU, memory, and disk space.
6. Complete the Ubuntu installer.
7. After installation, unmount/eject the installer ISO and boot from the virtual disk.

A practical starting point for this project is:

```text
CPU: 4+ virtual cores
RAM: 8 GB or more if available
Disk: 30–40 GB or more (I got it done with like 25)
```

## 3. Install the Ubuntu Desktop

```bash
sudo apt update
sudo apt install ubuntu-desktop -y
sudo reboot
```

## 4. Optional: Enable SSH for VS Code Remote Development

```bash
sudo apt update
sudo apt install openssh-server -y

sudo systemctl enable --now ssh
hostname -I
```

From the Mac:

```bash
ssh <ubuntu-user>@<VM-IP>
```

You can then use VS Code Remote - SSH to edit on the Mac while compiling and running everything inside Ubuntu.

---

# Software Setup Inside Ubuntu

## 1. Update Ubuntu and Install Basic Tools

```bash
sudo apt update
sudo apt upgrade -y

sudo apt install -y \
  git \
  curl \
  wget \
  build-essential \
  cmake \
  python3-pip
```

---

## 2. Install ROS 2 Humble

Configure the locale:

```bash
sudo apt update
sudo apt install locales -y

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

export LANG=en_US.UTF-8
```

Enable Universe:

```bash
sudo apt install software-properties-common -y
sudo add-apt-repository universe
```

Add the ROS 2 package repository:

```bash
sudo apt update
sudo apt install curl -y

sudo curl -sSL \
  https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Install ROS 2 Humble and development tools:

```bash
sudo apt update
sudo apt upgrade -y

sudo apt install ros-humble-desktop -y
sudo apt install ros-dev-tools -y
```

Source ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

Optional automatic sourcing:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

Install PX4/ROS Python dependencies:

```bash
pip install --user -U empy==3.3.4 pyros-genmsg setuptools
```

Verify:

```bash
source /opt/ros/humble/setup.bash

echo $ROS_DISTRO
ros2 --help
colcon --help
```

Expected:

```text
humble
```

---

# Clone the Project

```bash
cd ~

git clone --recurse-submodules \
  https://github.com/cameroncho1/uav_ugv_project.git

cd ~/uav_ugv_project
```

If cloned without submodules:

```bash
git submodule update --init --recursive
```

Verify:

```bash
git submodule status
```

Expected PX4 commit:

```text
d6f12ad1c4f70ad3230afd7d86e971421e02fef4
```

Expected `px4_msgs` commit:

```text
86d8239e962f6939e05c3737784f60c02fa884db
```

## Testing the Feature Branch Before Merge

```bash
git switch cameron/simple-uav-ugv-sim
git submodule update --init --recursive
```

---

# Install PX4 + Gazebo Dependencies

The repository already contains the pinned PX4 source.

```bash
cd ~/uav_ugv_project

bash external/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx
```

This installs the PX4 simulator/development dependencies, including Gazebo for Ubuntu 22.04.

Reboot:

```bash
sudo reboot
```

Verify Gazebo:

```bash
gz sim --versions
```

This project uses Gazebo Harmonic / `gz-sim8` and was tested with Gazebo Sim 8.14.0.

A fresh install may have a newer Harmonic 8.x patch version.

---

# Install Micro XRCE-DDS Agent

PX4 runs a lightweight XRCE-DDS client. The Agent connects PX4 to the ROS 2 DDS network.

For ROS 2 Humble, install Micro XRCE-DDS Agent 2.4.2:

```bash
cd ~

git clone -b v2.4.2 \
  https://github.com/eProsima/Micro-XRCE-DDS-Agent.git

cd Micro-XRCE-DDS-Agent

mkdir -p build
cd build

cmake ..
make -j$(nproc)

sudo make install
sudo ldconfig /usr/local/lib/
```

Verify:

```bash
MicroXRCEAgent --help
```

The simulation uses UDP port `8888`.

---

# Build PX4

```bash
cd ~/uav_ugv_project

./scripts/build_px4.sh
```

Manual equivalent:

```bash
cd ~/uav_ugv_project/external/PX4-Autopilot
make px4_sitl
```

---

# Build the ROS 2 Workspace

## First Full Build

```bash
cd ~/uav_ugv_project/ros2_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash
```

The first build can take several minutes because `px4_msgs` generates and compiles many ROS 2 message types.

`--symlink-install` is useful during development. C++ files still require recompilation after changes.


## Normal Incremental Development

You do not need to rebuild `px4_msgs` every time.

Example:

```bash
cd ~/uav_ugv_project/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build \
  --packages-select uav_control \
  --symlink-install

source install/setup.bash
```

For another package:

```bash
colcon build \
  --packages-select route_planning \
  --symlink-install
```

Use a full:

```bash
colcon build --symlink-install
```

mainly for fresh setup or when intentionally rebuilding the complete workspace.

---

# Start the Simulation

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

If needed:

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

Leave it running.

## Terminal 4 — ROS 2 Development Shell

There is intentionally no `ros.sh` wrapper so this terminal can be used for any ROS 2 command or node.

```bash
source /opt/ros/humble/setup.bash
source ~/uav_ugv_project/ros2_ws/install/setup.bash
```

Examples:

```bash
ros2 topic list | grep /fmu
```

```bash
ros2 topic echo /fmu/out/vehicle_odometry --once
```

```bash
ros2 run uav_control odometry_listener
```

## Terminal 5 — Gazebo GUI

Run this from the Ubuntu graphical desktop:

```bash
cd ~/uav_ugv_project
./scripts/gui.sh
```

The current UTM setup uses Ogre 1 because Ogre 2 has crashed in this VM configuration.

The GUI is optional.

---

# Test PX4 ↔ ROS 2

Make sure Gazebo, PX4, and the Micro XRCE-DDS Agent are running.

```bash
ros2 topic list | grep /fmu
```

Typical output includes:

```text
/fmu/out/vehicle_odometry
/fmu/out/vehicle_global_position
/fmu/out/vehicle_local_position_v1
/fmu/out/vehicle_status_v1

/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
/fmu/in/vehicle_command
```

Some topics are versioned (`_v1`). Use the exact names shown by `ros2 topic list`.

Test PX4 → ROS 2:

```bash
ros2 topic echo /fmu/out/vehicle_odometry --once
```

Inspect:

```bash
ros2 topic info /fmu/out/vehicle_odometry
```

Expected:

```text
Type: px4_msgs/msg/VehicleOdometry
Publisher count: 1
```

Communication path:

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

# Test the C++ ROS 2 Odometry Node

```bash
cd ~/uav_ugv_project/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build \
  --packages-select uav_control \
  --symlink-install

source install/setup.bash

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

A UAV about 2.3 m above the local origin may therefore report:

```text
z ≈ -2.3 m
```

---

# Test UAV Takeoff

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

---

# Test UGV Motion

```bash
gz topic \
  -t /simple_ugv/cmd_vel \
  -m gz.msgs.Twist \
  -p 'linear: {x: 0.5}'
```

---

# Normal Startup Summary

```text
Terminal 1: ./scripts/world.sh
Terminal 2: ./scripts/px4.sh
Terminal 3: ./scripts/dds.sh
Terminal 4: source ROS 2 + workspace, then run desired ROS 2 nodes
Terminal 5: ./scripts/gui.sh
```

The Gazebo GUI is optional.

---

# Adding New ROS 2 C++ Code

Adding a `.cpp` file does not automatically create a ROS 2 executable.

Add the target to the package's `CMakeLists.txt`.

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

Build only that package:

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
CMakeLists.txt  = how C++ code is compiled
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
MicroXRCEAgent --help
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

# Troubleshooting Notes

- Do not use `commander arm -f` for normal testing.
- `NAV_DLL_ACT=0` disables the GCS/data-link-loss requirement for this headless ROS 2 simulation.
- ROS 2 Offboard control has its own failsafe behavior and should be configured separately later.
- A one-time transient simulated sensor timeout can occur in a VM. If PX4 reports `Ready for takeoff!` and `commander check` returns `OK`, the vehicle recovered.
- Repeated sensor timeouts indicate a simulation/performance issue.
- `px4_msgs` contains PX4 ROS 2 message definitions; it is not the simulated vehicle.
- Micro XRCE-DDS transports data between PX4 and the ROS 2 DDS network.
- If the Gazebo GUI crashes using Ogre 2 in UTM, use `gui.sh`, which launches it with Ogre 1.

---

# Reference Documentation

- UTM Ubuntu 22.04 guide: https://docs.getutm.app/guides/ubuntu/
- PX4 Ubuntu environment: https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu
- PX4 Gazebo simulation: https://docs.px4.io/main/en/sim_gazebo_gz/
- PX4 ROS 2 user guide: https://docs.px4.io/main/en/ros2/user_guide
- PX4 uXRCE-DDS bridge: https://docs.px4.io/main/en/middleware/uxrce_dds
