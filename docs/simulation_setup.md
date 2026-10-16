# Simulation dependencies

## Supported configuration

Run the simulator inside Ubuntu 22.04 on either host. Use ARM64 Ubuntu on Apple Silicon with UTM, or x86-64 Ubuntu in a Windows VM or WSL2. [Host setup](../README.md) covers both paths. The original macOS/UTM record covers the basic simulation; the audit validation covers x86-64 WSL2. A fresh ARM64 build of the audit branch is not recorded.

The simulator uses Gazebo Harmonic 8.6 or newer and ROS 2 Humble on Ubuntu 22.04. The exact source revisions are in [dependencies.json](../simulation/dependencies.json):

| Component | Revision | Purpose |
| --- | --- | --- |
| PX4 | `0462e3a31e7f0a6f6f05a5e094bee8a52e8759ec` | Converts the corrected Gazebo magnetic field from FLU/tesla to FRD/gauss. |
| px4_msgs | `109e8c8e151938f03fee2e62e3e93dc5ff5e2281` | Matches the wire fields and versions of the PX4 messages consumed by this project. |
| PX4 Gazebo models | `a15af9628536914ff7201c992fce5e3cb5d70db9` | Specifies X500 magnetometer noise as `3e-7` tesla per axis. |

Both project worlds set `use_units_gauss=false` and `use_earth_frame_ned=false` on their magnetometer plugin. Standalone worlds declare their own plugins, so PX4's `server.config` does not configure them. The [upstream PX4 fix](https://github.com/PX4/PX4-Autopilot/pull/28877) explains the unit and frame contract.

The Offboard controller derives the vehicle-status topic suffix from `VehicleStatus::MESSAGE_VERSION`; this configuration uses `/fmu/out/vehicle_status_v4`. The owned camera model publishes `/world/<world>/model/cooperative_camera_uav_0/link/camera_link/sensor/camera/image` at 5 Hz. It inherits the aligned X500 flight model and uses the mounting geometry in [perception](cooperative_localization/perception.md). The goal-demo launch supplies this topic through the mapper's existing `camera_topic` parameter.

## Build and check

Complete the [dependency installation](#dependency-metadata-and-harmonic-overlay) first. Generate the PX4 Gazebo target before starting the world scripts:

```bash
cd external/PX4-Autopilot
DONT_RUN=1 make px4_sitl gz_x500
cd ../..
```

Run these commands in Linux from the repository root:

```bash
git submodule update --init --recursive
source /opt/ros/humble/setup.bash
python3 scripts/check_sim_dependencies.py

cd external/PX4-Autopilot
make px4_sitl_default -j6
cd ../../ros2_ws
colcon build --symlink-install
source install/setup.bash
cd ..
```

Rebuild every ROS package that consumes px4_msgs after changing its revision. Sourcing an older workspace can select incompatible generated types. The dependency checker verifies source revisions, consumed message schemas, world plugin settings, model noise units, and the minimum Gazebo version. The world launchers run it before starting Gazebo. They also expose the built `ugv_control` actuator-watchdog library through `GZ_SIM_SYSTEM_PLUGIN_PATH`.

Simulation ROS nodes use Gazebo `/clock`. Localization trials set `UXRCE_DDS_SYNCT=0` before PX4 starts. The DDS domain matches `ROS_DOMAIN_ID`. The standalone operational estimator defaults to wall time. The [localization interfaces](cooperative_localization/interfaces.md) describe topics and trial commands.

## Display and launchers

Camera runs require `DISPLAY` in the Ubuntu process environment, including runs with RViz or the Gazebo GUI disabled. Open the terminal in the Ubuntu desktop or a WSLg session. Check:

```bash
echo "$DISPLAY"
gz sim --versions
```

The camera runners use the Ogre server renderer. The original UTM setup also uses Ogre for the GUI. A pure SSH session without display forwarding does not supply camera rendering.

`scripts/start_sim.sh` opens the basic `uav_ugv` simulation in GNOME Terminal tabs. Without GNOME Terminal it falls back to `scripts/start_sim_tmux.sh`. Install `tmux` with `sudo apt install tmux` for that path. These launchers do not start the complete localization/navigation demo; use the [goal-demo launch](../README_simple_goal_demo.md#run) for that system. The basic `px4.sh` uses DDS time synchronization unless `PX4_DDS_SYNCT=0` is set. The camera demo's `px4_goal_demo.sh` selects simulation timestamps directly.

The basic simulation has no camera navigation assembly. Do not reuse it as the camera demo with only `start_simulation:=false`; start the matching goal-demo world, camera UAV and PX4 script first.

## Live anchor evaluation

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
SEED=513 GPS_CASE=startup_off scripts/run_live_cooperative_trial.sh /tmp/live_513_startup
SEED=514 GPS_CASE=off scripts/run_live_cooperative_trial.sh /tmp/live_514_outage
SEED=515 GPS_CASE=on scripts/run_live_cooperative_trial.sh /tmp/live_515_gps
scripts/run_aligned_anchor_trials.sh /tmp/aligned_matrix

# Evaluation-only dependency; use a virtual environment if needed.
python3 -m pip install pyulog
python3 scripts/score_px4_heading.py /path/to/flight.ulg \
  /tmp/live_513_startup/trial_live.json /tmp/live_513_startup/heading_check
```

`GPS_CASE` accepts `on`, `off`, or `startup_off`. The live runner uses actual PX4 anchors by default. The live runner defaults to camera observations. `RELATIVE_MODE=simulated` selects the vector fixture; with that fixture, `ANCHOR_MODE=simulated` selects independently noisy synthetic anchors. `run_aligned_anchor_trials.sh` explicitly uses simulated vectors; `run_camera_anchor_trials.sh` uses rendered pixels. Each run drives a paired baseline and assisted UGV filter on identical UGV measurements, then stops its simulation processes. Accuracy and uncertainty results belong in [evaluation results](cooperative_localization/evaluation_results.md).


## Dependency metadata and Harmonic overlay

The estimator no longer links Gazebo. `simulation_support` declares transport/message development dependencies; the simulated actuator driver declares Sim/transport/messages/plugin dependencies. The mapper consumes ROS camera data and map anchors instead of linking Gazebo or raw PX4.

[Project rosdep rules](../simulation/rosdep.yaml) map the declared keys to the OSRF Jammy development packages and the Harmonic bridge. The upstream [base definitions](https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml) do not supply those version-specific keys. If the PX4 setup has not already registered the OSRF repository, add it with the [Gazebo Ubuntu installation steps](https://gazebosim.org/docs/harmonic/install_ubuntu/):

```bash
sudo apt install -y curl lsb-release gnupg
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt update
```

On amd64, use `ros-humble-ros-gzharmonic-bridge`. The OSRF Jammy package indexes checked on October 15 list that bridge for amd64 but not arm64. ARM64 uses the source build below. The default Humble `ros_gz` binary targets Fortress. [Gazebo's ROS pairing instructions](https://gazebosim.org/docs/harmonic/ros_installation/) describe the separate Harmonic packages.

Register the project overlay before the default sources:

```bash
# Once, if rosdep has not been initialized.
sudo rosdep init

# Run from this repository in Linux. as_uri handles paths containing spaces.
python3 -c 'from pathlib import Path; print("yaml " + Path("simulation/rosdep.yaml").resolve().as_uri())' \
  | sudo tee /etc/ros/rosdep/sources.list.d/00-uav-ugv-harmonic.list
rosdep update --rosdistro humble
```

### ARM64 Harmonic bridge

Run this section on ARM64 after registering the overlay above. The [upstream Humble branch](https://github.com/gazebosim/ros_gz/tree/856e319ee4da713609b3f6e58c74ac8db7662482) supports `GZ_VERSION=harmonic`. Build only the bridge and its interface package:

```bash
source /opt/ros/humble/setup.bash
mkdir -p "$HOME/ros_gz_harmonic_ws/src"
cd "$HOME/ros_gz_harmonic_ws/src"
git clone --branch humble https://github.com/gazebosim/ros_gz.git
git -C ros_gz checkout 856e319ee4da713609b3f6e58c74ac8db7662482
cd ..
export GZ_VERSION=harmonic
export IGNITION_VERSION=
rosdep install --from-paths src/ros_gz/ros_gz_bridge src/ros_gz/ros_gz_interfaces \
  --ignore-src --rosdistro humble -y
MAKEFLAGS=-j1 colcon build --packages-up-to ros_gz_bridge \
  --executor sequential --parallel-workers 1 --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

Keep this install sourced when building the project. The project workspace records the sourced underlay in its setup chain. This source recipe is checked against upstream build metadata; an ARM64 build is not recorded in the audit results.

### Install project dependencies

Return to the project root. On amd64, rosdep installs the binary bridge. On arm64, source the bridge underlay and skip the binary bridge key:

```bash
cd "$HOME/uav_ugv_project"
source /opt/ros/humble/setup.bash
if [ "$(dpkg --print-architecture)" = arm64 ]; then
  source "$HOME/ros_gz_harmonic_ws/install/setup.bash"
  rosdep install --from-paths ros2_ws/src --ignore-src --rosdistro humble \
    --skip-keys ros_gz_bridge -y
else
  rosdep install --from-paths ros2_ws/src --ignore-src --rosdistro humble -y
fi
```

The full build includes the driver plugin required by both worlds. After changing PX4 message definitions, rebuild every consumer and source the current install.

The runtime also needs `MicroXRCEAgent` from the [agent build](../README.md#install-micro-xrce-dds-agent). If Python packages in the user site override Ubuntu packages, OpenCV and ROS plotting need NumPy below version 2. The recorded setup uses `empy==3.3.4` and `setuptools==65.7.0` for message generation.
