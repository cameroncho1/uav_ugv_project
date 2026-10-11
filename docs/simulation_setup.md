# Simulation dependencies

## Supported configuration

The simulator uses Gazebo Harmonic 8.6 or newer and ROS 2 Humble on Ubuntu 22.04. The exact source revisions are in [dependencies.json](../simulation/dependencies.json):

| Component | Revision | Purpose |
| --- | --- | --- |
| PX4 | `0462e3a31e7f0a6f6f05a5e094bee8a52e8759ec` | Converts the corrected Gazebo magnetic field from FLU/tesla to FRD/gauss. |
| px4_msgs | `109e8c8e151938f03fee2e62e3e93dc5ff5e2281` | Matches the wire fields and versions of the PX4 messages consumed by this project. |
| PX4 Gazebo models | `a15af9628536914ff7201c992fce5e3cb5d70db9` | Specifies X500 magnetometer noise as `3e-7` tesla per axis. |

Both project worlds set `use_units_gauss=false` and `use_earth_frame_ned=false` on their magnetometer plugin. Standalone worlds declare their own plugins, so PX4's `server.config` does not configure them. The [upstream PX4 fix](https://github.com/PX4/PX4-Autopilot/pull/28877) explains the unit and frame contract.

The Offboard controller derives the vehicle-status topic suffix from `VehicleStatus::MESSAGE_VERSION`; this configuration uses `/fmu/out/vehicle_status_v4`. The camera model publishes `/world/<world>/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image`. The goal-demo launch supplies this topic through the mapper's existing `camera_topic` parameter.

## Build and check

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

Rebuild every ROS package that consumes px4_msgs after changing its revision. Sourcing an older workspace can select incompatible generated types. The dependency checker verifies source revisions, consumed message schemas, world plugin settings, model noise units, and the minimum Gazebo version. The world launchers run it before starting Gazebo.

ROS nodes use Gazebo `/clock`. Localization trials set `UXRCE_DDS_SYNCT=0` before PX4 starts, preserving its simulation acquisition timestamps. The DDS domain must match `ROS_DOMAIN_ID`. The [localization interfaces](cooperative_localization/interfaces.md) describe topics and trial commands.

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

`GPS_CASE` accepts `on`, `off`, or `startup_off`. The live runner uses actual PX4 anchors by default. `ANCHOR_MODE=simulated` selects the independent noisy anchor fixture. Each run drives a paired baseline and assisted UGV filter on identical UGV measurements, then stops its simulation processes. Accuracy and uncertainty results belong in [evaluation results](cooperative_localization/evaluation_results.md).
