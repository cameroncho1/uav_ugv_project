# Collect sensor data

Run in Ubuntu 22.04 with ROS 2 Humble and Gazebo Harmonic. Open every terminal at the repo root. Complete [setup](../README.md) and [dependencies](simulation_setup.md) first.

## 1. Build

```bash
git submodule update --init --recursive
source /opt/ros/humble/setup.bash
python3 scripts/check_sim_dependencies.py
(
  cd external/PX4-Autopilot
  DONT_RUN=1 make px4_sitl gz_x500
)
(
  cd ros2_ws
  colcon build --symlink-install
)
source ros2_ws/install/setup.bash
ros2 bag record --help
```

If `ros2 bag` is missing:

```bash
sudo apt install ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins
```

## 2. Configure a run

```bash
mkdir -p datasets/run_001
cp ros2_ws/src/fault_management/config/measurement_model.yaml datasets/run_001/noise.yaml
cat > datasets/run_001/session.env <<'EOF'
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=61
export GZ_PARTITION=uav_dataset_001
export GZ_IP=127.0.0.1
export DATASET_SEED=513
export DATASET_PROFILE=nominal
export DATASET_GPS_ENABLED=true
export DATASET_SHARED_STDDEV=0.0
export DATASET_SYSTEMATIC_STDDEV=0.0
EOF
nano datasets/run_001/noise.yaml
```

Edit the values under `sim_ugv_sensors: ros__parameters:`. Save with Ctrl+O, Enter; exit with Ctrl+X.

| Noise parameter | Default | Unit |
| --- | ---: | --- |
| `gps_horizontal_stddev` | 1.5 | m |
| `gps_vertical_stddev` | 3.0 | m |
| `odom_xy_stddev_per_sqrt_meter` | 0.05 | m / sqrt(m) |
| `odom_yaw_stddev_per_sqrt_meter` | 0.02 | rad / sqrt(m) |
| `odom_linear_velocity_stddev` | 0.05 | m/s |
| `odom_angular_velocity_stddev` | 0.02 | rad/s |
| `imu_orientation_stddev` | 0.02 | rad |
| `imu_angular_velocity_stddev` | 0.01 | rad/s |
| `imu_linear_acceleration_stddev` | 0.1 | m/s^2 |

- Lower values reduce noise. For almost-zero added noise, use `0.000001`. Exact zero GPS covariance is rejected by the estimator.
- Restart the simulation after changing noise. No rebuild is needed for the run file.
- `DATASET_SEED` overrides `random_seed` in the YAML.
- `nominal` still adds the configured Gaussian noise. `drift` also adds 15% wheel-speed error and 0.002 rad/s heading drift.
- `DATASET_SHARED_STDDEV` adds a persistent shared GPS/UAV position error in meters. `DATASET_SYSTEMATIC_STDDEV` adds a pose uncertainty prior. For a known 0.8 m shared-error trial, set both to `0.8`; navigation can hold on uncertainty.
- These controls affect UGV sensors and the stated shared-position fault. Camera pixels and PX4 sensor noise have separate settings.

## 3. Terminal A: record

```bash
source datasets/run_001/session.env

git rev-parse HEAD > datasets/run_001/git_commit.txt
git diff HEAD -- > datasets/run_001/working_tree.patch
git status --short > datasets/run_001/git_status.txt
git submodule status --recursive > datasets/run_001/submodules.txt
cp simulation/dependencies.json datasets/run_001/dependencies.json
cp ros2_ws/src/simulation_support/config/simple_goal_demo.yaml datasets/run_001/world.yaml
cp "ros2_ws/src/fault_management/config/measurement_${DATASET_PROFILE}.yaml" datasets/run_001/profile.yaml
cp ros2_ws/src/uav_sensing/config/camera_calibration.yaml datasets/run_001/camera_calibration.yaml

ros2 bag record --storage sqlite3 --use-sim-time \
  --output datasets/run_001/bag \
  /clock \
  /simulation/ugv/gps /simulation/ugv/imu /simulation/ugv/wheel_odom \
  /ugv/sensors/gps /ugv/sensors/imu /ugv/sensors/wheel_odom \
  /ugv/sensors/uav_relative_pose /uav/localization/pose \
  /ugv/measurements/gps_pose /ugv/measurements/uav_pose \
  /ugv/localization/pose /ugv/localization/odometry \
  /uav/camera/image /uav/camera/camera_info \
  /evaluation/ugv_truth \
  /global_goal_pose /ugv/navigation/route /ugv/cmd_vel \
  /diagnostics /tf /tf_static
```

- Leave recording running. It waits for `/clock` and discovers topics as they appear.
- Use a new bag directory for each run.
- Omit `/uav/camera/image` for numerical-only datasets.

## 4. Terminal B: publish truth labels

```bash
source datasets/run_001/session.env
ros2 run simulation_support ugv_truth --ros-args \
  -p use_sim_time:=true -p world:=simple_goal_demo
```

Leave it running. Labels use the UGV model origin in ENU `map`. For 3D chassis labels, apply the model-to-chassis transform from the world geometry.

## 5. Terminal C: run the simulation

```bash
source datasets/run_001/session.env
test -n "$DISPLAY" || { echo 'Use an Ubuntu desktop or WSLg terminal.'; exit 1; }

ros2 launch mission_manager simple_goal_demo.launch.py \
  start_rviz:=false \
  measurement_config:=datasets/run_001/noise.yaml \
  measurement_profile:="$DATASET_PROFILE" \
  seed:="$DATASET_SEED" \
  initial_gps_enabled:="$DATASET_GPS_ENABLED" \
  shared_position_stddev:="$DATASET_SHARED_STDDEV" \
  systematic_position_stddev:="$DATASET_SYSTEMATIC_STDDEV" \
  2>&1 | tee datasets/run_001/simulation.log
```

- This starts Gazebo, PX4, DDS, sensors, camera detection, localization, mapping, planning, and control. Run only one assembly.
- The UAV climbs to 4.5 m and settles. The UGV drives toward `(4, 0)` when its inputs pass navigation checks.
- Recording includes startup and stationary samples after arrival.

## 6. Terminal D: check data

```bash
source datasets/run_001/session.env
ros2 param dump /sim_ugv_sensors > datasets/run_001/effective_sensor_parameters.yaml
ros2 topic echo /ugv/sensors/imu --once
ros2 topic echo /evaluation/ugv_truth --once --qos-reliability best_effort
ros2 topic echo /ugv/sensors/uav_relative_pose --once
ros2 topic echo /ugv/localization/pose --once
```

- Check the saved noise values and seed. Ctrl+C cancels an echo waiting for missing data.
- For GPS denial from startup, set `DATASET_GPS_ENABLED=false` before launching.
- For a mid-run GPS outage:

```bash
ros2 service call /evaluation/set_gps_enabled std_srvs/srv/SetBool '{data: false}'
```

Use `true` to restore GPS. Save the action and simulation time in run notes; service calls are not recorded by this topic list.

## 7. Stop and inspect

- Ctrl+C in Terminal A. Wait for the bag to finish writing.
- Ctrl+C in Terminal C, then Terminal B. Wait for shutdown before another run.
- In Terminal D:

```bash
ros2 bag info datasets/run_001/bag | tee datasets/run_001/bag_info.txt
```

- Check nonzero counts for sensors, truth, camera images if selected, and relative observations. No noisy GPS messages are expected during a declared outage.
- Output: `datasets/run_001/bag/metadata.yaml` and `.db3` files containing serialized ROS messages.
- Raw sensors: `/simulation/ugv/*`. Corrupted sensors: `/ugv/sensors/*`. Estimates: `/ugv/localization/*`. Labels: `/evaluation/ugv_truth`.
- Raw readings precede added corruption. Wheel pose and IMU components also undergo adaptation; compare matching fields.
- Export using ROS message types. Align by acquisition stamps and frames. Keep whole runs in separate train/validation/test splits.
- Repeat with `datasets/run_002`, replacing the run path in every command and choosing another seed. Keep the whole run directory.

## Files

| Repo-root path | Use |
| --- | --- |
| `ros2_ws/src/fault_management/config/measurement_model.yaml` | Default UGV noise. |
| `ros2_ws/src/fault_management/config/measurement_nominal.yaml` | No extra drift. |
| `ros2_ws/src/fault_management/config/measurement_drift.yaml` | Wheel-scale and heading-drift faults. |
| `ros2_ws/src/fault_management/scripts/sim_ugv_sensors.py` | Noise implementation and GPS gate. |
| `ros2_ws/src/fault_management/launch/sim_ugv_sensors.launch.py` | Raw bridge and noise producer. |
| `ros2_ws/src/mission_manager/launch/simple_goal_demo.launch.py` | Complete camera simulation. |
| `ros2_ws/src/fault_management/config/sim_uav_sensor.yaml` | Synthetic UAV-vector noise; unused by this camera demo. |
| `docs/cooperative_localization/interfaces.md` | Topics, frames, timestamps, and covariance. |

[ROS 2 Humble recorder options](https://github.com/ros2/rosbag2/blob/humble/ros2bag/ros2bag/verb/record.py)
