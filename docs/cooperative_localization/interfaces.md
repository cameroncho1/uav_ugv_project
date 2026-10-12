# Cooperative localization interfaces

## Topics and frames

Estimator inputs preserve acquisition timestamps. Localization nodes use Gazebo `/clock`; PX4 must share this clock with `UXRCE_DDS_SYNCT=0` when its pose is fused. Raw sensors use best-effort sensor QoS. Prepared measurements and fused poses use reliable QoS.

| Topic | ROS message | Frame and meaning |
| --- | --- | --- |
| `/ugv/sensors/wheel_odom` | `nav_msgs/Odometry` | Gazebo differential-drive odometry, 20 Hz, relative to spawn; child `simple_ugv/chassis`. |
| `/ugv/sensors/imu` | `sensor_msgs/Imu` | Chassis-mounted inertial measurement, 50 Hz; `simple_ugv/chassis/imu`. |
| `/ugv/sensors/gps` | `sensor_msgs/NavSatFix` | WGS84 GPS, 5 Hz; `simple_ugv/chassis/gps`. |
| `/ugv/measurements/wheel_odom` | `nav_msgs/Odometry` | Noisy pose anchored to map spawn; body forward speed and zero lateral speed. |
| `/ugv/measurements/imu` | `sensor_msgs/Imu` | Noisy yaw and turn rate in the chassis sensor frame. |
| `/ugv/measurements/gps_pose` | `geometry_msgs/PoseWithCovarianceStamped` | GPS projected to ENU `map`, with meter noise and covariance; position only. |
| `/uav/camera/image` | `sensor_msgs/Image` | Rendered pixels, 5 Hz; acquisition stamp and `uav/camera_optical_frame`. |
| `/uav/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics/distortion in the same optical frame and image dimensions. |
| `/uav/localization/pose` | `geometry_msgs/PoseWithCovarianceStamped` | Independent UAV map position and FLU attitude with covariance. |
| `/ugv/sensors/uav_relative_pose` | `geometry_msgs/PoseWithCovarianceStamped` | UAV-to-UGV vector in `uav/base_link` (FLU), with position covariance; orientation unused. |
| `/ugv/measurements/uav_pose` | `geometry_msgs/PoseWithCovarianceStamped` | UGV map position derived from timestamp-matched UAV pose and relative observation; orientation unused. |
| `/ugv/localization/odometry` | `nav_msgs/Odometry` | Planar EKF output in `map`, 30 Hz. |
| `/ugv/localization/pose` | `geometry_msgs/PoseWithCovarianceStamped` | EKF pose/time/covariance with the configured systematic x/y uncertainty prior added. |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Anchor/correction freshness and rejection counts; camera visibility and reprojection fit. |

The bridge maps Gazebo `/simple_ugv/odom`, `/simple_ugv/imu`, `/simple_ugv/gps`, and `/clock` to ROS. Raw wheel and IMU covariance is zero and GPS covariance is unknown. The adapter supplies an explicit uncertainty model while raw topics remain available. GPS is converted through WGS84 ECEF to local ENU. The UGV map spawn is `(1, 0)` in `uav_ugv` and `(-4, 0)` in `simple_goal_demo`.

The baseline EKF fuses body forward and zero lateral wheel velocity, IMU yaw and turn rate, and GPS x/y. It does not fuse integrated wheel pose or GPS placeholder orientation. UAV assistance adds absolute x/y position with a five-sigma innovation threshold. The chassis-to-IMU TF is identity. The EKF does not publish map-to-chassis TF.

## UAV observation contract

The relative sensor reports the UGV chassis origin relative to the UAV body origin. A perception producer must apply calibrated sensor-to-body extrinsics before publishing. Its position covariance must be finite and positive in the same FLU axes. It must use the shared acquisition clock. A hardware producer can replace the simulator through this interface.

`uav_pose.py` accepts PX4 `POSE_FRAME_NED` with valid position, quaternion, and variances. It converts world axes from NED to ENU and body axes from FRD to FLU. `map_origin` supplies the local-to-map translation; the two world YAML files contain the configured origins. `uav_require_airborne:=true uav_initial_settle_time:=10.0` initializes the live reference after takeoff and a reset-free interval. `/fmu/out/vehicle_land_detected` supplies the takeoff state. Later pose resets stop anchor output until realignment and restart. Clock mismatches are rejected.

`uav_correction.py` uses an exact anchor or interpolates position and quaternion at observation time. The maximum anchor gap is 0.25 s, maximum age 0.5 s, and maximum relative range 30 m. It propagates UAV position and attitude covariance plus rotated relative covariance. It rejects invalid inputs, duplicate stamps, and jumps exceeding a configured motion bound plus five standard deviations. It does not extrapolate or subscribe to UGV truth. [Measurement design](uav_measurement_design.md) gives equations and limits.

The synthetic vector fixture uses relative standard deviations `(0.25, 0.15, 0.25)` m in FLU. Simulated UAV position deviations are `(0.3, 0.3, 0.5)` m in map and attitude deviations `(0.01, 0.01, 0.02)` rad in body axes. Independent seeded random streams make an unchanged message sequence reproducible. These are simulation assumptions, not calibrated sensor specifications.

## Services and launch modes

- `/ugv/localization/set_gps_enabled`: `std_srvs/SetBool`; disables or restores prepared GPS while raw GPS continues.
- `/ugv/localization/set_uav_enabled`: `std_srvs/SetBool`; disables or restores UAV corrections while other sensors continue.
- `ugv_sensors.launch.py`: bridges and prepares measurements. `world` selects spawn and GPS origin; `measurement_profile` selects `nominal` or `drift`.
- `ugv_localization.launch.py`: starts the baseline, or adds UAV adapters and fusion with `uav_assistance:=true`. `uav_odometry_topic` selects live PX4 or a compatible test source.
- `sim_uav_sensor.launch.py`: generates noisy relative observations. Empty `uav_model` selects a fixed observer at map `(2, -2, 4)`; a named model selects its actual Gazebo pose. `publish_anchor:=false` uses an independently supplied UAV anchor.
- `simple_goal_demo.launch.py`: starts assisted localization with actual PX4 anchors and calibrated relative observations from camera pixels. `uav_assistance:=false` selects the baseline. Navigation consumes `/ugv/localization/pose` with freshness and uncertainty gates; `/ugv/cmd_vel` carries velocity commands.

## Evaluation and reproducibility

`ugv_evaluation.launch.py` evaluates the baseline. `ugv_cooperative_evaluation.launch.py` runs baseline and assisted filters together on identical sensor messages. `/evaluation/ugv_truth` supplies true pose to the scorer, `/evaluation/ugv_cmd_vel` drives the route, and `/evaluation/baseline_pose` carries the baseline. `/simulation/vehicle_poses` supplies synchronized true poses to the synthetic sensor producer. `/simulation/uav_odometry` supplies noisy PX4-format anchor samples in fixed-observer trials. Those trials omit UAV flight dynamics.

The scorer matches estimates to interpolated truth at their timestamps and rejects truth gaps over 0.1 s. CSVs hold every scored position, heading error, covariance, and planar normalized estimation error squared (NEES). JSON summaries contain conditions, sensor counts, rejection counts, and error metrics. NEES uses the EKF x/y covariance; 5.991 is the 95% chi-square threshold for two coordinates. Ellipse coverage is a descriptive check on temporally correlated samples, not an independent-sample statistical claim.

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-up-to cooperative_localization mission_manager --symlink-install
source install/setup.bash
cd ..

# PX4 hover with actual PX4 anchors and rendered camera sensing.
scripts/run_live_cooperative_trial.sh /tmp/cooperative_live

# Three GPS cases across three seeds.
scripts/run_cooperative_trials.sh /tmp/cooperative_short

# 91-second route with wheel-scale and IMU-heading faults.
ROUTE=long DURATION=91 MEASUREMENT_PROFILE=drift \
  scripts/run_cooperative_trials.sh /tmp/cooperative_drift

# GPS loss, outliers, and a 20-second correction outage.
GPS_CASES=off ROUTE=long DURATION=91 MEASUREMENT_PROFILE=drift \
  OUTLIER_EVERY=10 DROPOUT_START=35.0 DROPOUT_END=55.0 \
  scripts/run_cooperative_trials.sh /tmp/cooperative_robust
```

`SEEDS` and `GPS_CASES` accept space-separated lists. `off` disables GPS at route time 10 s; `startup_off` starts with GPS disabled. `ROUTE=long` repeats the short forward/turn/stop pattern. The drift profile injects 15% forward wheel-speed error and 0.002 rad/s growing IMU heading bias. Its covariance still describes Gaussian noise, so these biases are deliberate faults. Outliers add 8 m in body x every tenth sensor sample. Dropout times are relative to `/evaluation/trial_start`.

Each trial starts a fresh world and stops its processes. Results include scored CSVs, a baseline CSV for paired trials, JSON summaries, and `summary.csv` with one row per strategy. `GZ_PARTITION` can select an isolated Gazebo transport partition; the default is `uav_ugv_sim`. Avoid another world or controller in the same Gazebo partition and ROS domain. The baseline runner remains `scripts/run_localization_trials.sh`. [Measured results](evaluation_results.md) describe tested conditions and stored summaries.

`scripts/run_live_cooperative_trial.sh` flies a PX4 X500 and compares both UGV filters. Its default `ANCHOR_MODE=px4` uses the actual PX4 estimator and a ten-second post-takeoff initialization interval. `GPS_CASE` selects `on`, `off`, or `startup_off` (default). `RELATIVE_MODE=simulated ANCHOR_MODE=simulated` selects independent noisy anchors and vector observations on the flying model. Camera mode uses actual PX4 anchors. `ANCHOR_TIMEOUT` bounds startup in wall seconds (default 240); `TRIAL_TIMEOUT` bounds scoring in wall seconds (default 300). These timeouts do not change the 31-second simulation route or timestamp/uncertainty gates. Both modes write the source and observer model into their trial summaries. `scripts/run_aligned_anchor_trials.sh` repeats the vector fixture across all three GPS cases and seeds 513-515. `scripts/run_camera_anchor_trials.sh` repeats rendered-camera trials with the same cases and seeds. [Perception](perception.md) defines camera calibration and shared-bias fault settings. [Simulation setup](../simulation_setup.md) specifies the aligned dependency configuration and heading-scoring command.
