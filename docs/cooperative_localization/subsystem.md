# Cooperative UGV Localization

## Purpose and Current State

This subsystem supplies a fused UGV position from wheel motion, IMU heading, and GPS. Its input layer bridges the simulated sensors into ROS 2. A measurement adapter projects GPS into the map frame, anchors wheel odometry to the UGV spawn pose, and publishes noisy readings with configured covariance. A planar EKF combines them and publishes `/ugv/localization/pose`. Both Gazebo worlds contain the sensors. [Interfaces](interfaces.md) lists the live topics, frames, error settings, and commands to check them.

The fused pose combines wheel speed, IMU heading, GPS when available, and optional UAV-derived UGV map position. A simulated relative-position sensor, PX4 frame adapter, and timestamp-matched correction producer supply the UAV input. GPS and UAV gates support controlled outages. The goal demo plans and drives from `/ugv/localization/pose`. Both consumers check frame, acquisition age, receipt age, and position/heading uncertainty before allowing navigation.

## Sensor Boundary

`cooperative_localization` owns the ROS-facing raw sensor topic names and their Gazebo bridge configuration:

- `/ugv/sensors/wheel_odom`: `nav_msgs/msg/Odometry`, 20 Hz in Gazebo.
- `/ugv/sensors/imu`: `sensor_msgs/msg/Imu`, 50 Hz in Gazebo.
- `/ugv/sensors/gps`: `sensor_msgs/msg/NavSatFix`, 5 Hz in Gazebo.
- `/clock`: Gazebo simulation time for consumers that enable `use_sim_time`.

These are raw simulation measurements. Wheel odometry is relative to the UGV spawn pose; GPS is geodetic; IMU is in its chassis sensor frame. Their raw covariance fields do not provide a useful uncertainty model. `ugv_measurements.py` publishes prepared wheel odometry and GPS position in ENU `map`, and an IMU reading in its original sensor frame. It adds configurable independent Gaussian noise and corresponding covariance; the raw inputs remain unchanged. The values are test assumptions, not hardware calibration.

## Fused Pose and Boundaries

The implemented baseline publishes one UGV estimate on `/ugv/localization/pose`, type `geometry_msgs/msg/PoseWithCovarianceStamped`, in the ENU `map` frame. `robot_localization` runs the 2D EKF. It predicts from wheel forward speed, constrains lateral speed to zero for the differential-drive UGV, tracks heading with IMU yaw and turn rate, and corrects map x/y with GPS. The output adapter copies the EKF pose and covariance into the agreed message type. An identity TF relates the chassis and the IMU mounted at its origin.

The UAV-assisted correction uses a measured UAV-to-UGV vector in UAV FLU body axes and an independently localized UAV pose in ENU `map`. `uav_pose.py` converts PX4 NED/FRD odometry, preserving sample time and transforming uncertainty. `uav_correction.py` interpolates anchors at observation time, rotates the vector into map, and propagates relative, UAV position, and UAV attitude covariance. The EKF uses the resulting x/y observation. [Measurement design](uav_measurement_design.md) defines equations, rejection rules, research sources, and assumptions.

The path planner and UGV controller consume the fused topic; [navigation](../navigation.md) defines their limits and verification. Terrain mapping, obstacle detection, route planning, and flight control stay in their existing packages.

## GPS Degradation and Evaluation

The measurement adapter exposes `/ugv/localization/set_gps_enabled` (`std_srvs/srv/SetBool`). Setting `data: false` stops prepared GPS pose messages while wheel and IMU measurements continue. Setting `data: true` restores GPS. The raw Gazebo GPS topic continues publishing, which lets the same simulator run both cases. Sensor types have separate seeded random streams so GPS gating does not change wheel or IMU noise.

`ugv_evaluation.launch.py` adds a test-only command bridge and true-pose reader. `scripts/run_localization_trials.sh` starts a fresh `uav_ugv` world for each seed and GPS case, drives a forward/turn route, and tests GPS always on, GPS switched off at 10 seconds, and GPS off from startup. The scorer matches estimates to true positions by simulation timestamp and records each position and heading error. It reports mean, RMSE, 95th percentile, and maximum position error for the 10.5–31 second window. Truth is read only by the scorer; the EKF never receives it. The estimate topic is configurable so later localization strategies can use the same route and scoring method.

`ugv_cooperative_evaluation.launch.py` runs independent baseline and assisted EKFs on the same sensor messages and route. The scorer records both estimates, planar covariance, position and heading error, normalized error, and 95% ellipse coverage. A 91-second route repeats turns and stops. The `drift` profile injects 15% wheel-speed scale error and 0.002 rad/s heading bias growth. The relative sensor supports periodic 8 m outliers and timed dropouts. Fault timing uses the runner's published simulation start time.

The fixed-observer evaluation models a UAV at map `(2, -2, 4)` with noisy independent localization; those trials omit flight dynamics. Named-model sensing generates relative observations and independent noisy anchors from a flying Gazebo UAV. Actual PX4 anchors use the aligned dependency configuration in [simulation setup](../simulation_setup.md). Gaussian noise values are test assumptions. Correlated GNSS bias, calibrated camera perception, physical traction faults, and navigation using the fused pose remain outside the measured fixed-observer conditions. Claims apply only to the simulated conditions tested.

[Measured results](evaluation_results.md) list baseline-only trials and paired baseline/assisted trials with their sensor assumptions and uncertainty scores.

The initial scope assumes the UAV retains reliable localization while UGV GPS is degraded. It does not cover loss of both vehicles' global reference or full SLAM.
