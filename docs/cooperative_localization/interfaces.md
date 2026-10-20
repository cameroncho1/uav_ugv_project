# Cooperative localization interfaces

[Documentation index](../README.md) · [Localization guide](README.md)

## Stable contract and ownership

- Localization owns measured-input validation/fusion and one authoritative UGV pose. Relative sensing is independent of sensing technology.
- Shared global positions use ENU `map`; ROS body axes use FLU. PX4 NED/FRD conversion, covariance, and local-origin alignment are explicit in the UAV interface owner.
- Acquisition stamps are preserved. Simulation assembly uses `/clock`; actual PX4 anchors use `UXRCE_DDS_SYNCT=0` and the matching ROS DDS domain.
- Simulator truth is used only for synthesis and scoring. Estimation and navigation consume the stated measurement interfaces. The fused UGV pose is never fed into its UAV correction producer.

| Owner | Responsibility |
| --- | --- |
| `cooperative_localization` | Geodetic adaptation, validity/time/uncertainty handling, cooperative correction matching, EKF, authoritative pose. |
| `uav_control` | PX4 interface, reference initialization/resets, independent map-anchor publication. |
| `uav_sensing` | Camera calibration/detection and body-relative position producer. |
| `traversability_mapping` | Acquisition-matched camera-to-grid projection. |
| `simulation_support` | World/model integration, raw sensor bridge, shared `/clock` bridge, truth readers. |
| `fault_management` | Synthetic measurements, explicit injected outages/errors, paired trials and scoring. |
| `route_planning` / `ugv_control` | Clearance routes associated with goal revisions / path following and actuator command expiry. |
| `localization_common` | Frame, covariance and geodesy math without ROS or simulator imports. |

Integration owns UGV bridging and the shared clock. Global map-to-body TF publication and hardware sensor calibration remain open ownership decisions.

## Topics and frames

Raw simulated data uses sensor QoS. Prepared inputs and fused poses use reliable publishers; acquisition timestamps are unchanged.

| Topic | ROS message | Meaning |
| --- | --- | --- |
| `/simulation/ugv/wheel_odom` | `nav_msgs/Odometry` | Raw Gazebo DiffDrive odometry; parent `simple_ugv/odom`, child `simple_ugv/chassis`. |
| `/simulation/ugv/imu` | `sensor_msgs/Imu` | Raw simulated IMU, frame `simple_ugv/chassis/imu`. |
| `/simulation/ugv/gps` | `sensor_msgs/NavSatFix` | Raw WGS84 simulator fix, frame `simple_ugv/chassis/gps`. |
| `/ugv/sensors/wheel_odom`, `/ugv/sensors/imu`, `/ugv/sensors/gps` | Corresponding sensor messages | Measured interface. Simulation assembly supplies explicit noise/covariance here; a hardware producer can supply measured values directly. |
| `/ugv/measurements/wheel_odom` | `nav_msgs/Odometry` | Validated body velocities and covariance. Integrated pose is not fused. |
| `/ugv/measurements/imu` | `sensor_msgs/Imu` | Validated components; `-1` availability markers remain attached to missing components. |
| `/ugv/measurements/gps_pose` | `geometry_msgs/PoseWithCovarianceStamped` | WGS84 projected into map ENU with known covariance rotated from the fix's tangent basis. Orientation is unused. |
| `/uav/localization/pose` | `geometry_msgs/PoseWithCovarianceStamped` | Independent UAV position/FLU attitude and covariance in map. |
| `/ugv/sensors/uav_relative_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Chassis pose relative to UAV body in `uav/base_link` FLU. Camera inputs include measured orientation and joint fixed-axis covariance; position-only inputs use large angular uncertainty. |
| `/ugv/measurements/uav_pose` | `geometry_msgs/PoseWithCovarianceStamped` | UGV map position from a timestamp-matched independent anchor plus relative observation. |
| `/ugv/measurements/uav_pose_with_heading` | `geometry_msgs/PoseWithCovarianceStamped` | Joint UGV map x/y/yaw correction, including cross-covariance. An observation uses this topic or the position-only topic, never both. |
| `/ugv/localization/odometry` | `nav_msgs/Odometry` | Internal planar EKF output. |
| `/ugv/localization/pose` | `geometry_msgs/PoseWithCovarianceStamped` | Authoritative map pose/covariance, including the configured unreduced systematic x/y prior. |
| `/uav/camera/image`, `/uav/camera/camera_info` | `sensor_msgs/Image`, `sensor_msgs/CameraInfo` | Rendered/measured optical data; `uav/camera_optical_frame`. |
| `/uav/camera/heading_image` | `sensor_msgs/Image` | Best-effort bgr8 image with measured chassis-forward arrow; original acquisition stamp and optical frame. Visualization only. |
| `/local_grid` | `nav_msgs/OccupancyGrid` | Acquisition-stamped map projection with matched camera anchor and calibrated mount. |
| `/ugv/navigation/route` | `uav_ugv_msgs/PlannedRoute` | Goal revision, path and the conservative footprint-clearance grid used to plan it. |
| `/local_path` | `nav_msgs/Path` | Visualization and evaluation reference; does not authorize motion. |
| `/ugv/cmd_vel` | `geometry_msgs/Twist` | Controller command. Integration bridges it to `/simple_ugv/drive_cmd_vel`; the actuator watchdog alone forwards fresh commands to DiffDrive. |
| `/evaluation/ugv_truth` | `geometry_msgs/PoseStamped` | Scoring-only simulator state. |

## Validity, uncertainty and clocks

The operational adapter checks frames, finite coordinates, component availability and acquisition age (0.5 s). Duplicate/out-of-order stamps are rejected within an epoch; an actual ROS-clock rewind clears epoch state. Known GNSS covariance is preserved and transformed; invalid known covariance is rejected. Unknown covariance uses explicit fallback priors from `measurements.yaml`, not injected randomness.

IMU orientation requires a valid unit quaternion. Missing/invalid orientation is marked unavailable while valid rate/acceleration components remain usable. The EKF uses yaw and turn rate; missing heading does not become a synthetic zero-heading observation. GNSS and UAV x/y innovations use five-sigma rejection thresholds. An unknown global origin starts with `1e6 m^2` x/y variance, allowing an initial absolute anchor without pretending wheel/IMU data locate the origin.

The independent UAV adapter accepts NED/FRD PX4 samples with finite variances and a matching clock. `map_origin` is a stated alignment. Optional takeoff initialization requires airborne status and a reset-free interval. A later PX4 reference reset stops anchors pending realignment/restart. A ROS-clock rewind resets temporal initialization state. Origin guesses from UGV truth or fused pose are prohibited.

Corrections use exact anchors or interpolation with at most 0.25 s anchor gaps, 0.5 s acquisition age, and 30 m range. UAV position/attitude and relative uncertainty are propagated. No extrapolation is performed. Pending batches are detached before processing so clock resets cannot invalidate queue iteration.

`uav_heading:=true` enables joint camera x/y/yaw corrections. The standalone estimator defaults to false; the camera demo and camera evaluation default to true. Heading selection requires a valid relative quaternion, positive angular uncertainty, a horizontal forward-axis projection of at least 0.5, and map heading standard deviation at most 0.35 rad. Otherwise the position-only correction remains available. Joint covariance includes source position/orientation correlations and UAV attitude error. Both filter inputs use five-sigma innovation rejection. [Camera heading](camera_heading.md) defines geometry, controls, and validation.

## Launch modes and controls

| Launch / service | Implemented role |
| --- | --- |
| `cooperative_localization ugv_localization.launch.py` | Operational estimator only. Defaults to wall time and no assistance. Accepts explicit GNSS datum and systematic prior; no simulator, PX4 or camera producer starts here. |
| `uav_control uav_pose.launch.py` | Independent anchor producer with explicit topic, map origin, clock, airborne and settle settings. |
| `uav_sensing uav_camera_sensor.launch.py` | Camera bridge and optional detector (`detect_ugv`). |
| `simulation_support ugv_sensors.launch.py` | Raw Gazebo sensors and shared clock. |
| `fault_management sim_ugv_sensors.launch.py` | Explicit simulated sensor corruption; nominal/drift profiles and seeded uncertainty. |
| `fault_management ugv_evaluation.launch.py` | Baseline evaluation assembly. |
| `fault_management ugv_cooperative_evaluation.launch.py` | Paired baseline/assisted filters receiving identical UGV messages inside each trial. |
| `/evaluation/set_gps_enabled` | Evaluation-only `std_srvs/SetBool` gate on simulated measured GPS. Raw simulated GPS continues. |
| `/ugv/localization/set_uav_enabled` | Estimator selection of UAV corrections; observation/anchor producers remain independent. |

Camera evaluation defaults select `simple_goal_demo`, `cooperative_camera_uav_0`, actual PX4 odometry, no synthetic anchor publisher, and live initialization. Synthetic defaults select the fixed observer in `uav_ugv`. Incompatible camera/synthetic-anchor combinations fail before launch. `GZ_PARTITION` and `ROS_DOMAIN_ID` propagate through the assembly; avoid unrelated publishers in the same partition/domain.

## Evaluation and reproducibility

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
colcon test --executor sequential
colcon test-result --verbose
source install/setup.bash
cd ..

# Nine nominal paired trials: three seeds, three GPS cases.
scripts/run_cooperative_trials.sh /tmp/cooperative_nominal

# Same matrix with long wheel/heading drift.
ROUTE=long DURATION=91 MEASUREMENT_PROFILE=drift \
  scripts/run_cooperative_trials.sh /tmp/cooperative_drift

# Three long GPS-loss trials with outliers and a 20 s relative-sensor outage.
GPS_CASES=off ROUTE=long DURATION=91 MEASUREMENT_PROFILE=drift \
  OUTLIER_EVERY=10 DROPOUT_START=35.0 DROPOUT_END=55.0 \
  scripts/run_cooperative_trials.sh /tmp/cooperative_robust

# Acquisition-stamped rendered observations and actual PX4 anchors.
scripts/run_camera_perception_trial.sh /tmp/camera_visibility
scripts/run_camera_anchor_trials.sh /tmp/camera_anchors
scripts/run_goal_navigation_trial.sh /tmp/camera_goal

# Actual planner/controller and actuator-loss checks.
ROS_DOMAIN_ID=55 python3 scripts/test_navigation_guards.py /tmp/navigation_guards.json
ROS_DOMAIN_ID=56 python3 scripts/test_actuator_watchdog.py /tmp/watchdog

# Physical closed-loop tracking under the same configured seeds/map/goal.
ROS_DOMAIN_ID=57 python3 scripts/run_path_tracking_trials.py /tmp/path_tracking
```

`off` disables measured GPS at route time 10 s; `startup_off` starts disabled. The drift profile injects 15% forward-speed error and 0.002 rad/s growing heading bias. Relative outliers add 8 m every tenth sample; relative dropout uses `/evaluation/trial_start`. Noise/fault values are controlled assumptions, not hardware specifications.

The localization scorer interpolates truth at acquisition times and rejects truth gaps over 0.1 s. It records per-sample position/heading/covariance, planar NEES, coverage, sensor counts and conditions. Repeated samples are temporally correlated. Heading-log scoring also rejects out-of-range estimate times, nonmonotonic logs and gaps over 0.1 s.

Closed-loop scoring records route snapshots and truth matched to the most recent planned route at or before acquisition time (maximum reference age 1 s). CSVs preserve physical point-to-polyline cross-track error; summaries include RMSE, mean, maximum, p95, traveled distance and reduction in physical distance to the fixed goal. Separate baseline/assisted closed-loop worlds use the same configured seeds/map/goal, but their trajectories and sensor sequences differ. The free-grid fixture isolates tracking from camera-map accuracy.

Arrival requires a full fresh one-second stopped interval, estimated distance at most 0.5 m, and the declared physical tolerance (0.85 m for the camera demo). A deadline during dwell cannot pass. Hold checks require fresh sensors and sufficient evidence. Localization and camera evaluation runners bound process-group cleanup and stop their owned processes. Use persistent directories for retained results; `/tmp` may clear when WSL restarts.

[Heading validation](heading_validation.md), [navigation](../navigation.md), [perception](perception.md) and [simulation setup](../simulation_setup.md) state measured conditions and remaining limits.
