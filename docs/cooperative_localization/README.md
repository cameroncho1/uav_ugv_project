# Cooperative UGV localization

The system estimates the ground robot's position and heading, with pose uncertainty. Wheel speed and IMU heading track motion. GPS and UAV-derived measurements provide absolute map position. Calibrated camera observations also supply optional heading corrections and an image direction indicator.

The output is `/ugv/localization/pose`, a `geometry_msgs/msg/PoseWithCovarianceStamped` in ENU `map`. Covariance describes the modeled uncertainty. An unknown error shared by GPS and the UAV's global reference cannot be recovered from relative measurements alone.

## Run the demo

Complete [Ubuntu setup](../../README.md) and [simulation dependency setup](../simulation_setup.md) first. From the repository root inside Ubuntu:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
cd ..
ros2 launch mission_manager simple_goal_demo.launch.py initial_gps_enabled:=false
```

This starts the simulated world, PX4, DDS, raw sensors, simulated measurement noise, camera detector, UAV anchor, estimator, mapper, planner, controller, and RViz. The UAV hovers while the UGV navigates to `(4, 0)`. The world has flat ground and no physical obstacles. Use Ctrl+C in the launch terminal to stop the demo.

Use `uav_assistance:=false` for baseline-only fusion. Use `uav_heading:=false` for camera position without camera heading. RViz's **Camera heading** display shows `/uav/camera/heading_image`. Use `start_rviz:=false` to omit RViz. Camera rendering still needs a valid display. [Goal-demo instructions](../../README_simple_goal_demo.md) list the separate-terminal commands.

## Data flow

1. `simulation_support` bridges raw Gazebo wheel, IMU, GPS, and clock data.
2. `fault_management` adds the selected simulation error model and publishes `/ugv/sensors/*`.
3. `cooperative_localization` validates those measurements and converts GPS into map coordinates.
4. `uav_control` publishes the UAV's independent map position and attitude.
5. `uav_sensing` detects the roof tag and publishes the UGV pose relative to the UAV, joint covariance, and a measured forward-direction image overlay.
6. `cooperative_localization` matches the relative reading to the UAV anchor at acquisition time and propagates both sources' uncertainty.
7. The `robot_localization` extended Kalman filter combines wheel velocity, IMU yaw/rate, GPS x/y, and optional UAV-derived x/y or joint x/y/yaw. Each camera observation supplies one update.
8. The output adapter publishes pose, acquisition time, and covariance. Planning and control consume that output.

Simulator truth feeds synthetic sensors and scoring. The estimator does not use truth as its operational pose.

## Code locations

All paths below are under `ros2_ws/src/`.

| Responsibility | Package |
| --- | --- |
| Measured-input validation, correction matching, filter configuration, fused output | `cooperative_localization` |
| PX4 map anchor | `uav_control` |
| Camera detector, calibration, relative orientation and image overlay | `uav_sensing` |
| Raw bridges, shared clock, truth readers | `simulation_support` |
| Sensor noise, faults, outages, trial assembly, scoring | `fault_management` |
| Shared frame, covariance, and geodesy math | `localization_common` |
| Terrain mapping | `traversability_mapping` |
| Planning and driving | `route_planning`, `ugv_control` |

`localization_common` contains numerical functions without ROS or Gazebo imports. `cooperative_localization` contains ROS nodes and estimator configuration. The nodes call the numerical functions.

## Run the estimator separately

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch cooperative_localization ugv_localization.launch.py
```

This starts the adapter, filter, pose output, and chassis-to-IMU static transform. It starts no simulator, measurement producer, PX4 adapter, or detector. It defaults to wall time and baseline-only fusion. Supply measured sensor topics before expecting a pose.

For simulation, set `use_sim_time:=true` and supply `/clock`. Set the GNSS datum with `origin_latitude`, `origin_longitude`, and `origin_altitude`. Match the expected sensor frames or remap/configure the producer. Assistance requires `uav_assistance:=true` plus independent publishers for `/uav/localization/pose` and `/ugv/sensors/uav_relative_pose`.

## Extend a component

| Change | Interface to preserve |
| --- | --- |
| Replace the tag detector | Publish the UGV chassis origin relative to the UAV body in FLU on `/ugv/sensors/uav_relative_pose`, with acquisition time and covariance. |
| Replace PX4 localization | Publish the independent UAV pose and covariance in ENU `map` on `/uav/localization/pose`. |
| Add hardware UGV sensors | Publish measured `/ugv/sensors/{gps,imu,wheel_odom}` with valid frames, timestamps, covariance, and unavailable-component markers. Omit simulation noise producers. |
| Change estimator math | Keep `/ugv/localization/pose`, map coordinates, acquisition time, and covariance. Rerun paired localization trials. |
| Change mapping or planning | Keep the [navigation contracts](../navigation.md), including map acquisition time, goal revisions, clearance, and command expiry. |

The relative interface requires a 3D position vector. A single range value needs a different observation model or additional geometry.

## Verify a change

From the repository root inside Ubuntu:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
colcon test --executor sequential
colcon test-result --verbose
source install/setup.bash
cd ..
scripts/run_cooperative_trials.sh "$HOME/uav_ugv_results/paired_nominal"
```

[Interfaces](interfaces.md#evaluation-and-reproducibility) lists drift, outlier, outage, camera, tracking, and command-loss checks.

## Design and interfaces

| Document | Contents |
| --- | --- |
| [Subsystem](subsystem.md) | Estimator behavior and ownership. |
| [Interfaces](interfaces.md) | Topics, frames, clocks, controls, launch modes, and evaluation commands. |
| [UAV measurement design](uav_measurement_design.md) | Coordinate conversion, acquisition-time matching, propagated covariance, and rejection rules. |
| [Camera observations](perception.md) | Tag geometry, calibration assumptions, visibility, and shared-error tests. |
| [Camera heading](camera_heading.md) | Direction overlay, relative orientation, joint heading fusion, controls, and uncertainty. |
| [Navigation integration](../navigation.md) | Pose consumers, goal revisions, clearance, command expiry, and tracking metrics. |
| [Research references](references.md) | Source summaries and limits on their application to this simulation. |

## Validation

| Document | Scope |
| --- | --- |
| [Heading validation](heading_validation.md) | Executed checks, camera/filter measurements, paired position-only trials, and limits. |
| [Heading artifacts](results/heading/) | Machine-readable summaries and an annotated camera frame. |

## Presentations

| Presentation | Contents |
| --- | --- |
| [How localization works](how_it_works_presentation.html) | Localization pipeline and navigation guards. |
| [Localization overview](initial_presentation.html) | Scope, inputs, output, and integration. |
| [Research papers](research_papers_presentation.html) | Literature summaries and source links. |

[Project documentation](../README.md) covers shared setup and project-wide design.
