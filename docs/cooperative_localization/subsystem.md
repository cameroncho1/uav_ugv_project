# Cooperative UGV Localization

## Purpose and Current State

This subsystem is intended to supply the UGV's position to the rest of the project, including when UGV GPS is degraded. Its implemented input layer bridges wheel odometry, IMU, GPS, and simulation time into ROS 2. A measurement adapter projects GPS into the map frame, anchors wheel odometry to the UGV spawn pose, and publishes noisy readings with configured covariance. Both Gazebo worlds contain the sensors. [Interfaces](interfaces.md) lists the live topics, frames, error settings, and commands to check them.

There is no fused pose estimator, GPS outage control, UAV-to-UGV measurement, or localization accuracy result in the repository. The goal demo drives and plans from its existing `/ugv_pose` topic, which the UGV controller makes from Gazebo odometry.

## Sensor Boundary

`cooperative_localization` owns the ROS-facing raw sensor topic names and their Gazebo bridge configuration:

- `/ugv/sensors/wheel_odom`: `nav_msgs/msg/Odometry`, 20 Hz in Gazebo.
- `/ugv/sensors/imu`: `sensor_msgs/msg/Imu`, 50 Hz in Gazebo.
- `/ugv/sensors/gps`: `sensor_msgs/msg/NavSatFix`, 5 Hz in Gazebo.
- `/clock`: Gazebo simulation time for consumers that enable `use_sim_time`.

These are raw simulation measurements. Wheel odometry is relative to the UGV spawn pose; GPS is geodetic; IMU is in its chassis sensor frame. Their raw covariance fields do not provide a useful uncertainty model. `ugv_measurements.py` publishes prepared wheel odometry and GPS position in ENU `map`, and an IMU reading in its original sensor frame. It adds configurable independent Gaussian noise and corresponding covariance; the raw inputs remain unchanged. The values are test assumptions, not hardware calibration.

## Fused Pose Contract

The integration target is one authoritative UGV estimate on `/ugv/localization/pose`, type `geometry_msgs/msg/PoseWithCovarianceStamped`, in the ENU `map` frame. That topic has no publisher yet. Its estimator must account for sensor frames, timestamps, and measurement uncertainty. The measurement adapter handles the configured GPS origin and wheel-odometry spawn offset.

An EKF through `robot_localization` is the starting approach. The prepared GPS input is already in the local map frame. A UAV-assisted correction also needs a measured UGV-to-UAV relative position and a UAV pose converted from PX4 NED to map ENU. PX4 UAV odometry by itself is not a UGV correction.

Once the fused topic is validated, the path planner and UGV controller can consume it in place of `/ugv_pose`. Terrain mapping, obstacle detection, route planning, and flight control stay in their existing packages.

## GPS Degradation and Evaluation

The target behavior during UGV GPS loss is a continuously published pose based on remaining measurements, with uncertainty that grows when the estimate drifts. No GPS loss behavior is implemented yet.

The evaluation compares nominal GPS, GPS loss without UAV help, and GPS loss with UAV help. It records position error, heading error, uncertainty versus observed error, and UGV path-following error. Gazebo's true state is a scoring reference, never the operational localization output. Claims apply only to the simulated conditions tested.

The initial scope assumes the UAV retains reliable localization while UGV GPS is degraded. It does not cover loss of both vehicles' global reference or full SLAM.
