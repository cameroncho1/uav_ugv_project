# Cooperative UGV Localization

## Purpose and Current State

This subsystem supplies a fused UGV position from wheel motion, IMU heading, and GPS. Its input layer bridges the simulated sensors into ROS 2. A measurement adapter projects GPS into the map frame, anchors wheel odometry to the UGV spawn pose, and publishes noisy readings with configured covariance. A planar EKF combines them and publishes `/ugv/localization/pose`. Both Gazebo worlds contain the sensors. [Interfaces](interfaces.md) lists the live topics, frames, error settings, and commands to check them.

The fused pose is a GPS, wheel, and IMU baseline. GPS outage control, a UAV-to-UGV measurement, and localization accuracy results are not implemented. The goal demo still drives and plans from its existing `/ugv_pose` topic, which the UGV controller makes from Gazebo odometry.

## Sensor Boundary

`cooperative_localization` owns the ROS-facing raw sensor topic names and their Gazebo bridge configuration:

- `/ugv/sensors/wheel_odom`: `nav_msgs/msg/Odometry`, 20 Hz in Gazebo.
- `/ugv/sensors/imu`: `sensor_msgs/msg/Imu`, 50 Hz in Gazebo.
- `/ugv/sensors/gps`: `sensor_msgs/msg/NavSatFix`, 5 Hz in Gazebo.
- `/clock`: Gazebo simulation time for consumers that enable `use_sim_time`.

These are raw simulation measurements. Wheel odometry is relative to the UGV spawn pose; GPS is geodetic; IMU is in its chassis sensor frame. Their raw covariance fields do not provide a useful uncertainty model. `ugv_measurements.py` publishes prepared wheel odometry and GPS position in ENU `map`, and an IMU reading in its original sensor frame. It adds configurable independent Gaussian noise and corresponding covariance; the raw inputs remain unchanged. The values are test assumptions, not hardware calibration.

## Fused Pose and Boundaries

The implemented baseline publishes one UGV estimate on `/ugv/localization/pose`, type `geometry_msgs/msg/PoseWithCovarianceStamped`, in the ENU `map` frame. `robot_localization` runs the 2D EKF. It predicts from wheel forward speed, tracks heading with IMU yaw and turn rate, and corrects map x/y with GPS. The output adapter copies the EKF pose and covariance into the agreed message type. An identity TF relates the chassis and the IMU mounted at its origin.

A UAV-assisted correction needs a measured UGV-to-UAV relative position and a UAV pose converted from PX4 NED to map ENU. PX4 UAV odometry by itself is not a UGV correction.

The path planner and UGV controller can consume the fused topic in place of `/ugv_pose` after accuracy and GPS-loss tests. Terrain mapping, obstacle detection, route planning, and flight control stay in their existing packages.

## GPS Degradation and Evaluation

The target behavior during UGV GPS loss is a continuously published pose based on remaining measurements, with uncertainty that grows when the estimate drifts. GPS loss injection and this behavior's validation are not implemented yet.

The evaluation compares nominal GPS, GPS loss without UAV help, and GPS loss with UAV help. It records position error, heading error, uncertainty versus observed error, and UGV path-following error. Gazebo's true state is a scoring reference, never the operational localization output. Claims apply only to the simulated conditions tested.

The initial scope assumes the UAV retains reliable localization while UGV GPS is degraded. It does not cover loss of both vehicles' global reference or full SLAM.
