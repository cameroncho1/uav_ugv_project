# Cooperative localization subsystem

[Documentation index](../README.md) | [Localization guide](README.md) | [Interfaces](interfaces.md)

## Inputs and output

`cooperative_localization` estimates the UGV's map position and heading. Inputs are wheel odometry, IMU, GPS, and optional UAV-derived position observations. Measurements carry acquisition timestamps, coordinate frames, and covariance.

The output is `/ugv/localization/pose`, a `geometry_msgs/msg/PoseWithCovarianceStamped` in ENU `map`. It contains position, orientation, acquisition time, and covariance. Planning and control consume this authoritative pose.

The UAV retains an independent global position reference while UGV GPS is degraded. Loss of both vehicles' global references is outside the implemented localization model.

## Estimation

A planar `robot_localization` extended Kalman filter combines:

- Wheel forward velocity and a zero lateral-velocity constraint for the differential-drive model.
- IMU yaw and turn rate.
- GPS map x/y when valid and available.
- UAV-derived map x/y when assistance is enabled; optional camera-derived yaw is fused jointly with x/y.

Integrated wheel pose and placeholder GPS or position-only UAV orientation are excluded from fusion. The filter starts with large x/y uncertainty until an absolute position anchors it. Camera observations provide measured orientation with joint covariance when heading is enabled.

The measurement adapter validates frames, component availability, timestamps, and covariance. Known GPS covariance is preserved and rotated into map coordinates. Unknown covariance uses configured fallback values. Missing IMU orientation remains unavailable. GPS and UAV position updates have five-sigma innovation gates.

## UAV correction

The correction combines an independently localized UAV pose with a measured 3D vector from UAV body origin to UGV chassis origin. Anchors are matched at observation acquisition time. UAV position, UAV attitude, and relative-measurement uncertainty enter the correction covariance.

The camera producer detects a roof AprilTag in rendered pixels and publishes relative chassis position/orientation with joint covariance. The estimator consumes the relative-position interface with optional orientation. A different producer can supply position only with large unused orientation covariance. A scalar range alone requires a different observation model or additional geometry. [Camera heading](camera_heading.md) defines the image direction indicator, joint x/y/yaw updates, uncertainty, and fallback behavior.

## GPS loss and uncertainty

Wheel and IMU prediction continues when GPS stops. UAV observations can supply position corrections during that outage. Position uncertainty and error depend on the remaining measurements and their error models.

The public pose adapter adds a configured systematic x/y uncertainty prior. The operational default is zero. Evaluation may set the prior to a known injected shared-reference error. This setting does not detect or remove unknown common global bias.

Navigation requires a fresh pose within configured uncertainty limits. A publishing estimate does not by itself authorize motion.

## Package boundaries

| Package | Implemented responsibility |
| --- | --- |
| `cooperative_localization` | Measured-input validation, GPS adaptation, acquisition-time matching, filter configuration, and authoritative pose. |
| `localization_common` | Frame, covariance, and geodesy calculations without ROS or simulator imports. |
| `uav_control` | PX4 subscriptions, NED/FRD conversion, reference initialization/reset policy, and independent ENU map anchors. |
| `uav_sensing` | Camera calibration, tag detection, sensor transform, and body-relative observations. |
| `simulation_support` | Raw Gazebo bridges, shared clock, world/model geometry, and truth readers. |
| `fault_management` | Simulated corruption, outages, synthetic observations, trial assembly/driving, and scoring. |
| `traversability_mapping` | Camera-to-grid projection using acquisition-matched independent UAV anchors. |
| `route_planning` | Routes associated with goal revisions and robot footprint clearance. |
| `ugv_control` | Path following and simulated actuator command expiry. |

The operational estimator starts no simulator, PX4 adapter, camera detector, synthetic sensor, or test driver. The demo and evaluation launches assemble those components explicitly. Simulator truth is confined to synthesis and scoring. The fused UGV pose is not fed into its relative-observation producer.

## Evaluation

Paired trials compare baseline and assisted filters on identical UGV messages. Conditions include nominal GPS, GPS dropout, GPS denied from startup, wheel/heading drift, relative outliers, and observation outages. Scores include position/heading error, reported uncertainty, normalized error, and ellipse coverage.

Rendered trials test tag visibility, occlusion, recovery, and off-axis geometry. Closed-loop trials measure physical route cross-track error and goal arrival. Command-loss trials measure stopping after controller or bridge loss.

[Interfaces](interfaces.md#evaluation-and-reproducibility) lists commands and output files. [Heading validation](heading_validation.md) records executed checks, measured conditions, and limits.

## Limits

Validation uses simulated flat ground, known vehicle/tag geometry, and configured sensor errors. Hardware calibration, physical wheel slip, uneven terrain, unknown shared global bias, and semantic obstacle detection are not established by these trials. Global map-to-body TF ownership and production fault classification remain open.
