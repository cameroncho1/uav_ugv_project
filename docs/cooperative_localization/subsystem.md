# Cooperative Localization Subsystem Specification

## Purpose

The cooperative localization subsystem estimates the UGV's current pose. Its purpose is to maintain a useful pose estimate when UGV GNSS measurements are degraded or unavailable.

The subsystem supports resilient UGV navigation in degraded-GNSS defense and search-and-rescue scenarios. Its scope does not include complete GNSS denial affecting both the UAV and UGV. The initial operating assumption is that the UAV retains reliable localization while UGV GNSS is degraded.

## Subsystem Boundary

`cooperative_localization` owns:

- Combining localization-related measurements.
- Maintaining the UGV pose estimate.
- Handling degraded or unavailable UGV GNSS.
- Incorporating UAV-assisted corrections.
- Publishing the authoritative pose and its covariance.

It does not own:

- Terrain classification or traversability assessment.
- Obstacle detection.
- Path generation.
- UAV path planning or flight control.
- Mission coordination.
- Full simultaneous localization and mapping (SLAM).

## Expected Inputs

The estimator is expected to accept these future raw inputs:

- UGV GNSS measurements.
- UGV wheel odometry.
- UGV IMU measurements.
- A UAV-assisted relative-position measurement.

Raw topic names are not yet established. Each measurement used by the estimator requires a timestamp, an identified coordinate frame, and an uncertainty model.

## UAV-Assisted Measurement

For the initial system, UAV assistance is a noisy relative-position observation of the UGV with respect to the UAV. The UAV is assumed to retain reliable localization while UGV GNSS is degraded.

The simulation and estimator interface remain independent of the physical sensing technology. UWB, AprilTags, and camera tracking are possible real-world implementations, not architectural commitments.

## Estimation Approach

A standard state estimator, such as an extended Kalman filter, is the expected starting point. Existing ROS 2 estimation tooling is preferred over implementing a filter from scratch when it satisfies the required state, measurement, frame, and failure-handling behavior.

The estimator must tolerate missing, degraded, stale, or rejected measurements without substituting Gazebo ground truth for operational sensor data. Measurement validity, timing, and uncertainty must be represented consistently enough for the estimator to distinguish nominal data from degraded data.

## GNSS Degradation Behavior

When UGV GNSS becomes degraded or unavailable:

- The authoritative pose output remains continuous.
- Odometry and IMU measurements continue contributing to the estimate.
- Pose uncertainty may increase as uncorrected drift accumulates.
- UAV-assisted relative-position observations may reduce drift and uncertainty.

A UAV correction improves the estimate according to its modeled uncertainty; it does not make localization exact or eliminate all accumulated error.

## Authoritative Output

The subsystem will publish one authoritative UGV localization estimate using the preferred interface:

- Topic: `/ugv/localization/pose`
- Message type: `geometry_msgs/msg/PoseWithCovarianceStamped`
- Frame: `map`
- Coordinate convention: ENU

This interface is the project default unless a future integration issue requires a documented change. It is a design contract and is not yet implemented in the repository.

The message contains:

- Position.
- Orientation.
- Timestamp.
- Covariance, representing the estimator's uncertainty in the pose.

The path planner and other downstream navigation components consume this single authoritative pose. They do not subscribe directly to GNSS, wheel odometry, IMU, or UAV-assisted correction topics as independent pose sources.

## Ground-Truth Separation

Three data categories must remain distinct:

- **Gazebo ground truth:** The simulator's actual UGV state.
- **Simulated sensor measurements:** Noisy, delayed, degraded, or unavailable observations derived from the simulation.
- **Operational localization estimate:** The estimator output published on `/ugv/localization/pose`.

Ground truth may be used to:

- Generate simulated measurements.
- Debug the simulation and estimator.
- Evaluate estimator error.

Ground truth must not be forwarded directly as the operational localization estimate consumed by the path planner. When ground truth contributes to a simulated input, it must pass through an explicit sensor or measurement model.

## Evaluation

Evaluation should compare at least these operating cases:

- Nominal UGV GNSS.
- UGV GNSS loss without UAV assistance.
- UGV GNSS loss with UAV assistance.

Relevant metrics include:

- Position root-mean-square error.
- Mean position error.
- Maximum position error.
- Heading error.
- Reported localization uncertainty and its consistency with observed error.
- Downstream UGV path-following error.

Results should identify the tested GNSS degradation, measurement noise, outage duration, estimator configuration, and ground-truth reference. Claims should remain limited to the simulated conditions evaluated.

## Scope Limits

The initial subsystem assumes a reliably localized UAV can provide useful relative-position information to a UGV experiencing degraded GNSS. It does not establish localization when both vehicles lack a reliable global reference, and it does not provide mapping, perception, planning, control, mission management, or full SLAM.
