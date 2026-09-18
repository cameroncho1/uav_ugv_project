# Cooperative Localization Interface Specification

## Status

This document defines the preferred project interfaces and ownership boundaries for cooperative UGV localization. These contracts guide future integration but are not yet implemented in the repository. Items listed under Open Decisions remain intentionally unresolved.

## Confirmed Repository Baseline

- The project targets ROS 2 Humble and Gazebo Harmonic using `gz-sim8`.
- The Gazebo world uses ENU coordinates.
- PX4 UAV vehicle odometry uses local NED coordinates.
- The UGV is an inline Gazebo SDF model named `simple_ugv` with a differential-drive plugin.
- UGV differential-drive odometry is configured for 20 Hz, but its output topic is not explicitly configured or bridged into ROS 2.
- The UGV has no configured IMU or GNSS/NavSat sensor.
- No TF tree currently exists.
- `cooperative_localization`, `route_planning`, `traversability_mapping`, and `ugv_control` are package scaffolds without subsystem nodes.
- UAV odometry is ROS-visible at `/fmu/out/vehicle_odometry` as `px4_msgs/msg/VehicleOdometry`.
- No UAV-to-UGV localization measurement interface currently exists.

## System Invariants

- There is one authoritative UGV pose output.
- Downstream navigation components use the shared ENU `map` frame.
- Ground truth is never used directly as the operational localization output.
- The path planner does not depend directly on raw localization sensors.
- Localization continues publishing during UGV GNSS degradation or loss.
- UAV assistance is represented as a noisy relative-position observation.
- Initial scope assumes UAV localization remains available.

## Authoritative Localization Output

`cooperative_localization` will publish one authoritative UGV pose estimate through the preferred interface:

- Topic: `/ugv/localization/pose`
- Message type: `geometry_msgs/msg/PoseWithCovarianceStamped`
- Frame: `map`
- Coordinate convention: ENU

This interface is the project default unless a future integration issue requires a documented change. The message contains position, orientation, timestamp, and covariance. Covariance represents the estimator's uncertainty in the pose.

Downstream navigation components consume this output instead of treating GNSS, wheel odometry, IMU, or UAV-assisted observations as independent pose sources.

## Raw Localization Inputs

The estimator is expected to consume:

- UGV odometry.
- UGV IMU.
- UGV GNSS.
- A UAV-assisted relative-position measurement.

Raw topic names are not finalized. Candidate ROS representations are:

| Input | Candidate representation | Current repository state |
|---|---|---|
| UGV odometry | `nav_msgs/msg/Odometry` | Gazebo DiffDrive odometry is configured but not bridged into ROS 2. |
| UGV IMU | `sensor_msgs/msg/Imu` | No simulated sensor or ROS topic exists. |
| UGV GNSS | `sensor_msgs/msg/NavSatFix` | No simulated sensor or ROS topic exists. |
| UAV-assisted relative position | Stamped relative-position measurement with uncertainty and declared frame | No message type, topic, or producer exists. |

The UAV-assisted input is a noisy relative-position observation of the UGV with respect to the UAV. It does not assume a physical sensing technology. UWB, AprilTags, and camera tracking are possible real-world implementations only.

## Subsystem Ownership

### `cooperative_localization`

Owns:

- Combining localization-related measurements.
- Maintaining the UGV pose estimate.
- Handling degraded or unavailable UGV GNSS.
- Incorporating UAV-assisted corrections.
- Publishing pose and covariance on the authoritative output.

Does not own terrain classification, obstacle detection, path generation, UAV path planning, mission coordination, or full SLAM.

### `ugv_control`

- Owns the ROS-facing UGV motion-control boundary.
- Is the likely source or exposure point for UGV wheel odometry.
- May expose UGV IMU and GNSS measurements if sensor bridging is assigned to this package.
- Keeps raw UGV measurements separate from the fused localization output.

UGV sensor-bridge ownership remains open.

### `uav_control`

- Owns the PX4/UAV ROS interface.
- Exposes UAV state needed to interpret a relative-position observation.
- Currently receives UAV state from `/fmu/out/vehicle_odometry`.

PX4 state is NED and must be converted before use in project-level localization logic. UAV state alone is not a UGV correction; the estimator requires a separate relative-position observation.

### `route_planning`

The planner receives:

- The authoritative UGV pose from `/ugv/localization/pose`.
- Traversability or map information.
- A goal or mission target.

All planner inputs that describe positions, obstacles, or goals use the ENU `map` frame. The planner does not depend on estimator implementation details or subscribe directly to raw localization sensors.

### `traversability_mapping`

Localization and traversability mapping are separate subsystems:

- Localization answers: "Where is the UGV?"
- Traversability mapping answers: "Where can the UGV safely drive?"

Their shared dependency is the ENU `map` frame. UGV pose, traversability data, obstacles, and goals must refer to the same coordinate system so the planner can combine them consistently.

### `fault_management`

Fault management may later:

- Inject UGV GNSS dropout or noise for testing.
- Report sensor faults.
- Consume localization-health information.

No final fault-management API is defined.

## Coordinate-Frame Contract

The project-facing navigation frame is `map` using ENU coordinates.

- Localization output is expressed in `map`.
- Traversability mapping uses `map` for project-facing map data.
- Path planning uses `map` for pose, obstacles, paths, and goals.
- Every localization input identifies its source frame and timestamp.
- Frame conversion is explicit; coordinate conventions are not inferred from topic names.

The Gazebo world already uses ENU, which is consistent with this project-facing convention. PX4 UAV state uses NED and therefore requires conversion before it contributes to localization in `map`. This contract does not define a complete TF tree or assign TF publication ownership.

## GNSS Degradation Contract

When UGV GNSS is degraded or unavailable:

- `/ugv/localization/pose` continues publishing without a deliberate discontinuity.
- Odometry and IMU continue contributing to the estimate.
- Reported covariance may increase as drift accumulates.
- UAV-assisted relative-position observations may reduce drift and covariance.

UAV assistance does not make the estimate exact. Its effect depends on measurement uncertainty, timing, geometry, and estimator behavior.

## Ground-Truth Separation

The system distinguishes:

- Gazebo ground truth: the simulator's actual state.
- Simulated sensor measurements: observations derived from simulation with explicit noise, timing, degradation, or outage behavior.
- Operational localization estimate: the fused pose published on `/ugv/localization/pose`.

Ground truth may generate simulated measurements, support debugging, and provide an evaluation reference. It must not be published directly as the operational pose consumed by the planner.

## Open Decisions

- Full TF tree.
- TF publication ownership.
- Exact raw sensor topic names.
- Raw sensor QoS.
- Exact sensor update rates.
- Exact sensor noise values.
- EKF or UKF tuning.
- Whether UAV relative-position measurements are 2D or 3D.
- Gazebo-to-ROS bridge ownership.
- Final fault-management API.
