# UAV-assisted pose measurements

## Implemented model

The sensor boundary uses a full 3D UGV position relative to the UAV in `uav/base_link` (forward, left, up). `PoseWithCovarianceStamped` carries the relative vector, covariance, and acquisition time. The camera detector also supplies measured chassis orientation and joint fixed-axis covariance using known marker geometry, CameraInfo intrinsics, and body-camera extrinsics. Position-only producers leave orientation unused with large angular covariance. [Camera heading](camera_heading.md) defines orientation composition, joint x/y/yaw covariance, and heading-quality fallback; [perception](perception.md) describes camera calibration assumptions. A range-only UWB reading does not supply the relative vector; it needs a different observation model or additional geometry.

The UAV anchor is independently localized in ENU `map`. `uav_control/scripts/uav_pose.py` converts PX4 NED position, FRD attitude, and their variances. The configured `map_origin` translates PX4's local origin into the shared map. This origin is an external alignment assumption, not an estimate obtained from UGV truth. The goal demo and live trial runner require airborne status and a reset-free ten-second initialization interval before accepting the reference. The standalone anchor launch defaults to no airborne requirement and zero settling delay. Later PX4 resets stop output and require realignment and restart. Independent noisy simulated anchors remain available for controlled observation tests.

The position correction computes `p_ugv = p_uav + R_uav * relative`. With independent anchor and relative errors, its covariance is `J * C_uav * J.T + R * C_relative * R.T`, where `J = [I, -skew(R * relative)]`. Anchor attitude errors are about fixed map axes. The planar EKF selects map x/y, with optional camera-derived yaw fused jointly when enabled and usable. Full position/orientation composition retains source cross-covariance. The Jacobians are small-angle approximations.

`simulation_support` owns the Gazebo pose reader; `fault_management` owns the synthetic relative-observation producer. Evaluation assemblies select these explicitly. At 5 Hz the synthetic producer generates independent Gaussian relative position, UAV position, and UAV attitude errors. The fixed-observer evaluation places the UAV at map `(2, -2, 4)`; it models an independently localized hovering observer without running flight dynamics. The same relative sensor can read a named PX4 UAV model in a live world. True poses supply synthetic sensor data and scoring; the estimator receives only the noisy measurements. The camera demo selects rendered observations and an actual PX4 anchor instead of this synthetic relative sensor.

## Design choices checked by evaluation

| Choice | Evidence and implemented behavior |
| --- | --- |
| Full relative vector and 3D rotation | Range alone cannot determine one UGV position from one UAV. Rotation round trips and lever-arm covariance tests cover nonzero roll, pitch, and yaw. The planar EKF selects x/y or joint x/y/yaw. |
| UAV uncertainty in every correction | Monte Carlo checks exercise position propagation and joint position/orientation covariance, including correlated errors. Attitude error grows into position error with distance. |
| Acquisition-time matching | Delayed readings wait for a bracketed anchor, expire without extrapolation, and retain their original timestamp. Clock-mismatch and reset tests reject invalid anchors. |
| Independent streaming EKFs | Paired filters receive identical UGV messages; neither estimate feeds the observation producer. Heading checks isolate the effect of selecting camera yaw. |
| Jump gate plus EKF innovation gate | Synthetic trials inject 8 m position outliers and a 20-second outage. The heading fixture injects 170-degree orientation outliers and a two-second observation outage. |
| Aligned PX4 anchor as the live default | The dependency checker verifies pinned wire definitions, firmware, and ENU/tesla magnetic settings before live evaluation. |

[Heading validation](heading_validation.md) gives the executed conditions and measured results.

## Timing and rejection

The PX4 adapter preserves `timestamp_sample`; ROS and PX4 must share a clock. In Gazebo, ROS nodes use `/clock` and PX4 uses `UXRCE_DDS_SYNCT=0`. Samples whose clock differs from ROS are rejected. Receipt time is not substituted for acquisition time.

The correction buffers UAV anchors and interpolates position and quaternion at the relative reading's timestamp. It requires an exact sample or a bracket no wider than 0.25 seconds. Inputs expire at 0.5 seconds; extrapolation is disabled. Wrong frames, invalid covariance, nonfinite data, duplicates, and ranges over 30 m are rejected. A motion-and-uncertainty bound rejects impossible jumps between accepted corrections. The EKF also applies a five-sigma innovation threshold. `/diagnostics` reports adapter counters and `/ugv/localization/set_uav_enabled` controls corrections.

## Research basis and limits

- [PX4 ROS 2 guide](https://docs.px4.io/main/en/ros2/user_guide#ros-2-px4-frame-conventions): NED/FRD versus ENU/FLU conversion and shared Gazebo time. The pinned [VehicleOdometry definition](https://github.com/PX4/px4_msgs/blob/109e8c8e151938f03fee2e62e3e93dc5ff5e2281/msg/VehicleOdometry.msg) defines sample timestamps, local origins, body attitude variance, and reset counters.
- [REP 103](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0103.rst): right-handed ROS frames, SI units, and fixed-axis orientation covariance.
- [robot_localization Humble configuration](https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/humble-devel/doc/state_estimation_nodes.rst): absolute pose inputs, selected state components, and Mahalanobis rejection thresholds.
- [Pryor thesis](https://etd.auburn.edu/handle/10415/7871): cooperative navigation depends on relative measurement information and an absolute reference. The full-vector model here supplies more information than one scalar range.
- [Cooperative localization with adaptive UWB weighting](https://www.mdpi.com/2504-446X/10/3/175): measurement quality and outlier handling matter. Its range-based factor graph is a different estimator; its accuracy numbers are not predictions for this simulation.

Noise values are controlled simulation assumptions, not sensor calibration. UAV uncertainty is included so a precise relative reading cannot create an unrealistically perfect global anchor. The filters do not exchange their estimates, avoiding estimator feedback. The independent Gaussian fixture omits persistent shared errors. Separate [perception and shared-bias tests](perception.md) exercise actual pixels, occlusion, and a configured common global offset. Covariance propagation assumes independent anchor and relative errors; the public fused-pose interface retains a configured systematic position prior for shared absolute-reference uncertainty.

The paired evaluation uses two EKFs on the same sensor messages and trajectory. It measures position error and reported planar uncertainty under nominal noise, injected wheel-scale and IMU-heading drift, relative-measurement outliers, and correction dropout. [Heading validation](heading_validation.md) records results and conditions.

## Live PX4 compatibility

The dependency manifest pins PX4, matching ROS messages, and Gazebo models to the [upstream magnetometer fix](https://github.com/PX4/PX4-Autopilot/pull/28877). Both standalone worlds select ENU magnetic components in tesla, and the aligned model uses tesla noise. [Simulation setup](../simulation_setup.md) gives exact revisions and a fail-fast compatibility check. Gaussian covariance propagation cannot compensate for an unmodeled systematic bias. `ANCHOR_MODE=simulated` selects the independent synthetic anchor fixture.
