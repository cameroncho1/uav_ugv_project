# UAV-assisted position measurements

## Implemented model

The sensor boundary uses a full 3D UGV position relative to the UAV in `uav/base_link` (forward, left, up). `PoseWithCovarianceStamped` carries the relative vector, its position covariance, and its acquisition time. Its orientation is unused. The implemented camera detector supplies this vector using known marker geometry, CameraInfo intrinsics, and body-camera extrinsics; [perception](perception.md) describes the calibration assumptions. A range-only UWB reading does not supply this vector; it needs a different observation model or additional geometry.

The UAV anchor is independently localized in ENU `map`. `uav_control/uav_pose.py` converts PX4 NED position, FRD attitude, and their variances. The configured `map_origin` translates PX4's local origin into the shared map. This origin is an external alignment assumption, not an estimate obtained from UGV truth. The live PX4 anchor waits for land detection to report airborne and for a reset-free ten-second initialization interval before accepting its first anchor. Later PX4 resets stop output and require realignment and restart. The goal demo and live trial runner use actual PX4 anchors. Independent noisy simulated anchors remain available for controlled observation tests.

The correction computes `p_ugv = p_uav + R_uav * relative`. With independent anchor and relative errors, its covariance is `J * C_uav * J.T + R * C_relative * R.T`, where `J = [I, -skew(R * relative)]`. Anchor attitude errors are about fixed map axes. Full 3D rotation and covariance are retained; the planar EKF selects only map x/y. The Jacobian is a small-angle approximation.

`simulation_support` owns the Gazebo pose reader; `fault_management` owns the explicit noise/fault producer. They start only in evaluation assemblies. At 5 Hz it generates independent Gaussian relative position, UAV position, and UAV attitude errors. The fixed-observer evaluation places the UAV at map `(2, -2, 4)`; it models an independently localized hovering observer without running flight dynamics. The same relative sensor can read a named PX4 UAV model in a live world. True poses supply synthetic sensor data and scoring; the estimator receives only the noisy measurements.

## Design choices checked by evaluation

Numerical results in this section are retained October 11 runs. [October 12 remediation](../project/remediation_2026-10-12.md) records the current reruns; they are not promises of general accuracy.

| Choice | Evidence and implemented behavior |
| --- | --- |
| Full relative vector and 3D rotation | Range alone cannot determine one UGV position from one UAV. Rotation round trips and lever-arm covariance tests cover nonzero roll, pitch, and yaw. Only x/y is fused by the existing planar EKF. |
| UAV uncertainty in every correction | A 10,000-sample Monte Carlo check agrees with the propagated covariance diagonals within 5%. Attitude error grows into position error with distance. |
| Acquisition-time matching | Delayed readings wait for a bracketed anchor, expire without extrapolation, and retain their original timestamp. Clock-mismatch and reset tests reject invalid anchors. |
| Independent streaming EKFs | The existing robot_localization filter accepts absolute pose x/y. A second baseline filter receives identical UGV messages for each paired comparison; neither estimate feeds the observation producer. |
| Jump gate plus EKF innovation gate | Synthetic 8 m outliers generate 31-38 adapter jump rejections per fault trial. Observations resume after the 20-second outage and assisted RMSE stays below 0.389 m. |
| Aligned PX4 anchor as the live default | Nine live trials have 0.124-0.151 m assisted RMSE and 0.79-0.91 degree UAV heading RMSE. The legacy magnetic configuration has a measured 17.94 degree bias. [Recorded conditions](evaluation_results.md) distinguish these dependency configurations. |

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

The paired evaluation uses two EKFs on the same sensor messages and trajectory. It measures position error and reported planar uncertainty under nominal noise, injected wheel-scale and IMU-heading drift, relative-measurement outliers, and correction dropout. Results and conditions are recorded in `evaluation_results.md`.

## Live PX4 compatibility

The dependency manifest pins PX4, matching ROS messages, and Gazebo models to the [upstream magnetometer fix](https://github.com/PX4/PX4-Autopilot/pull/28877). Both standalone worlds select ENU magnetic components in tesla, and the aligned model uses tesla noise. [Simulation setup](../simulation_setup.md) gives exact revisions and a fail-fast compatibility check. The nine live trials measure 0.79-0.91 degree heading RMSE; the residual is consistent with the upstream discussion of earth-field model differences. This is a measured simulation result, not a guarantee of hardware heading accuracy. Gaussian covariance propagation cannot compensate for an unmodeled systematic bias. `ANCHOR_MODE=simulated` selects the independent synthetic anchor fixture.
