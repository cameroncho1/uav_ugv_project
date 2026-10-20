# Camera heading validation

The evaluated system uses ROS 2 Humble on Ubuntu 22.04, Gazebo Harmonic 8.15.0,
the pinned PX4/message definitions, the owned 5 Hz camera, and the calibration in
`uav_sensing/config/camera_calibration.yaml`. [Conditions and source
fingerprints](results/heading/conditions.json) identify the numerical libraries,
configuration, tested source, and commands. Sensor noise and calibration
uncertainties are declared simulation assumptions.

## Build and registered checks

All eleven first-party packages build in Linux against the pinned consumed-message
underlay. Registered checks report 170 tests, zero errors/failures, and seven
cppcheck skips with installed cppcheck 2.7.
[Verification](results/heading/verification.json) records the check result.
Checks cover joint covariance, heading projection, angle wrap,
mount alignment, acquisition time, duplicate suppression, position-only fallback,
image annotation, and the other package contracts.

The joint-pose numerical check uses 6,000 correlated Monte Carlo samples. The
camera check uses 600 samples with independent corner noise and a shared
principal-point error, comparing position and heading variance with the model.
The XML linter uses a local catalog containing the official ROS package schemas
because the schema host is unreachable from this runtime. The schema sources and
hashes are included in the conditions; XML checks remain enabled.

## Rendered camera and direction indicator

The static camera body is at map `(0, 0, 4)` with zero attitude. The marker is at
z=0.37 m; its represented chassis is 0.12 m lower. Each scored phase receives at
least twelve camera frames. Visible phases require at least five detections.

| Case | Observations | Maximum 3D position error | Maximum heading error |
| --- | ---: | ---: | ---: |
| Forward | 12 | 0.0230 m | <0.001 degrees |
| Partial occlusion | 0 | No observation | No observation |
| Full occlusion | 0 | No observation | No observation |
| Recovery | 13 | 0.0230 m | <0.001 degrees |
| Off-axis `(1, -0.5)` m | 16 | 0.0113 m | 0.139 degrees |
| Left, +90 degrees | 14 | 0.0230 m | <0.001 degrees |
| Reverse, 180 degrees | 14 | 0.0230 m | <0.001 degrees |
| Right, -90 degrees | 13 | 0.0230 m | <0.001 degrees |

[Rendered summaries](results/heading/rendered_heading.json) preserve counts,
geometry errors, and detector diagnostics. [The annotated frame](results/heading/heading_image.png)
shows the camera-measured forward arrow. Callback checks verify that the image
keeps acquisition metadata and contains no retained arrow after target loss.
These ideal static images do not establish real-camera heading accuracy.

## Heading fusion in real EKFs

Two actual `robot_localization` EKFs consume identical wheel/IMU measurements and
camera correction messages. One selects camera x/y; the other selects joint
x/y/yaw. The fixture supplies declared stationary geometry with map heading
zero, UAV map yaw 0.7 rad, and relative chassis yaw -0.7 rad. No GPS is supplied.
This isolates fusion behavior from rendering accuracy.

IMU yaw bias grows from zero to 0.18 rad over twelve seconds. IMU orientation
standard deviation is 0.1 rad; relative orientation and anchor orientation each
have 0.02 rad standard deviation. Wheel/IMU messages are scheduled at 20 Hz and
camera messages at 5 Hz. Means use up to the last thirty received poses of each phase.

| Phase | Position-only mean absolute heading error | Joint mean absolute heading error |
| --- | ---: | ---: |
| Drift | 9.51 degrees | 4.93 degrees |
| Two-second camera outage | 10.31 degrees | 10.31 degrees |
| Recovery | 10.31 degrees | 5.65 degrees |
| Final recovery after outliers | 10.31 degrees | 5.07 degrees |

One second of 170-degree camera-heading outliers leaves the assisted estimate
within 10.32 degrees of the declared heading. Valid corrections resume afterward.
Independent UAV anchors continue during the camera outage. The adapter emits
113 joint updates and zero duplicate position updates.
[Filter summary](results/heading/heading_fusion.json) stores phase metrics and
sample counts. The reduction applies to this fixture and its stated covariance,
rates, bias, and outage; it is not a universal improvement guarantee.

## Live PX4 anchor and rendered camera

Three fresh worlds use seed 513, the 4.5 m hover setpoint, actual PX4 anchors,
rendered tag observations, and the 31-second nominal route. Each trial compares
filters on identical UGV messages. The assisted filter receives joint camera
x/y/yaw; the baseline receives wheel, IMU, and available GPS. These trials do not
isolate heading fusion from camera position assistance.

| GPS case | Baseline position RMSE | Assisted position RMSE | Baseline mean heading error | Assisted mean heading error | Joint updates |
| --- | ---: | ---: | ---: | ---: | ---: |
| On | 0.340 m | 0.154 m | 0.692 degrees | 0.691 degrees | 106 |
| Lost at 10 s | 0.313 m | 0.132 m | 0.731 degrees | 0.730 degrees | 113 |
| Denied from startup | 3.914 m | 0.129 m | 0.729 degrees | 0.737 degrees | 113 |

Each estimate has 594-600 acquisition-matched scored poses. GPS-loss summaries
contain zero GPS messages after the switch; startup denial contains zero GPS
messages throughout. Every received camera correction in these runs is joint.
Heading error is nearly equal between filters under this nominal IMU condition;
assistance does not improve every metric in every run. Planar 95% ellipse coverage
is 94.6-98.8% for the assisted estimates, describing temporally correlated samples.
[Live summaries](results/heading/camera_anchors/) preserve errors, counts,
uncertainty metrics, timing rejections, and the selected heading mode.

## Closed-loop camera navigation

The complete camera goal demo uses seed 514 with GPS disabled from startup.
The UGV arrives with 0.573 m physical goal distance, inside the declared 0.85 m
tolerance. Estimated distance is 0.469 m, inside the controller's 0.5 m tolerance.
Physical route cross-track RMSE is 0.0544 m across 3,195 matched samples; 2,639
truth readings without a sufficiently fresh reference route are skipped.

The scorer receives 937 wheel readings, 67 relative observations, 57 joint
corrections, and zero GPS readings. Arrival includes a complete stopped dwell
with fresh truth, estimate, and command evidence. Maximum linear command is
0.425 m/s. Arrival takes 154.4 wall seconds, with motion beginning at 70.2 wall
seconds. [Goal summary](results/heading/camera_goal.json) retains the conditions,
counts, freshness checks, and physical tracking metrics.

This establishes goal arrival under the stated tolerance in the flat, obstacle-free
world. It does not isolate the effect of heading fusion or establish a 0.5 m
physical arrival guarantee.

## Position-only assistance

Fixed-observer synthetic inputs supply position only, with heading fusion disabled.
Seeds 513-515 each exercise GPS available, lost at ten seconds, and denied from
startup. Nominal routes last 31 seconds; drift routes last 91 seconds with 15%
wheel-speed error and 0.002 rad/s heading drift. Three additional GPS-loss drift
trials inject an 8 m relative-position outlier every tenth sample and suppress
relative observations from route seconds 35 to 55.

| Group | Trials | Baseline position RMSE range | Assisted position RMSE range |
| --- | ---: | ---: | ---: |
| Nominal | 9 | 0.280-1.011 m | 0.180-0.233 m |
| Drift | 9 | 0.374-1.736 m | 0.208-0.221 m |
| Outliers and outage | 3 | 0.935-1.489 m | 0.360-0.464 m |

All 21 comparisons have lower assisted RMSE in these runs. Each trial has at least
500 acquisition-matched poses per estimate and at least fifty corrections. GPS
gates produce zero messages after declared loss and throughout startup denial.
The largest assisted position error in the outlier/outage group is 1.018 m.
These trials check position-only assistance with heading fusion disabled; they do
not measure the benefit of camera heading.

[Aggregate metrics](results/heading/paired_summary.json),
[nominal summaries](results/heading/paired_nominal/),
[drift summaries](results/heading/paired_drift/), and
[outlier/outage summaries](results/heading/paired_robust/) preserve conditions,
counts, error metrics, and the baseline/assisted comparisons. Sample counts report
the matched evidence; scoring does not extrapolate missing truth coverage.

## IMU heading bias limits

The filter treats each IMU heading sample as independent noise with its declared
standard deviation. It has no IMU bias state. A slow bias therefore pulls the
estimate toward the IMU, and the filter reports less heading uncertainty than the
bias error. The camera heading corrects part of the bias. When the gap between
the camera heading and the estimate exceeds the five-sigma gate, the filter
rejects the whole camera update, including its position.

The bias sweep repeats the filter fixture at several final biases and two
declared IMU standard deviations. The acceptance criteria stay fixed. The
outlier phase rejects every camera update for one second, so its maximum error
reaches the IMU bias. That criterion (0.2 rad) can only pass when the bias is
below 0.2 rad.

| IMU std | Final bias | Runs passed | Assisted recovery error | Position-only error | Result |
| ---: | ---: | ---: | ---: | ---: | --- |
| 0.1 rad | 0.18 rad | 3 of 3 | 4.8-5.3 degrees | 10.31 degrees | Camera heading accepted; criteria met. |
| 0.1 rad | 0.24 rad | 0 of 3 | 6.4-6.7 degrees | 13.75 degrees | Recovery and final criteria met; outlier maximum equals the bias. |
| 0.1 rad | 0.30 rad | 0 of 3 | 7.8-8.1 degrees | 17.19 degrees | Camera heading accepted; error above 6.88 degrees. |
| 0.1 rad | 0.36 rad | 0 of 3 | 10.2-20.6 degrees | 20.63 degrees | Gate rejects camera updates in some phases of every run. |
| 0.02 rad | 0.02-0.10 rad | 0 of 8 | 90-93% of position-only | 1.15-5.73 degrees | IMU outweighs the camera; improvement below the 30% criterion. |
| 0.02 rad | 0.18 rad | 0 of 2 | 10.31 degrees | 10.31 degrees | Gate rejects every camera update after the outage. |

[Bias sweep](results/heading/heading_bias_sweep.json) stores every run.
[Stress diagnostic](results/heading/heading_stress_gate.json) stores the
original 0.36 rad run. Each run has zero duplicate position updates.

The failures come from the filter model and the test conditions, not from the
heading measurement. The camera heading, its covariance, and the topic selection
behave as specified in every run. The supported operating limits are:

- Camera heading improves the estimate when the declared IMU heading standard
  deviation is several times the camera heading standard deviation (0.1 rad
  against about 0.03 rad here) and the IMU bias stays near or below 0.18 rad.
- With the 0.02 rad IMU heading standard deviation that the simulated UGV
  sensors declare, the IMU dominates. Camera heading reduces heading error by
  7-10%, and a bias near 0.15-0.18 rad locks out camera updates. The
  drift profile's 0.002 rad/s bias reaches that range after about 75-90 seconds.
- A gate lockout also drops camera position. Without GPS, the estimate then
  relies on wheel odometry and IMU until the IMU and camera agree again.
- `uav_heading:=false` keeps camera position assistance under any IMU bias. Use it
  for runs with a known or growing IMU heading bias.

The adapter's `heading_accepted` counter reports publication, not the filter's
gate decision. Bias estimation, adaptive IMU weighting, and gating heading
separately from position are not implemented.

## Reproduce

[Camera heading](camera_heading.md#validation) lists build, registered, rendered,
filter, paired-localization, and goal-navigation commands. The filter fixture's
`--imu-bias` option selects the final bias and `--imu-stddev` the declared IMU
heading standard deviation (default 0.1 rad). `--imu-bias 0.36` runs the stress
condition; `--imu-stddev 0.02` matches the simulated UGV IMU. Outputs include strict pass/fail results and phase samples.

For a runtime that cannot reach the XML schema host, cache the official schemas
and rerun registered checks with a local catalog:

```bash
HEADING_SCHEMA_DIR=/tmp/uav_heading_schemas
mkdir -p "$HEADING_SCHEMA_DIR"
for schema in package_format3 package_common; do
  curl -fsSL "https://raw.githubusercontent.com/ros-infrastructure/rep/master/xsd/${schema}.xsd" \
    -o "$HEADING_SCHEMA_DIR/${schema}.xsd"
done
cat > "$HEADING_SCHEMA_DIR/catalog.xml" <<EOF
<catalog xmlns="urn:oasis:names:tc:entity:xmlns:xml:catalog">
  <uri name="http://download.ros.org/schema/package_format3.xsd" uri="$HEADING_SCHEMA_DIR/package_format3.xsd"/>
</catalog>
EOF
cd ros2_ws
XML_CATALOG_FILES="$HEADING_SCHEMA_DIR/catalog.xml" \
  ROS_DOMAIN_ID=76 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  colcon test --executor sequential
colcon test-result --verbose
```

The numerical checks use one BLAS thread and one OpenCV thread to avoid CPU
oversubscription. Monte Carlo sample counts and test time limits are not reduced.

All runtime assemblies stop their owned processes. In both heading modes, the
fused pose is `/ugv/localization/pose` in ENU `map` with acquisition time. Hardware mounting
calibration, motion blur, uneven terrain, shared UAV heading bias, and simultaneous
loss of both global references require separate validation.
