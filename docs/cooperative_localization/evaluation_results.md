# UGV Localization Evaluation

The results below are retained October 11 evidence. [October 12 remediation and validation](../project/remediation_2026-10-12.md) records the current ownership, fixes, paired reruns, and physical tracking results. Earlier numbers are not reclassified as tests of the repaired code.

## Baseline-only short route

The `uav_ugv` Gazebo world runs a 31-second forward-and-turn route. The scorer compares `/ugv/localization/pose` with the `simple_ugv` model's true map pose at matching simulation timestamps. Each row below uses 615 pose samples from 10.5 to 31 seconds. Position error is the 2D distance between estimated and true x/y. The measurement model uses 1.5 m horizontal GPS standard deviation and the wheel and IMU settings in `measurement_model.yaml`.

`on` keeps GPS throughout the route. `off` disables prepared GPS at 10 seconds, after the filter has a GPS fix. `startup_off` disables prepared GPS before the first sensor reading. The UGV keeps publishing wheel and IMU measurements in both GPS-denied cases. The tests use a fresh Gazebo world for each seed and case.

| Seed | GPS case | Mean error (m) | RMSE (m) | 95th percentile (m) | Max (m) | Mean heading error (deg) |
| ---: | --- | ---: | ---: | ---: | ---: | ---: |
| 513 | on | 0.384 | 0.406 | 0.584 | 0.730 | 0.728 |
| 513 | off at 10 s | 0.454 | 0.455 | 0.482 | 0.487 | 0.726 |
| 513 | off from startup | 0.922 | 0.922 | 0.944 | 0.954 | 0.727 |
| 514 | on | 0.298 | 0.328 | 0.627 | 0.722 | 0.721 |
| 514 | off at 10 s | 0.306 | 0.306 | 0.323 | 0.329 | 0.709 |
| 514 | off from startup | 0.999 | 0.999 | 1.016 | 1.022 | 0.709 |
| 515 | on | 0.257 | 0.288 | 0.489 | 0.605 | 0.767 |
| 515 | off at 10 s | 0.116 | 0.116 | 0.131 | 0.143 | 0.712 |
| 515 | off from startup | 1.009 | 1.009 | 1.019 | 1.025 | 0.760 |

Every GPS-off trial publishes zero prepared GPS messages after the switch. Every startup-off trial publishes zero prepared GPS messages throughout. The fused pose continues at 30 Hz in every case.

The GPS-on result is not always the smallest error. In this route, wheel motion is stable while GPS has injected 1.5 m noise; losing GPS after an initial fix can therefore leave a steady estimate. With GPS off from startup, the estimate is about 1 m from truth because the UGV spawns at map x=1 m and the filter receives no global position anchor. The seed controls injected noise, but startup timing can change which readings the filter sees first, so repeated runs vary. These results describe this short simulated route and measurement model. They do not establish real GPS-denied performance or longer-term drift.

The live runner writes per-sample CSV and JSON summaries and accepts another estimate topic and launch file. [Interfaces](interfaces.md#evaluation-and-reproducibility) gives the command and output format.

## Paired baseline and UAV-assisted evaluation

![Paired position RMSE comparison](results/position_rmse.svg)

Both filters receive the same wheel, IMU, and GPS messages in one fresh Gazebo world per trial. Only the assisted filter receives UAV-derived x/y. The observer is fixed at map `(2, -2, 4)`. Independent Gaussian noise models relative position, UAV position, and UAV attitude; the anchor passes through the PX4 NED/FRD adapter. [Measurement design](uav_measurement_design.md) specifies the model and assumptions.

### Short route, nominal measurements

The route lasts 31 seconds; metrics cover 10.5-31 seconds. Each estimate contributes 615 matched samples. [JSON summaries and comparison CSV](results/short_nominal/summary.csv) preserve error metrics, sensor counts, and conditions.

| Seed | GPS case | Baseline RMSE (m) | Assisted RMSE (m) | Assisted mean NEES | Assisted 95% coverage |
| ---: | --- | ---: | ---: | ---: | ---: |
| 513 | off | 0.436 | 0.246 | 1.45 | 99.5% |
| 513 | on | 0.401 | 0.241 | 1.45 | 100.0% |
| 513 | startup_off | 0.917 | 0.250 | 1.50 | 99.5% |
| 514 | off | 0.282 | 0.209 | 1.11 | 99.3% |
| 514 | on | 0.303 | 0.214 | 1.21 | 99.3% |
| 514 | startup_off | 0.981 | 0.210 | 1.12 | 99.5% |
| 515 | off | 0.222 | 0.186 | 0.82 | 99.8% |
| 515 | on | 0.311 | 0.189 | 0.89 | 99.0% |
| 515 | startup_off | 1.007 | 0.185 | 0.82 | 99.8% |

All nine paired short trials have lower assisted position RMSE. With no GPS at startup, assistance supplies the missing global anchor: assisted RMSE is 0.185-0.250 m versus 0.917-1.007 m for the baseline. GPS disabled after an initial fix can leave a stable short-route baseline, as the baseline-only table also shows. Separate world runs cannot be compared as identical sensor sequences; the paired trials avoid that mismatch.

NEES is squared x/y error weighted by the EKF covariance. A correctly calibrated independent Gaussian 2D estimate has expected NEES 2 and 95% coverage below 5.991. Samples here are temporally correlated. Coverage near 99% and mean NEES below 2 indicate conservative uncertainty in these conditions; they do not establish calibrated 95% confidence.

### Long route with wheel and heading drift

The route lasts 91 seconds and repeats forward motion, turns, and stops. Metrics cover 10.5-91 seconds. The `drift` profile adds 15% wheel-speed scale error and 0.002 rad/s growing IMU heading bias. These biases deliberately exceed the covariance's independent noise model. Each strategy has 2,389-2,415 matched samples per trial. [Comparison CSV](results/drift/summary.csv) and per-trial JSON summaries store complete metrics.

| Seed | GPS case | Baseline RMSE (m) | Assisted RMSE (m) | Assisted maximum (m) | Assisted mean NEES | Assisted 95% coverage |
| ---: | --- | ---: | ---: | ---: | ---: | ---: |
| 513 | off | 1.393 | 0.218 | 0.531 | 1.16 | 99.8% |
| 513 | on | 0.394 | 0.219 | 0.514 | 1.23 | 99.9% |
| 513 | startup_off | 1.586 | 0.216 | 0.529 | 1.13 | 99.8% |
| 514 | off | 1.219 | 0.208 | 0.478 | 1.08 | 99.5% |
| 514 | on | 0.411 | 0.209 | 0.556 | 1.12 | 98.2% |
| 514 | startup_off | 1.675 | 0.214 | 0.617 | 1.15 | 98.3% |
| 515 | off | 0.943 | 0.213 | 0.627 | 1.10 | 99.2% |
| 515 | on | 0.378 | 0.204 | 0.595 | 1.05 | 99.0% |
| 515 | startup_off | 1.734 | 0.213 | 0.557 | 1.11 | 99.3% |

All nine assisted long trials have lower position RMSE. With GPS lost at 10 seconds, assisted RMSE is 0.208-0.218 m versus baseline 0.943-1.393 m. With GPS denied at startup, assisted RMSE is 0.213-0.216 m versus 1.586-1.734 m. UAV observations supply x/y only: mean heading error remains about 6.1-6.5 degrees in both strategies. Position correction does not establish heading observability.

### Outliers and a UAV observation outage

The 91-second drift route loses GPS at 10 seconds. Every tenth relative sample adds an 8 m body-x outlier. Relative observations stop from route time 35 to 55 seconds; UAV anchor samples continue. Metrics cover the whole 10.5-91 second window, including the outage and recovery. [Comparison CSV](results/robust/summary.csv) and JSON summaries store counts and complete metrics.

| Seed | Baseline RMSE (m) | Assisted RMSE (m) | Assisted maximum (m) | Jump rejections | Assisted 95% coverage |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 513 | 1.280 | 0.388 | 0.738 | 32 | 99.4% |
| 514 | 1.464 | 0.388 | 0.805 | 31 | 99.4% |
| 515 | 0.626 | 0.320 | 0.641 | 38 | 99.8% |

All three assisted fault trials remain below 0.389 m RMSE and 0.806 m maximum error. The baseline reaches 0.626-1.464 m RMSE. The correction resumes after the outage without a filter restart. Counts include the producer's pre-route startup interval and the most recent one-second diagnostic report; they are not exact counts restricted to the scoring window. Nonzero expired counts indicate readings discarded when acquisition-time matching cannot finish within 0.5 seconds.

### Flying UAV and live PX4 anchor diagnostics

A PX4 X500 climbs and hovers while the UGV follows the 31-second nominal route with GPS off from startup. Seed 513 is used; both filters have 615 scored samples. The two rows are separate world runs with paired filters inside each run. They do not share identical sensor sequences across rows.

| UAV anchor source | Baseline RMSE (m) | Assisted RMSE (m) | Assisted mean NEES | Assisted 95% coverage | Stored summary |
| --- | ---: | ---: | ---: | ---: | --- |
| Independent noisy simulated pose | 0.962 | 0.208 | 1.12 | 98.4% | [JSON](results/flight_simulated/trial_live.json) |
| Actual PX4 pose, legacy magnetic path | 0.969 | 1.172 | 24.80 | 0.0% | [JSON](results/live_px4_anchor/trial_live.json) |

The simulated-anchor flight validates full-body rotation, acquisition-time matching, and integration with an actual flying Gazebo model. It does not validate PX4's independent absolute-position accuracy or calibrated perception.

The legacy configuration uses PX4 `d6f12ad1c4f70ad3230afd7d86e971421e02fef4` and px4_msgs `86d8239e962f6939e05c3737784f60c02fa884db`. Its actual PX4 anchor worsens position error and reports overconfident uncertainty. A [timestamp-matched heading check](results/live_px4_anchor/heading_check.json) compares `vehicle_attitude` with `vehicle_attitude_groundtruth` over simulation time 55-75 seconds: mean signed NED heading error is 17.94 degrees across 1,111 samples. [Matched heading samples](results/live_px4_anchor/heading_check.csv) preserve the inputs. The [upstream fix](https://github.com/PX4/PX4-Autopilot/pull/28877) explains the legacy frame/unit incompatibility. This source bias is not represented by the reported covariance.

### Aligned PX4, models, and magnetic settings

These nine fresh-world live trials use the exact revisions and tesla/ENU settings in [simulation setup](../simulation_setup.md). The actual PX4 X500 hovers while both UGV filters receive identical sensor messages and simulated relative vectors. Each trial scores 615 UGV samples over route seconds 10.5-31. UAV heading compares estimated and true ULog attitudes over the same acquisition-time window. [Summary CSV](results/aligned_px4/summary.csv), per-trial JSON, and [representative heading samples](results/aligned_px4/heading_samples_seed_513_startup_off.csv) store the measurements.

| Seed | GPS case | Baseline RMSE (m) | Assisted RMSE (m) | UAV heading RMSE (degrees) |
| ---: | --- | ---: | ---: | ---: |
| 513 | off | 0.220 | 0.149 | 0.856 |
| 513 | on | 0.329 | 0.151 | 0.854 |
| 513 | startup_off | 0.982 | 0.135 | 0.809 |
| 514 | off | 0.029 | 0.134 | 0.810 |
| 514 | on | 0.371 | 0.132 | 0.894 |
| 514 | startup_off | 1.004 | 0.129 | 0.820 |
| 515 | off | 0.276 | 0.129 | 0.791 |
| 515 | on | 0.384 | 0.130 | 0.912 |
| 515 | startup_off | 1.027 | 0.124 | 0.845 |

All assisted runs have 100% planar 95% ellipse coverage under these conditions. Startup without GPS improves in all three seeds. The seed-514 dropout baseline has lower RMSE than the assisted estimate: a lucky initial GPS fix and stable short-route wheel motion can outperform noisy additional corrections. Assistance is not guaranteed to improve every route. Heading RMSE is 0.79-0.91 degrees with the aligned configuration; the upstream fix describes a residual caused by different earth-field models. These trials still synthesize relative vectors and do not validate camera perception or navigation.

The aligned PX4 and full ROS workspace build successfully. Registered checks report 83 tests, zero failures, and three skipped cppcheck checks due to cppcheck 2.7. Message compatibility tests check wire-field order, shapes, and message versions; world-setting tests reject the legacy magnetic configuration. VehicleStatus uses its declared version-4 topic. Bash syntax and Python lint checks pass. A goal-demo smoke run using the Ogre server renderer receives live PX4-derived corrections and a fused map pose with the camera topic supplied through the mapper's launch parameter. PX4's console runs under a terminal even when ROS launch redirects stdin. This smoke run does not score navigation accuracy.

## Navigation guards and unknown startup position

The planner and controller use the fused pose with the limits in [navigation](../navigation.md). [Nineteen ROS integration cases](results/navigation/guards.json) verify continuous zero commands on bad/stale/uncertain inputs, frozen ROS time, missing or blocked maps, and arrival. Valid inputs restore motion. The old raw-pose topic cannot authorize motion. Registered checks report 111 tests, zero failures, and six skipped cppcheck checks due to cppcheck 2.7.

The EKF initialization uses large x/y uncertainty for an unknown map origin. [Nine fresh-world paired trials](results/navigation/summary.csv) repeat seeds 513-515 with all three GPS cases and the fixed-observer simulated sensor. Each strategy has 591-615 scored samples. Assisted RMSE is 0.184-0.250 m. Startup without GPS has assisted RMSE 0.185-0.242 m versus baseline 0.933-1.016 m. Assisted 95% ellipse coverage is 99.0-99.8%. The unanchored baseline retains approximately `1e6 mÂ²` x/y variance; coverage of its unknown origin does not demonstrate accurate localization. No navigation controller drives these paired scoring routes.

A [physical goal trial with the upstream 30 Hz camera](results/navigation/goal_upstream_camera.json) fails to arrive within 300 wall seconds. It starts with GPS disabled, begins motion after 30.0 wall seconds, and stops repeatedly on stale fused poses. The world then stops advancing; true remaining goal distance is 7.762 m. This run verifies the hold behavior under a stalled runtime and does not establish goal arrival. The runner stops its owned process groups, including a renderer that does not finish graceful shutdown. The camera-specific evaluation below uses the owned 5 Hz model and reports physical arrival separately.

## Calibrated rendered perception

The camera pipeline uses rendered pixels, CameraInfo intrinsics, a 0.4 m AprilTag, and known simulated mounting geometry. The detector has no simulator-pose input. [Perception](perception.md) specifies frames, covariance assumptions, visibility gates, and commands.

### Visibility and metric geometry

The owned camera hovers at world z=4 m in a static fixture. The roof marker is at z=0.37 m, the represented chassis at z=0.25 m, and the camera mount 0.18 m below the UAV body. Each phase scores at least twelve delivered frames after settling, so software-rendering speed does not change the amount of evidence. [Fixture JSON](results/camera/fixture.json) stores observation counts and diagnostics.

| Case | Observations | Maximum 3D position error (m) | Result |
| --- | ---: | ---: | --- |
| Visible | 10 | 0.0230 | Pass |
| Partial visual occlusion | 0 | — | Pass |
| Full visual occlusion | 0 | — | Pass |
| Recovery | 14 | 0.0230 | Pass |
| Off-axis target `(1, -0.5)` m | 15 | 0.0113 | Pass |

The [default-threshold fixture](results/camera/fixture_default_threshold.json) detects the centered target and recovery but misses the off-axis target. The captured image contains the correct ID. An offline detector-parameter sweep identifies adaptive threshold constant 15 as a successful candidate; constants 7 then 15 recover it without changing dictionary or geometry gates. [Research summary](results/camera/detector_research.json) and the [regression image](../../ros2_ws/src/uav_sensing/test/fixtures/off_axis_tag.png) preserve this case. Partial/full occlusion still produce no observations with both attempts. These checks cover the stated synthetic images; they do not quantify a real-camera false-detection rate.

### Actual PX4 anchors and camera observations

Nine fresh-world trials use the aligned PX4 dependency configuration, the owned 5 Hz camera, a 4.5 m altitude setpoint, and the 31-second nominal route. Both filters receive identical wheel/IMU/GPS messages; only the assisted filter receives camera-derived corrections. Each strategy contributes 614–615 timestamp-matched poses over route seconds 10.5–31. The summaries include acquisition-time rejection counts, GPS gates, wheel counts, and observation source. [Comparison CSV](results/camera/anchors/summary.csv) and per-trial JSON preserve the measured values; [conditions](results/camera/conditions.json) record runtime versions and model/calibration fingerprints.

| Seed | GPS case | Baseline RMSE (m) | Assisted RMSE (m) | Assisted 95% coverage | UAV corrections | Wheel readings |
| ---: | --- | ---: | ---: | ---: | ---: | ---: |
| 513 | off | 0.242 | 0.105 | 100.0% | 62 | 625 |
| 513 | on | 0.354 | 0.124 | 100.0% | 59 | 596 |
| 513 | startup_off | 3.916 | 0.139 | 100.0% | 52 | 598 |
| 514 | off | 0.269 | 0.102 | 100.0% | 84 | 600 |
| 514 | on | 0.348 | 0.110 | 100.0% | 83 | 598 |
| 514 | startup_off | 3.897 | 0.157 | 100.0% | 50 | 597 |
| 515 | off | 0.293 | 0.107 | 100.0% | 89 | 597 |
| 515 | on | 0.359 | 0.096 | 100.0% | 59 | 597 |
| 515 | startup_off | 3.867 | 0.128 | 100.0% | 95 | 597 |

Assisted RMSE is 0.096–0.157 m under these conditions. The startup-without-GPS baseline retains its roughly 4 m unknown-origin error in this world; assistance supplies an absolute anchor. Dropout cases retain 398 wheel readings after GPS loss, and every startup-off case records zero GPS readings. Coverage of 100% describes these correlated samples and indicates conservative uncertainty; it is not a calibrated hardware confidence guarantee. Separate configurations do not share identical sensor sequences.

### Goal arrival using camera corrections

The [arrival summary](results/camera/goal.json) scores the complete guarded navigation loop in a fresh goal-demo world. Seed 514 uses the owned camera at a 4.5 m altitude setpoint, actual PX4 anchors, and UGV GPS disabled from startup. The scorer receives 717 correctly framed wheel measurements, 41 relative observations, 34 accepted UAV corrections, and zero prepared GPS messages. The controller begins moving after 66.6 wall seconds and meets the arrival criterion after 320.0 wall seconds. Software rendering and freshness holds affect wall duration.

Estimated goal distance is 0.478 m, inside the controller's 0.5 m estimated tolerance. Independently scored physical distance is 0.607 m, inside the trial's declared 0.85 m physical tolerance. Maximum commanded linear speed is 0.425 m/s. This demonstrates arrival under that tolerance in this flat world; it does not establish a 0.5 m physical accuracy guarantee. The scoring window allows 600 wall seconds and requires fresh truth, estimate, and commands at arrival, more than 100 wheel readings, more than 20 relative observations/corrections, and no prepared GPS when startup GPS is disabled.

[Startup diagnostic logs](results/camera/startup_diagnostic/conditions.json) retain a live-anchor initialization timeout that produces no accepted accuracy result. The live runner allows 240 wall seconds for a settled anchor; this timeout does not weaken the ten-second simulated settling interval or acquisition-time gates. The successful nine-case camera matrix uses the same frame and calibration configuration.

### Navigation hold with a common position error

The [hold summary](results/camera/hold.json) uses seed 513, actual PX4 anchors, rendered camera observations, GPS enabled, and the same 0.8 m shared-position-error model applied to both prepared GPS and the PX4 anchor. The public fused pose retains the configured 0.8 m uncertainty prior. Flight control receives raw PX4 data.

The scorer observes 463 valid wheel readings, 120 GPS readings, 33 relative observations, and 21 accepted UAV corrections. Position standard deviation stays at or above 0.851 m, beyond the 0.5 m navigation limit. All 1,217 commands remain at zero speed; the UGV stays at its spawn. The check completes after 125.6 wall seconds with truth/pose/command receipt ages of 0.001/0.019/0.081 seconds. Inputs are present and fresh at validation, so this is evidence for the uncertainty hold with an active perception/anchor chain.

Two diagnostic snapshots preserve strict failed checks: [a stale deadline receipt](results/camera/hold_receipt_diagnostic.json) and [insufficient corrections within 240 wall seconds](results/camera/hold_observation_diagnostic.json). Both show zero movement, but neither meets all evidence requirements. The scorer completes on a qualifying fresh sample inside the bounded window, requiring more than 100 wheel readings/pose samples/commands and more than 20 relative observations/corrections. GPS must be present in this case. The 600-wall-second cap accommodates software rendering; it changes no acquisition-age, uncertainty, or count thresholds.

### Wheel-frame integration check

The [frame diagnostic](results/camera/frame_diagnostic/conditions.json) records a payload integration fault: merging the roof marker before the chassis selects `fiducial_link` as Gazebo's default canonical body. The raw DiffDrive output names `simple_ugv/fiducial_link`, while the EKF expects `simple_ugv/chassis`. Without that transform the filter cannot use wheel velocity. The [raw frame messages](results/camera/frame_diagnostic/wheel_frame_default_body.txt) and [baseline trace](results/camera/frame_diagnostic/baseline_513_off.csv) show the mismatch and growing prediction uncertainty. Camera corrections can mask this missing input. Those diagnostic runs do not validate the intended GPS/wheel/IMU baseline.

Both operational worlds explicitly select `chassis` as canonical link and specify the DiffDrive parent/child frame IDs. [The contract check](results/camera/frame_diagnostic/wheel_frame_chassis.txt) confirms the chassis child frame. The scorer rejects a mismatched frame and requires at least 100 wheel measurements, including continuation after a GPS switch. Accepted result summaries record wheel counts. The diagnostic folder retains five camera summaries and six shared-bias summaries under the missing-wheel condition, plus one route-arrival summary; these are separate from the correctly framed comparisons.

### Persistent position error shared by GPS and UAV

Six paired fixed-observer/vector trials use GPS on, the 31-second nominal route, and a persistent seeded ENU x/y bias drawn with 0.8 m standard deviation. Both UGV GPS and the UAV anchor receive the same offset; [fault conditions](results/shared_bias_conditions.json) record its ENU value for each seed. Independent measurement noise retains its existing seed. Each trial compares baseline and assisted filters on identical messages. Separate prior configurations use fresh worlds and can have different startup timing.

| Seed | Assisted RMSE, no prior (m) | Mean NEES, no prior | Coverage, no prior | Assisted RMSE, 0.8 m prior (m) | Mean NEES, prior | Coverage, prior |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 513 | 1.862 | 41.22 | 0.0% | 1.875 | 4.85 | 96.9% |
| 514 | 0.260 | 0.81 | 100.0% | 0.261 | 0.09 | 100.0% |
| 515 | 0.387 | 1.81 | 100.0% | 0.391 | 0.21 | 100.0% |

[No-prior summaries](results/shared_independent/summary.csv) and [prior summaries](results/shared_bounded/summary.csv) store both strategies and full conditions. Seed 513 demonstrates overconfidence: roughly 1.9 m error, mean NEES 41.22, and zero coverage without the prior. Retaining the 0.8 m public covariance prior gives mean NEES 4.85 and 96.9% coverage in that seed. The prior represents a Gaussian standard deviation, not a hard error bound or a guarantee of 95% coverage in three seeded trials. It changes public uncertainty, not the estimated position. Relative sensing cannot observe a translation shared by all global anchors.

## Verification and remaining conditions

The aligned PX4 and all nine ROS packages build. Registered checks report 140 tests, zero failures, and six skipped cppcheck checks caused by the installed cppcheck 2.7 performance issue. Numerical and callback tests cover NED/FRD and ENU/FLU axes, random rotations, covariance lever arms, a 10,000-sample Monte Carlo covariance check within 5%, timestamp interpolation, missing anchors, wrong frames, disabled corrections, outlier recovery, PX4 clock mismatches, resets, and takeoff initialization. Bash runners pass syntax checks. A goal-demo smoke check uses the documented Ogre backend with the GUI and RViz disabled: PX4 confirms Offboard and armed state, and the actual `x500_mono_cam_down_0` model produces timestamped map corrections. The headless Ogre2 renderer crashes in this environment, so that backend is not part of the validated demo configuration. This smoke check does not score navigation accuracy.

The recorded groups cover nominal, drift, and fault routes, aligned PX4 flights, guarded navigation, and rendered perception. All GPS-off summaries report zero prepared GPS messages after the switch, and every startup-off summary reports zero prepared GPS throughout. Every accepted paired localization trial has at least 500 timestamp-matched samples in the scoring window and at least 50 received UAV corrections. The EKF continues publishing during outages. Coverage is descriptive for correlated samples, and the short baseline's large uncertainty can also cover biased estimates.

Run [the evaluation commands](interfaces.md#evaluation-and-reproducibility) to generate full per-sample UGV CSVs, per-trial JSON, logs, and comparison summaries. Checked-in results keep JSON summaries, comparison CSVs, and the PX4 heading diagnostic. Startup scheduling can change which seeded measurements each filter first sees; exact error values vary between fresh worlds. Within a trial both strategies receive the same UGV messages.

The runtime is ROS 2 Humble on Ubuntu 22.04, Gazebo Sim 8.15.0, and robot_localization 3.5.4. Each flight section identifies its dependency configuration; the aligned configuration is pinned in `simulation/dependencies.json`. Noise and fault values are controlled simulation assumptions. Shared position bias and camera visibility are tested under the stated simulation conditions. Physical wheel slip, uneven terrain, hardware camera calibration, unknown global bias, weather, motion blur, and obstacle-mapping accuracy remain untested.
