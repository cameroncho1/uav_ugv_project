# Implementation report: October 11, 2026

Historical report: behavior and numerical results below describe October 11. Current code links follow the October 12 ownership moves; [remediation](remediation_2026-10-12.md) records the fixes and replacement validation.

## Summary and scope

- The project runs a simulated UAV-assisted UGV navigation system: camera pixels produce a relative UGV position; a timestamp-matched PX4 anchor converts it into map position; an EKF fuses it with wheel, IMU, and available GPS measurements; guarded navigation produces velocity commands.
- This report covers all 11 commits dated October 11, 2026 in `America/Los_Angeles`, reachable from `feat/rylan-gps` at `625b5ea2a0fefa20a3392e6f3b1e8c7f0371bb60`. It includes both merges and the preflight fix made on `main` and merged into this branch.
- Each commit's patch is reviewed, including implementation, configuration, documentation, tests, and recorded results. Merge contributions are checked against their parents so imported work is attributed correctly.
- The implementation commits are local; nothing is pushed. Commit links target the repository's GitHub remote and become available there when those commits are published. Use `git show <commit>` locally to inspect them now. For a merge, use `git diff <commit>^1 <commit>` to inspect what it brings into this branch.
- The reported tests are retained implementation results. Preparing this report does not rerun PX4 or ROS trials. Test totals at different commits describe successive checks of overlapping code; they should not be added together.
- The unrelated `.gitignore` working-tree edit is preserved. The traversability mapper source has no changes from the localization work.
- Main evidence: [evaluation results](../cooperative_localization/evaluation_results.md), [interfaces](../cooperative_localization/interfaces.md), [perception](../cooperative_localization/perception.md), and [navigation](../navigation.md).

## 1. Integrate Cameron's goal demo

- **Commit:** [a3b99bc — Merge main into feat/rylan-gps](https://github.com/cameroncho1/uav_ugv_project/commit/a3b99bc9a8d34a53d1220162f11b300dbc9a7951), 11:56 PDT.
- **Changed:** imports Cameron's goal-demo world, mission goal publisher, stationary-XY UAV controller, camera-luminance mapper, local A* planner, UGV path follower, launch configuration, and RViz configuration.
- **Why:** supplies the simulation and navigation loop into which localization can connect.
- **Decision:** preserve the branch's localization documentation and simulation-launcher fixes while integrating the demo from `main`.
- **Attribution:** the mapper and original goal-demo implementation come from Cameron Cho's `b782b72`, already merged into `main`. This merge integrates that work; it does not establish authorship of it as localization work.
- **Verification:** the first-parent patch identifies the imported demo files; the second-parent comparison shows the branch documentation and launcher work retained alongside them. Full workspace builds and the final goal trial provide integration evidence for the resulting system.
- **Limit:** the mapper classifies image brightness. The demo world has no physical obstacles. Importing the demo does not establish obstacle detection accuracy or localization accuracy.
- **Evidence:** [goal-demo instructions](../../README_simple_goal_demo.md), [mapper source](../../ros2_ws/src/traversability_mapping/scripts/traversability_mapper), [demo launch](../../ros2_ws/src/mission_manager/launch/simple_goal_demo.launch.py), and [goal world](../../simulation/worlds/simple_goal_demo.sdf).

## 2. Apply the PX4 preflight setting after startup

- **Commit:** [3f11f0b — Apply PX4 preflight setting in goal demo](https://github.com/cameroncho1/uav_ugv_project/commit/3f11f0b1e49bdcccfa252c4f42ddacd0be236654), 12:13 PDT.
- **Changed:** the goal-demo launcher waits for commander startup and sets `NAV_DLL_ACT=0` through PX4's parameter command.
- **Why:** the airframe startup script overwrites the environment-provided default. Applying the parameter after startup lets this simulation arm without QGroundControl.
- **Decision:** keep the setting in the simulation launcher and bound the startup polling loop. The launcher cleans up its background helper on exit.
- **Verification:** the patch replaces the environment-only setting with the post-startup command. Recorded flight and demo checks confirm Offboard and armed state in the supported simulation configuration.
- **Limit:** this is a simulation preflight configuration. It does not validate physical flight operation.
- **Evidence:** [PX4 goal-demo launcher](../../scripts/px4_goal_demo.sh).

## 3. Merge the preflight fix into this branch

- **Commit:** [2423af5 — Merge main into feat/rylan-gps](https://github.com/cameroncho1/uav_ugv_project/commit/2423af506838f00e4f86806574f4439380591e0f), 12:13 PDT.
- **Changed:** brings the `3f11f0b` preflight launcher fix from `main` into `feat/rylan-gps`.
- **Why:** keeps the branch's goal demo able to arm under the same startup configuration.
- **Decision:** integrate the existing fix through Git history. The first-parent diff contains that launcher change.
- **Verification:** the merged launcher contains the post-startup parameter command. This is the same fix described in section 2, not an additional independent implementation or experiment.
- **Evidence:** [PX4 goal-demo launcher](../../scripts/px4_goal_demo.sh).

## 4. Connect UGV simulation sensors to ROS 2

- **Commit:** [b257e4d — Connect UGV simulation sensors to ROS 2](https://github.com/cameroncho1/uav_ugv_project/commit/b257e4d58e5763cfc501604df52592787a159dbd), 12:36 PDT.
- **Changed:** adds chassis IMU and GPS sensors to both operational worlds, explicitly exposes wheel odometry, and bridges all three measurements plus `/clock` into ROS 2.
- **Why:** the estimator needs stated sensor interfaces, timestamps, and coordinate frames.
- **Decision:** `cooperative_localization` owns the bridge configuration. Raw sensors remain available separately from prepared measurements and the fused pose.
- **Decision:** use Gazebo-to-ROS bridge direction, sensor QoS for raw readings, and simulation-clock QoS for `/clock`.
- **Result:** configured raw rates are 20 Hz wheel odometry, 50 Hz IMU, and 5 Hz GPS. The demo launch includes the sensor chain; docs and presentations describe those interfaces.
- **Verification:** subsequent localization trials exercise the bridged inputs. Final camera comparisons also require valid wheel messages and verify GPS presence or absence.
- **Limit:** raw simulation covariance is zero or unknown. These raw messages require the measurement model described in section 5 before fusion.
- **Evidence:** [bridge configuration](../../ros2_ws/src/simulation_support/config/ugv_sensor_bridges.yaml), [sensor launch](../../ros2_ws/src/simulation_support/launch/ugv_sensors.launch.py), and [interfaces](../cooperative_localization/interfaces.md).

## 5. Prepare measurements in the map frame

- **Commit:** [e3103fd — Prepare UGV localization measurements in map frame](https://github.com/cameroncho1/uav_ugv_project/commit/e3103fddf9fcf2b84e77852987263412c280c2cf), 12:52 PDT.
- **Changed:** adds the measurement adapter, shared noise settings, and world-specific GPS origins and UGV spawn configuration.
- **Why:** GPS latitude/longitude and spawn-relative wheel odometry cannot be treated as the same map coordinates.
- **Decision:** convert WGS84 geodetic position through Earth-centered coordinates into local ENU meters. Anchor prepared wheel pose to the configured spawn and accumulate motion.
- **Decision:** preserve acquisition timestamps, leave raw messages intact, and publish configured noise with matching covariance. Unused pose components carry large uncertainty.
- **Verification:** tests cover the GPS origin, an approximately one-meter east displacement, invalid GPS fixes, spawn offsets, uncertainty growth, and motion occurring before the adapter starts.
- **Limit:** noise values are simulation assumptions. Prepared wheel pose includes a configured spawn offset, but the EKF uses wheel velocity; that configured pose does not supply a global anchor to the filter.
- **Evidence:** [measurement adapter](../../ros2_ws/src/cooperative_localization/scripts/ugv_measurements.py), [noise configuration](../../ros2_ws/src/fault_management/config/measurement_model.yaml), and [world configuration](../../ros2_ws/src/simulation_support/config/simple_goal_demo.yaml).

## 6. Add the baseline UGV estimator

- **Commit:** [77bff8b — Add baseline UGV position estimator](https://github.com/cameroncho1/uav_ugv_project/commit/77bff8b003963e2d4154004eecd3c0d6e81d7321), 13:02 PDT.
- **Changed:** runs a planar `robot_localization` EKF and exposes its output as `/ugv/localization/pose`, including timestamp and covariance.
- **Why:** provides one defined position estimate for evaluation and downstream navigation.
- **Decision:** use the existing ROS filter. Wheel forward speed predicts movement, IMU yaw and turn rate supply heading information, and GPS x/y correct global position.
- **Decision:** exclude integrated wheel pose and GPS placeholder orientation from fusion. Reusing integrated wheel pose would count information from the same wheel source twice.
- **Decision:** use a small output adapter to convert filtered odometry into the agreed pose message. It performs no second filter step. The current adapter also retains the systematic uncertainty described in section 11.
- **Verification:** pose-contract tests check map frame, timestamp, position, and covariance preservation. Baseline route trials in section 7 measure actual error.
- **Evidence:** [EKF configuration](../../ros2_ws/src/cooperative_localization/config/ugv_ekf.yaml), [localization launch](../../ros2_ws/src/cooperative_localization/launch/ugv_localization.launch.py), and [public pose adapter](../../ros2_ws/src/cooperative_localization/scripts/ugv_fused_pose.py).

## 7. Evaluate localization with and without GPS

- **Commit:** [2ba4e8d — Test UGV localization with and without GPS](https://github.com/cameroncho1/uav_ugv_project/commit/2ba4e8d4eb34ab9571a4c0ed6348cc44031013d3), committed 13:43 PDT.
- **Changed:** adds a GPS enable service, GPS-denied startup, a test-only truth reader, reusable route scorer, trial runner, and zero lateral wheel-speed fusion.
- **Why:** a publishing estimator is not enough evidence. Error must be measured against an independent reference while GPS actually stops contributing.
- **Decision:** use separate seeded noise streams for wheel, IMU, and GPS. Disabling GPS does not change the other streams.
- **Decision:** match estimates to interpolated truth at acquisition time and reject excessive truth gaps. Simulator truth belongs to sensor synthesis and scoring; it is not an operational estimator input.
- **Decision:** include a zero lateral wheel-speed measurement for this differential-drive model. It constrains sideways drift under the model's assumptions.
- **Result:** nine baseline trials cover three seeds and GPS always on, lost at 10 seconds, and disabled from startup. Each baseline trial scores 615 poses from route seconds 10.5 to 31. GPS-off cases contain zero prepared GPS messages after the switch; the fused pose continues at 30 Hz.
- **Result:** baseline RMSE is 0.288-0.406 m with GPS on, 0.116-0.455 m after dropout, and 0.922-1.009 m with GPS denied from startup.
- **Verification:** the commit records 33 registered checks, zero failures, and one skipped check. Tests cover GPS gating, independent noise streams, timestamp interpolation, angle wrapping, and metric calculations.
- **Limit:** stable short-route wheel motion can outperform noisy GPS after a good initial fix. These results do not establish long-term drift performance. The startup-denied error reflects an unknown global origin.
- **Evidence:** [baseline results](../cooperative_localization/evaluation_results.md#baseline-only-short-route), [scorer](../../ros2_ws/src/fault_management/scripts/localization_trial.py), and [trial runner](../../scripts/run_localization_trials.sh).

## 8. Add UAV-assisted localization and harder trials

- **Commit:** [a855324 — Add simulated UAV-assisted UGV localization](https://github.com/cameroncho1/uav_ugv_project/commit/a855324343ce1d620765470dff725e5588296e42), 14:59 PDT.
- **Changed:** adds the relative-vector interface, synthetic observation fixture, PX4 frame adapter, timestamp-matched correction producer, assisted EKF input, diagnostic counters, and UAV correction enable service.
- **Why:** a localized UAV plus a measured UAV-to-UGV vector can supply the global position that the UGV loses when GPS is unavailable.
- **Decision:** use a full 3D relative vector and full body rotation. One scalar range from one observer cannot determine a unique UGV position.
- **Decision:** explicitly convert PX4 world NED to ENU and body FRD to FLU, including covariance and the configured local-to-map origin.
- **Decision:** match the anchor to observation acquisition time. Readings wait for an exact or bracketed anchor, expire after 0.5 seconds, and are not extrapolated. Anchor brackets must be no wider than 0.25 seconds.
- **Decision:** propagate UAV position, UAV attitude, and relative-measurement uncertainty into each correction. Attitude error produces larger position error at longer viewing distances.
- **Decision:** use an adapter motion/jump gate plus the EKF innovation gate. Neither filter estimate feeds the observation producer.
- **Decision:** compare baseline and assisted filters on identical UGV sensor messages inside each trial. Separate fresh-world runs can have different startup scheduling even with the same seed.
- **Results from synthetic-vector trials:**
  - Nine nominal trials have assisted RMSE 0.185-0.250 m.
  - Nine 91-second drift trials inject 15% wheel-speed scale error and 0.002 rad/s growing heading bias. Assisted RMSE is 0.204-0.219 m; GPS-dropout baseline RMSE is 0.943-1.393 m.
  - Three fault trials inject an 8 m outlier every tenth relative sample and a 20-second observation outage. Assisted RMSE is 0.320-0.388 m, with 31-38 jump rejections and recovery without a filter restart.
  - A flying-model trial with a synthetic independent anchor has 0.208 m assisted RMSE.
- **Failure recorded:** the actual PX4 anchor under the legacy magnetic configuration produces 1.172 m assisted RMSE and 0% uncertainty coverage. A timestamp-matched heading check measures a 17.94-degree mean signed UAV heading error. That finding motivates the dependency alignment in section 9.
- **Verification:** 74 registered checks report zero failures and three skipped cppcheck checks. A 10,000-sample Monte Carlo check agrees with propagated covariance diagonals within 5%. Callback tests cover timestamps, missing anchors, frames, disabled corrections, outliers, PX4 resets, and airborne initialization.
- **Documentation:** adds the measurement-design research and equations, retained result summaries and plot, shared `AGENTS.md`, and the `CLAUDE.md` import.
- **Limit:** these controlled vector trials do not validate pixel perception. Position-only UAV corrections do not solve heading drift; both strategies retain about 6.1-6.5 degrees mean heading error in the drift trials.
- **Evidence:** [measurement design](../cooperative_localization/uav_measurement_design.md), [frame/covariance math](../../ros2_ws/src/localization_common/localization_common/frames.py), [correction adapter](../../ros2_ws/src/cooperative_localization/scripts/uav_correction.py), [nominal results](../cooperative_localization/results/short_nominal/summary.csv), [drift results](../cooperative_localization/results/drift/summary.csv), and [fault results](../cooperative_localization/results/robust/summary.csv).

## 9. Align PX4, UAV models, and magnetometer settings

- **Commit:** [7f1902a — Align PX4 simulation dependencies and validate live anchors](https://github.com/cameroncho1/uav_ugv_project/commit/7f1902acac29454e8a37cf2b1496e93e2df6accd), 15:57 PDT.
- **Changed:** pins compatible PX4, ROS message definitions, and UAV models; both operational worlds select magnetic components in ENU and tesla, aligned with the upstream magnetometer fix.
- **Why:** heading and frame errors rotate relative positions into incorrect map locations. The anchor must be validated before judging fusion quality.
- **Decision:** check exact dependency revisions, minimum Gazebo version, consumed wire fields and message versions, world flags, and model noise. Derive the VehicleStatus topic suffix from its declared message version.
- **Decision:** use actual PX4 anchors after takeoff and a ten-simulation-second reset-free settling interval. PX4 samples and ROS consumers share simulation acquisition time.
- **Decision:** score estimated and true UAV heading over the same route window. Run PX4 under a terminal when launch redirects stdin, avoiding its console EOF loop.
- **Result:** nine live-anchor/vector trials have assisted RMSE 0.124-0.151 m and UAV heading RMSE 0.79-0.91 degrees. Startup without GPS improves in all three seeds.
- **Exception recorded:** seed 514 after GPS dropout has baseline RMSE 0.029 m versus assisted 0.134 m. Additional noisy corrections do not improve every trial.
- **Verification:** aligned PX4 and the full ROS workspace build. The commit records 83 registered checks, zero failures, and three skipped cppcheck checks; schema and world-setting tests reject incompatible inputs.
- **Limit:** the legacy mean signed heading error and aligned heading RMSE are different metrics under different configurations. They should not be presented as a precise percentage improvement. These trials still use synthetic relative vectors.
- **Evidence:** [dependency manifest](../../simulation/dependencies.json), [dependency checker](../../scripts/check_sim_dependencies.py), [setup and upstream fix](../simulation_setup.md), and [aligned trial results](../cooperative_localization/results/aligned_px4/summary.csv).

## 10. Drive navigation from guarded fused poses

- **Commit:** [a61f0e7 — Drive navigation from guarded fused poses](https://github.com/cameroncho1/uav_ugv_project/commit/a61f0e7e2c6e7433bd7ce23fd06714126d6cde60), 16:21 PDT.
- **Changed:** planner and controller consume `/ugv/localization/pose`, validate it through a shared guard, and hold when pose, map, or route is invalid. Commands use ROS `/ugv/cmd_vel` with a separate Gazebo bridge.
- **Why:** navigation needs one defined estimate and must stop when it is too old or uncertain.
- **Decision:** check both ROS acquisition age and steady-clock receipt age, each at most 0.5 seconds. A frozen `/clock` cannot keep an old pose valid indefinitely.
- **Decision:** require map frame, finite values, normalized quaternion, valid positive-definite x/y/yaw covariance, and monotonic samples. A clock rewind requires a fresh accepted sample.
- **Decision:** use the largest XY covariance eigenvalue, including correlation. Position standard deviation must be at most 0.5 m and yaw standard deviation at most 0.2 rad.
- **Decision:** start the EKF with x/y variance of `1e6 m^2`. Wheel and IMU updates constrain movement and heading but cannot prove an unknown global origin.
- **Decision:** publish a zero command every 100 ms during holds. Bound route age to one second and grid age to two seconds. A changed goal clears the cached route.
- **Decision:** centralize limits in YAML and separate path-following math from ROS callbacks. Standard `Twist` commands allow a hardware driver to use the same interface.
- **Result:** 19 real ROS integration scenarios pass, including valid motion, bad/stale/uncertain poses, correlated XY error, frozen clock, map loss, malformed/blocked grids, recovery, and arrival. Each hold requires only zero commands in its final half-second.
- **Result:** nine paired synthetic-vector trials with the large startup prior have assisted RMSE 0.184-0.250 m. Startup without GPS has assisted RMSE 0.185-0.242 m versus baseline 0.933-1.016 m.
- **Failure recorded:** the upstream 30 Hz camera goal trial fails to arrive within 300 wall seconds as the renderer/world stalls. Guards hold; true remaining goal distance is 7.762 m. That run does not establish arrival.
- **Verification:** the commit records 111 registered checks, zero failures, and six skipped cppcheck checks. Native tests cover pose guards, clock resets, path lookahead, and speed/turn bounds. The runner uses bounded cleanup of its own process groups.
- **Limit:** stop commands depend on the controller and transport still running. An actuator-side command timeout is needed for their failure. Valid map messages do not establish accurate obstacle classification.
- **Evidence:** [navigation rules](../navigation.md), [shared pose guard](../../ros2_ws/src/uav_ugv_msgs/include/uav_ugv_msgs/pose_guard.hpp), [navigation settings](../../ros2_ws/src/mission_manager/config/navigation.yaml), [guard results](../cooperative_localization/results/navigation/guards.json), and [failed upstream-camera trial](../cooperative_localization/results/navigation/goal_upstream_camera.json).

## 11. Use calibrated camera observations and retain shared uncertainty

- **Commit:** [625b5ea — Use calibrated camera observations for cooperative localization](https://github.com/cameroncho1/uav_ugv_project/commit/625b5ea2a0fefa20a3392e6f3b1e8c7f0371bb60), 19:27 PDT.
- **Changed:** the default demo and live camera trials derive relative UGV position from rendered AprilTag pixels and CameraInfo. Synthetic vectors remain an explicit fixture for controlled fault experiments.
- **Why:** validates the perception boundary instead of supplying the relative vector directly from simulated vehicle positions.
- **Decision:** the detector subscribes only to images and CameraInfo. The independent PX4 anchor enters through a separate timestamp-matched adapter. The detector has no truth, GPS, UAV odometry, or fused-UGV-pose input.
- **Decision:** use AprilTag 36h11 ID 7 with a 0.4 m black edge. Apply optical-to-body calibration and marker-to-chassis offset, preserve image acquisition time, and report position covariance. Missing tags produce no observation; old positions are not reused.
- **Decision:** keep numerical geometry separate from ROS callbacks. Evaluate square-specific and iterative PnP candidates under positive-depth, finite-geometry, size, and reprojection gates. Duplicate target IDs are rejected.
- **Decision:** model independent corner errors plus shared principal-point/focal-length errors, marker-size uncertainty, and mount position/angle uncertainty. Precise pixels do not imply perfect mounting or global localization.
- **Decision:** use an owned 5 Hz camera model inheriting the aligned X500. Exact mounting geometry and a 4.5 m altitude setpoint provide marker visibility across the eight-meter route. Native SDF tag visuals avoid renderer-specific texture materials. The mapper receives the camera topic through its existing parameter.
- **Decision:** explicitly select the UGV chassis canonical link and DiffDrive frame IDs in both worlds. Scorers reject incorrect wheel frames and require wheel measurements to continue after GPS loss.
- **Decision:** inject one persistent seeded position offset into both UGV GPS and the UAV anchor. A separate configured uncertainty prior remains in the public fused-pose covariance without changing the EKF's state or feeding it back into inputs.
- **Why for the prior:** relative vision cannot observe a translation shared by all absolute references. Repeated independent-noise updates can otherwise report high confidence in a biased global position. The prior represents externally known uncertainty; it does not estimate, remove, or detect unknown bias.
- **Results:**
  - Five actual rendered visibility cases pass: visible, partial occlusion, full occlusion, recovery, and off-axis target. Both tested occlusions produce zero observations. Maximum visible/recovery 3D error is 0.0230 m; off-axis maximum is 0.0113 m.
  - Nine valid paired live-camera trials have assisted position RMSE 0.096-0.157 m. Each strategy contributes 614-615 scored poses; each trial receives at least 50 camera-derived corrections and correctly framed wheel input.
  - GPS-denied startup cases contain zero prepared GPS messages. Dropout cases retain 398 wheel measurements after GPS loss.
  - Six valid fixed-observer/vector shared-error trials expose overconfidence. Seed 513 has assisted RMSE 1.862 m and 0% reported 95% coverage without the prior. With the 0.8 m prior, RMSE remains 1.875 m while coverage is 96.9%. The covariance changes; the global bias remains.
  - The GPS-denied camera navigation trial receives 717 wheel measurements, 41 relative observations, and 34 accepted corrections. Estimated goal distance is 0.478 m; physical goal distance is 0.607 m. It passes the declared 0.85 m physical criterion after 320 wall seconds.
  - The known shared-error hold trial has fresh perception/localization inputs, position standard deviation at least 0.851 m, and 1,217 zero commands. The hold is caused by uncertainty exceeding the navigation limit.
- **Verification:** PX4 and all nine ROS packages build. The retained verification record reports 140 registered checks, zero errors, zero failures, and six skipped cppcheck checks due to installed cppcheck 2.7. Twenty focused perception tests pass, including a 600-sample shared-principal-point covariance experiment.
- **Limit:** calibration is known simulation geometry, not a measured physical-camera calibration. The covariance is a local approximation conditional on correct target identity. The goal result does not establish 0.5 m physical arrival accuracy or real-time performance.
- **Evidence:** [perception design](../cooperative_localization/perception.md), [geometry implementation](../../ros2_ws/src/uav_sensing/scripts/fiducial_math.py), [sensor adapter](../../ros2_ws/src/uav_sensing/scripts/uav_fiducial_sensor.py), [calibration](../../ros2_ws/src/uav_sensing/config/camera_calibration.yaml), [camera trial results](../cooperative_localization/results/camera/anchors/summary.csv), [arrival](../cooperative_localization/results/camera/goal.json), [hold](../cooperative_localization/results/camera/hold.json), and [verification](../cooperative_localization/results/camera/verification.json).

## What research and iteration caught

- **Off-axis detection failure:** default detector settings miss a rendered tag. Twenty parameter trials identify adaptive threshold constant 15 as the successful tested change. The detector tries constants 7 and 15 with the same dictionary and geometry checks. The failing image remains a regression fixture. See [detector research](../cooperative_localization/results/camera/detector_research.json).
- **Front-facing square solver failure:** an exactly front-parallel square exposes a numerical singularity in the square-specific solver. The additional iterative candidate and a known-geometry regression test cover that case.
- **Wheel-frame integration fault:** merging the roof marker changes Gazebo's default canonical link to the marker. DiffDrive emits the marker child frame, which the EKF cannot transform into the chassis frame. Camera corrections can mask the missing wheel input. Explicit frame configuration fixes the contract, all affected comparisons are repeated, and earlier runs remain labeled diagnostic data. See [frame diagnosis](../cooperative_localization/results/camera/frame_diagnostic/conditions.json).
- **Renderer stalls:** the upstream-camera run fails arrival and exercises freshness holds. The owned 5 Hz configuration completes the later arrival trial. Time allowances accommodate software rendering; they do not relax acquisition-age, uncertainty, or sensor-count requirements.
- **Hold scoring at a stale deadline:** two retained snapshots fail their evidence requirements despite zero motion. The scorer completes when all sensor-count, uncertainty, and freshness criteria are met within a bounded window. No quality threshold is lowered. See [receipt diagnostic](../cooperative_localization/results/camera/hold_receipt_diagnostic.json) and [observation diagnostic](../cooperative_localization/results/camera/hold_observation_diagnostic.json).
- **Anchor startup timeout:** a seed-515 initialization attempt produces no accepted accuracy score. Its logs remain recorded separately from the successful camera matrix. See [startup conditions](../cooperative_localization/results/camera/startup_diagnostic/conditions.json).
- **Shared global bias:** the bias experiment confirms that camera-relative accuracy does not guarantee global accuracy. The prior makes reported uncertainty more honest while preserving the measured failure to remove the shared translation.

## How to judge confidence in the implementation

- **Check the boundaries:** camera perception receives pixels and calibration; correction receives an independent anchor and relative observation; navigation receives only the public fused pose. Truth stays in simulation synthesis and scoring.
- **Check the math:** known projection cases test metric geometry and axes. Rotation round trips test NED/FRD conversion. The 10,000-sample experiment checks anchor/relative covariance propagation within 5%; the 600-sample perception experiment checks shared principal-point covariance within its stated 35% tolerance.
- **Check the actual processes:** the 19 navigation scenarios exercise real ROS planner/controller nodes. Rendered occlusion moves an opaque visual in front of the actual camera. These checks cover behavior beyond isolated math functions.
- **Check fair comparisons:** paired filters receive the same UGV sensor messages inside a trial, use matching truth timestamps, and record actual sensor counts. Fresh-world comparisons across configurations are not identical sensor sequences.
- **Check complete integration:** the GPS-denied goal trial uses the camera, actual PX4 anchor, wheel/IMU fusion, planner, controller, and command bridge together. Physical distance is scored separately from estimated distance.
- **Check failures as well as successes:** legacy-heading, wheel-frame, rendering, startup, and scoring failures remain labeled with their conditions. They are not counted as successful intended-system comparisons.
- **Check the limits on uncertainty claims:** 100% ellipse coverage in the nine camera trials indicates conservative uncertainty under those conditions. Temporally correlated samples and three seeds do not prove calibrated 95% confidence or hardware reliability.
- **Reproduce in Linux:** [simulation setup](../simulation_setup.md), [interfaces](../cooperative_localization/interfaces.md#evaluation-and-reproducibility), [perception commands](../cooperative_localization/perception.md#research-and-verification), and [navigation commands](../navigation.md#scoring-and-verification) document builds and runners. Retain new results in a separate output directory with their conditions.

## Remaining limits

- Unknown position bias shared by GPS and the UAV anchor remains unobservable from relative sensing alone. The uncertainty prior must come from an external error model or calibration.
- The tested camera observes a marked UGV with known simulation intrinsics and mounts. Physical calibration, motion blur, damaged tags, weather, complex terrain, and arbitrary moving occlusion are unvalidated.
- Synthetic wheel-scale and IMU-heading faults test measurement errors. They do not establish behavior under physical wheel slip or uneven terrain.
- The mapper remains Cameron's luminance heuristic. Valid map messages and successful flat-world arrival do not establish reliable obstacle detection or camera-grid calibration.
- Continuous stop commands protect the running control loop. Controller death or command-transport loss requires an actuator-side timeout.
- Additional assistance does not improve every trial, particularly when absolute references share a bias or a short-route baseline has a favorable initial fix.

## Next steps

1. Add an actuator command timeout, then test controller termination and command-transport loss.
2. Add an independent global reference if navigation must remain accurate under shared GPS/UAV bias; define the map alignment and uncertainty assumptions.
3. Test moving occlusion and representative physical obstacles with the mapper owner, retaining accuracy, hold, recovery, and failed-run evidence.
4. Measure physical camera intrinsics, mounting geometry, and uncertainty before hardware deployment; repeat the perception and fusion evaluations with those measurements.
