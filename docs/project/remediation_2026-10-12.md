# October 12 audit remediation

- Requested by Rylan Wade: fix all findings in [the audit](review_2026-10-12.md), create one detailed local commit, and do not push.
- Audited implementation: `c87bf1b`. Requirements baseline: `65ba304`; the relevant baseline documents match `2423af5`, before the localization implementation.
- The historical audit/evidence and October 11 results remain identifiable as historical. The results linked here come from the repaired implementation.
- No original Linux checkout or unrelated working-tree edits are used as the implementation under test.

## Findings and implemented fixes

| ID | Problem / reason to care | Implemented fix and owner | Verification |
| --- | --- | --- | --- |
| F01 | Clock rewind can crash correction queue processing and remove assistance. | Localization observes clock epochs before processing and detaches the pending batch. | Multiple queued readings, rewind, and resumed matching in `test_uav_correction.py`. |
| F02 | Discarded GNSS covariance hides degraded fixes; ungated innovations can move the estimate. | Localization retains and rotates known covariance, rejects invalid known covariance, uses a stated prior only for unknown covariance, and configures a five-sigma GNSS innovation gate. Simulation adds uncertainty upstream. | Adapter regressions; actual EKF [startup/degradation/outlier/recovery](remediation_2026-10-12_results/gnss_innovation.json). |
| F03 | Missing IMU orientation becomes a confident heading observation. | Localization preserves unavailable markers, validates/normalizes orientation, and independently retains valid rate/acceleration components. | Unavailable orientation, zero/nonfinite quaternion, known covariance and invalid-component regressions. |
| F04 | A delayed route for a previous goal can authorize motion. | Mission preserves a goal revision stamp; planning publishes `PlannedRoute` with goal, path and clearance grid. Control requires the matching revision/coordinates and endpoint. | Message/header unit tests and delayed-obsolete-route/recovery cases against the actual planner/controller. |
| F05 | A deadline during stop dwell or stale final evidence can falsely pass arrival. | Evaluation requires an explicit complete continuous fresh one-second stopped interval, estimated tolerance, physical tolerance and fresh final receipts. | Deadline/reset dwell tests and scored physical arrivals. |
| F06 | Camera mode defaults select incompatible world/model/anchor inputs. | Evaluation derives compatible camera defaults and rejects camera/synthetic-anchor combinations before starting nodes. | Launch-resolution regressions; actual camera/PX4 mission assembly. |
| F07 | Partition/domain mismatches disconnect the simulated camera and PX4 state. | Integration preserves `GZ_PARTITION`; UAV scripts and mission launch derive PX4's DDS domain from `ROS_DOMAIN_ID`. | Camera fixture in domain 101/custom partition; camera/PX4 goal in domain 102/custom partition. |
| F08 | Unbounded waits or surviving helper sessions hang evaluation. | Runners track scorers as owned groups, wait with interruptible shell builtins, and bound INT/TERM/KILL escalation. Cleanup discovers descendant groups before terminating parents, including PTY sessions. | Registered resistant-process and separate-session descendant tests; trial cleanup and owned-process inspection. |
| F09 | Old images are treated as clock resets and bypass monotonic/rate checks. | Sensing tracks ROS-clock progression separately, rejects duplicate/out-of-order images, and resets only on a real clock rewind. Invalid future images do not advance the watermark. | Reordering, true rewind/recovery, five-Hz boundary and future-image regressions. |
| F10 | Heading scoring fabricates coverage through endpoint interpolation. | Evaluation requires ordered finite acquisition logs, estimate time coverage and gaps no greater than 0.1 s; summaries count rejected samples. | Empty/truncated/duplicate/reversed/nonfinite/gapped numerical log tests. |
| F11 | A fresh grid stamp can conceal stale UAV projection state. | Mapping consumes ROS images/CameraInfo and independent ENU anchors matched at image acquisition time, applies calibrated extrinsics, and preserves image stamps. Missing coverage expires without a map. | Projection/origin/axis tests; anchor-loss and matched-UGV-mask tests; actual rendered camera navigation. |
| F12 | Endpoint snapping, corner shortcuts and point-only planning do not establish footprint clearance. | Planning rejects blocked endpoints, inflates occupied/unknown/boundary cells for a 0.5 m enclosing radius plus 0.05 m clearance, and checks every connection. Control requires visible lookahead and clear predicted motion, turning before translating through large heading error. | Inflation, blocked endpoint, diagonal/corner/bend tests; real planner/controller guard suite. |
| F13 | Undeclared Gazebo dependencies break metadata-based setup and burden the estimator. | Integration declares Harmonic dependencies where used and supplies a Jammy [rosdep overlay](../../simulation/rosdep.yaml). Localization and mapping no longer link Gazebo. | `rosdep update` and dependency-resolution dry run; Linux builds of all 11 first-party packages. This is not a fresh-OS installation test. |
| F14 | Localization owns PX4 integration promised to the UAV interface subsystem. | `uav_control` owns independent anchor publication, PX4 sample-clock/reference-reset policy, and its configuration/launch/tests. Localization consumes `/uav/localization/pose`. | Moved timing/reset tests; actual PX4 camera run. No fused-UGV feedback into the producer. |
| F15 | Camera detection/calibration becomes an estimator responsibility. | Separate `uav_sensing` owns the detector, calibration, sensor TF, bridge and fixtures. Mission/evaluation selects the replaceable producer. | Moved camera unit/Monte Carlo tests; rendered visibility/occlusion/recovery/off-axis fixture. |
| F16 | Synthetic corruption, truth and test controls enter the normal estimator path. | `simulation_support` owns raw bridges/truth/clock/geometry; `fault_management` owns sensor corruption, outages, synthetic observations and route-driving evaluation. The normal estimator launch starts none of these. | Simulated sensor tests; paired matrices; wall-clock GNSS fixture uses the operational launch without Gazebo/PX4. |
| F17 | Rewritten implementation docs obscure original requirements and ownership. | Stable sensing-independent requirements, exclusions and ownership are restored. New ownership choices are recorded separately; setup/current links and historical result labels are updated. | Baseline contract comparison and local link/reference checks. |
| F18 | Numerical frame/geodesy code requires importing a ROS callback module. | `localization_common` contains transport-independent numerical frame, geodesy and covariance functions; callbacks only adapt messages. | Pure axis/geodetic round trips and existing covariance/lever-arm/Monte Carlo tests. |
| F19 | Goal distance alone does not measure promised route tracking. | Evaluation records acquisition-matched truth against the most recent planned polyline, route snapshots, cross-track RMSE/mean/p95/max, traveled distance and fixed-goal distance reduction. | Three seeded baseline/assisted closed-loop comparisons, with CSVs and route snapshots retained. |
| F20 | Controller/bridge disappearance leaves the last actuator command active. | `ugv_control` supplies a Gazebo actuator-side watchdog that uses a steady clock and expires commands after 0.35 s with a 50 ms publication tick. Both worlds bridge to its input, not directly to DiffDrive. | Actual controller death and bridge loss: [watchdog results](remediation_2026-10-12_results/watchdog/summary.json). |

## Ownership and compatibility

- `cooperative_localization`: validated measured sensor adaptation, correction matching, EKF configuration and public fused-pose/covariance output. Its operational launch defaults to wall time and baseline-only fusion.
- `uav_control`: independent PX4 state and flight control. `uav_sensing`: camera-specific relative-observation production. `localization_common`: pure reusable numerical operations.
- `simulation_support`: raw Gazebo observations, truth readers, world geometry and shared `/clock`. `fault_management`: explicit simulation fault/noise models, test outages and evaluation assembly/scoring.
- Planning/control use `/ugv/navigation/route` (`uav_ugv_msgs/PlannedRoute`) to authorize motion; `/local_path` remains available for display and scoring. A stable positive goal stamp identifies a revision.
- Normal sensor inputs are `/ugv/sensors/{gps,imu,wheel_odom}`. Evaluation's measured-GPS gate is `/evaluation/set_gps_enabled`; it does not disable raw simulator GPS. Launch commands for truth/synthetic producers now use their owning packages.
- Numerical frame and timestamp conventions remain ENU `map`, FLU ROS body frames and original acquisition times. PX4 NED/FRD conversion includes covariance and the stated origin. Simulator truth stays in synthesis/scoring.

## Validation conditions and results

- Persistent isolated Linux workspace: `/home/rylanwwade/uav-fix-20261012`; ROS 2 Humble, Ubuntu 22.04/WSL, Gazebo 8.15.0, OpenCV 4.5.4 and robot_localization 3.5.4.
- PX4 `0462e3a31e7f0a6f6f05a5e094bee8a52e8759ec`, matching `px4_msgs` `109e8c8e151938f03fee2e62e3e93dc5ff5e2281`, models `a15af9628536914ff7201c992fce5e3cb5d70db9`. The full PX4 SITL target builds in the isolated workspace.
- All 11 first-party ROS packages build. ROS validation reuses the prior isolated audit's six-consumed-type PX4 message underlay generated from the pinned schemas; unchanged `px4_msgs` is skipped. This does not claim a fresh complete message build in this validation run.
- Registered tests: **155 checks, zero errors, zero failures, seven skipped**. [Validation record](remediation_2026-10-12_results/validation.json) retains commands, conditions, source hashes, skips and the relation of each matrix to final edits. Seven skipped entries are cppcheck cases disabled by the installed ament tooling for cppcheck 2.7 performance issues; they are not behavioral tests.
- GNSS integration starts wheel/IMU before an initial map position `(1200, -400)` m, preserves a reported `100 m²` variance, rejects a `1000 m` innovation, and recovers on valid fixes. The fixture exercises the actual operational adapter and robot_localization EKF.
- Navigation checks run the actual planner/controller and cover stale/frozen/invalid/uncertain inputs, blocked endpoints, delayed obsolete routes and recovery. Watchdog checks terminate the actual controller or command bridge, then measure physical motion beyond the expiry window.

| Paired evaluation | Conditions | Baseline position RMSE (m) | Assisted position RMSE (m) | Evidence |
| --- | --- | ---: | ---: | --- |
| Nine nominal trials | 31 s route; seeds 513/514/515 × GPS on/off/startup-off; scores after 10.5 s | 0.111–1.030 | 0.185–0.240 | [Summary](remediation_2026-10-12_results/paired_nominal/summary.csv) |
| Nine drift trials | 91 s route; same seeds/GPS cases; 15% wheel scale and 0.002 rad/s growing heading bias | 0.383–1.761 | 0.206–0.221 | [Summary](remediation_2026-10-12_results/paired_drift/summary.csv) |
| Three fault trials | 91 s drift route; GPS off at 10 s; 8 m relative outlier every tenth reading; relative dropout 35–55 s | 1.136–1.272 | 0.413–0.499 | [Summary](remediation_2026-10-12_results/paired_robust/summary.csv) |

- Both filters receive identical measured sensor messages inside each paired world. GPS-disabled cases record zero measured GPS after the gate; wheel/IMU continue. Nominal strategies have 588–616 matched samples; drift 2,375–2,416; fault 2,360–2,404.
- Assistance improves all drift/fault cases. The nominal seed-515 GPS-off baseline is better than assistance; short reliable-wheel scenarios do not establish universal improvement. Noise, timing, seeded faults and shared-reference alignment are simulation conditions.
- Closed-loop comparisons use fresh worlds with the same configured seed/map/goal, a declared free 20 × 20 m grid with 0.25 m cells, start `(1, 0)`, goal `(4, 0)`, drift measurements and fixed-observer relative observations. GPS stops no earlier than two simulation seconds after first motion and eleven received fixes. Trajectories/sensor sequences differ under closed-loop control; this is not an identical-message comparison.
| Seed | Strategy | Cross-track RMSE (m) | Goal error (m) | Goal-distance reduction | Confirmed arrival |
| ---: | --- | ---: | ---: | ---: | --- |
| 513 | baseline | 0.420 | 2.402 | 19.9% | No: uncertainty hold |
| 513 | assisted | 0.145 | 0.470 | 84.3% | Yes |
| 514 | baseline | 0.121 | 2.343 | 21.9% | No: uncertainty hold |
| 514 | assisted | 0.125 | 0.333 | 88.9% | Yes |
| 515 | baseline | 0.178 | 2.257 | 24.8% | No: uncertainty hold |
| 515 | assisted | 0.131 | 0.419 | 86.0% | Yes |

- All three assisted runs arrive; all three baselines hold after uncertainty grows. [Tracking summary, per-run CSV and route snapshots](remediation_2026-10-12_results/path_tracking/summary.json) preserve failures as measured outcomes.
- Cross-track distance is physical point-to-polyline distance, including route endpoints. The reference is the latest planned route at or before truth acquisition, at most one second old. Empty/expired routes contribute skip counts, not fabricated errors. Completion is fractional reduction in physical distance to the fixed goal, not arc-length traversal or a collision guarantee. Holding baselines therefore have fewer scored samples; low error over a short partial route does not prove better navigation.
- The rendered fixture passes visible, partial occlusion, full occlusion, recovery and off-axis cases. Occlusions produce no relative observations; nominal and off-axis 3D errors are approximately 0.023 m and 0.011 m. [Fixture results](remediation_2026-10-12_results/camera_fixture/results.json).
- The actual PX4/camera goal trial starts with GPS disabled and confirms stopped arrival in 69.5 wall seconds. Physical goal error is 0.549 m against the unchanged 0.85 m tolerance; cross-track RMSE is 0.064 m over 1,541 matched samples. It records 71 relative observations, 58 corrections and zero GPS fixes. [Goal evidence and route traces](remediation_2026-10-12_results/camera_goal/results.json).
- Camera mapping is a flat-ground luminance heuristic. The known robot's 0.5 m radius / 0.5 m height cylinder and cell coverage remove its projected roof/body silhouette only with a timestamp-matched bounded-uncertainty public pose. The goal's visual-only ring is brighter than the dark-obstacle threshold. These fixture decisions do not establish terrain classification or detection of obstacles occluded by the robot.
- An early tracking run reached the goal but lacked the required GPS sample count before its requested outage. [Diagnostic result](remediation_2026-10-12_results/diagnostics/tracking_513_assisted_insufficient_gps.json) remains separate from the final benchmark; the arrival/evidence thresholds were retained. The subsequent harness waits for the required sample count before disabling GPS.
- An initial camera navigation run held on projected robot/goal visuals after removal of unsafe endpoint snapping. It was stopped for diagnosis; the projection and visual-only goal annotation are corrected instead of relaxing blocked-endpoint checks.
- Per-trial JSON, comparison summaries, tracking CSV/routes and guard/watchdog summaries are committed here. Full paired per-sample CSVs, simulator/launch logs and PX4 logs remain in the persistent Linux workspace's `evaluation/` directory. `validation.json` records final checks and source hashes; earlier matrix code differences are noted there.

## Reproduce

```bash
source /opt/ros/humble/setup.bash
# Register the project rosdep overlay as documented in ../simulation_setup.md.
rosdep install --from-paths ros2_ws/src --ignore-src --rosdistro humble --simulate
cd ros2_ws
colcon build --symlink-install
colcon test --executor sequential
colcon test-result --verbose
source install/setup.bash
cd ..

ROS_DOMAIN_ID=103 python3 scripts/test_gnss_innovation.py /tmp/gnss_innovation.json
ROS_DOMAIN_ID=97 python3 scripts/test_navigation_guards.py /tmp/navigation_guards.json
ROS_DOMAIN_ID=98 python3 scripts/test_actuator_watchdog.py /tmp/watchdog
ROS_DOMAIN_ID=100 python3 scripts/run_path_tracking_trials.py /tmp/path_tracking --timeout 90
ROS_DOMAIN_ID=101 GZ_PARTITION=fix_camera_fixture \
  scripts/run_camera_perception_trial.sh /tmp/camera_fixture
ROS_DOMAIN_ID=102 GZ_PARTITION=fix_camera_goal \
  scripts/run_goal_navigation_trial.sh /tmp/camera_goal

ROS_DOMAIN_ID=99 GZ_PARTITION=localization_fix_nominal \
  scripts/run_cooperative_trials.sh /tmp/paired_nominal
ROS_DOMAIN_ID=111 GZ_PARTITION=localization_fix_drift \
  ROUTE=long DURATION=91 MEASUREMENT_PROFILE=drift \
  scripts/run_cooperative_trials.sh /tmp/paired_drift
ROS_DOMAIN_ID=112 GZ_PARTITION=localization_fix_robust \
  GPS_CASES=off ROUTE=long DURATION=91 MEASUREMENT_PROFILE=drift \
  OUTLIER_EVERY=10 DROPOUT_START=35.0 DROPOUT_END=55.0 \
  scripts/run_cooperative_trials.sh /tmp/paired_robust
```

- Use persistent directories for retained evidence; WSL can clear `/tmp` on restart. Camera runs require the Linux display/rendering session and the pinned PX4/model build. Runners stop their owned processes with bounded escalation.
- Remaining limits are explicit: flat simulated ground, known calibration/geometry, temporally correlated measurements, configurable global-reference uncertainty, no hardware tests and no production fault-classification policy. A simulated watchdog protects the Gazebo actuator; a physical driver needs the corresponding actuator-side expiry implementation.
