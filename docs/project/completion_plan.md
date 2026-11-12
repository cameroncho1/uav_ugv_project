# Completion plan

Team: Sophie Siemsgluess, Rylan Wade, Ethan Palosh, Jeremy Yin, Cameron Cho.

[Documentation index](../README.md) | [Project overview](project_overview.md) | [Technical plan](technical_plan.md) | [System diagram](../system_design.html)

Plan baseline: 2026-11-03; W10 update: 2026-11-11. Final report: 10 pages in the supplied Fall 2025 course introduction. Applicable presentation/submission dates require the current course instructions; the supplied slides do not establish 2026 deadlines. See [course source provenance](../verification/supervisor_course_alignment.md).

This plan defines the final demonstration, the requirements it is checked against, the verification methods, and the work remaining. Owners and dates are proposed and open for team confirmation (section 10).

## 1. Scope

### 1.1 Final demonstration

One UAV and one UGV in a Gazebo world with static obstacles. The UGV drives to a goal under degraded GPS. The system provides:

- UAV-assisted UGV localization with covariance.
- A camera traversability grid, A* route planning with footprint clearance, and path following.
- One dynamic hazard: a box that appears on the route during the run, which forces a replan or a stop.
- A learned sensor-fault detector that runs online and drives a supervisor hold.
- Deterministic guards (stale pose, pose uncertainty, command expiry) that hold the UGV independent of the detector.
- A runtime-monitored, falsification-tested, partly model-checked safety argument over all of the above.

### 1.2 Changes from the proposal

| Proposed | Completion scope |
| --- | --- |
| UAVs survey the route ahead | One UAV follows the UGV fused pose at camera height. Scouting ahead is not built. |
| Multi-UAV scouting | Not built. |
| Terrain classification (slope, mud, water) | Not built. The mapper is a camera-luminance heuristic. Hazards are dark boxes on flat ground. |
| Energy-aware global routing | Not built. Path length and traversal time are reported as the energy proxy. |
| Dynamic hazards trigger replanning | One appearing box per run. |
| Sensor-fault detection | Learned online detector (section 4) plus deterministic guards. Mechanical and actuator fault diagnosis is not built. |
| Evaluation across route length, terrain, obstacle density | Obstacle layout (4), route length (2), hazard (2), fault class (5). Terrain is flat. |

## 2. Baseline on `main`

- Implemented: PX4 SITL and Gazebo Harmonic world; EKF localization with wheel, IMU, GPS and UAV measurements; camera heading; luminance mapper with map memory; A* planner with partial routes; path follower; actuator watchdog; UAV follow; four generated obstacle worlds. See [navigation](../navigation.md) and [localization](../cooperative_localization/README.md).
- Verification: reusable R1-R8 evaluation, frozen experiment campaigns, headless demo/replay, source-bound supervisor checking, and scheduled dynamic obstacles with camera-observation scoring. See [verification](../verification/README.md), [dynamic obstacles](../dynamic_obstacles/README.md), and [combined validation](results/integration/README.md).
- Measured: nine paired short-route localization trials and goal-navigation trials on five courses. The localization results are dated 2026-10-11 and predate the planner, mapper and world changes ([evaluation results](../cooperative_localization/evaluation_results.md)). They are rerun in this plan.
- Not present: a learned detector in the ROS graph, detector-driven closed-loop evaluation, and expanded dynamic-hazard coverage across the full scenario matrix.

## 3. Operating envelope

The envelope bounds every requirement in section 5. A scenario outside it is recorded and excluded from pass/fail counts, not silently dropped.

| Parameter | Value | Source |
| --- | --- | --- |
| UGV footprint radius, planner clearance | 0.50 m, 0.05 m | `local_astar_planner` |
| UGV maximum linear / angular speed | 0.5 m/s, 1.0 rad/s | `mission_manager/config/navigation.yaml` |
| Controller completion / R2 estimated arrival | 0.35 m / 0.5 m | `navigation.yaml` / [R2](../verification/requirements.md#r2-goal-arrival-with-dwell) |
| Pose guard: age, position stddev, heading stddev | 0.5 s, 0.5 m, 0.2 rad | `navigation.yaml` |
| Grid age limit, path age limit | 2.0 s, 1.0 s | `navigation.yaml` |
| Actuator command expiry | 0.35 s | `command_watchdog.cpp` |
| UAV altitude | 4.5 m | [sensor dataset](../sensor_dataset.md) |
| GPS horizontal / vertical stddev | 1.5 m, 3.0 m | `measurement_model.yaml` |
| Wheel-speed scale error (fault) | 1.15 | `measurement_drift.yaml` |
| IMU yaw bias rate (fault) | 0.002 rad/s | `measurement_drift.yaml` |
| Shared position error | 0 to 0.8 m | `simple_goal_demo.launch.py` arguments |
| Ground | flat, luminance-separable obstacles | world files |
| Obstacle layouts | sparse, dense, corridor, narrow | `simulation/worlds/simple_goal_demo_*.sdf` |
| Goal courses | UGV start (-4, 0); goals (4, 0) and (9, 3) | [goal demo](../../README_simple_goal_demo.md); commit `4d94701` |
| Hazard (initial values) | 0.6 m dark box, appears at time `t_h` in [5 s, 0.8 T_nominal], at distance `d_h` in [1.0 m, 3.0 m] ahead of the UGV on its route | to be set in W1 |
| Hazard validity | `d_h >= d_min = v_max (tau_obs + tau_r) + r + c` | section 5, R5 |

Noise and fault values are simulation assumptions. Results apply to these conditions only.

## 4. Fault-detection subsystem

### 4.1 Current state

Code: [`uav_ugv_project_fault_detection/`](../../uav_ugv_project_fault_detection/README.md), added in `0a887cf` (JeremyYin-99, 2026-10-16). Sensor and simulation reference: [`SensorsDocs.md`](../../uav_ugv_project_fault_detection/SensorsDocs.md).

Implemented:

- Three PyTorch models: an autoencoder (one linear encoder, one linear decoder, ReLU, sigmoid output; reconstruction error is the anomaly score), a 2-layer RNN classifier and a 2-layer BiLSTM classifier. Hidden size 50, input length 100, Adam at 0.001, 100 epochs, batch 32 (`models/utils/ModelParameters.json`).
- A trainer with periodic checkpoints, best/final weights and `loss_history.json`.
- Class-weighted cross-entropy for RNN and BiLSTM.
- A pipeline over the CMU ALFA UAV fault dataset (link in the README): one sensor channel (`mavros-nav_info-yaw`, `field.measured`) per flight, resampled to 100 samples, z-scored, in time or FFT-magnitude form. 46 scenarios are used: 36 faulty, 10 normal. The label is per flight.
- A 20% scenario-level validation split (about 9 scenarios) with a fixed seed.
- Precision, recall, F1 and accuracy (`metrics.py`).
- Weights and loss plots for both domains, committed.

Observed in the committed artifacts:

- `loss_history.json`, final epoch, train / validation loss:

| Model, domain | Train | Validation |
| --- | ---: | ---: |
| RNN, time | 0.271 | 1.107 |
| RNN, frequency | 0.018 | 2.306 |
| BiLSTM, time | 0.488 | 0.411 |
| BiLSTM, frequency | 0.154 | 1.363 |
| Autoencoder, time | 0.657 | 0.799 |
| Autoencoder, frequency | 88.24 | 88.60 |

- The frequency-domain autoencoder loss moves from 94.6 to 88.2 over 100 epochs. The decoder uses a sigmoid output, which limits outputs to (0, 1); the z-scored time inputs and FFT magnitudes fall outside that range.
- The autoencoder threshold is the maximum error over the normal windows of the set being evaluated.
- No precision/recall/F1 values are stored in the repository. `main.py` prints them.

Not implemented:

- Any ROS 2 interface. There is no inference node, no topic input and no output.
- Online operation. Each input is a whole flight resampled to 100 points.
- Training data from this simulator. ALFA records flights of a different aircraft with engine and control-surface failures. The simulator has no such faults, and the demo's UGV sensors are not in ALFA.
- Reproducibility from a clean clone. `datasets/` is gitignored, and the dataset is not included.
- Any use of the output by navigation.

### 4.2 Integration design

The existing models, trainer and metrics are kept. The data source, window definition and runtime interface change.

**Data.** Record simulator runs with the procedure in [sensor dataset collection](../sensor_dataset.md). Each run records the UGV sensor topics, the estimator output and the truth topic. The fault schedule (class, onset, end) is written to the run directory as `faults.json` and is the label source. Runs are split into train, validation and test by run, never by window. Initial target: 60 labeled runs (20 fault-free, 10 per fault class), seeds disjoint across splits.

**Inputs.** The detector reads cross-sensor residuals, not a raw channel:

| Channel | Definition |
| --- | --- |
| `r_gps` | GPS position minus fused position (m) |
| `r_yaw` | wheel-odometry yaw rate minus IMU yaw rate (rad/s) |
| `r_speed` | commanded speed minus wheel-odometry speed (m/s) |
| `r_uav` | UAV-relative position minus fused position (m); masked when no measurement |
| `valid_uav`, `valid_gps` | message-fresh flags |

Windows: 100 samples at 20 Hz (5 s), stride 10 samples (0.5 s). Models take `(100, C)` input; the existing `input_dim` parameter and flattened autoencoder input already follow the channel count.

**Output.** W10 implements `uav_ugv_msgs/FaultStatus`: schema version, acquisition/window-end stamp, source-session identity, sequence, window validity, and four per-class flags. W9 supplies the future `fault_detector` producer using acquisition timestamps and `/clock`, consuming measurements only. Model scores can be separate diagnostics; the supervisor consumes the versioned decision contract.

**Baseline.** A residual-threshold detector (per-channel CUSUM or fixed multiple of the nominal residual stddev) runs on the same data. The learned detector is selected over it only if it has equal or better detection and false-alarm results on the test runs.

**Model selection.** RNN, BiLSTM and autoencoder are trained on the same splits. Selection uses validation runs. The test runs are scored once.

**Weights and code.** Training stays in `uav_ugv_project_fault_detection/`. The node lives in a new `fault_detection` package that loads the selected weights. `AGENTS.md` component ownership is updated to list it.

### 4.3 Fault classes and responses

| Class | Injection | Control | Detector | Supervisor response |
| --- | --- | --- | --- | --- |
| F1 GPS outage | GPS gate | `/evaluation/set_gps_enabled`, `initial_gps_enabled` | flag `gps_out` | continue; estimator uses wheel, IMU, UAV |
| F2 Wheel-speed scale | scale on wheel odometry | `wheel_speed_scale` | flag `odom_scale` | hold |
| F3 IMU yaw drift | bias rate on IMU yaw | `imu_yaw_bias_rate` | flag `imu_drift` | hold |
| F4 UAV measurement loss | gate on `/ugv/sensors/uav_relative_pose` | new `uav_dropout` control (W1) | flag `uav_loss` | continue if GPS valid; otherwise the pose guard holds |

A shared GPS/UAV position error is not detected by cross-sensor residuals. It is covered by the systematic uncertainty prior and the pose guard, and it is recorded as an undetected class.

### 4.4 Supervisor

W10 implements the persistent supervisor in the goal controller. The [technical specification](../verification/supervisor.md) defines the shared priority policy, input validation, recovery memory, goal revisions, and clock-reset assumptions. The detector producer remains W9 work; detector participation defaults to disabled and is fixed at startup.

| State | Entry |
| --- | --- |
| `INIT` | no accepted active goal; clock reset |
| `NAVIGATE` | pose valid, route valid, no hold flag |
| `HOLD_GUARD` | stale pose, pose stddev above limit, stale route or grid, invalid route |
| `HOLD_FAULT` | detector flag in {F2, F3}, or F1/F4 with the pose guard failing |
| `GOAL` | completion latched from a valid estimated pose; persists until a new accepted goal revision |

Validated inputs include active/new goal, full pose/route validity, estimated goal attainment, distinct clear/alarm/unavailable detector events, recovery maturity, and clock reset. Only `NAVIGATE` permits motion; all other modes publish zero each command tick. Fault recovery requires ten distinct valid clear decisions **and** at least five simulation seconds from the first clear acquisition stamp. New goals clear completion and invalidate old routes without clearing fault recovery. Missing or invalid enabled-detector evidence holds. Completion is an estimated-distance decision; it does not establish R2 physical arrival/dwell.

The deterministic guards (`HOLD_GUARD`) do not read the detector. R1 and R3 are evaluated with the detector enabled, disabled, and forced to miss.

## 5. Requirements

Signals come from ROS topics or the simulator truth. Truth is used for scoring only. Time is simulation time. Initial constants are marked and calibrated from measurements in W2.

| ID | Requirement | Formal form |
| --- | --- | --- |
| R1 | The UGV footprint never intersects an obstacle. | `G[0,T] (d_clear >= 0)` |
| R2 | In scenarios with no fault or only F1, the UGV reaches the goal within the time bound. | `F[0,T_goal] (dist_goal <= 0.5)` |
| R3 | When the pose guard fails, the velocity command is zero within `tau_g` and stays zero until the guard clears. | `G ((pose_age > 0.5 \| sigma_pos > 0.5 \| sigma_yaw > 0.2) -> F[0,tau_g] (v_cmd = 0))` |
| R4 | When commands stop arriving, actuator speed is zero within `tau_w`. | `G (cmd_age > 0.35 -> F[0,tau_w] (v_act = 0))` |
| R5 | When a hazard becomes visible on the route (`d_h >= d_min`), a route that avoids it is published or the UGV is stopped within `tau_r`. | `G (hazard_in_grid -> F[0,tau_r] (route_clear \| v_cmd = 0))` |
| R6 | With a fresh UAV measurement and a valid pose, position error stays below 0.5 m. | `G ((uav_fresh & pose_valid) -> e_pos <= 0.5)` |
| R7 | After onset of F2, F3 or F4, the detector raises the matching flag within `tau_d`. | `G (onset(Fk) -> F[0,tau_d] flag_k)` |
| R8 | In fault-free runs the detector raises no flag. | `G (!fault_active -> !flag)` over `N` runs; reported as an upper bound |

Constants: `T_goal = 2 L / 0.5 + 30 s` for route length `L` (initial); `tau_g = 0.5 s`; `tau_w = 0.35 s + 1 control tick`; `tau_r = 2 s`; `tau_d = 5 s`; `d_min = 0.5 (tau_obs + tau_r) + 0.5 + 0.05` with `tau_obs` the measured map-update latency. `d_clear` is the distance from the UGV footprint disk to the nearest obstacle surface, from simulator truth.

R1 and R3 are the safety requirements. R2, R5 and R6 are performance requirements. R7 and R8 apply to the detector and are statistical (section 6.4).

## 6. Verification and assurance

### 6.1 Runtime monitors on recorded traces

- Every trial records a bag with the topics in the [dataset procedure](../sensor_dataset.md), plus the hazard schedule, the fault schedule and `FaultStatus`.
- A monitor script computes the signals of section 5 and the STL robustness of each requirement. Tool: RTAMT if the course tools allow it; otherwise a small in-repo robustness evaluator for the `G` and `F` forms used above.
- Output per trial: pass/fail and robustness per requirement, written to JSON and a summary CSV.
- Monitors are tested on hand-built traces that violate each requirement, so a monitor that never fails is detected.

### 6.2 Supervisor model check

- The reachable model includes mode, fault latch, clear count, recovery-age class, detector enablement, and completed-goal memory. An independent completion monitor extends exploration; five modes times 32 Boolean inputs is insufficient.
- Eleven independent temporal safety/next-step obligations cover navigation authorization, alarm response, recovery thresholds/counting/memory, completion creation/persistence, ready response, reset, and bounds. Nine unsafe mutations produce shortest counterexamples.
- The model and generated C++ priority table share a versioned policy. Every explored reachable context/input is compared with the compiled transition; source and binary hashes bind the result. Properties are independent of that table.
- Recorded results cover 92 reachable states and 7,120 transitions. This is a finite decision-logic proof under the documented input/timer abstraction. R3/R4 scheduling and physical behavior remain trace-monitor claims. [W10 evidence](results/supervisor/README.md) and [paper material](results/supervisor/paper/paper_section.md) retain assumptions and quantitative results.

### 6.3 Falsification

- Search space: obstacle layout (categorical), goal course (categorical), hazard `t_h`, `d_h`, GPS outage onset, fault class and onset, fault magnitude (within the envelope), noise seed, initial heading.
- Objective: minimize the robustness of R1, R3 and R5 (separate runs per requirement).
- Method: Latin-hypercube initial sample, then local minimization of robustness (CMA-ES or Nelder-Mead) from the lowest-robustness samples. A pairwise covering array over the categorical factors fixes the initial coverage (about 20 to 25 rows, two seeds per row).
- Two stages:
  - Surrogate: a Python model of the UGV kinematics, command watchdog, guard latencies, localization error process and hazard sensing latency, calibrated from Gazebo traces and using the supervisor transition table of section 6.2. It runs the broad search.
  - Gazebo: every surrogate counterexample and the fixed covering-array set are replayed in the full stack. Only Gazebo results are reported as results of the system. Surrogate results are reported as search guidance.
- Each counterexample is classified as a code defect (fixed and rerun), an out-of-envelope input (envelope entry added or tightened), or a requirement defect (requirement revised and recorded). The counterexample table is part of the report.
- Detector configurations: enabled, disabled, and forced-miss. Violation counts for R1 are reported for each. The detector is credited with a reduction only if the counts show one.

### 6.4 Detector evaluation

- Per test run: detected (flag within `tau_d` of onset), latency, and false alarms outside any fault.
- Fault-free runs: with `N` runs and zero false alarms, the 95% upper bound on the per-run false-alarm probability is `1 - 0.05^(1/N)`, about `3/N`. Reported with `N`.
- Detection rate per class with a Clopper-Pearson 95% interval.
- Window-level precision, recall and F1 as secondary metrics.
- The ALFA results are a separate offline benchmark. Precision, recall and F1 are saved to JSON, the threshold is fit on training data only, and the dataset location and checksum are recorded.

### 6.5 Code-level checks

- Property-based tests for pure numerical code: grid inflation never shrinks an obstacle; frame conversion round-trips; covariance remains positive definite after conversion.
- Existing `colcon test` suites and `scripts/test_navigation_guards.py`, `scripts/test_actuator_watchdog.py` run in the final CI pass.
- New unit tests for `route_planning` (blocked start, blocked goal, outside grid, stale grid, no route, diagonal corner, partial route).

### 6.6 Assurance case

| Claim | Evidence |
| --- | --- |
| Discrete hold logic is correct | 11 independent obligations and compiled conformance over 92 reachable states / 7,120 transitions; source-bound W10 evidence |
| UGV holds on guard failure within `tau_g` | R3 monitor over all trials; falsification counterexample table |
| UGV does not collide in the envelope | R1 monitor over the covering-array set; falsification with detector enabled, disabled, forced-miss |
| Hazards are avoided or stopped for | R5 monitor; hazard sweep over `t_h`, `d_h` |
| Detector detects injected faults | R7 and R8 on held-out runs |
| Localization bound holds | R6 over the rerun paired trials |

The report states each claim with the search budget, the number of trials, the seeds and the envelope. A falsification search with no counterexample is reported as no counterexample found in that budget, not as proof.

## 7. Work remaining

| ID | Work | Owner (proposed) | Depends on | Done when |
| --- | --- | --- | --- | --- |
| W1 | Rerun localization and goal-navigation trials on current `main`; store results | Rylan | none | results in `docs/` with commands |
| W2 | Initial infrastructure implemented: validated spawn/remove scenarios and confirmed world mutations | Sophie, Cameron | none | [Small camera example](../dynamic_obstacles/README.md); all-four-layout coverage remains pending |
| W3 | Initial camera example observes an appearing box and evaluates the existing replanner/controller response | Ethan | W2 | [R5 example](../dynamic_obstacles/README.md); ten-run response characterization remains pending |
| W4 | `uav_dropout` control and `faults.json` writer | Cameron | none | F1-F4 selectable per run; schedule recorded |
| W5 | Run-recording script for a scenario row; batch runner | Cameron | W4 | one command runs and records a row |
| W6 | Dataset collection: 60 labeled runs | Jeremy, Cameron | W5 | bags and `faults.json` on shared storage |
| W7 | Feature extraction from bags; windowing; run-level splits | Jeremy | W6 | `(N, 100, C)` arrays with labels and run IDs |
| W8 | Retrain RNN, BiLSTM, autoencoder; threshold baseline; select by validation | Jeremy | W7 | metrics JSON for all candidates |
| W9 | `fault_detection` node, `FaultStatus` message | Jeremy, Ethan | W8 | flags published at 2 Hz in the full stack |
| W10 | Implemented: persistent supervisor, finite model check, compiled conformance, controlled ROS tests, paper artifacts | Rylan | none | [Source-bound results](results/supervisor/README.md); online detector accuracy remains W9 |
| W11 | Requirement monitors and monitor tests | Rylan | W5 | all R1-R8 evaluated from a bag |
| W12 | Surrogate model and calibration | Rylan, Ethan | W3, W11 | surrogate reproduces Gazebo R1 and R5 outcomes on 10 calibration rows |
| W13 | Falsification search and Gazebo replay | Rylan, Sophie | W9, W11, W12 | counterexample table complete |
| W14 | `route_planning` tests; property tests | Ethan | none | tests registered in `colcon test` |
| W15 | Final scenario matrix, plots, tables | Sophie | W13 | results in `docs/project/results/` |
| W16 | Report and presentation | all | W15 | 10-page report; slides |

## 8. Schedule

| Week | Dates | Targets |
| --- | --- | --- |
| 1 | 11/3-11/8 | Scope and envelope agreed. W1, W2, W4, W5, W10 started. Detector feature set agreed with Jeremy. Requirements R1-R8 reviewed by the team. |
| 2 | 11/9-11/15 | W3, W6, W7, W11 done. W8 in progress. Constants in section 5 replaced by measured values. |
| 3 | 11/16-11/22 | W8, W9, W12, W14 done. W13 running. Code freeze 11/22. |
| 4 | 11/23-11/30 | Final trials on frozen code. W15, W16. Report and slides done 11/29. |

Any item not done at the end of week 3 is cut from the demonstration and stated as not built in the report. Cut order: F4 injection and class, surrogate (replace with Gazebo-only search at a smaller budget), property tests, `GOAL` re-entry logic.

## 9. Deliverables

- Final demonstration: one command launches the full stack with a layout, goal, hazard schedule and fault schedule.
- `verification/` directory: monitors, surrogate, falsification driver, supervisor model and conformance test.
- `docs/project/results/`: trial summaries, counterexample table, detector evaluation, rerun localization results, each with the commit hash, seeds and commands.
- 10-page report: system description, envelope, requirements, verification methods, results, limits.
- Presentation.

## 10. Open items

- Course project page: confirm required report sections and the report due date.
- Verification tools covered in lecture (RTAMT, Breach, S-TaLiRo): confirm before W11.
- Owners in section 7 are proposals.
- Detector feature set and window parameters (section 4.2): confirm with Jeremy before W7.
- Hazard `d_min` and constants in section 5 are initial values until measured in W3.
- Terrain classification is not covered by any team member's work and is cut (section 1.2).
