# Verification implementation reference

Status: research and source inspection, 2026-11-10. No implementation or simulator validation.

[Specification](milestone_plan.md) | [Verification index](README.md)

## 1. Dependencies and APIs

| Area | Decision | Reference |
| --- | --- | --- |
| Numerical core | Python >=3.10, NumPy, PyYAML; no Torch or ROS import | Existing [dependency constraints](../simulation_setup.md) |
| Bag reader | Humble `rosbag2_py.SequentialReader`; `StorageOptions`, CDR `ConverterOptions`, `StorageFilter`; deserialize selected types with `rclpy.serialization.deserialize_message` and `rosidl_runtime_py.utilities.get_message` | [Humble reader test](https://github.com/ros2/rosbag2/blob/humble/rosbag2_py/test/test_sequential_reader.py), [options helper](https://github.com/ros2/rosbag2/blob/humble/rosbag2_py/test/common.py) |
| Confidence intervals | SciPy `binomtest(k, n).proportion_ci(confidence_level=0.95, method='exact')` for two-sided binomial intervals | [SciPy 1.10.1 API](https://docs.scipy.org/doc/scipy-1.10.1/reference/generated/scipy.stats._result_classes.BinomTestResult.proportion_ci.html) |
| Figures | Matplotlib; load only for report generation | Existing [requirements](../../requirements.txt) |
| Monitor comparison | RTAMT is an optional test oracle for supported `always`/`eventually` formulas, not a runtime dependency | [RTAMT documentation](https://github.com/nickovic/rtamt), [tool paper](https://arxiv.org/abs/2005.11827) |

Use the Ubuntu 22.04 ROS environment's Python interpreter for bag decoding. A virtual environment needs access to installed ROS packages, for example via `--system-site-packages`. Pin versions after the Linux compatibility checks; no dependency set is claimed tested here. Core fixtures must run without importing the ROS adapter.

The RTAMT discrete-time API has sampling assumptions; do not feed irregular sensor timestamps into it as though samples were seconds. A comparison must state time units, sampling/interpolation, and horizon coverage. Its result does not replace the project's data-quality and trigger-coverage verdicts. [RTAMT timing documentation](https://github.com/nickovic/rtamt#working-with-time-units-and-timing-assumptions)

## 2. Current signal inventory

Types below omit the `/msg/` segment. Present means implemented at the inspected baseline, not confirmed in a newly recorded run.

| Canonical signal | Existing source | Timestamp and limits |
| --- | --- | --- |
| `truth_pose` | `/evaluation/ugv_truth`, `geometry_msgs/PoseStamped` | Gazebo acquisition stamp; model origin in ENU `map`; no velocity or obstacle geometry |
| `estimated_pose` | `/ugv/localization/pose`, `geometry_msgs/PoseWithCovarianceStamped` | EKF acquisition stamp and public covariance, including systematic prior |
| `uav_observation` | `/ugv/sensors/uav_relative_pose`, `geometry_msgs/PoseWithCovarianceStamped` | Relative observation; retain declared frame. Do not treat its coordinates as global UGV position |
| `goal` | `/global_goal_pose`, `geometry_msgs/PoseStamped` | Header stamp is the stable goal revision, not a freshness clock |
| `route` | `/ugv/navigation/route`, `uav_ugv_msgs/PlannedRoute` | Includes goal revision, path, clearance grid, and `partial`; `/local_path` is visualization/scoring history only |
| `grid` | `/local_grid`, `nav_msgs/OccupancyGrid` | Image acquisition stamp; publisher is transient-local, depth 1 |
| `requested_command` | `/ugv/cmd_vel`, `geometry_msgs/Twist` | No header; source emission/consumer receipt times cannot be recovered from the message |
| `actuator_input` | Gazebo `/simple_ugv/drive_cmd_vel`, `gz.msgs.Twist` | Receipt is private to the watchdog; no current trace |
| `actuator_output` | Gazebo `/simple_ugv/cmd_vel`, `gz.msgs.Twist` | Watchdog emission, not physical speed; no current receipt/expiry trace |
| `guard_state` | `PoseGuard::check()` and `goal_controller::control_tick()` | Guard result and hold reason exist internally; only changing text status is logged |
| `obstacles` | Selected world SDF `<collision>` elements | Static boxes and upright cylinders; dynamic spawn/transform events need recording |
| `detector_decision` | Offline fault-model pipeline | No `FaultStatus` message or online detector node found in `ros2_ws/src` |
| Dataset CSV | `imu.csv`, `wheel_odom.csv`, `gps.csv`, `truth.csv`, `cmd.csv`, `metadata.json` | Acquisition seconds; command CSV includes sim stamp. No navigation estimate, route, grid, or online detector decision |

Source paths: [truth](../../ros2_ws/src/simulation_support/src/ugv_truth.cpp), [pose adapter](../../ros2_ws/src/cooperative_localization/scripts/ugv_fused_pose.py), [route](../../ros2_ws/src/uav_ugv_msgs/msg/PlannedRoute.msg), [mapper](../../ros2_ws/src/traversability_mapping/scripts/traversability_mapper), [controller](../../ros2_ws/src/ugv_control/src/goal_controller.cpp), [watchdog](../../ros2_ws/src/ugv_control/src/command_watchdog.cpp), [dataset recorder](../../ros2_ws/src/fault_management/scripts/ugv_dataset_recorder.py).

### Recording requirements

- Extend the [current bag topic list](../sensor_dataset.md) with `/local_grid` and evaluation telemetry. A clearance grid embedded in a route cannot establish when a new hazard first entered the mapper's grid if no new route is published.
- Filter numerical topics before deserialization. Read message types from bag metadata; custom `PlannedRoute` decoding requires the matching built workspace. Reject unsupported storage/schema explicitly; do not copy APIs from newer ROS distributions.
- Record `/clock`, but preserve source/receipt timestamps separately. With `--use-sim-time`, rosbag2 stamps records using the latest received `/clock` value and waits for the first clock message. Equal bag times are therefore legitimate and cannot resolve watchdog timing during a frozen clock. [Humble recorder semantics](https://github.com/ros2/rosbag2/blob/humble/README.md#simulation-time)
- Preserve acquisition order within streams and receipt order for causal checks. A late sensor message is not evidence of a global clock reset; only the recorded clock lifecycle establishes a new epoch.
- Request compatible QoS. Record the transient-local goal, grid, and route with compatible durability; use sensor-compatible reliability for best-effort sources. Verify retained startup messages and nonzero topic counts. Humble supports `--qos-profile-overrides-path`; the exact override file is a recording artifact. [Humble QoS guide source](https://github.com/ros2/ros2_documentation/blob/humble/source/How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback.rst)

### Evaluation telemetry, proposed

Use versioned JSON payloads on evaluation-only channels. ROS transport: `std_msgs/msg/String`; Gazebo transport: `gz.msgs.StringMsg` with a one-way bridge. The bridge is outside the command path; the watchdog retains no ROS dependency. Confirm the mapping against the installed Harmonic overlay. [Bridge mapping/configuration](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md)

| Record | Minimum payload |
| --- | --- |
| Common envelope | `schema_version`, `source`, `sequence`, `clock_id`, `epoch`, `steady_ns`; `sim_ns` when available |
| Control tick | Pose source stamp/receipt, `guard_valid`, guard reason, overall hold reason, active goal revision, emitted linear/angular command |
| Watchdog input | Input sequence, accepted/rejected flag, receipt steady time, configured timeout |
| Watchdog output | Output sequence, associated input sequence, input age, expiry flag, emitted command, actual emission steady time |
| Applied fault | Event ID, sensor/channel, class, new state, actual application stamp; retain request/ack separately |

Sequence gaps make affected timing intervals incomplete. Telemetry must not block command delivery or write files while command-state locks are held. Validate timing overhead in pilot runs. Publish each control tick and watchdog transition/output needed to reconstruct intervals; text status changes alone are insufficient. Same-source elapsed timing is valid without synchronizing unrelated monotonic clock origins. [ROS clock design](https://design.ros2.org/articles/clock_and_time.html)

R4 expiry trials require a nonzero pre-loss output and no later accepted input during the observation interval. Position differences provide interval-average physical velocity, not an exact stopping instant. Record the stop-time interval implied by the sample spacing; a deadline overlapping that uncertainty is inconclusive. A commanded zero and a watchdog-emitted zero are separate from measured settling.

## 3. Temporal evaluation rules

The following are project semantics. The partial-trace literature motivates retaining undecided outcomes when a future horizon is missing; this specification does not claim to implement that paper's robustness-interval algorithm. [Deshmukh et al., Robust Online Monitoring of STL](https://arxiv.org/abs/1506.08234)

- Use integer nanoseconds and closed response windows `[t0, t0 + tau]`. A response exactly at the deadline meets the timing bound; one nanosecond later does not.
- Discrete values are right-continuous until the next sample or their maximum validity gap, whichever comes first. No signal is extended indefinitely through a recording gap. Missing validity invalidates dependent obligations.
- Start one obligation at each trigger's false-to-true transition. An initially true trigger starts at the declared evaluation interval boundary. Repeated true samples do not reset its deadline. Log initial-state versus transition triggers.
- R3: find the first quiet command within the response window; it must stay quiet until guard recovery. A brief recovery does not cancel an already-created response obligation. A quiet command at onset gives zero response latency; later nonzero commands before recovery violate the hold.
- A response observed before trace end can decide bounded eventuality without its entire horizon. A hold still needs coverage until recovery or the declared run end. If no response exists and the recording ends before the deadline, return inconclusive; after complete deadline coverage, return failure.
- Do not create a cross-epoch obligation. Retain already witnessed violations; report unresolved obligations at clock resets as inconclusive.
- At equal timestamps, use explicit same-source sequence. If a cross-source tie changes the verdict and ordering is unobserved, report ambiguity rather than choosing a favorable ordering.
- Match R6 estimates at acquisition time to bracketed truth for offline scoring within the configured gap. This interpolation is scoring only; guard/route validity uses only data available at the recorded consumer tick.

### Required analytic fixtures

Times below are seconds for readability; fixture storage uses nanoseconds. Assume complete coverage unless stated. R3 fixtures use `tau = 0.5`.

| Case | Trace | Expected result |
| --- | --- | --- |
| Deadline boundary | Guard invalid at 1.0; first quiet command at 1.5; quiet through recovery at 2.0 | Pass, response 0.5 s |
| Late response | Same, first quiet at 1.500000001 | Fail, timing slack -1 ns |
| Truncated obligation | Invalid at 1.0; no quiet command; recording ends 1.4 | Inconclusive |
| Sustained hold | Quiet at 1.1; nonzero at 1.3; guard recovers 2.0 | Fail despite timely first response |
| Deadline reset bug | Invalid samples at 1.0, 1.2, 1.4; first quiet 1.6 | Fail; start remains 1.0 |
| Already quiet | Quiet command valid at invalid onset 1.0; remains quiet until recovery | Pass with zero latency |
| Coverage gap | Timely quiet command followed by an uncovered interval before recovery | Inconclusive unless a violation is separately witnessed |
| Unexercised guard | No invalid interval | Empirical verdict inconclusive; trigger count zero |
| Stale route | R5 trigger followed only by a route for a different goal revision | No qualifying route response |
| Missing alarm horizon | Fault onset 10.0; detection bound 5.0; no alarm by trace end 12.0 | Inconclusive, not a detector miss |
| Pre-existing alarm | Alarm active before fault onset, no new matching decision afterward | Not a new detection; count pre-existing alarm |
| Frozen simulation | Sim time constant; watchdog steady time advances | Evaluate expiry using watchdog clock; no physical-settling conclusion from pause |

## 4. Geometry and uncertainty

Both boxes and cylinders are required: [world generator](../../scripts/generate_obstacle_worlds.py) emits cylinders in sparse/dense layouts. Extract collision geometry from the selected SDF, not visual geometry or camera occupancy. Exclude the ground and robot self-collisions using explicit entity selection. The visual goal marker is not an obstacle.

For a disk radius `r`, a box with center `c`, planar rotation `R`, and half-extents `h`:

```text
q = abs(transpose(R) * (p - c)) - h
signed_box_distance = norm(max(q, 0)) + min(max(q.x, q.y), 0)
clearance = signed_box_distance - r

upright cylinder: clearance = norm(p - c) - cylinder_radius - r
```

These are reference calculations for the declared planar geometry model. Compose model/link/collision poses. Resolve `relative_to` and includes using SDFormat semantics or require an already-resolved geometry export; unsupported transforms/shapes invalidate coverage rather than disappearing. A missing obstacle source differs from an explicitly verified empty obstacle set. [SDF collision specification](https://sdformat.org/spec/1.9/collision/), [frame semantics](https://sdformat.org/tutorials/specification/pose_frame_semantics/)

Analytic geometry checks: axis-aligned box half-extents `(1,1)` and `r=0.5` give clearance `0.5` at `(2,0)`, `0` at `(1.5,0)`, and `-1.5` at `(0,0)`. Cylinder radius `0.4`, `r=0.5`, and center distance `1.0` give `0.1`. Repeat after a common translation/rotation; results are invariant.

R1 checks the declared disk footprint. Disk overlap is not automatically evidence of physical chassis contact. Report proxy clearance separately from simulator contact evidence. Default to sampled clearance; conservative intersample bounds require an explicit relative-motion bound and cannot bridge obstacle-spawn discontinuities.

Align pose reference points before computing localization error: the truth publisher reports the model origin, while sensor/body frames may refer to the chassis. Preserve the model-to-body transform, including the chassis height, rather than assuming planar coordinates remain identical under tilt.

Position standard deviation uses the largest x/y covariance eigenvalue: `sqrt((c00 + c11 + hypot(c00 - c11, 2*c01))/2)`. The [current guard](../../ros2_ws/src/uav_ugv_msgs/include/uav_ugv_msgs/pose_guard.hpp) also validates x/y/yaw positive definiteness, symmetry, quaternion norm, frame, future stamps, receipt age, and monotonic input. `sqrt(max(c00,c11))` is not an equivalent check. Runtime guard verdicts come from telemetry; independent numerical fixtures check the monitor's interpretation.

## 5. Detector and statistical evaluation

- Current model decisions are faulty/clean per channel window; `fault_type` metadata is ground truth, not a predicted class. Label R7 as binary/channel-scoped unless an actual class-specific output exists. Keep original class-specific R7 unavailable rather than inferring predictions from labels. [Model input/output definitions](../../uav_ugv_project_fault_detection/MODELS.md)
- For offline injected data, fault offsets use the minimum first stamp across the loaded IMU/wheel/GPS streams, not necessarily `metadata.start_sim_time`. A short-fault envelope can contain clean gaps: use per-sample masks and actual first affected samples for events. Preserve sample precision and source IDs. [Fault application](../../uav_ugv_project_fault_detection/fault_injection.py)
- Online latency ends at decision emission, not at window start/center. Store window start/end and emission separately. The current 100-sample, non-overlapping windows span different durations by sensor; they do not establish a uniform five-second online bound.
- Use two-sided 95% Clopper-Pearson intervals for independent binary run outcomes. Multiple events/variants from one drive are clustered; report event counts separately without presenting them as independent trials. With no eligible runs, return an undefined estimate with `n=0`. [Exact interval API](https://docs.scipy.org/doc/scipy-1.10.1/reference/generated/scipy.stats._result_classes.BinomTestResult.proportion_ci.html)
- With zero false-alarm runs out of `N` independent nominal runs, the one-sided 95% upper bound is `1 - 0.05^(1/N)`, derived from `(1-p)^N = 0.05`. This differs from a two-sided 95% interval. For `N=3`, it is about 0.632; three seeds provide coverage, not a strong false-alarm claim. Retain alarm count and nominal exposure for events/hour; do not attach a Poisson interval without that model assumption.

## 6. Required adjacent corrections

Source inspection establishes the branches and missing interfaces below. Runtime consequences remain to be reproduced in Linux. Changes belong in M2/M3 before final trials; this document applies none of them.

| ID | Source finding | Required correction and regression |
| --- | --- | --- |
| D01 | [Scorer](../../scripts/score_goal_navigation.py) unconditionally requires UAV correction/relative-message counts; [launch](../../ros2_ws/src/mission_manager/launch/simple_goal_demo.launch.py) disables tag detection with `uav_assistance:=false` | Separate physical-arrival verdict from configuration-specific input checks. Baseline with zero UAV observations is valid input configuration. Keep assisted checks; test both and an assisted run missing corrections. Distinguish assistance-disabled camera trials from synthetic tracking trials where the observation producer remains active |
| D02 | [World launcher](../../scripts/world_goal_demo.sh) and [trial runner](../../scripts/run_goal_navigation_trial.sh) hardcode the base SDF; runner does not forward alternate goal/assistance settings to launch and scorer | Parameterize selected world file and goal/assistance once; pass identical effective settings to launch, scoring, manifest, and geometry extraction. Keep internal world name `simple_goal_demo` for current variants. Test all four paths and both goals without editing generated SDFs |
| D03 | [Scorer](../../scripts/score_goal_navigation.py) records GPS-off time when requesting the service; [sensor callback](../../ros2_ws/src/fault_management/scripts/sim_ugv_sensors.py) applies it later | Emit applied-state event in the callback; retain request/ack/applied separately. Test delayed and unsuccessful requests; neither establishes onset. Do not retroactively relabel historical request timestamps as exact application time |
| D04 | Existing [arrival scorer](../../scripts/score_goal_navigation.py) measures timeout/dwell using wall time; R2 is specified in simulation time | Preserve legacy fields with clock labels; compute R2 using declared sim start/deadline and retain an independent wall watchdog for stalled runs. Test non-unit real-time factor and frozen clock |
| D05 | [Watchdog fixture](../../scripts/test_actuator_watchdog.py) checks displacement after 0.75 wall seconds; it does not measure the first zero output or actual stopping latency | Retain displacement regression and add telemetry-based expiry/settling measurements. The 50 ms worker sleep is nominal, not a hard scheduling guarantee; report observed scheduling delay and calibrated bounds |
| D06 | [Bag procedure](../sensor_dataset.md) omits raw `/local_grid`, guard state, actuator timing, and applied fault events | Extend explicit recording/QoS configuration; add a missing-topic/sequence-gap check. Older bags cannot support full R3/R4/R5 automatically |

Full PX4 trials run serially by default. ROS domain and Gazebo partition isolation do not isolate the fixed XRCE UDP port 8888 or PX4 process instances. Parallel full-stack runs require separate ports/instances/resources and a dedicated isolation test. Use the existing [dependency checker](../../scripts/check_sim_dependencies.py) and [process cleanup](../../scripts/process_cleanup.sh); camera trials also require a working Linux display as documented in [simulation setup](../simulation_setup.md).
