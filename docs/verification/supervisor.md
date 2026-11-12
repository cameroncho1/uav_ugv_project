# Navigation supervisor and finite-state verification

[Verification index](README.md) | [Course alignment](supervisor_course_alignment.md) | [Completion plan W10](../project/completion_plan.md#62-supervisor-model-check)

The UGV controller has a persistent navigation supervisor. It decides whether the existing path follower may produce a command. Every other mode publishes zero linear and angular velocity on each controller timer tick. The model checks this discrete permission policy; it does not prove continuous collision freedom, physical stopping, detector accuracy, or callback scheduling.

## Implementation

| Component | Function |
| --- | --- |
| [Policy](../../verification/supervisor/policy.json) | Versioned ordered mode-selection table and recovery bounds; shared by the model and generated C++ table |
| [Pure C++ transition](../../ros2_ws/src/ugv_control/include/ugv_control/supervisor.hpp) | Modes, fault latch, clear-window count, recovery-age class, and completed-goal memory; no ROS/Gazebo imports |
| [Numerical input validation](../../ros2_ws/src/ugv_control/include/ugv_control/supervisor_inputs.hpp) | Detector identity, timestamps, continuity, exact elapsed time, and accepted goal revisions |
| [ROS controller](../../ros2_ws/src/ugv_control/src/goal_controller.cpp) | Existing pose/path validation, subscriptions, permission-gated path following, zero holds, and consumer telemetry |
| [Independent checker](../../verification/src/uav_ugv_verification/supervisor.py) | Breadth-first reachable-state/product-monitor exploration, independent properties, negative controls, and C++ conformance |

The policy header is generated during the `ugv_control` build. The compiled probe embeds the policy digest and implementation-source digest; the checker rejects a probe built from different inputs. The proof checks the compiled transition used by the controller, with controller wiring tested separately through ROS.

## Modes and precedence

The first matching row applies after memory updates:

| Priority | Condition | Mode | Motion permission |
| --- | --- | --- | --- |
| 1 | No accepted active goal | `INIT` | Zero |
| 2 | Fault latch set | `HOLD_FAULT` | Zero |
| 3 | Full pose guard invalid | `HOLD_GUARD` | Zero |
| 4 | Current goal completed | `GOAL` | Zero |
| 5 | Current route invalid | `HOLD_GUARD` | Zero |
| 6 | All navigation conditions satisfied | `NAVIGATE` | Existing follower allowed |

`NAVIGATE` permits the follower; the follower can still return zero. The existing pose guard remains authoritative for frames, covariance, quaternion, monotonic acquisition stamps, and acquisition/steady receipt ages. Existing path validation retains goal association, freshness, connected map geometry, and clearance checks.

State includes detector enablement, mode, fault latch, count `0..10`, recovery age (`NONE`, `YOUNG`, `MATURE`), and completed-goal memory. Detector participation is fixed at startup. The supplied deployment has no online detector producer and uses `detector_enabled: false`.

## Goal completion and revisions

The controller latches completion after one valid estimated pose lies inside the configured goal tolerance. This prevents localization drift from restarting the same goal. It is a controller decision, **not** proof of R2's physical tolerance and quiet dwell; R2 remains a separate recorded-trace check.

The first supervisor decision that sees an accepted revision consumes its new-goal event, including detector callbacks before the timer. A later timer cannot replay that event and erase completion already observed for the revision.

Completion survives pose/route failures, fault holds, and clock rewind. A new accepted valid goal revision clears completion and invalidates the previous route. It does not clear fault recovery. A repeated revision with changed coordinates, an older revision, an invalid goal, or a goal more than 50 ms ahead of the local ROS clock cannot authorize a new mission.

A ROS clock rewind invalidates active goal/route/decision anchors and starts a local epoch. Navigation requires an accepted goal and fresh evidence in that epoch. Messages do not carry an explicit epoch: identifying a newly published goal after rewind assumes the producer obeys acquisition/revision semantics. Numerically coincident old messages cannot be intrinsically distinguished by the current message types.

## Detector boundary and recovery

[FaultStatus](../../ros2_ws/src/uav_ugv_msgs/msg/FaultStatus.msg) carries schema version 1, acquisition/window-end stamp, producer-session identity, sequence, window validity, and class flags. `/fault_detection/status` is the default input. `detector_enabled` and `fault_status_topic` are read-only startup parameters; changing them requires restart.

- Wheel-scale or IMU-drift flags request a hold. GPS/UAV-loss flags request a fault hold when the full pose guard is invalid; otherwise degraded navigation remains possible.
- An enabled but missing, stale, invalid, restarted, or discontinuous detector input holds. Initial enabled state is latched until recovery evidence arrives.
- Recovery requires **ten distinct valid clear decisions and at least five simulation seconds since the first clear decision's acquisition stamp**. A clear decision means no hold-requesting flag under this policy, rather than every fault flag being false.
- Source sequence and acquisition stamps increase strictly. Acquisition/receipt freshness and consecutive-decision acquisition/receipt gaps are at most one second; future acquisition allowance is 50 ms. A source-session change or gap breaks recovery. A subsequent contiguous decision starts a new clear streak.
- An identical duplicate neither increments the counter nor renews the receipt lease. Conflicting duplicates and old decisions invalidate recovery. Controller ticks never increment the counter.
- Ten endpoints spaced 0.5 seconds apart span 4.5 seconds. The latch stays set until the separate five-second condition holds; a later control tick may release it if the latest clear decision remains fresh.

Every received detector decision updates the latch, including alarm/clear bursts between command ticks. Actual `Twist` publication remains on the 100 ms wall timer. Timing and physical settling require trace evidence; the finite proof does not establish a hard timer bound.

## Model and properties

The finite abstraction uses Boolean validated pose/route/goal facts, a distinct-clear/alarm/unavailable/no-new-decision event, a valid new-goal event, clock reset, and recovery maturity. The runtime canonicalizes irrelevant maturity flags. Exact nanosecond comparisons establish the `YOUNG`/`MATURE` boundary; they are tested independently at five seconds minus one, exactly five seconds, and plus one nanosecond.

The checker explores all reachable memory states from detector-disabled and detector-enabled initialization. It also carries an independently maintained completed-goal monitor. Input constraints are explicit: a new valid goal requires an active goal; a valid route requires its goal; estimated goal attainment requires a valid pose/goal; timer maturity requires a started recovery interval. This is exhaustive finite-state safety model checking under that abstraction, not arbitrary LTL parsing or unconditional liveness checking.

| Obligation | Evidence checked |
| --- | --- |
| Navigation authorization | Permission requires accepted goal, valid pose/route, no fault latch, and no completion latch |
| Alarm/unknown response | Enabled alarm or unavailable input sets the fault latch |
| Recovery threshold | Fault hold cannot release before count and maturity conditions |
| Distinct-window counting | No new clear decision means no counter increment |
| Completed-goal hold | Independent monitor forbids resumed permission until a new valid goal |
| Completion memory | Guard/fault/reset transitions do not erase completion |
| Ready response | Pre-state/input-derived readiness requires next-decision navigation permission |
| Reset hold | Reset yields `INIT`, unknown enabled detector, zero recovery count/age |
| Memory bounds | Count and age remain within the finite representation |
| Completion integrity | Completion is created only by an eligible estimated-threshold observation and persists correctly |
| Recovery memory integrity | Every clear increments once, maturity requires its validated input, and resets/releases update all memory consistently |

Temporal safety and next-step predicates are independent of the shared decision table. Nine deliberately unsafe mutations test the checker: unsafe navigation, early recovery, forgotten completion, spurious hold, permanent fault hold, missing completion creation, spurious completion, double counting, and invented maturity. Detected violations retain shortest breadth-first counterexamples.

The compiled probe is compared against the model on every reachable context/input and resulting memory/output. These contexts are reached through explored multi-step sequences. Exact graph and conformance counts, source hashes, mutation witnesses, and runtime results are recorded with the evidence; five named modes times 32 Boolean combinations is not the proof domain.

## Commands

Offline model checking, without ROS:

```bash
python scripts/check_navigation_supervisor.py --output verification/outputs/supervisor/model.json
python -m pytest verification/tests/test_supervisor.py
```

Linux, with ROS 2 Humble and the project workspace dependencies available:

```bash
cd ros2_ws
colcon build --packages-select uav_ugv_msgs ugv_control mission_manager --symlink-install
source install/setup.bash
colcon test --packages-select uav_ugv_msgs ugv_control mission_manager
colcon test-result --verbose
cd ..
python scripts/check_navigation_supervisor.py \
  --probe ros2_ws/install/ugv_control/lib/ugv_control/supervisor_transition_probe \
  --output verification/outputs/supervisor/model_and_conformance.json
ROS_DOMAIN_ID=67 python scripts/test_navigation_supervisor.py verification/outputs/supervisor/controlled
```

Use fresh output directories for retained trials. The controlled fixture supplies declared ROS pose, free-route, and detector inputs to the actual controller. It does not evaluate learned detection or physical dynamics. Its compact numerical traffic uses a local UDP profile; camera trials retain the project shared-memory transport. Consumer observations, actual `Twist` checks, recordings, and request logs remain separate artifacts.

The existing guard/watchdog checks and an assisted headless navigation demonstration validate integration separately. Estimator measurements and filters are unchanged. Historical M3/M4 evidence is preserved; the supervisor proof applies only to matching supervisor source/policy versions.
