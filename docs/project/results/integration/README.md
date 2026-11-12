# Combined integration validation

Date: 2026-11-11. Integration candidate: `c68a955`, containing PRs #5-#7 and the corrections below. Documentation-only changes follow this candidate. [Validation metadata](validation.json) | [Source equivalence](source_equivalence.json)

## Corrections

- The extensionless mapper uses LF on Windows checkouts and exports. A CRLF interpreter line prevented Linux startup in the initial export.
- Frozen source identity includes extensionless scripts. Regression tests verify that mapper and obstacle-runner edits invalidate an existing source snapshot.
- The guard fixture selects UDP before ROS imports load middleware, including when its parent uses camera shared memory. Its compact grid retains both goals, footprint clearance, and all 23 cases.
- The watchdog fixture confirms input silence at the actuator before declaring command loss. A queued command after producer termination no longer races the event marker. Deadlines still use the last accepted input; subsequent accepted input remains inconclusive.
- Deployed controller completion is 0.35 m inside R2's unchanged 0.5 m estimated-arrival boundary. This leaves margin for post-stop estimate variation during the one-second dwell. It does not bound localization drift or change completion-latch semantics.

## Results

| Check | Recorded result |
| --- | --- |
| Clean Linux build | All 12 packages build against the ROS Humble underlay |
| Registered checks | 389 checks, zero errors/failures; nine cppcheck skips |
| Existing fault-injection tests | Seven pass |
| Compiled supervisor | 92 reachable states, 7,120 transitions; 11 properties pass; nine unsafe mutations detected |
| Controlled supervisor | All 29 ROS cases pass |
| Headless demo | Three attempts/four recordings pass: camera R1/R2/R6, guard R3, both watchdog R4 recordings |
| Dynamic obstacle | R1/R2/R5/R6 pass; expected, observed, and recorded hazard IDs match |
| Gazebo lifecycle | Five create/duplicate/remove/respawn cases pass |
| Cancellation | Exit 130 after confirmed obstacle creation; recorder and both obstacle nodes finalize; no owned processes remain |
| Saved-result publication | Eight historical CSV tables reproduce exactly; six-page verification PDF and two-page supervisor PDF compile; obstacle evidence/timeline reproduce byte-for-byte |

Evidence: [registered checks](colcon-results.txt), [model](model.json), [supervisor cases](controlled.json), [demo summary](demo_summary.json), [demo ledger](demo_ledger.json), [demo requirements](demo_requirements.json), [dynamic summary](dynamic_summary.json), [dynamic requirements](dynamic_requirements.json), [applied operations](hazard_operations.json), [observations](hazard_observations.json), [lifecycle](lifecycle.json), and [interruption](interruption.json).

## Reproduction

Use the [Linux dependency/build setup](../../../verification/simulator_integration.md). From a sourced workspace with the verification environment active:

```bash
cd ros2_ws
colcon build --parallel-workers 2 --symlink-install
colcon test --parallel-workers 2 --event-handlers console_direct+
colcon test-result --all
source install/setup.bash
cd ..

export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/config/fastdds_local_shm.xml"
python scripts/verification_demo.py verification/outputs/integration-demo
python scripts/run_dynamic_obstacle_trial.py verification/outputs/integration-obstacle --seed 744
python scripts/test_obstacle_lifecycle.py verification/outputs/integration-lifecycle

FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/config/fastdds_fixture_udp.xml" ROS_DOMAIN_ID=67 \
  python scripts/test_navigation_supervisor.py verification/outputs/integration-supervisor
python scripts/check_navigation_supervisor.py \
  --probe ros2_ws/install/ugv_control/lib/ugv_control/supervisor_transition_probe \
  --output verification/outputs/integration-model.json
```

Use fresh output directories and run simulator trials serially. All simulator checks are headless. [Demo/replay commands](../../../verification/demo.md) and [obstacle report commands](../../../dynamic_obstacles/README.md) regenerate reports without another simulation.

## Evidence scope

[Development outcomes](development_outcomes.json) retain unsuccessful integration attempts separately. Historical feature results are unchanged. This validation reruns the combined demo and one flat-world box example; it does not rerun the 45-case matrix or resolve its historical dense-layout outcomes. Camera R3 remains unexercised; its controlled fixture is checked separately. Online R7/R8 remain excluded because no detector producer is deployed.

[Raw availability](raw_availability.json) identifies the source archive, bags, and checksums at `Ubuntu-22.04:/home/rylanwwade/uav-integration`. Raw bags and normalized streams remain machine-local. All 238 frozen source files match the candidate export, including both extensionless runtime executables. [Artifact checksums](artifacts.json) identify the portable result files.
