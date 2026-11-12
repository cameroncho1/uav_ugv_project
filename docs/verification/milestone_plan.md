# Verification specification

Status: M1-M4 implemented. Optional E1 remains unimplemented. Updated: 2026-11-11. [Validation status](status.md) | [M3 experiments](../project/results/verification/m3/README.md) | [M4 report/demo](../project/results/verification/m4/README.md).

[Verification index](README.md) | [Implementation reference](implementation_reference.md) | [Completion plan](../project/completion_plan.md) | [Navigation](../navigation.md)

## 1. Scope

Evaluate recorded system behavior against R1-R8. Produce per-run verdicts, violation intervals, metrics, and reproducible report figures. Numerical evaluation is independent of ROS and Gazebo; recording adapters provide simulator-specific inputs.

| Milestone | Output | Dependency |
| --- | --- | --- |
| M1: offline evaluator | Schemas, profiles, R1-R8 monitors, CLI, semantic tests | Existing numerical/scoring code |
| M2: simulator integration | Recording instrumentation, adapters, cross-setup validation | M1 API |
| M3: experiments and failure search | Fixed scenario matrix, run ledger, counterexample replay | M2 capabilities and calibrated profiles |
| M4: report and demo | Figures, tables, evidence index, repeatable demonstration | M3 results; reporting can start against M1 fixtures |

Optional: supervisor model checking (E1). Excluded: surrogate models, advanced optimizers, live dashboards, detector training, and additional simulator integrations.

Include scoring, launch, and recording corrections D01-D06 in the [implementation reference](implementation_reference.md#6-required-adjacent-corrections). Preserve existing behavior outside the corrected cases. Reproduce each case and add its regression before final experiments.

Requirement IDs refer to [completion-plan section 5](../project/completion_plan.md#5-requirements). Changes to their definitions require a recorded version.

## 2. Existing interfaces

Baseline: `origin/main` at `5a366ea03bd7a3384cca8bad4e382e6a273752ad`.

- [`scripts/evaluation_math.py`](../../scripts/evaluation_math.py): numerical arrival, matching, and tracking helpers.
- [`scripts/score_goal_navigation.py`](../../scripts/score_goal_navigation.py): scorer behavior and output conventions.
- [`scripts/test_navigation_guards.py`](../../scripts/test_navigation_guards.py), [`scripts/test_actuator_watchdog.py`](../../scripts/test_actuator_watchdog.py): controlled integration fixtures.
- [`PlannedRoute.msg`](../../ros2_ws/src/uav_ugv_msgs/msg/PlannedRoute.msg): authoritative route, clearance grid, and goal revision.
- [Navigation](../navigation.md), [localization interfaces](../cooperative_localization/interfaces.md), [recording procedure](../sensor_dataset.md): frames, timing, and collection paths.
- [Detector documentation](../../uav_ugv_project_fault_detection/MODELS.md): fault labels, window durations, and model inputs.

## 3. Architecture and data contracts

```text
Existing runner -> recording + actual events + manifest
               -> setup adapter -> normalized numerical streams
               -> derived signals -> requirement monitors
               -> result JSON/CSV -> paper figures and demo summary
```

### Package layout

Installable Python project: `verification/`, containing `pyproject.toml`, `src/uav_ugv_verification/`, `profiles/`, and `tests/`. Register checks through `fault_management` in M2. ROS wrappers call the library; numerical logic has a single implementation.

| Package area | Behavior | Dependencies |
| --- | --- | --- |
| `contracts.py`, `validation.py` | Typed records, schema checks, profiles, manifests, results | No ROS/Gazebo |
| `adapters/` | Normalized JSONL, dataset CSV, ROS bag readers and setup mappings | ROS imports confined to bag adapter |
| `signals/` | Matching, covariance, geometry, validity, event intervals | Numerical code only |
| `monitors/` | R1-R8 and shared bounded-response/hold operations | Canonical signals; no topic names or world-specific branches |
| `reporting/`, `__main__.py` | CLI, JSON/CSV, static plots, aggregation | Results and plotting dependencies |

Constraints:

- Use versioned JSON manifests/results, JSONL numerical streams/events, and YAML profiles. Validate finite values and required fields; encode unavailable metrics as null with a reason. Pin dependency versions actually tested.
- Implement only temporal operations needed by R1-R8, with mathematical definitions and tested finite-trace behavior. No generic STL parser, engine plug-in framework, or mandatory RTAMT dependency. Any later engine substitution must match the same semantic fixtures.
- Report physical clearance/error margins and response-time slack with units. Do not call these general STL robustness or combine unlike units into one score. Search one requirement's declared scalar objective at a time.
- Preserve legacy scorer behavior when extracting shared helpers; update both callers and regression tests together.
- Support a planar disk footprint, oriented boxes, and upright cylinders; sparse/dense worlds already contain cylinders. Configuration covers names, dimensions, thresholds, and frames; adapters handle different message/geometry semantics.

### Canonical run and signal contract

A run directory contains `manifest.json`, `events.jsonl`, source recordings, and optional cached numerical streams. The manifest records source paths/checksums, adapter/profile/schema versions, effective parameters, world/geometry identity, seed, code commit/dirty patch, dependency revisions, mission interval, and completion status. Existing CSV and `faults.json` sources are converted through adapters.

Canonical streams are `truth_pose`, `estimated_pose`, `uav_observation`, `goal`, `route`, `grid`, `requested_command`, `actuator_input`, `actuator_output`, `measured_motion`, `guard_state`, `obstacles`, and `detector_decision`. Include available streams only; profiles declare required capabilities. Events identify actual fault onset/recovery, hazard appearance/observation, goal changes, and run lifecycle. Planned schedules remain separate from observed events.

Records preserve acquisition stamp, frame, units, validity, source identity, clock domain/epoch, and available consumer receipt timing. Use integer nanoseconds internally. Positions use ENU `map`, body data FLU; transforms include covariance and local origin. Simulation and steady time are separate. Do not subtract unsynchronized clocks or treat bag storage time as actuator/controller receipt time.

Read incrementally with bounded matching buffers and skip camera image deserialization. Cache numerical streams by source/adapter hashes. Validate per-stream ordering and split clock resets into epochs. Declare matching gaps; interpolate continuous pose only within those gaps. Never interpolate discrete events/commands or use future samples for causal checks.

Evaluation accepts a normalized trace reader and profile and returns typed requirement results. Adapters do not decide verdicts; reports do not recompute monitor logic. Field schemas and the public API are defined in [interfaces](interfaces.md). Apply the reference's [temporal rules and analytic fixtures](implementation_reference.md#3-temporal-evaluation-rules); boundary behavior is part of the interface.

### Verdict and result contract

Each requirement result includes ID/version, verdict/reason, interval/clock, parameters, eligible duration, trigger/completed-obligation counts, missing-data counts, margin/latency units, and supporting timestamps. Execution/data-quality status stays separate.

| Verdict | Meaning |
| --- | --- |
| `PASS` | Property holds over the declared interval with sufficient evidence and required trigger coverage. |
| `FAIL` | A witnessed violation exists, even if another interval is incomplete. |
| `INCONCLUSIVE` | Required data, trigger coverage, or deadline coverage is missing/invalid. |
| `NOT_APPLICABLE` | Profile intentionally excludes the property, with a reason. |

An untriggered implication is unexercised and inconclusive for empirical verification; mathematical vacuity may be reported separately. Startup failures, aborted/out-of-envelope runs, timeouts, and excluded intervals remain in aggregate counts. Freeze thresholds before held-out evaluation and retain original results after code/requirement revisions.

### CLI

Implemented in M1; run in the [package environment](../../verification/README.md#install):

```bash
python3 -m pip install -e './verification[test]'
python3 -m uav_ugv_verification inspect RUN --profile PROFILE
python3 -m uav_ugv_verification evaluate RUN --profile PROFILE --output OUTPUT
python3 -m uav_ugv_verification report RESULTS --output OUTPUT
python3 -m pytest verification/tests
```

`inspect` lists capabilities/missing evidence. `evaluate` writes per-run JSON/CSV and returns 0 when all mandatory applicable requirements pass, 1 for a witnessed mandatory failure, and 2 for invalid configuration/data or inconclusive mandatory evidence. Failure takes precedence over inconclusive evidence. Optional results stay visible; configuration failure and evaluated inconclusiveness are distinct structured statuses. `report` consumes retained results without rerunning monitors.

Overall success requires at least one mandatory applicable requirement; an empty or entirely excluded requirement set returns 2 with `no_applicable_requirements`.

### Additional setups

Each setup supplies topic/field mappings, transforms, footprint/obstacle geometry, clocks, event sources, and capability/profile declarations. Adapter conformance requires a small recording, expected normalized output, and a regression fixture. Sensor-only profiles exclude navigation requirements. Setup changes do not modify monitors unless a requirement changes.

## 4. Requirements

Formal definitions, executable interpretations, dependencies, and scope are defined in [requirements](requirements.md). Retain R1-R8 IDs and version revisions. Constants remain provisional until M2 calibration; unmeasured bounds are labeled uncalibrated.

| Requirement | M1: implement against normalized fixtures | M2: supply or validate real evidence |
| --- | --- | --- |
| R1: clearance | Disk-to-box/cylinder clearance and contact tolerance. Report a sampled guarantee unless conservative swept bounds are implemented; missing intersample coverage cannot establish continuous collision freedom. | Independent collision geometry/transforms, footprint, sampling limits; distinguish proxy overlap from physical contact |
| R2: arrival | Goal-revision deadline and stopped dwell. Separate physical/estimated arrival. Preserve existing scorer tolerances as named parameters; specify mission start and reference length `L`. | Goal/start events, truth, commands, estimate; compare legacy scorer |
| R3: pose guard | Bounded response plus zero linear/angular commands until recovery. Include invalid frame/covariance/stamp and receipt age. Explicitly label acquisition-only scope. | Actual guard validity/reason and consumer timing; zero tolerances |
| R4: command loss | Separate watchdog-output expiry and physical settling. Measure expiry from last accepted actuator input, counting delivery once; use steady time. Paused simulation cannot establish physical settling. | Actuator instrumentation, existing 0.35 s timeout/50 ms delivery, calibrated physical bound |
| R5: hazard | Separate appearance-to-grid and grid-to-response latency. Accept a fresh clear route for the active goal or sustained zero commands; report physical stop separately. Validate route against independent geometry. | Actual spawn/observation, raw grids, route revisions, geometry; calibrate `d_min` from latency/braking with consistent distance definitions |
| R6: position error | Match estimate/truth within gaps and check conditional 0.5 m bound. Also report eligible duration, UAV availability, full-run error, missing coverage, and startup scope. | Acquisition stamps, covariance, truth, UAV observations |
| R7: detection | Configurable alarm scope and per-profile latency. Require new matching alarms after onset; count already-active alarms separately. Version binary/channel checks separately from class-specific R7; never derive predicted classes from labels. | Actual detector/event schema; current binary decisions, `short`/`noise`/`constant` labels, window spans ~2 s IMU, 5.2 s wheel, 20 s GPS |
| R8: false alarms | Separate fault-free per-run probability, alarm events/time, and window metrics. Declare warmup/recovery/event merging. Confidence intervals/counts use independent source runs, not correlated windows. | Held-out source IDs, nominal exposure, detector decisions; separate offline accuracy from closed-loop benefit |

## 5. Milestones

### M1: complete offline evaluator

**Dependencies:** existing numerical/scoring source. No simulator dependency.

**Files:** `verification/`, `docs/verification/requirements.md`, `docs/verification/interfaces.md`.

**Implement:**

1. Define typed input/profile/result contracts and capability validation. Add navigation, GPS-outage, pose-guard, command-loss, hazard, detector, and sensor-only profiles with mandatory/optional requirement declarations.
2. Add normalized JSONL/current dataset CSV readers, CLI, JSON/CSV results, and basic static plots. Implement R2/R3/R6 first, then R1/R4/R5/R7/R8 through the same result interface. Complete all eight offline checks in this milestone.
3. Add compact fixtures with independently specified expected outcomes: pass, violation, incomplete, unsupported, untriggered, and deadline boundaries. Include every analytic case in the implementation reference, box/cylinder geometry invariance, and the empty-requirement exit case.
4. Document schemas, installation, commands, profile values, sampled-evidence limits, and supported geometry. Add a sample run/profile/result for dependent work.

**Acceptance:** imports without ROS/Gazebo; CLI evaluates a fixture end to end; every requirement has a known pass/failure and relevant missing-data/boundary cases; exit codes match the contract; repeated evaluation reproduces numerical output. Expected outcomes cannot depend solely on the helper being tested.

**Validation:** install the package, run the section 3 commands against committed small fixtures, and regenerate reports without raw sources. Separate generated-at metadata from reproducible numerical fields.

**Outputs:** package/API, schemas, profiles, fixtures, sample results, and test results.

### M2: simulator integration and cross-setup reuse

**Implemented:** [integration commands and contracts](simulator_integration.md), [pilot evidence and calibration](../project/results/verification/m2/README.md). Camera and controlled setups use the same monitors. Paused physical settling remains inconclusive; hazard and online detector producers remain unavailable.

**Dependencies:** M1 API/fixtures and existing recording paths. Unavailable hazard/detector signals are reported as missing capabilities.

**Files:** bag/setup adapters, integration fixtures, recording/instrumentation in `fault_management`, `simulation_support`, `ugv_control`, and required message definitions; D01-D06 scorer/launcher corrections under `scripts/` and `mission_manager/launch/`. Schema changes are versioned.

**Implement:**

1. Add the ROS bag reader and setup mappings. Extend recording for goal revisions, authoritative routes, guard state, independent geometry, actuator receipt/output, and measured motion. Record receipt at the consumer/actuator rather than inferring it from ROS publication.
2. Consume actual fault/hazard logs and detector decisions where producers exist. Test absent interfaces with contract fixtures and report unavailable evidence. Do not add new hazard behavior, detector models, or supervisor policy to fill a dependency.
3. Reproduce and correct D01-D06 with targeted regressions. Reuse controlled guard/watchdog fixtures; check normal, slow, and frozen simulation clocks. Compare metrics with explicit clock/scope labels. Calibrate new bounds on separate pilot runs and freeze profiles before M3.
4. Evaluate camera navigation and controlled navigation/watchdog trials using unchanged applicable monitors. Ingest a sensor-only CSV run with explicit navigation exclusions. Register checks through `fault_management`'s `colcon test` workflow and declare dependencies; required checks cannot silently skip.

**Acceptance:** a real navigation recording reaches the complete pipeline; controlled cases provide R3/R4 evidence; box/cylinder geometry supports R1; two setups share monitor code; sensor-only data cannot claim navigation success. D01-D06 have regressions and verified corrections for enabled capabilities. Integrate R5/R7/R8 when their producers are available, naming omissions. Incomplete historical recordings stay inconclusive.

**Validation in Linux:** build/test touched packages, run guard/watchdog and camera-navigation trials, and evaluate their recordings. Rerun paired localization trials when measurement/filter behavior changes. Stop processes started by the trials. Fixture results do not satisfy real-recording checks.

**Outputs:** setup profiles, recording commands, results, source checksums, pilot IDs, calibrated bounds, and capability matrix.

### M3: experiments and failure search

**Dependencies:** M2 capabilities and calibrated profiles. Unsupported factors are explicitly excluded.

**Files:** scenario/record/evaluate integration in `fault_management` and scripts, experiment profiles, search/replay under `verification/`, and compact results under `docs/project/results/verification/`. Reuse existing launch/cleanup helpers.

**Implement:**

1. Connect every attempted run to its manifest, actual events, recording, verdicts, and terminal status. Add bounded readiness, isolated ROS/Gazebo domains, cleanup on all exits, deterministic resume keys, and separately identified rerun attempts. Run full PX4 trials serially unless port/instance/resource isolation is tested.
2. Publish the fixed matrix before execution: four layouts, two goals, nominal/GPS-outage cases, and controlled guard/command-loss rows. Use three predetermined seeds per selected row as coverage, not high-confidence probability estimates. Record selected combinations and total run count.
3. Add supported hazard and detector-enabled/disabled/forced-miss comparisons. Separate training/calibration/validation source runs from held-out tests. Pair seeds/worlds/goals without implying identical feedback trajectories.
4. Add seeded random or stratified direct Gazebo search with trial/wall-time budgets, one objective per requirement, and a durable ledger. Defaults: minimum sampled clearance for R1; deadline slack for R3/R5. Missing evidence is an invalid attempt, not a favorable objective. Replay candidate failures in fresh worlds.

**Acceptance:** every attempt appears in aggregate counts, including startup failures/aborts/exclusions; the matrix and budgeted search run in Linux; replay counterexamples or report the exact completed budget with none found. Preserve original failures and identify reruns after fixes. No surrogate or advanced optimizer is required.

**Validation:** test interruption/resume/status propagation, then execute the frozen matrix/search. Store launch/evaluation/replay commands, code/profile hashes, search seed, and ledger. Code changes after freeze start a new cohort; do not merge incompatible versions into one success rate.

**Outputs:** machine-readable matrix, attempt ledger, per-run provenance/results, counterexamples, exclusions, and a versioned result cohort.

### M4: report and demo

**Dependencies:** M1 result schema; M3 cohort for final results.

**Files:** `verification/reporting/`, demo wrapper/profiles, usage documentation, and figures/tables under `docs/project/results/verification/`.

**Implement:**

1. Generate architecture/requirement/capability tables, experiment matrix, localization/arrival plots, guard/watchdog timing, clearance/hazard plots, and supported detector statistics. Every quantitative figure identifies cohort, profile/version, counts/exposure, and exclusions. Use static exportable figures for report sections 3-6.
2. Produce a claim/evidence/limitation table separating sampled traces, statistical results, falsification budgets, and any actual discrete proof. Compare UAV assistance, deterministic guards despite detector failures, and hazard response only for completed supported trials.
3. Provide one documented demo command that reuses runner launch/record/evaluate/cleanup. Rehearse an assisted run plus guard, command-loss, or supported hazard response. Mandatory violations/incomplete evidence prevent a successful verification summary. Label deliberate fixture violations separately.
4. Provide a clearly labeled recorded replay and commands to regenerate tables from frozen results. Keep large recordings on shared storage with checksums/availability notes; commit compact evidence. Record measured rehearsal startup/run duration.

**Acceptance:** documented commands run without source edits; saved results reproduce numerical tables without simulation; live demonstration/cleanup succeeds in Linux; every claim links to evidence or is marked unmeasured. Offline detector accuracy does not establish closed-loop recovery.

**Validation:** regenerate reports and check provenance, rehearse the full demo including shutdown, and check interrupted cleanup. Record which checks require ROS/Gazebo versus saved results only.

**Outputs:** demo commands/replay, figures/tables, claim/evidence index, cohort IDs, and measurement limits. Absence of a counterexample within a finite search budget is not proof of continuous safety.

## 6. Optional E1: supervisor model and conformance

Optional extension corresponding to completion-plan W10. Independent of M1-M4.

- Files: `verification/supervisor/`, conformance tests, and `ugv_control`. Model reachable recovery counters, goal revisions, resets, and other memory; five named states times 32 Boolean inputs is incomplete when memory exists.
- Check temporal safety properties independently of the shared transition table, then compare C++ behavior over reachable sequences. Distinguish transition enumeration from temporal model checking.
- Build/test touched packages and rerun guard/watchdog checks in Linux. Report explored states/transitions, assumptions, counterexamples, and conformance coverage. Timing/physical stopping remain trace-monitor claims.
- If omitted, remove model-checking claims from the paper and retain measured guard evidence.

## 7. Validation

- Run core fixtures without ROS/Gazebo. Run ROS 2 Humble, PX4 SITL, Gazebo Harmonic, and touched packages' registered checks in Linux.
- Retain semantic counterexamples, exact commands, results, and source/profile versions. Report fixture and simulator validation separately; missing runtime data does not satisfy acceptance checks.
- Verify the reference's source findings against the implementation revision; report static findings separately from reproduced runtime failures. Runtime corrections require touched-package builds/tests and the relevant Linux regression trials.
- Record unavailable signals and pending checks. Stop processes started by validation; preserve unrelated processes and files.
- Require M1-M3 results before final M4 figures. Reporting may use M1 fixtures during development, labeled synthetic.
- Include formal requirements, architecture, assumptions, methods, quantitative results, and limits in the 10-page report. References: [report template](https://docs.google.com/document/d/1lMb6jLL3d95x2ixvJQQkNiAKz2m37j4iMq9wWyq-SWA/edit), [course introduction](https://jdeshmukh.github.io/teaching/cs513-autocps-fall-2026/slides/1-CourseIntro.pptx), [schedule](https://jdeshmukh.github.io/teaching/cs513-autocps-fall-2026/schedule.html).
