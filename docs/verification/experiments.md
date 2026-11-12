# Experiment execution

The frozen [matrix](../../verification/experiments/matrix.yaml) contains 45 attempts: 36 assisted camera trials, three assistance-disabled comparisons, three guard-fixture repetitions, and three watchdog-fixture repetitions. Each watchdog attempt produces separate controller-death and command-transport-loss recordings.

## Matrix

- Layouts: empty, sparse, dense, corridor. Goals: A `(4.0, 0.0)` and B `(3.0, 0.5)`.
- Nominal assisted trials cover both goals in every layout. GPS-outage trials cover goal A in empty/sparse and goal B in dense/corridor. Every outage has a nominal counterpart with the same world, goal, and seed.
- Assistance-disabled trials pair with empty/A nominal runs. R6 is excluded because no cooperative observations are produced; R1/R2 remain mandatory.
- Seeds: 631, 632, 633. M2 calibration uses seed 513. Seeds configure synthetic sensor corruption; PX4/render scheduling is not deterministic.
- Hazard response and detector enabled/disabled/forced-miss comparisons are excluded because their producers are unavailable. No detection or hazard benefit is inferred from guard behavior.

Limits are fixed before execution: 100 simulation seconds in the legacy scorer, an independent 160-second scorer wall watchdog, and a 215-second outer attempt watchdog. R2 retains its calibrated 60-second deadline from the first-drive mission boundary. A short or interrupted trace stays incomplete rather than becoming a witnessed deadline failure.

## Commands

Use the Linux environment from [simulator integration](simulator_integration.md). From the repository root:

```bash
python scripts/verification_experiments.py verification/outputs/m3 --stage all
```

The runner uses server-only Gazebo, disables RViz, and launches processes in the background. It checks world control-service responses, native PX4 writer creation, and actual ROS observations with bounded waits. Producer/consumer nodes start before the ROS observation barrier. The guard fixture observes a controller command before starting its cases.

ROS domain IDs use 20-79, within the [documented Linux range](https://github.com/ros2/ros2_documentation/blob/humble/source/Concepts/Intermediate/About-Domain-ID.rst). Gazebo partitions and process groups are isolated per attempt. M3 retains its original initialization/domain policy; the startup corrections form the separate M4 demo cohort.

The matrix, search, and replay stages can also run separately. Repeating a command resumes completed work. Interrupted attempts remain in the ledger and receive a fresh attempt directory on retry. A source, world, profile, or plan change requires a new cohort directory. Terminal attempts cannot be edited into successes. Do not run two campaigns concurrently against XRCE port 8888; the runner holds an exclusive lock.

`freeze.json` contains the expanded plan and source hashes before the first attempt, including report templates. Source consistency is checked before every matrix, search, and replay attempt. SQLite WAL with full synchronization records each attempt before launch and makes terminal transitions durable. `ledger.json` is an atomic portable snapshot. Each attempt preserves configuration, command, effective environment, log, recording, evaluator verdicts, and terminal status. Startup failure, timeout, interruption, invalid data, and witnessed requirement failure remain separate outcomes. An explicit failed readiness report retains `startup_failed` status even when partial recordings exist.

The runner tracks its own process groups, including children that create new sessions. Timeout/cancellation gives existing cleanup traps time to finalize recording, then escalates only those owned groups. A parent-death signal interrupts the trial runner if its coordinator exits unexpectedly. A Linux regression checks cleanup of independent child sessions.

## Failure search and replay

Search uses seed 20261110 and at most four fresh Gazebo trials within a 900-second wall budget, reserving time for cleanup. Candidate worlds use sparse, dense, corridor, and narrow layouts with independently drawn obstacle x/y offsets in `[-0.25, 0.25]` m. Every generated SDF and offset is retained. The objective is minimum sampled R1 disk clearance in meters; incomplete evidence is an invalid objective, never a favorable score.

Up to two R1 failure candidates, ordered by clearance when complete, are replayed in fresh worlds with the same configuration. Replay attempts link to their originals. A matching seed does not guarantee an identical feedback trajectory. Results distinguish reproduced, non-reproduced, and incomplete replays; no counterexample within the budget is not a safety proof.

## Export

```bash
python scripts/verification_experiments.py verification/outputs/m3 --stage export \
  --export-output docs/project/results/verification/m3
```

The portable export retains every attempted outcome, saved results, bound profiles, input manifests, source hashes, generated search worlds, and display series with separate checksums. Readiness diagnostics survive export even when recording never starts. Figure inputs are downsampled with discrete state transitions retained; verdicts always use full-resolution recordings. Export verifies the numerical stream hashes against the saved result. Missing artifacts remain listed. Raw bags remain at the archive path recorded in `campaign.json`; their availability is explicit.

Reporting uses saved results and exported display series without starting simulation or recomputing verdicts. Counts retain incomplete/failed attempts. Calibration and test cohorts remain separate; repeated deterministic guard/watchdog fixtures are repeatability checks rather than independent statistical trials.
