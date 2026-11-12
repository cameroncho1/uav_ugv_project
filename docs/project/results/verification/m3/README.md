# M3 experiment evidence

Date: 2026-11-11. Ubuntu 22.04/WSL, ROS 2 Humble, PX4 SITL, Gazebo 8.15.0. All simulations run server-only. M2 calibration runs and the M4 demo remain separate cohorts.

## Outcomes

| Category | Attempts | Passed | Requirement failure | Incomplete | Startup abort |
| --- | --- | --- | --- | --- | --- |
| Frozen matrix | 45 | 34 | 6 | 3 | 2 |
| Seeded clearance search | 4 | 4 | 0 | 0 | 0 |
| Post-hoc R2 diagnostic replay | 2 | 0 | 2 | 0 | 0 |
| Total | 51 | 38 | 8 | 3 | 2 |

[Campaign](campaign.json), [durable ledger snapshot](ledger.json), and [frozen inputs](freeze.json) retain every attempt. The export contains 52 evaluated recordings: watchdog attempts each produce two recordings; two aborted guard attempts produce none. Counts describe recorded outcomes, not independent success-probability estimates.

The [matrix protocol](../../../../verification/experiments.md) specifies four layouts, goals A `(4.0, 0.0)` and B `(3.0, 0.5)`, selected paired GPS outages, assistance-disabled comparisons, and repeated guard/watchdog fixtures. Seeds 631-633 are disjoint from M2 calibration seed 513. This is the recorded short-course envelope; the completion plan's longer `(9, 3)` goal, dynamic hazards, online detector comparisons, and discrete supervisor proof are not evaluated.

## Counterexamples and startup diagnostics

| Evidence | Observation | Interpretation |
| --- | --- | --- |
| Matrix attempts 16-18 and 31-33 | Dense/B nominal and GPS-outage R2 FAIL for all three seeds; R1/R6 PASS | Witnessed arrival deadline failures. Planner logs repeatedly report the goal cell blocked. The perception/grid/planning root cause remains unresolved. |
| [Replay plan](diagnostic_R2_plan.json), [outcomes](diagnostic_summary.json) | Attempts 50/51 replay original attempts 16/17 in fresh worlds; both R2 FAIL | Both selected failures reproduce. Selection is post-hoc, outside the held-out matrix and original search budget. Original failures remain unchanged. |
| Matrix attempts 1-3 | Camera readiness misses PX4 odometry; recording/evaluation incomplete | Native XRCE logs show participant/session setup failures. These are incomplete starts, not witnessed goal deadline misses. Historical ledger status remains `inconclusive`. |
| Matrix attempts 40/41 | Guard fixture aborts before receiving a controller command | Endpoint discovery does not establish command delivery. No valid positive-duration normalized interval is available. Attempt 42 completes the guard fixture. |
| Matrix attempts 37-39 | Assistance-disabled empty/A trials pass R1/R2 | Their assisted counterparts have incomplete startup evidence; these pairs cannot establish a navigation benefit from UAV assistance. |

[Diagnostic excerpts and source checksums](diagnostics/index.json) retain the startup assertions, XRCE errors, readiness reports, and blocked-goal messages. No thresholds, world geometry, or navigation policy change during this cohort. Readiness corrections are evaluated separately in M4.

## Failure search

The frozen budget is four trials and at most 900 driver wall seconds, with search seed 20261110. Actual process/cleanup time is **274.824255 seconds** across all four trials; normalization/export time is outside that measurement. The sole objective is minimum sampled R1 disk clearance. No R1 counterexample is found in this budget; this does not prove continuous safety.

| Attempt | Generated layout | Trial seed | Minimum sampled clearance (m) | R1/R2 |
| --- | --- | --- | --- | --- |
| 46 | sparse | 700 | 0.130211 | PASS/PASS |
| 47 | dense | 701 | 0.166927 | PASS/PASS |
| 48 | corridor | 702 | 0.224085 | PASS/PASS |
| 49 | narrow | 703 | 0.157927 | PASS/PASS |

Generated [worlds](search_worlds/) and per-obstacle offsets remain in the ledger. No R1 candidate qualifies for automatic replay. The two R2 diagnostics are explicitly supplemental. Missing evidence would have no scalar objective.

## Reproduction and availability

Activate the Linux environment described in [simulator integration](../../../../verification/simulator_integration.md), then run from the repository root:

```bash
python scripts/verification_experiments.py verification/outputs/m3 --stage all
python docs/project/results/verification/m3/diagnostic_replay.py verification/outputs/m3 \
  --repository "$PWD" --requirement R2 --limit 2
python scripts/verification_experiments.py verification/outputs/m3 --stage export \
  --export-output verification/outputs/m3-export
```

Use a new directory for changed code/profiles. Repeating an unchanged campaign resumes its completed cases; interrupted attempts receive new identities. Matching seeds do not guarantee identical feedback trajectories. The supplemental diagnostic driver is a retained experiment artifact, not a new monitor implementation.

Raw bags and normalized traces occupy about 1.21 GB at `/home/rylanwwade/uav-verification-m2/evidence/m3_v1`. They remain machine-local, are not committed, and are not uploaded to shared storage. Saved input manifests/results contain source checksums. [Source archive metadata](source_archive.json) identifies the exact frozen source archive and its checksum. Restoring that archive preserves the original source bytes for resume; fresh runs on later code form new cohorts.

The portable export retains saved verdicts, profiles, input manifests, display series, and source hashes. Display series are compacted without changing their numeric content and carry separate checksums in `campaign.json`. Full-resolution traces determine verdicts. Canonical LF text keeps checksums stable across Windows/Linux checkouts. [Artifact checksums](artifacts.json) cover the committed evidence.

[Validation metadata](validation.json), [registered Linux checks](m3-final-test-result.txt), [build output](m3-final-build.txt), and the [owned-process cleanup regression](m3-process-test.txt) record validation. R5 and online R7/R8 remain unavailable; guard repeats do not substitute for those interfaces. Measurement/filter behavior is unchanged.
