# M2 integration evidence

Date: 2026-11-10. Environment: Ubuntu 22.04 in WSL, Python 3.10, ROS 2 Humble, Gazebo 8.15.0. These runs calibrate integration profiles; they are excluded from M3 held-out results.

## Recorded outcomes

| Recording | Scope | Result | Measured value |
| --- | --- | --- | --- |
| [Camera nominal](camera-nominal/result.json) | Goal `(3.0, 0.5)`, assisted, GPS enabled, empty world | R1/R2/R6 PASS | Arrival 14.928 simulation seconds after first drive; maximum matched error 0.0822 m |
| [Camera GPS outage](camera-gps-outage/result.json) | Goal `(4.0, 0.0)`, assisted, actual GPS disable at 34.256 simulation seconds | R1/R2/R6 PASS | Arrival 16.752 simulation seconds after first drive; maximum matched error 0.0820 m |
| [Guard](guard/result.json) | 23 controlled cases, including slow/frozen clock, invalid frame/stamp/covariance, stale pose and route | R3 PASS | Five invalid-guard obligations; maximum observed command-publication response 0.114 ms |
| [Controller death](controller_death/result.json) | Normal clock | R4 PASS | Output expiry 385.8 ms; measured settling onset 456.8 ms |
| [Command transport loss](command_transport_loss/result.json) | Normal clock; independent diagnostic bridge | R4 PASS | Output expiry 383.5 ms; measured settling onset 460.9 ms |
| [Controller death, slow](controller_death_slow/result.json) | Requested real-time factor 0.2 | R4 PASS | Output expiry 358.6 ms; measured settling onset 606.8 ms |
| [Transport loss, slow](command_transport_loss_slow/result.json) | Requested real-time factor 0.2 | R4 PASS | Output expiry 361.6 ms; measured settling onset 604.4 ms |
| [Controller death, paused](controller_death_frozen/result.json) | Actual Gazebo pause acknowledged; truth acquisition stamp remains frozen | R4 INCONCLUSIVE | Output expiry 351.5 ms; physical settling unavailable |
| [Transport loss, paused](command_transport_loss_frozen/result.json) | Same paused-clock condition | R4 INCONCLUSIVE | Output expiry 351.9 ms; physical settling unavailable |

R4 times use steady time from the last accepted actuator input, not from the driver's kill timestamp. Settling is the onset of a confirmed 200 ms quiet dwell; position differencing supplies sampling uncertainty. The paused fixture's successful execution is separate from its deliberately inconclusive physical verdict.

Camera R3 remains optional and inconclusive: no guard onset occurs in the selected mission interval, and these early camera recordings precede the separate command-publication timestamp. The final controlled recording supplies that timestamp and exercises R3. Camera R1 covers an empty world; box/cylinder extraction and clearance have independent numerical regressions. This is not an obstacle-course safety result.

## Calibration

Frozen templates: [camera](../../../../../verification/profiles/m2_camera.yaml), [guard](../../../../../verification/profiles/m2_guard.yaml), [watchdog](../../../../../verification/profiles/m2_watchdog.yaml). Binding changes the boot clock only. The thresholds retain the M1 provisional values; integration pilots establish their use under these recorded conditions.

| Parameter | Frozen value | Pilot basis |
| --- | --- | --- |
| R3 response | 500 ms | Observed publication response at most 0.114 ms in five obligations |
| Steady guard freshness gap | 250 ms | Recorded control cadence at most 101.8 ms in the nominal camera mission; final fixture at most 97.4 ms |
| R4 expiry | 400 ms | Configured 350 ms timeout plus nominal 50 ms worker delivery; retained calibration runs reach 379.9 ms; final normal-clock rerun reaches 385.8 ms |
| R4 settling and dwell | 750 ms onset; 200 ms dwell | Retained calibration onset 432.9–606.8 ms; final normal reruns 456.8/460.9 ms; sampling uncertainty is checked by the monitor |
| R4 output/motion freshness | 100 ms | Output intervals about 50 ms; truth intervals about 20 ms in controlled runs |
| Camera simulation freshness | 500 ms | Nominal estimate gap at most 60 ms; relative-observation gap at most 200 ms |
| R2 arrival | 60 simulation seconds; 1 second dwell | Pilot arrivals 14.928 and 16.752 seconds after first drive |

The 50 ms delivery period is not a hard scheduling guarantee. Scheduling overruns, lost samples, and deadline/uncertainty overlap remain observable failures or incomplete evidence. `cadence.json` in each recording directory contains the measured positive acquisition/receipt intervals. These profiles do not establish bounds for arbitrary simulator loads or future setups; new conditions require separate calibration and a new cohort.

## Corrections and unsuccessful pilots

- D01: assistance-disabled scoring accepts zero unused UAV inputs; assisted scoring still requires them. Physical arrival and configuration checks are separate. Numerical regression covers both configurations and missing assisted input.
- D02: world, goal, and assistance settings propagate through launch/scoring/recording. The revised camera goal is exercised above; controlled slow trials load a separate generated SDF. Sparse/dense box/cylinder worlds are parsed in regressions.
- D03: the GPS callback emits actual applied transitions. The outage recording includes initial state and onset; request, acknowledgement, and application remain separate fields. Failed or delayed acknowledgement alone cannot establish onset.
- D04: simulation time drives arrival dwell/deadline; wall time bounds the process. Frozen/slow semantic tests and live guard/watchdog runs distinguish the clocks.
- D05: actuator receipt/output and measured motion supplement the original late-displacement regression. Frozen truth messages cannot establish physical settling.
- D06: recording includes `/local_grid` and actual guard, watchdog, motion, and fault telemetry. Native counter loss and missing historical topics have CDR regressions.
- The [first camera pilot](camera-startup-failure/legacy_result.json) physically arrived but failed configuration checks because a CRLF Python shebang prevented sensor startup. Python scripts now retain LF on Windows checkout. Its failure is retained.
- The [first paused fixture](frozen-fixture-failure/legacy_result.json) incorrectly expected no truth publications during pause. Gazebo continues publishing a frozen acquisition stamp. The fixture now checks the stamp; the physical verdict remains inconclusive.
- The initial guard pilot exposed an evaluator defect: quiet emissions after the response deadline were classified as resumed motion. The corrected monitor checks the sustained zero hold, with separate regressions for late zeros and actual resumed commands. A witnessed response also does not require command coverage before its first emission; subsequent hold coverage remains required.

## Artifacts and reproduction

[ledger.json](ledger.json) indexes nine evaluated recordings, two unsuccessful pilot attempts, and the initial guard-monitor diagnostic. Each evaluated directory contains saved results, CSV requirements, bound profile, input manifest, and cadence summary. Input manifests are metadata copies; the referenced numerical streams are not included in this compact evidence directory.

Raw bags and normalized streams remain at the Linux paths in the ledger under `/home/rylanwwade/uav-verification-m2/evidence`. Source bag/setup/world/event/adapter hashes are embedded in the saved manifests/results. These machine-local recordings are not portable repository fixtures. No images are recorded. Use the [integration commands](../../../../verification/simulator_integration.md) to create fresh runs.

The saved [Linux check output](colcon-test-result.txt) reports 197 checks, zero errors/failures, and eight cppcheck skips for headers or packages without C++ sources. [Environment versions](validation.json) identify the tested dependencies. Required functional tests, including two real CDR bag regressions, run without skips. The offline suite has 102 tests. The [sensor-only CSV result](sensor-only/result.json), `run_1030_noise_2`, evaluates all R1–R8 as explicitly not applicable and exits 2; it cannot claim navigation success.

R5 and online R7/R8 remain unavailable because hazard and detector-decision producers are absent. The adapter accepts their canonical event/decision contracts when present. Estimator measurement/filter behavior is unchanged; paired localization trials are not required for this instrumentation change. Trial processes are stopped after evaluation. Gazebo emitted one segmentation fault during teardown of the successful outage trial, after recording/scoring completed; no runtime arrival or localization failure is attributed to that shutdown.
