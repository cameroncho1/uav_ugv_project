# Verification validation

Date: 2026-11-11. M1-M4 implemented. Recorded requirement failures and unavailable capabilities remain explicit.

## Implemented behavior

- Installable numerical package, typed contracts, strict validation, normalized JSONL/current dataset CSV adapters, R1–R8 monitors, CLI, JSON/CSV results, and saved-result plots.
- Humble sqlite3 bag import with disk-backed acquisition sorting, clock epochs, native sequence gaps, source hashes, setup mappings, independent static box/cylinder extraction, and profile clock binding.
- Consumer guard and command-publication telemetry, actuator receipt/output diagnostics independent of command transport, applied GPS fault events, and acquisition/receipt-aware physical-motion measurements.
- Shared numerical recorder, configuration-aware navigation scoring, simulation-time arrival dwell/deadline, independent wall watchdog, consistent world/goal/assistance forwarding, and expanded bag topics/QoS.
- Sustained-guard regressions covering zero commands after the response deadline, actual resumed motion, and a witnessed response before command prehistory is available.
- Frozen serial experiment plans, durable attempt accounting, bounded readiness/search, interruption recovery, source consistency checks, and owned-process cleanup.
- Portable saved-result publication with evaluated bounds, assistance pairs, claim/evidence limits, eight static figures, and headless live/replay demonstration commands.

## Validation

| Check | Result |
| --- | --- |
| Offline Windows Python 3.10 suite | 122 tests pass; numerical-package Ruff lint/format pass |
| Linux touched packages | `uav_ugv_msgs`, `ugv_control`, `simulation_support`, `fault_management`, `mission_manager` build successfully |
| Registered Linux checks | 219 checks; zero errors/failures; eight cppcheck skips for headers or packages without C++ sources; functional checks run without skips |
| Real CDR regressions | Acquisition timestamps, storage insertion order across reset, unselected invalid image blobs, missing historical telemetry, native sequence loss, stable source-run identity, and mission boundaries |
| Controlled navigation | 23 cases pass, including slow/frozen clocks; R3 completes five obligations |
| Camera navigation | Revised-goal and applied-GPS-outage recordings pass R1/R2/R6 within the declared first-drive mission interval |
| Actuator command loss | Controller/bridge loss tested at normal and slow clocks; R4 passes. Paused clocks retain output-expiry evidence and return physical R4 inconclusive |
| Sensor-only CSV | `run_1030_noise_2` imports; all navigation/detector checks explicitly excluded; evaluation exits 2 |
| Synthetic example | R1–R8 pass; labeled synthetic, separate from simulation evidence |
| Fixed experiments | 45 matrix attempts, four clearance searches, two reproducing R2 diagnostic replays; every attempted outcome retained |
| Live demonstration | Assisted arrival, guard response, controller death, and transport loss pass; 159.175 wall seconds; first drive at 34.766 seconds |
| Interruption | Exit 130, durable interrupted ledger row, finalized recording, zero remaining owned processes; cleanup 6.076 seconds |
| Saved-result publication | Six-page PDF; eight static figures; eight numerical CSV tables reproduce byte-for-byte from a fresh wheel install; replay needs no ROS |

Linux environment: Ubuntu 22.04/WSL, ROS 2 Humble, Gazebo 8.15.0, Python 3.10. Tested Python dependencies are in [requirements-tested.txt](../../verification/requirements-tested.txt). pytest 8.4.2 is compatible with Humble's installed test plugin.

Commands, frozen pilot bounds, individual result files, source checksums, unsuccessful pilots, and recording locations are in the [integration guide](simulator_integration.md) and [M2 evidence index](../project/results/verification/m2/README.md). The evidence directory contains metadata/results, not large raw bags. Trial processes are stopped after evaluation.

## Scope and recorded limits

- R5 and online R7/R8 require actual hazard/detector producers. Their absence remains explicit; no new behavior/model fills those interfaces.
- Camera R3 is optional and unexercised in the scoped missions. Earlier camera telemetry also lacks the separate command-publication timestamp. The final controlled fixture establishes R3 timing.
- Calibrated profiles cover the recorded pilot conditions, not arbitrary loads or future setups. Sampling gaps, uncertainty, and scheduling overruns remain observable incomplete/failed outcomes.
- M3 executes 45 fixed matrix attempts, four clearance-search trials, and two supplemental R2 replays. Dense/B arrival failures reproduce; incomplete starts and aborted guard trials remain in the ledger. See the [M3 evidence](../project/results/verification/m3/README.md).
- M4 exports the final figures, claim/evidence table, and rehearsed live/recorded demo. [M4 evidence](../project/results/verification/m4/README.md) records timing, cleanup, package validation, and reproduction. No supervisor model checking is implemented.

Measurement/filter behavior is unchanged. This change instruments and evaluates existing interfaces; paired localization trials are not required by the repository's measurement-change rule.
