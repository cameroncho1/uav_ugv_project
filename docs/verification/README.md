# Requirements and verification

[Documentation index](../README.md) | [Completion plan](../project/completion_plan.md)

Offline evaluation of recorded behavior against R1-R8, using normalized signals and recording adapters.

Status: M1-M4 and W10/E1 implemented: offline evaluation, simulator integration, recorded experiments, and a rehearsed report/demo. Recorded failures and unavailable capabilities remain explicit.

- [Navigation supervisor](supervisor.md): persistent holds, recovery timing, goal revisions, exhaustive properties, and C++ conformance.
- [Supervisor evidence](../project/results/supervisor/README.md): source-bound proof, controlled ROS cases, simulator regressions, and two-page paper material.
- [Course alignment](supervisor_course_alignment.md): formal-method expectations, report sections, source provenance, and claim limits.
- [Technical specification](milestone_plan.md): architecture, data contracts, requirements, four milestones, and acceptance checks.
- [CLI and examples](../../verification/README.md): installation, evaluation, saved-result reporting, and tests.
- [Simulator integration](simulator_integration.md): Linux build/record/import commands, telemetry, setup extension, and calibrated profiles.
- [Experiment execution](experiments.md): frozen matrix, durable ledger, bounded search, replay, cleanup, and portable exports.
- [M3 evidence](../project/results/verification/m3/README.md): all 51 attempts, reproduced arrival failures, startup diagnostics, and the completed search budget.
- [Demo and paper commands](demo.md): headless live demonstration, cancellation check, recorded replay, and saved-result publication.
- [M4 evidence](../project/results/verification/m4/README.md): six-page report, live/replay timings, interruption cleanup, and reproducible tables.
- [M2 evidence](../project/results/verification/m2/README.md): camera, guard, and watchdog recordings, failed pilots, calibration, and limits.
- [Interfaces](interfaces.md): normalized records, profiles, adapter API, and result schemas.
- [Executable requirements](requirements.md): R1-R8 definitions, timing semantics, parameters, and limits.
- [Validation status](status.md): completed checks and remaining integration work.
- [Implementation reference](implementation_reference.md): verified APIs, signal inventory, temporal fixtures, geometry/statistics, and source-confirmed corrections.
- [Navigation](../navigation.md): existing guards, route contract, command watchdog, and scoring behavior.
- [Localization interfaces](../cooperative_localization/interfaces.md): measurement frames, acquisition timestamps, covariance, and paired trials.
- [Sensor recording](../sensor_dataset.md): current bag and CSV collection procedures.
