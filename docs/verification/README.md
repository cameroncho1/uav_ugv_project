# Requirements and verification

[Documentation index](../README.md) | [Completion plan](../project/completion_plan.md)

Offline evaluation of recorded behavior against R1-R8, using normalized signals and recording adapters.

Status: M1 offline evaluator implemented. Simulator integration and runtime validation are M2.

- [Technical specification](milestone_plan.md): architecture, data contracts, requirements, four milestones, and acceptance checks.
- [CLI and examples](../../verification/README.md): installation, evaluation, saved-result reporting, and tests.
- [Interfaces](interfaces.md): normalized records, profiles, adapter API, and result schemas.
- [Executable requirements](requirements.md): R1-R8 definitions, timing semantics, parameters, and limits.
- [Validation status](status.md): completed checks and remaining integration work.
- [Implementation reference](implementation_reference.md): verified APIs, signal inventory, temporal fixtures, geometry/statistics, and source-confirmed corrections.
- [Navigation](../navigation.md): existing guards, route contract, command watchdog, and scoring behavior.
- [Localization interfaces](../cooperative_localization/interfaces.md): measurement frames, acquisition timestamps, covariance, and paired trials.
- [Sensor recording](../sensor_dataset.md): current bag and CSV collection procedures.
