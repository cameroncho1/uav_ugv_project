# Verification validation

Date: 2026-11-10. M1: offline evaluator implemented. M2-M4: pending.

## M1

- Installable Python package, public trace-reader API, normalized JSONL and current dataset CSV adapters.
- Versioned schemas/profiles/results, capability inventory, strict validation, sequence/epoch/gap handling, deterministic exit codes.
- R1-R8 checks with box/cylinder geometry, arrival dwell, sustained guard holds, actuator expiry/settling, hazard responses, acquisition-matched error, detector scopes, and nominal false-alarm exposure.
- JSON/CSV output and saved-result SVG reporting; evaluator/source/profile hashes, separated cohorts, source-run-grouped exact detector statistics.
- Seven provisional setup profiles and a labeled synthetic example exercising all eight checks. No ROS/Gazebo imports or simulation processes.

Environment: Windows x86-64, Python 3.10.19. Direct tested dependencies are in [requirements-tested.txt](../../verification/requirements-tested.txt). Linux/ROS compatibility is not established by these checks.

## Validation commands

Run from the repository root after [installation](../../verification/README.md#install):

```bash
python -m pytest verification/tests
python -m uav_ugv_verification inspect verification/examples/full_run \
  --profile verification/examples/full_run.yaml
python -m uav_ugv_verification evaluate verification/examples/full_run \
  --profile verification/examples/full_run.yaml --output verification/outputs/example
python -m uav_ugv_verification report verification/outputs/example \
  --output verification/outputs/report
```

Additional development checks (install `verification[dev]`):

```bash
ruff check verification/src verification/tests
ruff format --check verification/src verification/tests
python -m build verification --outdir verification/dist --no-isolation
```

Validation: 85 tests pass; Ruff lint/format checks pass. Source distribution and wheel build successfully; an isolated wheel installation imports the package and evaluates the example without ROS/Gazebo. The tests include numeric boundary/counterexample cases, CSV labels/completion metadata, invalid schemas, public API, CLI status/error artifacts, report regeneration without recordings, source-run clustering, and bounded-memory alignment of 20,002 generated records, including discarded prehistory. The example returns `PASS` for R1-R8 and exit code 0; its signals are synthetic, not a simulated vehicle trajectory.

Existing CSV input checked: `uav_ugv_project_fault_detection/datasets/ugv_faulty_long/run_1030_noise_2`. Inventory: 15,000 IMU, 5,769 wheel, 1,500 GPS, 4,412 truth, 6,000 command samples, and two actual label-transition events. Source ID: `run_1030`. This confirms adapter compatibility with that recording; it supplies no navigation or online-detector verdict.

## M2 prerequisites and scope

- Implement Humble bag decoding and acquisition/receipt-aware recording; register library checks with `colcon test`.
- Add real guard, applied-fault, geometry, actuator receipt/output, and physical-motion telemetry. Normalize source clocks/reference points before evaluation.
- Reproduce/correct D01-D06 in the [implementation reference](implementation_reference.md#6-required-adjacent-corrections), including baseline scoring, world/goal settings, event onset, clock labels, stopping measurements, and missing topics.
- Run Linux builds/tests and controlled guard/watchdog plus camera-navigation trials. Demonstrate unchanged monitor code across two setups.
- Calibrate/freeze timing/gap/uncertainty profiles on pilot runs before M3. Preserve unavailable hazard/detector capabilities as explicit missing evidence.

Bag reading, runtime instrumentation, simulator trials, supervisor model checking, and final experiment/report claims are not completed by M1. Root `AGENTS.md` records the numerical library boundary; runtime localization, navigation, and actuator code are unchanged.
