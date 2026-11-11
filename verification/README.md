# Offline verification

R1-R8 monitors for normalized recordings and the existing sensor CSV dataset. The package imports without ROS, Gazebo, PX4, Torch, or a display.

- [Interfaces](../docs/verification/interfaces.md): schemas, streams, profiles, and results.
- [Requirements](../docs/verification/requirements.md): exact monitor behavior and scope.
- [Specification](../docs/verification/milestone_plan.md): M1-M4 acceptance checks.

## Install

Run from the repository root in an isolated Python >=3.10 environment:

```bash
python3 -m venv verification/.venv
source verification/.venv/bin/activate
python -m pip install -r verification/requirements-tested.txt -e './verification[test]'
```

Windows PowerShell activation: `.\verification\.venv\Scripts\Activate.ps1`. Alternatively, invoke `verification/.venv/Scripts/python.exe` directly. The tested dependency set is for Python 3.10; other supported Python versions may need the ranges in `pyproject.toml` instead.

## Synthetic example

```bash
python -m uav_ugv_verification inspect verification/examples/full_run \
  --profile verification/examples/full_run.yaml
python -m uav_ugv_verification evaluate verification/examples/full_run \
  --profile verification/examples/full_run.yaml --output verification/outputs/example
python -m uav_ugv_verification report verification/outputs/example \
  --output verification/outputs/report
```

The example exercises all eight checks with declared analytic signals. It is not a dynamically consistent robot trajectory or a simulator result. Expected verdicts are in `examples/expected.json`.

Evaluation writes `result.json` and `requirements.csv`. Reporting writes `summary.json`, `summary.csv`, and `verdicts.svg` using saved results only. Numerical results are deterministic; source/profile/evaluator hashes identify the inputs and implementation. Output directories represent individual attempts; use a new directory to retain each rerun.

Exit codes: `0` mandatory applicable checks pass; `1` a mandatory check fails; `2` mandatory evidence is incomplete, configuration/data is invalid, the run did not complete, or no mandatory requirement applies. Optional outcomes remain in the result. Untriggered response properties are inconclusive.

## Existing dataset

```bash
python -m uav_ugv_verification inspect PATH_TO_RUN \
  --profile verification/profiles/sensor_only.yaml
```

A CSV run requires `metadata.json`, `imu.csv`, `wheel_odom.csv`, `gps.csv`, `truth.csv`, and `cmd.csv`. Optional `faults.json` requires per-sample labels; events come from actual labeled samples, not fault envelopes. A dataset without navigation estimates or detector decisions cannot pass navigation/detection checks. Evaluating the sensor-only profile returns code `2` because all R1-R8 checks are explicitly excluded; inspection remains useful.

## Tests

```bash
python -m pytest verification/tests
```

The offline suite covers exact deadlines, incomplete horizons, sustained holds, missing streams, clock epochs, source loss, geometry, detector scope, false-alarm exposure, CLI statuses, and reporting without source recordings. It starts no simulation. Linux `colcon test` additionally runs real CDR bag regressions.

## Simulator integration

The [integration guide](../docs/verification/simulator_integration.md) provides Linux build, recording, import, profile-binding, and evaluation commands. Setup mappings are in `setups/`; calibrated pilot templates are `profiles/m2_*.yaml`. `bind-profile` resolves `steady:recording` from a recording's verified boot clock without changing thresholds.

`import-bag` supports Humble sqlite3 storage with bounded-memory acquisition sorting, clock resets, native telemetry counters, and numerical topic selection. It records missing capabilities and source/adapter hashes. It requires sourced ROS 2 Humble; other library commands retain their ROS-free numerical boundary. See the [M2 evidence index](../docs/project/results/verification/m2/README.md) for actual camera/guard/watchdog outcomes and sampling limits.


## Experiments, paper, and demo

Linux, with the ROS workspace sourced:

```bash
python scripts/verification_experiments.py verification/outputs/campaign --stage all
python scripts/verification_demo.py verification/outputs/demo
python scripts/test_verification_demo_cleanup.py verification/outputs/interruption
```

The [experiment guide](../docs/verification/experiments.md) defines the fixed matrix, source freeze, durable ledger, bounded search, and owned-process cleanup. The [M3 evidence](../docs/project/results/verification/m3/README.md) retains all completed attempts, including reproduced arrival failures and startup aborts. Live runs use fresh output directories and server-only Gazebo.

Saved-result reproduction works without ROS or source bags:

```bash
python scripts/verification_report.py docs/project/results/verification/m3 verification/outputs/paper
python scripts/verification_demo.py verification/outputs/replay --mode replay \
  --campaign docs/project/results/verification/m4/demo
```

Add `--compile-pdf` to the report command when `pdflatex` is installed. Paper output includes numerical tables, evaluated bounds, paired assistance outcomes, claim/evidence limits, and eight static figures. Figure bounds come from saved results. The [demo guide](../docs/verification/demo.md) documents statuses, timing, cancellation, and recorded replay.

## Additional simulator setups

The numerical package accepts the [normalized contract](../docs/verification/interfaces.md) independently of the simulator. Add a setup mapping/adapter, declare actual capabilities, transform global coordinates into the declared map/reference point, and preserve acquisition/receipt clocks. Calibrate a separate profile before held-out evaluation. Existing monitors and saved-result reports consume that profile without simulator imports.

The supplied process runner launches this project's Linux camera/guard/watchdog scenarios. Other simulators provide their own launch/recording path and feed the same offline CLI; they do not need to duplicate requirement logic. Unsupported signals remain unavailable. Future hazard/detector producers must supply actual events/decisions before those claims can be measured.
