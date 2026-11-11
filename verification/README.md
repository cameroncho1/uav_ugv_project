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

The semantic fixtures cover exact deadlines, incomplete horizons, hold violations, missing streams, clock epochs, source loss, geometry, detector scope, false-alarm exposure, CLI statuses, and reporting without source recordings. No simulation is started. Bag reading, telemetry, ROS test registration, and actual simulation trials are M2.
