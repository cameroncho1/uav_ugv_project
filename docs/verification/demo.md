# Verification demonstration and paper artifacts

The live demonstration runs assisted camera arrival, controlled guard response, and both controller-death/command-transport-loss checks. It reuses the experiment runner, recorder, adapters, frozen numerical profiles, and cleanup. Gazebo runs server-only; RViz and desktop preview windows stay disabled.

The numerical guard fixture selects `config/fastdds_fixture_udp.xml` for itself, its controller/planner, and its recorder. This overrides an inherited camera transport profile within that fixture; camera trials retain shared memory for image delivery.

The watchdog fixture confirms at least 100 ms of input silence in fresh actuator telemetry before recording its loss event. This lets an in-flight command finish delivery after producer termination. Expiry and settling deadlines remain measured from the last accepted actuator input; resumed input still makes the observation inconclusive.

## Live command

Activate the [Linux runtime](simulator_integration.md), then run from the repository root:

```bash
python scripts/verification_demo.py verification/outputs/demo
```

Use a new output directory. The default noise seed is 641, distinct from calibration and matrix seeds. The command writes its plan, durable attempt ledger, raw recordings, portable evidence, and `demo_summary.json`. A successful verification summary requires every planned attempt and mandatory evaluation to pass. A witnessed mandatory failure exits 1; incomplete or unavailable mandatory evidence exits 2. Legacy physical arrival alone cannot make the demo successful.

The summary records total wall duration and startup from the demo runner to the first nonzero drive tick, using the recorded steady clock. Command loss and guard holds run in separate controlled trials; no detector or hazard behavior is implied. Cancellation uses owned-process cleanup, retains the interrupted attempt, and exits 130. The Linux cleanup regression interrupts after recording starts and checks finalized metadata and absence of owned processes:

```bash
python scripts/test_verification_demo_cleanup.py verification/outputs/interruption
```

## Recorded replay

```bash
python scripts/verification_demo.py verification/outputs/replay --mode replay \
  --campaign docs/project/results/verification/m4/demo
```

Replay regenerates tables and static figures from saved verdicts and exported display series. It starts no simulator, opens no desktop windows, requires no ROS imports, and prints `mode: recorded_replay` and `new_simulation: false`. It preserves the original mandatory pass/failure/incomplete outcome. Raw bags are not needed for this report replay; re-evaluation of monitors still requires the normalized traces or source recordings.

## Paper output

```bash
python scripts/verification_report.py docs/project/results/verification/m3 \
  verification/outputs/paper --compile-pdf
```

The generator reads saved results, verifies checksums and attempt accounting, and writes:

- `paper.md`, `paper.tex`, and the compiled `paper.pdf` when requested.
- Architecture, full outcome, navigation, confirmed-arrival, conditional-error, clearance, guard, and watchdog figures in SVG, PDF, and PNG.
- `attempts.csv`, `metrics.csv`, `metric_ranges.csv`, `assistance_comparisons.csv`, `bounds.csv`, `claims.csv`, category counts, and `violations.csv`.
- `summary.json`, per-figure source/profile/cohort metadata, and artifact checksums.

PDF compilation uses `pdflatex` with shell escape disabled and an installed LaTeX distribution. MiKTeX package installation is disabled to prevent installer dialogs. The compiled report must fit within ten pages. Numerical table regeneration needs only the verification Python dependencies; LaTeX is optional.

Figures use full saved verdicts and downsampled display series. Confirmed-arrival plots omit unconfirmed arrivals explicitly; the outcome table retains every attempted outcome. Empty worlds have no finite nearest-obstacle clearance. Bounds come from saved evaluation parameters. Figure inputs and saved verdicts have separate checksums. Source/profile/evaluator cohorts remain identified, and descriptive totals are not a pooled success-probability estimate. The text covers the verification feature for the project report; unavailable project components remain unavailable.

The [M3 evidence](../project/results/verification/m3/README.md) records experiment/search outcomes. The [M4 evidence](../project/results/verification/m4/README.md) records the actual rehearsal, replay, report-generation checks, startup/run duration, and any unsuccessful attempts.
