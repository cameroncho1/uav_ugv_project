# M4 report and demonstration

Date: 2026-11-11. The [six-page report](paper/paper.pdf), [Markdown source](paper/paper.md), and [claim/evidence table](paper/claims.csv) use the frozen [M3 cohort](../m3/README.md). The live demo and interruption check use a separate source cohort. No desktop windows open during these checks.

## Measured checks

| Check | Result | Evidence |
| --- | --- | --- |
| Live demonstration | Three attempts pass; four evaluated recordings; exit 0 | [Summary](demo_summary.json), [ledger](demo/ledger.json), [saved results](demo/recordings/) |
| Startup to first drive | 34.766 wall seconds | Same-boot monotonic timestamps in demo summary and recording manifest |
| Full live command | 159.175 wall seconds, including export/report generation | [Summary](demo_summary.json) |
| Recorded replay | Exit 0; 17.734 wall seconds; no new simulation or ROS imports | [Replay summary](replay_summary.json) |
| Interrupted live run | Exit 130; durable `interrupted` attempt; finalized recording metadata | [Interruption result](interruption/result.json), [ledger](interruption/ledger.json), [recording](interruption/recording.json) |
| Interrupted cleanup | 6.076 wall seconds; zero remaining owned processes | [Owned process-group check](interruption/result.json), [final shutdown check](shutdown.json) |
| Offline suite | 122 tests pass | [Output](offline-tests.txt) |
| Registered Linux checks | 219 checks, zero errors/failures, eight cppcheck skips | [Output](colcon-test-result.txt) |
| Package distribution | Wheel and source distribution build; fresh isolated wheel install loads templates and publishes saved results | [Validation](validation.json) |
| Numerical reproduction | All eight CSV tables match byte-for-byte from the installed wheel | [Validation](validation.json) |
| Report compilation | Six pages; zero overfull LaTeX boxes; eight figures exported as SVG/PDF/PNG | [Paper](paper/paper.pdf), [figure provenance](paper/figure_sources.json) |

The demo uses seed 641, distinct from calibration seed 513 and matrix seeds 631-633. It exercises assisted arrival (R1/R2/R6), controlled invalid-guard response (R3), and controller-death plus command-transport-loss expiry/physical settling (R4). Optional and excluded outcomes remain in each saved result. A mandatory failure or incomplete recording prevents a successful demo summary.

## Startup corrections

- Native PX4 writer creation precedes ROS graph startup; actual ROS observations remain a separate bounded barrier.
- The guard fixture waits for an observed controller command, beyond endpoint discovery, before starting cases. This addresses the two M3 guard startup aborts.
- ROS domains use 20-79, within the documented Linux range. This is preventive isolation; the first M3 XRCE failures occurred on domains 91-93 and are not attributed to ephemeral-port overlap.
- Explicit readiness failure remains a startup failure even when partial recordings exist. Readiness reports survive portable export.
- Cancellation finalizes recording, updates the durable ledger, and stops owned process groups. Fixture ROS nodes/logs close before normalization; aborted recordings remain available without masking the original assertion with a normalization error.

The rehearsal validates these changes under its recorded conditions. It does not establish arbitrary-load startup reliability. The [demo source freeze](demo/freeze.json) and [source archive metadata](demo/source_archive.json) distinguish this code from M3. Original experiment failures and bounds remain unchanged.

## Reproduction

From the repository root with the Linux runtime sourced:

```bash
python scripts/verification_demo.py verification/outputs/demo
python scripts/test_verification_demo_cleanup.py verification/outputs/interruption
```

Use fresh output directories. Saved-result commands work without ROS:

```bash
python scripts/verification_demo.py verification/outputs/replay --mode replay \
  --campaign docs/project/results/verification/m4/demo
python scripts/verification_report.py docs/project/results/verification/m3 \
  verification/outputs/paper --compile-pdf
```

LaTeX is optional for numerical tables and figures. PDF compilation uses an installed `pdflatex`, disables shell escape, and disables MiKTeX package installation dialogs. See the [demo guide](../../../../verification/demo.md).

## Paper artifacts and limits

[Attempt outcomes](paper/attempts.csv), [metrics](paper/metrics.csv), [ranges](paper/metric_ranges.csv), [evaluated bounds](paper/bounds.csv), [assistance pairs](paper/assistance_comparisons.csv), [category counts](paper/claims_by_category.csv), and [counterexamples](paper/violations.csv) retain the recorded denominators. Evidence paths in the claim table are relative to the M3 campaign root. Figure inputs and saved verdicts have independent checksums; plotting never recomputes requirements.

The paper retains six dense-layout arrival failures and two reproducing diagnostic replays, plus incomplete/aborted startup attempts. It does not establish an assistance benefit from the incomplete paired trials. R5, online R7/R8, dynamic-hazard response, and discrete supervisor proof remain unavailable. The report covers this verification feature for integration into the project paper.

Raw demo recordings remain machine-local at `/home/rylanwwade/uav-verification-m2/evidence/m4_rehearsal_1`; interruption recordings remain at `m4_interruption_1` in the same parent directory. Raw bags and normalized streams are not committed or uploaded. [Interruption raw-file checksums](interruption/raw_files.json) and saved demo manifests identify the retained files. Earlier readiness-development pilots remain separately indexed in [development pilots](development_pilots.json), outside the frozen matrix and demo totals.

[Validation metadata](validation.json) and [artifact checksums](artifacts.json) cover the portable evidence. Numerical tables regenerate without simulation or raw bags. Re-evaluating monitors requires the original normalized traces or recordings.
