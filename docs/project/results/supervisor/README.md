# W10: verified navigation supervisor

Date: 2026-11-11. [Technical specification](../../../verification/supervisor.md) | [Course/report mapping](../../../verification/supervisor_course_alignment.md) | [Completion plan](../../completion_plan.md)

The UGV controller has persistent fault-recovery and completed-goal memory. The finite checker verifies its discrete motion-permission policy and compares every reachable transition with compiled C++. Controlled ROS inputs exercise the actual controller; separate headless simulator trials validate integration.

## Results

| Check | Result | Evidence |
| --- | --- | --- |
| Finite-state safety checking | 92 controller states, 92 product states, 7,120 transitions; 11 independent obligations pass | [Model, properties, and counterexamples](model.json) |
| Compiled C++ conformance | All 7,120 contexts/inputs agree; policy/source/binary digests retained | [Model conformance](model.json) |
| Negative controls | All nine unsafe mutations detected; shortest witnesses retained | [Mutation table](paper/mutations.csv) |
| Actual controller ROS fixture | 29 cases pass, including enabled missing input, recovery count/time, stale/duplicate windows, alarm bursts, completion drift, goal revisions, readonly configuration, paused clock, and rewind | [Cases](controlled/result.json), [consumer trace](controlled/control.jsonl) |
| Navigation regression | 23 existing guard cases pass | [Guard cases](guards/cases.json) |
| Actuator regression | Controller death and transport loss both pass; zero late displacement under the recorded normal-clock conditions | [Watchdog results](watchdog/summary.json) |
| Headless live demo | Three attempts pass; four recordings; 112.467 wall seconds; first drive at 24.198 seconds | [Demo summary](demo_summary.json), [ledger](demo/ledger.json) |
| Requirement monitors | Camera R1/R2/R6 pass; controlled guard R3 passes; both watchdog R4 checks pass | [Camera](demo/recordings/a0001-r0/result.json), [guard](demo/recordings/a0002-r0/result.json), [controller loss](demo/recordings/a0003-r0/result.json), [transport loss](demo/recordings/a0003-r1/result.json) |
| Cancellation | Exit 130; finalized recording; zero remaining owned processes; 3.332 s cleanup | [Interruption result](interruption/result.json) |
| Offline/registered tests | 139 offline tests; 254 Linux registered checks, zero errors/failures, eight static-analysis skips | [Validation](validation.json), [registered results](validation/colcon-results.txt) |
| Paper/package | Two-page PDF, no overfull boxes; three figures; three CSV tables and both paper sources reproduce byte-for-byte from a fresh wheel install | [Package check](validation/package.json) |

Camera R3 is unexercised and remains inconclusive. Hazard R5 and online detector R7/R8 remain unavailable/excluded. The standalone watchdog checks and the demo watchdog checks are separate recordings.

## Paper material

- [Two-page PDF insertion](paper/paper_section.pdf) and [editable LaTeX](paper/paper_section.tex).
- [Technical text for report sections 3?6](paper/paper_section.md): architecture/assumptions, method, implementation, and quantitative results.
- [Decision precedence](paper/figures/supervisor.svg), [recovery timing](paper/figures/recovery.svg), and [completed-goal memory](paper/figures/goal_memory.svg), each also available as PDF and PNG.
- [Property table](paper/properties.csv), [mutation table](paper/mutations.csv), [controlled-case table](paper/cases.csv), and [figure sources](paper/figure_sources.json).
- [Course sources](course_sources.json) preserve canonical URLs, source year, retrieval metadata, and available hashes. The supplied introduction is Fall 2025; no 2026 deadline or grading rule is inferred.

Reproduce saved material without ROS or source bags:

```bash
python scripts/supervisor_report.py docs/project/results/supervisor/model.json \
  docs/project/results/supervisor/controlled verification/outputs/supervisor-paper --compile-pdf
python scripts/verification_demo.py verification/outputs/supervisor-replay --mode replay \
  --campaign docs/project/results/supervisor/demo
```

Use fresh output paths. PDF compilation requires `pdflatex`; the report compiler disables shell escape and automatic package installation.

## Evidence provenance

[Source snapshot](source_snapshot.json) records 223 runtime files. The compiled proof and controlled recording bind the exact same five supervisor implementation sources and policy. [Source equivalence](validation/source_equivalence.json) verifies that all 223 runtime files match the checkout byte-for-byte against the retained archive. Historical M3/M4 artifacts remain unchanged.

Raw recordings and the exact source archive remain under `Ubuntu-22.04:/home/rylanwwade/uav-supervisor-w10/evidence/w10_final`. [Raw availability](raw_availability.json) lists paths, byte counts, and SHA-256 digests. These machine-local bags are not distributed by this PR; the committed evidence supports saved-result reproduction. The controlled bag also records the actual `FaultStatus` requests.

[Development pilots](development_pilots.json) retain unsuccessful startup/transport attempts and the superseded successful fixture. They are separate from the final 29-case result. A stale Linux-copy validation attempt aborted; checksum synchronization and rebuilding corrected that environment. No failed attempt is counted as a passing final case.

## Claim boundary

The proof covers reachable finite decisions under the documented validated-input and timer abstraction. Numerical timestamp validation is separately unit tested. ROS scheduling, middleware delivery, physical braking, perception accuracy, detector effectiveness, and continuous collision freedom remain experimental or outside scope. Completion records one valid estimated-distance observation; R2 still requires independently scored physical tolerance and quiet dwell. Clock epochs are locally inferred, with the message-identity limitation stated in the specification.
