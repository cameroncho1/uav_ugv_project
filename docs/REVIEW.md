# Documentation review against main

This list covers the Markdown and HTML files changed by `fix/subsystem-audit` relative to `main`, including the GPS-branch commits. No pull request is required to review the diff.

```bash
git diff main...fix/subsystem-audit -- '*.md' '*.html'
```

## Current setup and implementation

| File | Review scope |
| --- | --- |
| [Repository README](../README.md) | macOS/UTM and Windows Ubuntu setup, build, basic simulation, and watchdog command input. |
| [Goal-demo README](../README_simple_goal_demo.md) | Complete camera-assisted navigation launch and separate-process commands. |
| [Documentation index](README.md) | Entry point for setup, design, interfaces, results, history, and presentations. |
| [System diagram](system_design.html) | Proposed project with branch changes, inherited foundations, and unimplemented work marked separately. |
| [Simulation setup](simulation_setup.md) | Pinned dependencies, display, rosdep, amd64 bridge binaries, and ARM64 bridge source build. |
| [Localization README](cooperative_localization/README.md) | Pipeline, startup, old/new package locations, extension contracts, and verification commands. |
| [Subsystem](cooperative_localization/subsystem.md) | Implemented estimator and ownership boundaries. |
| [Interfaces](cooperative_localization/interfaces.md) | Topics, frames, clocks, controls, launch modes, and trial commands. |
| [UAV measurement design](cooperative_localization/uav_measurement_design.md) | Coordinate conversion, matching, propagated uncertainty, and rejection. |
| [Perception](cooperative_localization/perception.md) | Camera/tag geometry, simulated calibration, occlusion, and shared-reference errors. |
| [Navigation](navigation.md) | Goal revisions, map freshness, footprint clearance, following, expiry, and metrics. |
| [Project overview](project/project_overview.md) | Proposed scope and implemented prototype. |
| [Technical plan](project/technical_plan.md) | Current stack, subsystem roles, and original planned course schedule. |
| [References](cooperative_localization/references.md) | Research summaries, source limits, and implementation uses. |
| [Rendered detector fixture](../ros2_ws/src/uav_sensing/test/fixtures/README.md) | Retained test image, rendering conditions, and detector thresholds. |

## Results and historical records

| File | Tested revision or scope |
| --- | --- |
| [Audit fixes and validation](project/remediation_2026-10-12.md) | Audit implementation and replacement validation. |
| [October 11 evaluation](cooperative_localization/evaluation_results.md) | Historical GPS-branch results. |
| [October 11 implementation report](project/implementation_report_2026-10-11.md) | Historical changes, authorship, decisions, and evidence. |
| [October 12 review](project/review_2026-10-12.md) | Findings against GPS snapshot `c87bf1b`, before remediation. |

Historical reports retain their dates and measured values. They are separate from the current setup instructions.

## Presentations

| File | Contents |
| --- | --- |
| [How localization works](cooperative_localization/how_it_works_presentation.html) | Current pipeline, navigation guards, dated GPS results, and October 12 results. |
| [Localization overview](cooperative_localization/initial_presentation.html) | Scope, inputs/output, integration, and remaining work. |
| [Research papers](cooperative_localization/research_papers_presentation.html) | Literature summaries and source links. |

## Repository instructions

- [AGENTS.md](../AGENTS.md): shared runtime, ownership, validation, and Git instructions.
- [CLAUDE.md](../CLAUDE.md): imports `AGENTS.md`.
- This [review list](REVIEW.md) is also part of the documentation diff.

## Retained evidence files

- [GPS-branch results](cooperative_localization/results/): JSON summaries, CSV comparisons, charts, and diagnostic output. Individual trials are linked from the October 11 evaluation page.
- [Audit evidence](project/review_2026-10-12_evidence.json): reproduced findings and review conditions.
- [Audit validation](project/remediation_2026-10-12_results/validation.json): build/check commands, environment, and validation scope.
- [Audit trial output](project/remediation_2026-10-12_results/): paired trials, camera checks, physical tracking traces, watchdog checks, and logs.

List every changed evidence path with:

```bash
git diff --name-only main...fix/subsystem-audit -- docs
```

## Documentation validation

The documentation pass checks local links and anchors, referenced source paths, Bash snippet syntax, launch arguments, dependency pins, presentation rendering, and the system diagram. Prose excludes first-person wording. Runtime source and measurement/filter behavior are unchanged by this documentation commit.

The existing x86-64 Ubuntu workspace verifies the dependency checker and launch argument discovery. The setup checks the OSRF Jammy package indexes for amd64 and arm64 and the pinned upstream bridge build metadata. No fresh operating-system install, ARM64 compilation, or new simulation trials are claimed.
