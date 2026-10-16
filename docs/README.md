# Project documentation

This is the entry point for the cooperative localization work and its integration with the UAV-UGV demo.

## Run the system

| Document | Contents |
| --- | --- |
| [Host and Ubuntu setup](../README.md) | macOS with UTM, Windows with an Ubuntu VM or WSL2, ROS Humble, PX4, DDS, and basic simulation. |
| [Simulation dependencies](simulation_setup.md) | Pinned revisions, Harmonic bridge, rosdep rules, build checks, and display requirements. |
| [Goal demo](../README_simple_goal_demo.md) | Full camera-assisted navigation launch and separate-terminal commands. |
| [Localization guide](cooperative_localization/README.md) | Inputs, output, package locations, startup, extension points, and tests. |

## Design and interfaces

| Document | Contents |
| --- | --- |
| [System diagram](system_design.html) | Proposed project, implemented prototype, and work still proposed. Open in a browser. |
| [Project overview](project/project_overview.md) | Project goal and current limits. |
| [Technical plan](project/technical_plan.md) | Current stack, subsystem responsibilities, and original course schedule. |
| [Localization subsystem](cooperative_localization/subsystem.md) | Estimator behavior and ownership. |
| [Interfaces](cooperative_localization/interfaces.md) | Topics, frames, clocks, launch modes, and evaluation commands. |
| [UAV measurement design](cooperative_localization/uav_measurement_design.md) | Coordinate conversion, timestamp matching, covariance, and rejection rules. |
| [Camera observations](cooperative_localization/perception.md) | Tag geometry, calibration assumptions, visibility, and shared-error tests. |
| [Navigation](navigation.md) | Goal-associated routes, footprint clearance, command expiry, and scoring. |
| [Research references](cooperative_localization/references.md) | Source summaries and limits on their application to this simulation. |

## Results and history

| Document | Revision or scope |
| --- | --- |
| [Audit fixes and results](project/remediation_2026-10-12.md) | Implemented audit fixes, Linux build records, paired trials, tracking, and camera goal arrival. |
| [October 11 results](cooperative_localization/evaluation_results.md) | Historical GPS-branch trials. These are not reruns of the audit code. |
| [October 11 implementation report](project/implementation_report_2026-10-11.md) | Historical changes, attribution, decisions, and result links. |
| [October 12 audit](project/review_2026-10-12.md) | Findings against GPS snapshot `c87bf1b`, before the fixes. |
| [Documentation review list](REVIEW.md) | Every Markdown and HTML document changed from `main`, plus retained evidence directories. |

## Presentations

| Presentation | Scope |
| --- | --- |
| [How localization works](cooperative_localization/how_it_works_presentation.html) | Current pipeline, package boundaries, audit results, and dated GPS-branch results. |
| [Localization overview](cooperative_localization/initial_presentation.html) | Estimator scope, inputs, output, and integration. |
| [Research papers](cooperative_localization/research_papers_presentation.html) | Literature summaries and source links. |

## Recorded platforms

The original basic simulation records macOS/UTM on ARM64 with Gazebo 8.14.0. The audit records Ubuntu 22.04 under Windows WSL2 on x86-64 with Gazebo 8.15.0. Runtime commands use Linux paths and tools on both hosts. The audit record does not include a fresh ARM64 build or a fresh operating-system installation.

Keep trial outputs outside `/tmp` when retaining results. Camera runs require a Linux display session even when RViz and the Gazebo GUI are disabled.
