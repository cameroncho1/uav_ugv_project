# Project documentation

The project uses UAV observations to map terrain, estimate a ground robot's pose, and guide its navigation. This index covers project setup, system design, and subsystem documentation.

## Run the system

| Document | Contents |
| --- | --- |
| [Host and Ubuntu setup](../README.md) | macOS with UTM, Windows with an Ubuntu VM or WSL2, ROS Humble, PX4, DDS, and basic simulation. |
| [Simulation dependencies](simulation_setup.md) | Pinned revisions, Harmonic bridge, rosdep rules, build checks, and display requirements. |
| [Goal demo](../README_simple_goal_demo.md) | Full camera-assisted navigation launch and separate-terminal commands. |
| [Sensor dataset collection](sensor_dataset.md) | Per-run noise controls, complete simulation startup, ROS bag recording, truth labels, and dataset checks. |

## Project design

| Document | Contents |
| --- | --- |
| [System diagram](system_design.html) | Proposed project, implemented prototype, and work still proposed. Open in a browser. |
| [Project overview](project/project_overview.md) | Project goal and current limits. |
| [Technical plan](project/technical_plan.md) | Current stack, subsystem responsibilities, and original course schedule. |

## Subsystem documentation

| Document | Contents |
| --- | --- |
| [Cooperative localization](cooperative_localization/README.md) | Sensor fusion, UAV assistance, interfaces, extension points, results, research, presentations, and implementation history. |
| [Navigation](navigation.md) | Goal-associated routes, footprint clearance, path following, actuator expiry, and scoring. |
| [Camera heading](cooperative_localization/camera_heading.md) | Measured UGV direction overlay, relative orientation, joint heading fusion, and calibration. |
| [Heading validation](cooperative_localization/heading_validation.md) | Executed checks, measured simulation results, and operating limits. |

The [technical plan](project/technical_plan.md) lists responsibilities for simulation integration, perception and mapping, route planning, localization, and fault management.
