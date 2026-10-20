# Technical Plan

[Documentation index](../README.md) · [System diagram](../system_design.html)

The current implementation is a single-UAV, single-UGV simulation. The roles and course schedule below also include proposed work.

## Technology Stack

- Simulator: Gazebo Harmonic and PX4 SITL on Ubuntu 22.04.
- Middleware: ROS 2 Humble, Micro XRCE-DDS, and the Harmonic ROS-Gazebo bridge.
- Planning: local eight-neighbor A* with footprint clearance and goal-associated routes. Global energy-aware planning and dynamic-hazard avoidance remain proposed.
- Localization: the planar `robot_localization` EKF with wheel, IMU, GPS, and optional UAV position/heading measurements.

## Roles

### 1. Simulation and Systems Integration Lead

Owns the environment, world building, terrain and hazard representation, obstacles, UAV and UGV models, and the ROS 2 message architecture used by every module. This role also coordinates weekly integration builds.

### 2. UAV Perception and Traversability Mapping

Builds the aerial-survey-to-costmap pipeline. Processes UAV camera or depth input, classifies terrain such as obstacles, slopes, mud, water, and safe areas, and outputs a traversability grid or costmap for the UGV planner.

### 3. Path Planning and Dynamic Replanning

Consumes the traversability map, computes an optimized global route subject to fuel and battery constraints, and performs real-time replanning when the UAV scout reports a dynamic hazard such as a pedestrian, vehicle, or debris.

### 4. Cooperative UGV Localization

Owns resilient UGV pose estimation when UGV GNSS is degraded or unavailable. Fuses measured GNSS, wheel odometry, IMU, and UAV-relative observations through a sensing-independent interface into one pose-with-uncertainty output for planning. Perception, PX4 state exposure, simulation and injected faults remain with their stated owners. See the [Cooperative Localization Subsystem Specification](../cooperative_localization/subsystem.md).

### 5. Fault Detection and Evaluation Framework

Builds fault-injection and detection logic for sensor dropout, occlusion, and mechanical or actuator faults. Designs the evaluation harness, metrics, logging, and test matrix across route lengths, terrain types, and obstacle densities.

Current fault work implements synthetic sensor corruption, outage controls, occlusion trials, scoring, and command-loss checks. Mechanical diagnosis and production fault-classification/recovery policy remain proposed.

## Original 11-week course schedule (planned)

| Week | Milestone or deliverable | Ownership |
|---:|---|---|
| 1 | Create the simulation environment skeleton, survey tooling, and define exact metrics and test scenarios. | Everyone completes setup. |
| 2 | Move basic UAV and UGV models in simulation and stub message interfaces between modules. | Integration lead drives this work. |
| 3 | Develop the mapping pipeline, planner, localization fusion, and fault-injection framework in parallel against agreed interfaces. | Parallel module work with one shared stand-up. |
| 4 | Continue parallel module development against the agreed interfaces. | Parallel module work with one shared stand-up. |
| 5 | Integration Sprint 1: connect the modules and complete one end-to-end survey-to-map-to-plan-to-navigation run. | Module leads integrate their components. |
| 6 | Add dynamic replanning, GNSS-degraded localization corrections, and fault-detection triggers to the integrated loop. | Two members on replanning and localization, two on fault detection, and one on integration debugging. |
| 7 | Run the full scenario matrix across terrain, route length, and obstacle density; collect metrics and fix integration defects. | Everyone. |
| 8 | Continue scenario testing, metric collection, and integration fixes. | Everyone. |
| 9 | Complete final tuning. | Everyone. |
| 10 | Analyze results. | Everyone. |
| 11 | Complete report writing and presentation preparation. | Everyone. |
