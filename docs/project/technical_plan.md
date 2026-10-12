# Technical Plan

## Technology Stack

- **Simulator:** Gazebo provides ROS 2-native UAV and UGV multi-agent support and is the safer option for the project timeline. AirSim or Isaac Sim may provide better visuals and terrain variety but would add integration risk.
- **Middleware:** ROS 2 provides UAV-UGV communication through topics, services, and actions. Simulated bandwidth and latency constraints can be added to represent communication limits.
- **Planning baseline:** Use A*, D* Lite, or RRT* for global planning, with D* Lite or a local replanner such as TEB or DWA for dynamic obstacle avoidance. Adapt existing ROS 2 Nav2 implementations where practical.
- **Localization baseline:** Use EKF or UKF sensor fusion, preferably through established ROS 2 tooling such as `robot_localization`, with UAV-assisted relative-position corrections incorporated as measurement updates.

## Roles

### 1. Simulation and Systems Integration Lead

Owns the environment, world building, terrain and hazard representation, obstacles, UAV and UGV models, and the ROS 2 message architecture used by every module. This role also coordinates weekly integration builds.

### 2. UAV Perception and Traversability Mapping

Builds the aerial-survey-to-costmap pipeline. Processes UAV camera or depth input, classifies terrain such as obstacles, slopes, mud, water, and safe areas, and outputs a traversability grid or costmap for the UGV planner.

### 3. Path Planning and Dynamic Replanning

Consumes the traversability map, computes an optimized global route subject to fuel and battery constraints, and performs real-time replanning when the UAV scout reports a dynamic hazard such as a pedestrian, vehicle, or debris.

### 4. Cooperative UGV Localization

Owns resilient UGV pose estimation when UGV GNSS is degraded or unavailable. Fuses GNSS, wheel odometry, IMU, and UAV-assisted position corrections from calibrated camera observations and independent PX4 anchors into one pose-with-uncertainty output for planning. Synthetic observations support controlled fault trials. See the [Cooperative Localization Subsystem Specification](../cooperative_localization/subsystem.md).

### 5. Fault Detection and Evaluation Framework

Builds fault-injection and detection logic for sensor dropout, occlusion, and mechanical or actuator faults. Designs the evaluation harness, metrics, logging, and test matrix across route lengths, terrain types, and obstacle densities.

> **Project decision:** Decide early whether fault detection covers sensor, mechanical, and occlusion faults or a selected subset.

## 11-Week Timeline

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
