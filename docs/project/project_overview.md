# Drone-led UGV navigation

Team: Sophie Siemsgluess, Rylan Wade, Ethan Palosh, Jeremy Yin, Cameron Cho.

Project cluster: C1, Multi-Agent Systems.

[Documentation index](../README.md) | [System diagram](../system_design.html) | [Technical plan](technical_plan.md)

## Proposed project

UAVs survey the route ahead of a UGV. A terrain map identifies obstacles and hazardous surfaces. A planner selects a route, and the UGV follows it. New hazards trigger replanning. UAV observations also help estimate the UGV's position when its GPS is degraded or unavailable.

The proposed evaluation covers route length, terrain, obstacle density, localization error, fault response, energy use, and navigation completion.

## Implemented prototype

- One UAV takes off and hovers above a flat simulated route.
- A calibrated camera detector observes a roof marker on the UGV.
- Independent UAV map anchors and relative observations provide UGV position measurements.
- A planar filter combines wheel speed, IMU heading/rate, GPS, and optional UAV measurements.
- Camera brightness supplies a local terrain grid.
- Local A* plans with footprint clearance and goal-associated routes.
- The UGV controller follows valid routes and stops on invalid inputs. A Gazebo actuator watchdog expires missing commands.
- Fault injection and scoring cover GPS loss, drift, outliers, occlusion, shared-reference error, route tracking, and command loss under recorded conditions.

The camera goal-demo world has no physical obstacles. Multi-UAV scouting, semantic terrain classification, energy-aware global planning, dynamic-hazard evaluation, and production fault diagnosis remain proposed work.

[Localization](../cooperative_localization/README.md) describes the estimator, extension interfaces, test commands, and retained historical results.
