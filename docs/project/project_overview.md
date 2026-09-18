# Drone-Led Navigation for Unmanned Ground Vehicle for Search and Rescue Operations

## Project Information

- **Team members:** Sophie Siemsgluess, Rylan Wade, Ethan Palosh, Jeremy Yin, Cameron Cho
- **Project cluster:** C1 - Multi-Agent Systems

## Abstract

This project proposes a collaborative UAV-UGV navigation system in which a small swarm of aerial vehicles serves as the "eyes" for a ground vehicle operating in challenging scenarios that may affect the reliability and availability of sensor data.

The UAVs perform real-time aerial surveys to construct a traversability map and identify obstacles and hazardous surfaces, such as mud or water. The system uses this information to provide an optimized path for autonomous UGV navigation. The UAVs operate as forward scouts that continuously monitor the route ahead. When a new obstacle is detected, the system triggers real-time replanning so the UGV can reroute before encountering the hazard.

In GPS-denied environments, a UAV with superior line of sight to satellites or fixed beacons assists the UGV by relaying position corrections and navigation instructions. A key challenge is fault detection across sensor failures, mechanical issues, and image occlusion caused by obstacles.

The project will develop algorithms for fault detection, route planning, and localization under constraints that include route duration, vehicle fuel, UAV battery capacity, and GPS availability. Together, these capabilities form an integrated aerial-ground framework for robust, adaptive autonomous navigation in complex environments.

The proposed system will be evaluated in simulation across varied route lengths, terrain, and obstacles. Evaluation metrics will include safety, energy efficiency, fault tolerance, localization accuracy, and obstacle-detection and avoidance accuracy.
