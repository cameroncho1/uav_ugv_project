# Cooperative Localization: Reference Papers

## Purpose

This document lists the papers and tools that support the cooperative localization subsystem and the final project paper. Each entry gives a short summary, its relevance to localization, and its relevance to the project as a whole.

Terms used below:

- **EKF (extended Kalman filter):** A filter that combines noisy measurements into one estimate and an uncertainty value.
- **Relative position:** Where one vehicle is with respect to another.
- **Ground truth:** The true position, used only to generate test data and to score results.

## Summary

| # | Reference | Type | Main use for this project |
|---|---|---|---|
| 1 | Pryor (2021), Evaluation of Cooperative Navigation Strategies with Maneuvering UAVs | Master's thesis | Core evidence that UAV help works when UGV GPS is lost. Supports the EKF choice and the evaluation plan. |
| 2 | Akhihiero et al. (2024), Cooperative Localization for GNSS-Denied Subterranean Navigation | Journal paper | Shows how several noisy measurements at different rates are fused, and compares EKF with a heavier method. |
| 3 | De Silva and Katupitiya (2024), UAV-UGV Collaborative Localisation with Minimum Sensing | Journal paper | Closest match to the problem: UAVs with GPS localize a UGV without GPS. Shows how to reject impossible estimates. |
| 4 | Brunacci et al. (2026), Fly, Track, Land | Preprint | Shows how to treat a UAV-UGV relative position reading as a filter input, with outlier rejection. |
| 5 | Moore and Stouch (2014), `robot_localization` | Conference paper and software | The EKF package used by the GPS, wheel, and IMU baseline. |

---

## 1. Pryor (2021): Evaluation of Cooperative Navigation Strategies with Maneuvering UAVs

Master's thesis, Auburn University. [Link](https://etd.auburn.edu/handle/10415/7871)

### Summary

- Studies teams of UAVs and ground vehicles that lose GPS and share relative measurements to limit drift.
- Compares a centralized EKF (one filter for all vehicles) with a decentralized filter (one per vehicle).
- Tests relative range, range-rate, and bearing measurements, and UAV flight patterns that improve the estimate.
- Simulation and a field test with two UAVs and one ground vehicle.
- Field result: relative error dropped from over 100 m (inertial sensor only) to under 1 m. Absolute error dropped from over 70 m to meter level.
- If one vehicle has an accurate absolute position (for example a UAV with GPS) and the relative positions are accurate, every vehicle in the group gets that accuracy.
- Range plus bearing together give accurate relative position without any special flight pattern.
- Decentralized filters needed tuning that made their reported uncertainty wrong. For small groups the thesis recommends the centralized filter.
- Filter tuning is done by hand. Uncertainty settings often need inflating above the raw sensor values.

### Relevance to cooperative localization

- Directly supports our core claim: a UAV with a reliable position can reduce UGV drift during GPS loss.
- Supports a single centralized estimator, which matches our one authoritative pose output.
- Its results are grouped by which measurements are available. This is a model for our three test cases (nominal GPS, GPS loss without UAV, GPS loss with UAV).
- Its point that reported uncertainty must match real error supports our metric "uncertainty consistency with observed error."
- Tuning notes apply to our EKF setup.
- Our UAV reading is a relative position, which carries the same information as range plus bearing. This is the best case in the thesis. This is our interpretation, not a claim made by the thesis.

### Relevance to the project

- Gives the project a source for the claim that cooperative navigation reduces error growth.
- Shows that improving UGV pose helps every downstream module that reads it.
- Its flight-pattern results suggest that UAV path choices affect localization quality. This is a link between the UAV scouting role and localization.

### Cautions

- Simulated in MATLAB. The field test was processed after the run, not in real time.
- Its UGV was a car with a high-grade inertial sensor. Our simulated UGV will have cheaper sensors, so expect larger drift.
- Bearing measurements in the field test were simulated from GPS data.

---

## 2. Akhihiero et al. (2024): Cooperative Localization for GNSS-Denied Subterranean Navigation, A UAV-UGV Team Approach

NAVIGATION (Institute of Navigation), West Virginia University. [Link](https://navi.ion.org/content/71/4/navi.677)

### Summary

- A UGV maps a GPS-denied space and estimates the position of a light UAV that carries few sensors.
- The UAV pose is estimated from four inputs at different rates: inertial sensor, laser altimeter, lidar tracking from the UGV, and a UWB range radio.
- Each input has its own assumed noise (for example 0.1 m for the altimeter and range radio, 0.2 m for lidar).
- Compares an EKF with a factor graph method (a heavier method that re-solves the whole recent history).
- Adding measurements one at a time lowered error each time. Range radio gave the largest gain in position.
- With only lidar, range radio, and altimeter, EKF position errors were up to five times larger than the factor graph's. With all measurements added, both were similar.
- The authors judge the EKF the more practical choice when measurements are plentiful.
- Errors in the UGV pose carried into the UAV estimate. The UGV showed 0.4 m 3D position error and a peak near 1 m.
- Ran on ROS, tested in a motion-capture room, and processed from recorded data.

### Relevance to cooperative localization

- Same pattern as our estimator: several measurement types, different rates, each with stated noise, fused in one EKF.
- Gives example noise values we can use when setting simulated sensor noise (an open decision in `interfaces.md`).
- Shows the EKF versus factor graph tradeoff. It supports starting with an EKF and names the factor graph as a future option.
- Shows that errors in one vehicle's pose spread into the other vehicle's estimate. In our case, UAV pose errors will spread into the UGV correction. We should model UAV pose error, not assume it is perfect.
- The authors name the reverse direction (UAV helping UGV pose) as future work. That is our subsystem.

### Relevance to the project

- Supports the two-vehicle architecture where each vehicle contributes what it senses best.
- Confirms ROS is a workable base for this kind of team.
- Gives a real example of evaluating against ground truth from a motion-capture system.

### Cautions

- The direction is reversed. Here the UGV localizes the UAV. In our project the UAV helps localize the UGV.
- GPS is fully denied in this paper. Ours is degraded or lost on the UGV only, and the UAV keeps a reliable position.

---

## 3. De Silva and Katupitiya (2024): UAV-UGV Collaborative Localisation with Minimum Sensing

Sensors (MDPI), University of New South Wales. [Link](https://pmc.ncbi.nlm.nih.gov/articles/PMC11280856/)

### Summary

- Localizes a ground vehicle without GPS using two UAVs that have GPS.
- Each UAV measures distance to two radio tags on the ground vehicle. This also gives heading.
- With only two UAVs, some positions give two possible answers (a mirror image). A standard EKF can pick the wrong one.
- The fix is a constrained EKF that rejects answers the vehicle could not physically reach.
- A stochastic analysis using the Posterior Cramér-Rao Bound (a theoretical limit on how well any estimator can do) confirms the fix.
- Field test with a tractor and two hexacopters. Position error under 14 cm when still and about 23 cm when moving. Heading within about 4 degrees. Ground truth was RTK GPS (about 2 cm).
- Ran in real time at a 50 ms step in simulation.
- Needs a starting guess on the correct side of the ambiguous region.

### Relevance to cooperative localization

- Closest match to our problem statement: GPS-equipped UAVs help a UGV that has no usable GPS.
- Shows a practical safeguard for our estimator: reject measurements that conflict with what the UGV can physically do. This ties to our requirement that the estimator handles rejected measurements.
- Shows why measurement geometry matters. Our relative-position reading avoids this ambiguity, but if the team later switches to range-only sensing, this paper explains the risk.
- Gives an example of scoring position and heading error against a reference.

### Relevance to the project

- Names search and rescue, disaster, and battlefield use, which match our project goal.
- Supports the case for using few UAVs, which also limits battery use and cost. This links to the project's battery constraint.
- Its list of sensing options (camera, lidar, radar, UWB) and their weaknesses in smoke and darkness supports the project's focus on degraded conditions.

### Cautions

- Flat ground only. Hilly terrain and blocked range readings are listed as future work.
- Range-only sensing. Our design uses a relative-position reading.

---

## 4. Brunacci et al. (2026): Fly, Track, Land, Infrastructure-less Magnetic Localization for Heterogeneous UAV-UGV Teaming

arXiv preprint 2603.08926 (v2, July 2026). Universities of Perugia, ETH Zurich, and Czech Technical University. [Link](https://arxiv.org/pdf/2603.08926)

### Summary

- Focuses on a small UAV landing on a moving ground robot without GPS or external cameras.
- The ground robot emits a magnetic field. The UAV reads it to find its own 3D position relative to the robot.
- The reading is fed to the UAV's onboard EKF as an asynchronous absolute-position measurement, with noise settings chosen from tests.
- A jump larger than a set threshold is rejected as an outlier.
- Special handling was added for a sensor jump when the UAV leaves the robot's deck.
- Results against motion capture: about 5 cm error hovering and 8 to 11 cm while the robot moves. The baseline without the magnetic reading often failed.
- Attitude errors on the ground robot created position bias, because the estimator assumed the robot frame was fixed.
- The related work section states typical accuracy ranges: GNSS 1 to 3 m, UWB 10 to 30 cm.

### Relevance to cooperative localization

- Shows a clean way to treat a UAV-UGV relative-position reading: as a timestamped measurement with its own noise, fused by an EKF. This matches our planned interface.
- Shows a simple outlier check before fusing.
- The accuracy figures give reasonable ranges for our simulated sensor noise.
- The frame and attitude bias problem is a warning for our frame contract. A frame or heading mistake shows up as position error.

### Relevance to the project

- Confirms that UAV-UGV teaming is an active research area and that heterogeneous teams help in GPS-denied areas.
- Supports the idea of layered sensing: coarse sensing far away, precise sensing close in.
- Cites the Ingenuity helicopter scouting ahead of the Perseverance rover as a precedent for drone-led ground navigation.

### Cautions

- Preprint, not yet peer reviewed. Cite as a preprint.
- Its goal is close-range docking, not route following. Use it for the filter design and not for accuracy claims about our system.
- Hardware is a nano drone and a quadruped robot. It does not match our simulation.

---

## 5. Moore and Stouch (2014): A Generalized Extended Kalman Filter Implementation for the Robot Operating System

Proceedings of the 13th International Conference on Intelligent Autonomous Systems (IAS-13), Springer. Software: `robot_localization`. [Documentation](https://docs.ros.org/en/api/robot_localization/html/index.html)

### Summary

- Describes the `robot_localization` package: an EKF and an unscented Kalman filter (a variant that avoids some approximations) for ROS.
- Accepts any number of sensors as odometry, inertial, pose-with-covariance, or velocity-with-covariance messages.
- Lets the user choose which values from each sensor are fused.
- Keeps estimating from a motion model when data stops. A timeout setting controls when a sensor counts as lost.
- Has outlier rejection thresholds per sensor.
- Has a 2D mode for flat ground.
- Handles GPS through a companion node, and separates a continuous local frame from a global frame that can jump.
- Publishes its result as `nav_msgs/Odometry` on `odometry/filtered`.

### Relevance to cooperative localization

- The subsystem runs this package's 2D EKF for the GPS, wheel, and IMU baseline.
- Its continuous estimation and sensor timeout match our requirement that the pose keeps publishing when GPS is lost.
- Its pose-with-covariance input suits our UAV-derived UGV position.
- Its rejection thresholds cover our requirement for handling rejected measurements.
- Its output type is `Odometry`, and our contract is `PoseWithCovarianceStamped`. A small relay node is needed.
- The frame settings need care. Global data such as GPS belongs in the `map` frame, and another source must supply the local frame.

### Relevance to the project

- Standard, widely used ROS tool. Using it lowers integration risk and supports the timeline.
- Any other module that needs an estimate can rely on a known, documented behavior.

### Note on sources

- Only the package documentation was read in full. The 2014 paper PDF could not be downloaded, so paper-specific details are not cited here. Read the paper before citing it in the final report.

---

## Where each reference fits in the final paper

| Section | References |
|---|---|
| Introduction and motivation | 1, 3, 4 |
| Related work | 1, 2, 3, 4 |
| Method: estimator and tool choice | 1, 2, 5 |
| Method: handling bad measurements | 3, 4, 5 |
| Method: simulated sensor noise | 2, 4 |
| Evaluation design | 1, 2, 3 |
| Limitations and future work | 2 (factor graph option), 3 (range-only and terrain), 1 (flight patterns) |

## Gaps to address

- No reference here covers Gazebo or PX4 simulation of localization. Find one if the report needs a simulation-validity argument.
- No reference covers traversability mapping or path planning. Other team members should add their own.
- All four research papers use real sensors or MATLAB. Our results will be simulation only, so the report should limit its claims to the simulated conditions, as `subsystem.md` already requires.
