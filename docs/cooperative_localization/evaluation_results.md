# UGV Localization Evaluation

## Test Setup

The `uav_ugv` Gazebo world runs a 31-second forward-and-turn route. The scorer compares `/ugv/localization/pose` with the `simple_ugv` model's true map pose at matching simulation timestamps. Each row below uses 615 pose samples from 10.5 to 31 seconds. Position error is the 2D distance between estimated and true x/y. The measurement model uses 1.5 m horizontal GPS standard deviation and the wheel and IMU settings in `measurement_model.yaml`.

`on` keeps GPS throughout the route. `off` disables prepared GPS at 10 seconds, after the filter has a GPS fix. `startup_off` disables prepared GPS before the first sensor reading. The UGV keeps publishing wheel and IMU measurements in both GPS-denied cases. The tests use a fresh Gazebo world for each seed and case.

| Seed | GPS case | Mean error (m) | RMSE (m) | 95th percentile (m) | Max (m) | Mean heading error (deg) |
| ---: | --- | ---: | ---: | ---: | ---: | ---: |
| 513 | on | 0.384 | 0.406 | 0.584 | 0.730 | 0.728 |
| 513 | off at 10 s | 0.454 | 0.455 | 0.482 | 0.487 | 0.726 |
| 513 | off from startup | 0.922 | 0.922 | 0.944 | 0.954 | 0.727 |
| 514 | on | 0.298 | 0.328 | 0.627 | 0.722 | 0.721 |
| 514 | off at 10 s | 0.306 | 0.306 | 0.323 | 0.329 | 0.709 |
| 514 | off from startup | 0.999 | 0.999 | 1.016 | 1.022 | 0.709 |
| 515 | on | 0.257 | 0.288 | 0.489 | 0.605 | 0.767 |
| 515 | off at 10 s | 0.116 | 0.116 | 0.131 | 0.143 | 0.712 |
| 515 | off from startup | 1.009 | 1.009 | 1.019 | 1.025 | 0.760 |

Every GPS-off trial publishes zero prepared GPS messages after the switch. Every startup-off trial publishes zero prepared GPS messages throughout. The fused pose continues at 30 Hz in every case. Package build and tests pass: 33 tests, zero failures, one skipped check.

The GPS-on result is not always the smallest error. In this route, wheel motion is stable while GPS has injected 1.5 m noise; losing GPS after an initial fix can therefore leave a steady estimate. With GPS off from startup, the estimate is about 1 m from truth because the UGV spawns at map x=1 m and the filter receives no global position anchor. The seed controls injected noise, but startup timing can change which readings the filter sees first, so repeated runs vary. These results describe this short simulated route and measurement model. They do not establish real GPS-denied performance or longer-term drift.

The live runner writes per-sample CSV and JSON summaries and accepts another estimate topic and launch file. [Interfaces](interfaces.md#repeatable-accuracy-trials) gives the command and output format.

## Future Test Conditions

The current short route has steady wheel motion and a reliable IMU heading. Add these cases to make the evaluation harder and more useful:

- **Longer GPS outages:** Drive farther after the last GPS fix to measure how position error grows over time and distance.
- **Wheel slip:** Make wheel odometry disagree with the UGV's true motion, especially during turns or on low-traction ground.
- **IMU drift:** Add a persistent heading error and measure how it changes the predicted path during GPS loss.
- **Different routes:** Include repeated turns, stops, and direction changes so the filter is tested beyond one route shape.

Run each case with GPS on, lost during the route, and off from startup. Compare estimates with Gazebo truth using the same error metrics and report the test conditions alongside the results.
