# Navigation from the authoritative pose

## Inputs and route association

Planning and control consume `/ugv/localization/pose` in ENU `map`. The shared pose guard requires finite position, normalized attitude, positive-definite x/y/yaw covariance, monotonic acquisition stamps, and acquisition/steady receipt ages at most 0.5 s. Position standard deviation is the largest x/y eigenvalue's square root (limit 0.5 m); heading limit is 0.2 rad. A frozen clock does not keep the controller moving.

`/global_goal_pose` uses a stable positive acquisition stamp as its goal revision. Repeated publication preserves that stamp; an intentional new goal or mission-clock rewind creates a new revision. `/ugv/navigation/route` (`uav_ugv_msgs/PlannedRoute`) carries that goal, path, and the footprint-clearance grid used to plan it. The controller requires matching goal revision/coordinates, a matching endpoint, valid connected map segments, and a fresh path. A delayed route for another goal cannot authorize motion. `/local_path` is visualization/evaluation only.

## Clearance and following

The planner requires a valid acquisition-stamped map with acquisition/receipt ages at most 2 s. It rejects blocked/out-of-bounds endpoints; it does not relocate them across obstacles. The configured 0.5 m robot radius plus 0.05 m clearance is inflated conservatively with a square containing the circular footprint, plus cell extent. Unknown cells and map boundaries are blocked. The two-pass distance transform runs in linear grid size. These dimensions are simulation assumptions and need review for another robot.

The eight-neighbor A* avoids diagonal corner cutting. Start/goal connections are checked against the clearance grid. The follower chooses a visible lookahead along the forward route, turns before substantial translation through a bend, and checks predicted motion segments over 0.4 s. It cannot shortcut a blocked connection merely because a farther path point is free. Commands are limited to 0.5 m/s and 1 rad/s; estimated goal tolerance is 0.5 m.

Mapping remains a camera-luminance heuristic. It now uses ROS camera images/CameraInfo, a timestamp-matched independent map anchor, and calibrated optical/body extrinsics. Grid stamps retain image acquisition time. Missing anchor coverage produces no fresh map. A bounded-uncertainty, matched UGV pose masks its own visible body using a known 0.5 m radius / 0.5 m height cylinder projected through camera rays, with cell-coverage allowance. This removes the elevated roof's false ground obstacle; it does not feed the relative-observation producer. Flat-ground projection, known self-geometry masking and luminance classification do not establish semantic obstacle accuracy or detection behind the robot. The demo's visual-only goal ring stays above the dark-obstacle threshold.

## Actuator command expiry

The ROS controller publishes `/ugv/cmd_vel` every 100 ms, including zeros during holds. The bridge targets Gazebo `/simple_ugv/drive_cmd_vel`. The `ugv_control::CommandWatchdog` plugin runs inside the actuator's Gazebo process and alone forwards commands to `/simple_ugv/cmd_vel` for DiffDrive. It uses a steady-clock 0.35 s receipt timeout and a 50 ms delivery period, independently of the ROS controller/bridge and `/clock` progression. Missing or nonfinite commands produce zero drive commands.

Both operational worlds load this plugin. World runners set `GZ_SIM_SYSTEM_PLUGIN_PATH` to the built driver library. A hardware driver needs its own actuator-side implementation of the same expiry policy; the Gazebo plugin is not a hardware watchdog.

## Scoring and verification

The arrival scorer requires one continuous fresh second of stopped commands, estimated distance at most 0.5 m, and the declared physical tolerance (0.85 m in the camera trial). A deadline during dwell cannot pass. Physical tolerance remains distinct from estimated tolerance.

The scorer also logs planned-route snapshots and acquisition-matched physical truth. Point-to-polyline cross-track error uses the most recent plan at or before truth time, with at most 1 s reference age. `.tracking.csv` stores samples; `.routes.json` stores reference paths. JSON reports cross-track RMSE/mean/p95/max, travel distance and fractional reduction in physical distance to the fixed goal. Unmatched/stale references are counted as skipped, not silently scored.

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
colcon test --executor sequential
colcon test-result --verbose
source install/setup.bash
cd ..
ROS_DOMAIN_ID=55 python3 scripts/test_navigation_guards.py /tmp/navigation_guards.json
ROS_DOMAIN_ID=56 python3 scripts/test_actuator_watchdog.py /tmp/watchdog
ROS_DOMAIN_ID=57 python3 scripts/run_path_tracking_trials.py /tmp/tracking
scripts/run_goal_navigation_trial.sh /tmp/camera_goal
```

The ROS guard harness includes blocked starts and delayed obsolete-goal routes in addition to the original stale/uncertain/frame/grid/clock cases. The actuator harness kills the actual controller and command bridge in separate fresh worlds and scores physical stopping. It uses declared control-input fixtures independent of measured truth; it does not validate estimation accuracy.

Closed-loop baseline/assisted tracking uses separate fresh worlds with identical configured seeds, map and goal. Trajectories and sensor sequences differ under feedback. The fixture supplies a declared free grid, fixed-observer synthetic observations, and the drift profile. GPS outage begins no earlier than 2 s after first motion and after at least 11 received fixes, preserving the GPS evidence threshold. This isolates tracking from camera-grid accuracy. Baseline holds/failures remain recorded.

[Results](cooperative_localization/evaluation_results.md) and the [remediation record](project/remediation_2026-10-12.md) identify measured conditions. Uneven terrain, hardware camera calibration, unknown shared global bias and reliable semantic mapping remain outside these simulation claims.
