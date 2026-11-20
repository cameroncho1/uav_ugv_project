# Navigation from the authoritative pose

## Inputs and route association

Planning and control consume `/ugv/localization/pose` in ENU `map`. The shared pose guard requires finite position, normalized attitude, positive-definite x/y/yaw covariance, monotonic acquisition stamps, and acquisition/steady receipt ages at most 0.5 s. Position standard deviation is the largest x/y eigenvalue's square root (limit 0.5 m); heading limit is 0.2 rad. A frozen clock does not keep the controller moving.

`/global_goal_pose` uses a stable positive acquisition stamp as its goal revision. Repeated publication preserves that stamp; an intentional new goal or mission-clock rewind creates a new revision. `/ugv/navigation/route` (`uav_ugv_msgs/PlannedRoute`) carries that goal, path, and the footprint-clearance grid used to plan it. The controller requires matching goal revision/coordinates, a matching endpoint, valid connected map segments, and a fresh path. A delayed route for another goal cannot authorize motion. `/local_path` is visualization/evaluation only.

## Clearance and following

The planner requires a valid acquisition-stamped map with acquisition/receipt ages at most 2 s. It rejects blocked/out-of-bounds endpoints; it does not relocate them across obstacles. The configured 0.5 m robot radius plus 0.05 m clearance is inflated as a disk: a cell is blocked when the gap between its square and any non-free cell's square is at most that radius, so the robot center may lie anywhere in a free cell. Unknown cells and map boundaries are blocked. Only non-free cells that border free space stamp the disk. The default 0.1 m grid keeps cell quantization small; with 0.25 m cells the dense layout's 2 m gaps left a single free cell. These dimensions are simulation assumptions and need review for another robot.

The eight-neighbor A* avoids diagonal corner cutting. Start/goal connections are checked against the clearance grid. The planner then replaces the cell staircase with the longest clear straight shortcuts. The follower chooses a visible lookahead along the forward route, turns in place when the heading error exceeds 0.6 rad, otherwise drives while turning with speed reduced in proportion to the error, and checks predicted motion segments over 0.4 s. It cannot shortcut a blocked connection merely because a farther path point is free. Commands are limited to 0.5 m/s and 1 rad/s. The configured controller completes at an estimated distance of 0.35 m, leaving 0.15 m inside R2's 0.5 m estimated-arrival boundary for post-stop estimate variation. This margin is a simulation setting, not a bound on localization drift.

Mapping remains a camera-luminance heuristic. It now uses ROS camera images/CameraInfo, a timestamp-matched independent map anchor, and calibrated optical/body extrinsics. Grid stamps retain image acquisition time. Missing anchor coverage produces no fresh map. A bounded-uncertainty, matched UGV pose masks its own visible body using a known 0.5 m radius / 0.5 m height cylinder projected through camera rays, with cell-coverage allowance. This removes the elevated roof's false ground obstacle; it does not feed the relative-observation producer. Flat-ground projection, known self-geometry masking and luminance classification do not establish semantic obstacle accuracy or detection behind the robot. The demo's visual-only goal ring stays above the dark-obstacle threshold.

## Map memory and unseen ground

The mapper fuses every camera grid into one fixed map (`map_bounds`, default x -12 to 12 m and y -10 to 10 m) by clamped log-odds: each dark view adds 0.6, each clear view subtracts 0.6, within -1.5 to 2.5. One clear view erases a single false dark view; a long-seen obstacle needs about five. Never-observed cells stay unknown (-1). `map_memory:=false` restores per-frame grids.

The planner inflates two grids. The planning grid treats unknown cells as free and costs them twice as much, so A* can aim at a goal the camera has not seen while preferring observed ground. The clearance grid keeps unknown cells blocked. The published route is the route's prefix that stays clear in the clearance grid; if it stops before the goal, `PlannedRoute.partial` is true. The controller accepts a partial route for the current goal revision and drives only on observed, clear ground.

## UAV following

With `uav_follow:=true` (default) the offboard controller keeps its takeoff hold until PX4 reports Offboard, armed and at least 80% of the altitude setpoint. It then moves its horizontal setpoint toward the UGV fused pose, converted to PX4 NED with the same `map_origin` as the anchor adapter, at up to 1 m/s with a 0.15 m deadband. It holds whenever the fused pose is older than 0.5 s or its position standard deviation exceeds 0.5 m, so it never chases an unusable estimate. With GPS off at startup it holds until the camera's first UGV correction.

## Camera transport

The goal-demo launch sets `FASTRTPS_DEFAULT_PROFILES_FILE` to [config/fastdds_local_shm.xml](../config/fastdds_local_shm.xml) unless it is already set. Whole 3.7 MB camera images then pass through shared memory. With default UDP fragments and the default Linux receive buffer, an ARM64 UTM VM received about 0.3 of the 5 rendered images per second, and the map was usually older than the planner's 2 s limit. Processes started outside the launch need the same variable to receive images reliably.

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

[Historical results](cooperative_localization/evaluation_results.md) identify the tested GPS-branch conditions. Uneven terrain, hardware camera calibration, unknown shared global bias and reliable semantic mapping remain outside these simulation claims.

## Verified supervisor

The path follower runs only when the persistent supervisor selects `NAVIGATE`. Other modes publish zero every timer tick. Completed-goal memory prevents estimated drift from restarting the same goal. Enabled detector holds require ten distinct clear decisions and five simulation seconds to recover; new goals do not bypass recovery. Detector enablement defaults to false until a producer is deployed.

See the [supervisor specification](verification/supervisor.md), [finite proof and runtime evidence](project/results/supervisor/README.md), and [course/report mapping](verification/supervisor_course_alignment.md). The proof establishes discrete command permission under explicit input assumptions; the independent watchdog and recorded physical stopping checks remain necessary.

## Dynamic obstacle example

The [dynamic obstacle scenario](dynamic_obstacles/README.md) adds a scheduled box through Gazebo's world API. The UAV camera updates the existing map, the planner recomputes a route, and the controller follows it or holds. Scenario and truth geometry are evaluation inputs; navigation receives the same camera-map and measured-pose interfaces as before.
