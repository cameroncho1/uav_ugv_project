# Navigation from the fused pose

## Inputs and motion rules

The A* planner and UGV controller subscribe directly to `/ugv/localization/pose` (`PoseWithCovarianceStamped`). They require ENU `map`, a normalized quaternion, finite coordinates, and valid x/y/yaw covariance. Neither subscribes to simulator truth or raw wheel pose.

The shared [pose guard](../ros2_ws/src/uav_ugv_msgs/include/uav_ugv_msgs/pose_guard.hpp) checks acquisition age against ROS time and receipt age against a steady clock. Both must be at most 0.5 seconds. A frozen or missing `/clock` cannot keep navigation moving indefinitely. Duplicate and out-of-order samples are rejected. A clock rewind clears the accepted pose; a fresh sample is required to resume.

Position uncertainty is the square root of the largest eigenvalue of the x/y covariance, including the x/y correlation. Its limit is 0.5 m; yaw standard deviation is limited to 0.2 rad. The selected x/y/yaw covariance must be positive definite. Zero covariance does not mean known position. The EKF starts with x/y variance `1e6 m²` until an absolute position arrives; wheel and IMU updates alone do not locate an unknown global origin. These thresholds and initialization values are explicit simulation assumptions.

The planner publishes an empty route while pose, goal, or grid is invalid. Grids require `map`, matching dimensions/data, positive resolution, an axis-aligned origin, and acquisition/receipt ages at most 2 seconds. Goals require finite map coordinates. The existing eight-neighbor A* search avoids diagonal corner cutting and can adjust blocked endpoints within five cells.

The controller requires a nonempty finite map route with acquisition/receipt ages at most 1 second. A changed goal clears its cached route. It follows a 0.6 m lookahead with linear speed at most 0.5 m/s and angular speed at most 1 rad/s. Goal tolerance is 0.5 m in the estimated map pose. Actual goal error is scored separately; estimate tolerance is not a guarantee of physical arrival accuracy.

The controller publishes `/ugv/cmd_vel` (`Twist`) every 100 ms, including a zero command during every hold. The goal launch bridges this topic to Gazebo `/simple_ugv/cmd_vel`. This separates navigation from simulator transport. A hardware driver can consume the same ROS command interface. An actuator-side command timeout is still needed to handle loss of the controller process or transport itself.

Settings live in [navigation.yaml](../ros2_ws/src/mission_manager/config/navigation.yaml). The mapper remains the existing camera-luminance heuristic; these changes do not establish obstacle detection accuracy or camera-grid calibration.

## Verification and commands

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-up-to mission_manager --symlink-install
colcon test --packages-select uav_ugv_msgs ugv_control route_planning \
  cooperative_localization mission_manager
colcon test-result --verbose
source install/setup.bash
cd ..

# Use an unused domain; this script starts and stops its own planner/controller.
ROS_DOMAIN_ID=55 python3 scripts/test_navigation_guards.py /tmp/navigation_guards.json

# Repeat the paired localization cases for the configured EKF initialization.
scripts/run_cooperative_trials.sh /tmp/navigation_localization

# Navigation with GPS denied before the first sensor measurement.
ros2 launch mission_manager simple_goal_demo.launch.py initial_gps_enabled:=false

# Starts/stops its own world, PX4, demo, and truth-only arrival scorer.
scripts/run_goal_navigation_trial.sh /tmp/goal_navigation
```

The ROS integration check covers raw-pose-only startup, valid navigation, high position/yaw uncertainty, correlated x/y uncertainty, wrong frame, old/future stamps, pose loss, frozen clock, grid loss, malformed/blocked grids, recovery, and goal arrival. The final half-second of each hold must contain only zero commands. Numerical tests cover the covariance gate, clock resets, monotonic samples, path lookahead, and speed/turn bounds. Results are stored alongside [localization evaluations](cooperative_localization/evaluation_results.md).

The upstream [robot_localization filter implementation](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/src/filter_base.cpp) initializes unmeasured state components from its configured prior. A large prior is necessary for unknown global x/y. Freshness guards protect against absent data; covariance guards depend on the measurement model honestly representing its errors. A shared unknown global bias remains unobservable from relative sensing alone.
