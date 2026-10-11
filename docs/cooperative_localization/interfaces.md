# Cooperative Localization Interfaces

## Implemented Sensor Input Layer

Both Gazebo worlds, `simple_goal_demo` and `uav_ugv`, contain the `simple_ugv` differential-drive robot. Its chassis has an IMU and a NavSat sensor. The differential-drive system publishes wheel odometry. `cooperative_localization` launches one Gazebo-to-ROS bridge for the three measurements and simulation clock.

The goal demo starts this bridge through `mission_manager/launch/simple_goal_demo.launch.py`. The bridge also runs on its own with `ros2 launch cooperative_localization ugv_sensors.launch.py` when a world is already running. Its topic mapping is in `ros2_ws/src/cooperative_localization/config/ugv_sensor_bridges.yaml`.

| ROS 2 topic | Type | Gazebo topic | Rate configured in SDF | Frame in messages |
| --- | --- | --- | ---: | --- |
| `/ugv/sensors/wheel_odom` | `nav_msgs/msg/Odometry` | `/simple_ugv/odom` | 20 Hz | `simple_ugv/odom`, child `simple_ugv/chassis` |
| `/ugv/sensors/imu` | `sensor_msgs/msg/Imu` | `/simple_ugv/imu` | 50 Hz | `simple_ugv/chassis/imu` |
| `/ugv/sensors/gps` | `sensor_msgs/msg/NavSatFix` | `/simple_ugv/gps` | 5 Hz | `simple_ugv/chassis/gps` |
| `/clock` | `rosgraph_msgs/msg/Clock` | `/clock` | simulator clock | n/a |

The bridge direction is Gazebo to ROS. The three sensor topics use ROS `SENSOR_DATA` QoS (best effort); `/clock` uses `CLOCK` QoS. Messages keep Gazebo simulation timestamps. ROS consumers that compare message times against a node clock need `use_sim_time:=true`.

## Coordinates and Measurement Meaning

- Gazebo world coordinates use ENU. The NavSat sensor reports WGS84 latitude, longitude, and altitude using the world's `spherical_coordinates` origin.
- Wheel odometry is relative to the UGV's starting pose. Its `(0, 0)` is not the `map` origin. In `simple_goal_demo`, the UGV starts at map `(-4, 0)`; in `uav_ugv`, it starts at map `(1, 0)`.
- IMU orientation, angular velocity, and acceleration come from the sensor on the UGV chassis. The sensor frame is named in each message. No TF transforms are published for these frames.
- The bridged odometry and IMU covariance arrays contain zeros, and NavSatFix reports unknown position covariance. The worlds do not configure explicit sensor noise. A future estimator must assign defensible measurement uncertainty before treating these data as localization corrections.
- PX4 UAV odometry uses NED coordinates. Any UAV measurement joined with UGV data needs an explicit conversion to the shared ENU `map` frame.

## Current Demo Consumers

The goal demo's UGV controller still reads Gazebo odometry directly and publishes `/ugv_pose` as `geometry_msgs/msg/PoseStamped` in `map`. It applies the demo's configured spawn offset. The local A* planner reads `/ugv_pose`. The three `/ugv/sensors/*` topics are available to localization work; they do not drive the controller or planner yet.

## Localization Integration Contract

`cooperative_localization` owns the single fused UGV pose interface:

- Topic: `/ugv/localization/pose`
- Type: `geometry_msgs/msg/PoseWithCovarianceStamped`
- Frame: `map` (ENU)
- Contents: timestamp, position, orientation, and estimator covariance

This fused topic has no publisher in the current code. The estimator must combine raw UGV sensors and a UAV-assisted relative-position measurement, handle missing UGV GPS, and publish a continuous pose with uncertainty. The UAV-assisted measurement producer and topic are not implemented. The planner and controller can move from `/ugv_pose` to the fused topic when that publisher exists and the frame conversion is tested.

Gazebo's true state is for simulation checks and scoring only. It is not the fused localization output. GPS outage controls, measurement noise, estimator tuning, and accuracy evaluation are also separate work.

## Run and Check

Install a Gazebo Harmonic build of `ros_gz_bridge` alongside ROS 2 Humble. On this Ubuntu 22.04 setup the package is `ros-humble-ros-gzharmonic-bridge`. Build and source the workspace, then launch the demo or launch `ugv_sensors.launch.py` while the `uav_ugv` world is running.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 topic list -t | grep -E '/ugv/sensors/|^/clock'
ros2 topic echo --once /ugv/sensors/wheel_odom --qos-reliability best_effort
ros2 topic echo --once /ugv/sensors/imu --qos-reliability best_effort
ros2 topic echo --once /ugv/sensors/gps --qos-reliability best_effort
```

Expected: all three sensors publish stamped readings, and `/clock` advances while Gazebo runs. A listed topic alone does not prove that its sensor is publishing; check at least one message.
