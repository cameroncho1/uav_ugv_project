# Cooperative Localization Interfaces

## Implemented Sensor Input Layer

Both Gazebo worlds, `simple_goal_demo` and `uav_ugv`, contain the `simple_ugv` differential-drive robot. Its chassis has an IMU and a NavSat sensor. The differential-drive system publishes wheel odometry. `cooperative_localization` launches one Gazebo-to-ROS bridge for the three measurements and simulation clock.

The goal demo starts the bridge, measurement adapter, and EKF through `mission_manager/launch/simple_goal_demo.launch.py`. To run the full localization chain against an already running `uav_ugv` world, use `ros2 launch cooperative_localization ugv_localization.launch.py`. Pass `world:=simple_goal_demo` for that world. `ugv_sensors.launch.py` starts just the bridge and adapter. The bridge topic mapping is in `ros2_ws/src/cooperative_localization/config/ugv_sensor_bridges.yaml`.

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
- The bridged odometry and IMU covariance arrays contain zeros, and NavSatFix reports unknown position covariance. The worlds do not configure explicit sensor noise. These raw topics remain available for inspection; the adapter below makes noisy measurements with configured covariance for localization tests.
- PX4 UAV odometry uses NED coordinates. Any UAV measurement joined with UGV data needs an explicit conversion to the shared ENU `map` frame.

## Prepared Localization Measurements

`ugv_measurements.py` subscribes to the three raw ROS sensor topics and publishes these separate inputs. The two world configuration files in `ros2_ws/src/cooperative_localization/config/` contain each world's GPS origin and UGV spawn pose. `measurement_model.yaml` holds their shared error settings. These settings are simulation assumptions for repeatable tests, not measured hardware accuracy.

| ROS 2 topic | Type | Frame | Meaning |
| --- | --- | --- | --- |
| `/ugv/measurements/wheel_odom` | `nav_msgs/msg/Odometry` | `map`; child `simple_ugv/chassis` | Dead-reckoned planar pose anchored to the configured spawn pose, plus wheel-velocity readings. |
| `/ugv/measurements/imu` | `sensor_msgs/msg/Imu` | `simple_ugv/chassis/imu` | Planar yaw, angular velocity, and acceleration with configured noise. |
| `/ugv/measurements/gps_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `map` | WGS84 GPS fix converted to local east/north/up meters, then perturbed in meters. Position only; orientation is unknown. |

All outputs keep their input simulation timestamps and use reliable QoS so a later estimator can subscribe with its normal ROS 2 settings. The adapter subscribes to the raw best-effort sensor topics. The GPS origin is the world's `spherical_coordinates` origin. The conversion uses WGS84 Earth-centered coordinates and an east/north/up basis at that origin. The wheel odometry adapter starts at the configured UGV map spawn point and adds incremental motion, rather than treating its raw `(0, 0)` as the map origin. Both worlds spawn the UGV at zero yaw. The adapter does not publish TF or convert PX4's NED frame.

The configured GPS standard deviations are 1.5 m horizontally and 3 m vertically. Wheel position and yaw error grow with distance traveled, using 0.05 m and 0.02 rad per square root of meter. IMU yaw, angular-velocity, and acceleration standard deviations are 0.02 rad, 0.01 rad/s, and 0.1 m/s². Covariance fields contain the squared settings; unused 3D pose components carry large variance. A fixed random seed makes an unchanged message sequence reproducible. This simple independent Gaussian model does not represent all real GPS bias, wheel slip, or IMU drift, so filter tuning and evaluation must check its assumptions.

## Current Demo Consumers

The goal demo's UGV controller still reads Gazebo odometry directly and publishes `/ugv_pose` as `geometry_msgs/msg/PoseStamped` in `map`. It applies the demo's configured spawn offset. The local A* planner reads `/ugv_pose`. The raw and prepared localization topics do not drive the controller or planner yet.

## Fused UGV Position

`ugv_localization.launch.py` starts a planar `robot_localization` EKF and a small output adapter. It also publishes an identity TF from `simple_ugv/chassis` to `simple_ugv/chassis/imu`; the Gazebo IMU is mounted at the chassis link origin. The EKF configuration is `ros2_ws/src/cooperative_localization/config/ugv_ekf.yaml`. The EKF publishes at 30 Hz and continues prediction when an input is late. It does not publish a map-to-chassis TF; consumers use the pose message.

The EKF uses wheel forward velocity to predict movement, IMU yaw and turn rate to track heading, and GPS map x/y to correct position. It does not fuse the wheel's integrated pose a second time or use the GPS message's placeholder orientation. It publishes filtered odometry on `/ugv/localization/odometry`. The output adapter publishes the single fused pose interface:

- Topic: `/ugv/localization/pose`
- Type: `geometry_msgs/msg/PoseWithCovarianceStamped`
- Frame: `map` (ENU)
- Contents: EKF timestamp, position, orientation, and covariance

This is a GPS, wheel, and IMU baseline. The adapter copies the EKF's pose and covariance; it does not perform another filter step. The planner and controller still use `/ugv_pose`. The fused pose needs accuracy and GPS-loss tests before they can use it. A UAV-assisted measurement producer and correction are not implemented.

Gazebo's true state is for simulation checks and scoring only. It is not the fused localization output. GPS outage controls, estimator tuning, and accuracy evaluation are separate work.

## Run and Check

Install a Gazebo Harmonic build of `ros_gz_bridge` and `robot_localization` alongside ROS 2 Humble. On this Ubuntu 22.04 setup the packages are `ros-humble-ros-gzharmonic-bridge` and `ros-humble-robot-localization`. Build and source the workspace, then launch the demo or launch `ugv_localization.launch.py` while the `uav_ugv` world is running.

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 topic list -t | grep -E '/ugv/sensors/|^/clock'
ros2 topic echo --once /ugv/sensors/wheel_odom --qos-reliability best_effort
ros2 topic echo --once /ugv/sensors/imu --qos-reliability best_effort
ros2 topic echo --once /ugv/sensors/gps --qos-reliability best_effort
ros2 topic echo --once /ugv/measurements/wheel_odom --qos-reliability best_effort
ros2 topic echo --once /ugv/measurements/imu --qos-reliability best_effort
ros2 topic echo --once /ugv/measurements/gps_pose --qos-reliability best_effort
ros2 topic echo --once /ugv/localization/pose
```

Expected: all three raw sensors and their prepared measurements publish stamped readings, `/clock` advances, and the fused pose publishes in `map` with nonzero covariance. A listed topic alone does not prove that it is publishing; check at least one message. In the `uav_ugv` world, prepared wheel odometry starts near map `(1, 0)`; in `simple_goal_demo`, it starts near `(-4, 0)`. Individual GPS readings scatter around the spawn point because noise is enabled.
