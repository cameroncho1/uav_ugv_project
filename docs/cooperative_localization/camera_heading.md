# Camera heading and direction indicator

The UAV camera measures the UGV's forward direction from its roof AprilTag. The
annotated camera image draws a green arrow along chassis +X. This direction is
independent of the fused UGV estimate and remains meaningful while the UGV is
stationary or reversing. It represents facing direction, not velocity.

## Run and view

Run inside Ubuntu with the documented ROS, PX4, and Gazebo dependencies:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch mission_manager simple_goal_demo.launch.py initial_gps_enabled:=false
```

RViz includes a **Camera heading** image display. The topic is
`/uav/camera/heading_image`, a best-effort `sensor_msgs/Image` in `bgr8`. It keeps
the source image's acquisition timestamp and optical frame. The background is
grayscale; the annotation is colored. The detector generates annotated pixels
only while this topic has subscribers. The raw image and mapping input remain
`/uav/camera/image`.

An absent or rejected target produces a processed image labeled **Target
unavailable**, with no retained arrow. Invalid or stale source images produce no
fresh annotated image. A viewer may retain its last frame during a stopped
camera stream; check image timestamps before interpreting a frozen display.

The demo enables heading fusion. Set `uav_heading:=false` to use camera position
without camera heading; the measured arrow remains available. Set
`uav_assistance:=false` to disable all UAV corrections to the filter.

Camera trial scripts accept `UAV_HEADING=true` or `UAV_HEADING=false`. Summaries
record the selected mode and count joint heading corrections separately. Synthetic
position-only trials default to heading disabled.

For the standalone estimator:

```bash
ros2 launch cooperative_localization ugv_localization.launch.py \
  uav_assistance:=true uav_heading:=true
```

Supply the independent UAV anchor and measured relative pose. Standalone
defaults disable assistance and heading. Simulation also requires
`use_sim_time:=true` and `/clock`.

## Ownership and interfaces

| Owner | Responsibility |
| --- | --- |
| `uav_sensing` | Detect tag pixels, estimate chassis pose and uncertainty, publish the relative measurement and annotated image. |
| `uav_control` | Supply an independent UAV map anchor and its uncertainty. |
| `localization_common` | Compose poses, propagate joint covariance, and project heading without ROS imports. |
| `cooperative_localization` | Match acquisition times, validate corrections, select position-only or joint updates, and fuse them. |
| `fault_management` | Assemble evaluation inputs and score filter behavior. |

`/ugv/localization/pose` remains `PoseWithCovarianceStamped` in ENU `map`, with
position, orientation, acquisition time, and covariance. Planning and control
consume this authoritative output.

The camera producer publishes `/ugv/sensors/uav_relative_pose` using the same
message type and `uav/base_link` FLU parent frame. Position is the UGV chassis
origin relative to the UAV body origin. Orientation rotates chassis axes into
UAV body axes. The complete 6-by-6 covariance uses fixed-parent-axis angular
errors and includes position/orientation correlations. Position-only producers
use identity orientation with large angular covariance (`1e6 rad^2`); identity
alone does not mean that a heading is measured.

Each accepted observation generates exactly one filter input:

| Topic | Fused components | Selection |
| --- | --- | --- |
| `/ugv/measurements/uav_pose` | Map x/y | Heading disabled, unavailable, ill-conditioned, or too uncertain. |
| `/ugv/measurements/uav_pose_with_heading` | Map x/y/yaw jointly | Heading enabled and usable. |

Both inputs have independent five-sigma EKF innovation gates. The adapter never
publishes both inputs for the same accepted observation. This prevents duplicate
camera position updates and retains x/y/yaw correlations in the joint update.
Scorers count both topics. Diagnostics include `heading_accepted` and
`heading_unavailable`; counts describe adapter selection, not EKF acceptance.

## Geometry and uncertainty

The tag has known size and a rigid calibrated attachment to the chassis.
`marker_from_chassis_rotation` maps chassis axes into tag axes; its default is
identity, so tag +X points toward the robot's front. `marker_to_chassis` locates
the chassis origin in tag coordinates. A physically rotated tag needs the
corresponding rotation calibration. Camera extrinsics map optical coordinates
into UAV FLU. The image arrow projects the measured chassis forward axis at the
tag center; it does not use UAV odometry or simulator truth.

The solver transforms its Rodrigues-vector/translation covariance into joint
position and fixed-axis rotation covariance. Pixel, principal-point, focal-length,
tag-size, camera-mount position/angle, and tag-mount angle uncertainties are
included. `marker_mount_angle_stddev` defaults to `0.01 rad`; these values are
simulation assumptions, not hardware calibration results.

At the image acquisition time, let `R_a` rotate UAV body into map, `r` be the
relative chassis position, and `R_r` rotate chassis into UAV body:

```text
p_g = p_a + R_a r
R_g = R_a R_r
J_a = [[I, -skew(R_a r)], [0, I]]
J_r = blockdiag(R_a, R_a)
C_g = J_a C_a J_a^T + J_r C_r J_r^T
```

Anchor and relative errors are assumed independent. Internal position/angle
correlations in each source remain present. Heading is
`atan2(R_g[1,0], R_g[0,0])`. Its Jacobian includes roll/pitch effects and projects
the joint covariance into position/yaw coordinates. Only yaw is supplied to the
planar filter; roll/pitch carry large unused uncertainty.

The projected forward axis must have horizontal length at least `0.5` and map
heading standard deviation at most `max_heading_stddev` (`0.35 rad`). An invalid
quaternion or unknown angular covariance also prevents heading selection.
Usable position still contributes through the position-only input. Timing,
range, positional jump, frame, and covariance gates apply to both modes.

## Validation

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install
colcon test --executor sequential
colcon test-result --verbose
source install/setup.bash
cd ..
ROS_DOMAIN_ID=72 python3 scripts/test_camera_heading_fusion.py /tmp/heading_fusion.json
scripts/run_camera_perception_trial.sh /tmp/heading_pixels
scripts/run_cooperative_trials.sh /tmp/heading_position_regression
scripts/run_camera_anchor_trials.sh /tmp/heading_camera_anchors
scripts/run_goal_navigation_trial.sh /tmp/heading_goal
```

Numerical checks exercise rotations across the angle wrap, nonidentity tag mounts,
joint covariance against correlated Monte Carlo samples, and planar projection.
Callback checks cover acquisition stamps, duplicate suppression, fallback, and
overlay removal during occlusion. The rendered fixture measures pixel-derived
heading at four cardinal orientations, visibility, occlusion, recovery, and
off-axis geometry. The real-EKF fixture compares identical declared sensor
messages with and without camera yaw, including IMU bias, camera outage,
recovery, and heading outliers. [Validation results](heading_validation.md)
state the executed conditions and measured outcomes.

## Limits

The square-tag pose and its covariance are local estimates conditional on the
correct tag, camera calibration, and mounting geometry. Planar pose ambiguity,
small or blurred targets, and calibration bias can exceed this model. The map
heading inherits UAV heading error; a shared heading bias is not an independent
reference. Persistent disagreement can cause the joint innovation gate to reject
an entire camera update, including its position. Hardware calibration, uneven
terrain, motion blur, and simultaneous loss of both global references require
separate evaluation.
