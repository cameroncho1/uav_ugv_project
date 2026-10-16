# Sensor Simulation & Fault-Detection Reference

This document is the working reference for the sensor-simulation layer of the
UAV/UGV project and for wiring simulated sensor data into the
`uav_ugv_project_fault_detection/` tooling. It explains where sensor definitions
and Gaussian noise actually live, how to add or customize sensors, how to measure
the distance between the UAV and the UGV, how to inject noise, and how to verify
everything with `ros2 topic` / `gz topic` tooling.

**Scope of the repository.** The fault-detection tooling lives in
`uav_ugv_project_fault_detection/`, but the simulation that it drives is spread
across three places:

- `simulation/` — the Gazebo world `.sdf` files (environment, UGV model, plugin registration).
- `ros2_ws/` — the ROS 2 workspace (control/mapping/planning nodes and the `gz_bridge`).
- `external/PX4-Autopilot/` — the PX4 SITL firmware and the Gazebo `.sdf` models that
  actually **define the UAV sensors and their Gaussian noise**.

**Stack.**

| Layer | Tool | Version/notes |
|---|---|---|
| Middleware / ROS | ROS 2 | Humble |
| Gazebo | `gz` | Gazebo Harmonic (`gz-sim`, `gz-transport`, `gz-gui`, `ogre`/`ogre2` render engines) |
| Autopilot | PX4 SITL | In `external/PX4-Autopilot/`, built with `make px4_sitl <target>` |
| DDS bridge | Micro XRCE-DDS | `MicroXRCEAgent udp4 -p 8888` connects the PX4 SITL DDS agent to the ROS 2 graph |

All three parties (Gazebo, PX4 SITL, ROS 2) talk to each other over DDS using a
shared partition and loopback IP (see the Prerequisites section).

---

## 1. Overview

The sensor pipeline flows in one direction and is bridged at a single point:

1. **Gazebo simulates the world.** It loads a world `.sdf`
(`simulation/worlds/uav_ugv.sdf` or `simulation/worlds/simple_goal_demo.sdf`),
which defines the environment (ground plane, walls, obstacles, goal marker) and
the `simple_ugv` differential-drive robot, plus the sensor **system plugins**
that publish Gazebo sensor topics.

2. **PX4 SITL models the UAV and injects the noise.** The X500 airframe is a Gazebo
model composed by `make px4_sitl <target>`. The **sensor definitions and their
Gaussian noise live in the PX4 Gazebo model `.sdf` files** under
`external/PX4-Autopilot/Tools/simulation/gz/models/**/model.sdf` — **not** in the
project's world `.sdf` files. This is the single most important thing to internalize:
to change how noisy the IMU / magnetometer / barometer are, you edit the PX4 model,
not `simulation/worlds/*.sdf`.

3. **`gz_bridge` bridges Gazebo topics to uORB / ROS 2.** The PX4 module
`external/PX4-Autopilot/src/modules/simulation/gz_bridge/GZBridge.cpp` subscribes to
Gazebo topics of the form
`/world/<world>/model/<model>/link/<link>/sensor/<sensor>/<data>` and republishes
them as uORB topics (and, via Micro XRCE-DDS, as ROS 2 `/fmu/out/*` topics).

4. **ROS 2 nodes consume the bridged data.** The mission nodes
(`uav_control`, `ugv_control`, `traversability_mapping`, `mission_manager`,
`route_planning`) subscribe to `/fmu/out/*` and to the raw Gazebo topics and publish
the control / mapping / planning topics that close the loop.

Because of this split, "where does a sensor come from?" has two answers:
- **UGV** (`simple_ugv`): its only "sensor" is the differential-drive odometry topic
(`/simple_ugv/odom`, 20 Hz), produced by the `DiffDrive` system plugin inside the
world `.sdf`. There is **no Gaussian noise** on it.
- **UAV** (X500): IMU, magnetometer, air pressure, navsat, camera, and LiDAR are all
defined — with Gaussian noise — in the PX4 model `.sdf` files.

---

## 2. Project structure (where sensor simulation lives)

| Location | What it holds | Key files |
|---|---|---|
| `simulation/worlds/` | Gazebo world: environment, UGV model, sensor-system plugin registration. | [uav_ugv.sdf](../simulation/worlds/uav_ugv.sdf), [simple_goal_demo.sdf](../simulation/worlds/simple_goal_demo.sdf) |
| `external/PX4-Autopilot/Tools/simulation/gz/models/` | UAV sensor definitions + Gaussian noise. | [x500_base/model.sdf](../external/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf), `mono_cam/`, `x500_lidar_down/`, `x500_lidar_front/`, `x500_flow/`, `x500_depth/`, `x500/` |
| `ros2_ws/src/` | ROS 2 nodes + the `gz_bridge`. | `uav_control/`, `ugv_control/`, `traversability_mapping/`, `mission_manager/`, `route_planning/`, `px4_msgs/`, `uav_ugv_msgs/`; `external/PX4-Autopilot/src/modules/simulation/gz_bridge/GZBridge.cpp` |
| `scripts/` | Launch / orchestration shell scripts. | [start_sim.sh](../scripts/start_sim.sh), `world.sh`, `world_goal_demo.sh`, `px4.sh`, `px4_goal_demo.sh`, `dds.sh`, `gui.sh` |
| `config/` | Configuration (e.g. RViz configs shipped via the `mission_manager` package share dir). | `ros2_ws/src/mission_manager/config/*.rviz` |

### Sensor map (per sensor → model file, type, rate, noise)

| Sensor | Type | Model file | Rate | Gaussian noise (per axis / value) |
|---|---|---|---|---|
| `imu_sensor` | `imu` | [x500_base/model.sdf](../external/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf) | 250 Hz | gyro `x/y/z` stddev **0.0008726646** rad/s; accel `x/y` stddev **0.00637** m/s², accel `z` stddev **0.00686** m/s²; all `mean 0` |
| `magnetometer_sensor` | `magnetometer` | `x500_base/model.sdf` | 100 Hz | `x/y/z` stddev **0.0001** (gaussian) |
| `air_pressure_sensor` | `air_pressure` | `x500_base/model.sdf` | 50 Hz | `mean 0`, `stddev 3` (Pa) |
| `navsat_sensor` | `navsat` | `x500_base/model.sdf` | 30 Hz | **no noise** |
| `imager` (camera) | `camera` | `mono_cam/model.sdf` | 30 Hz | `hfov 1.74`, `1280×960`, `near 0.1`, `far 3000` |
| `lidar` | `gpu_lidar` (`<ray>`) | `x500_lidar_down/`, `x500_lidar_front/` | 50 Hz | `<ray> scan/range`, `range 0.1–100 m`, **no noise** |

> Verified facts: the world `.sdf` files (`uav_ugv.sdf`, `simple_goal_demo.sdf`) contain
> **no `<sensor>` elements and no `<noise>` elements** — they only register sensor *system
> plugins* and the UGV differential-drive model. All sensor **definitions** and **noise**
> for the UAV are in the PX4 model `.sdf` files.

---

## 3. Prerequisites & Setup

You will need a machine that matches the stack in the table above. The reference
machine is **Ubuntu 22.04** with the following.

### 3.1 Install dependencies

```bash
# ROS 2 Humble
sudo apt update
sudo apt install -y ros-humble-ros-base ros-humble-ros2cli \
    ros-humble-geometry-msgs ros-humble-nav-msgs ros-humble-rviz2
sudo ln -s /opt/ros/humble/setup.bash /opt/ros/humble/setup.bash  # already there

# Gazebo Harmonic (gz-sim, gz-transport, gz-gui) + Micro XRCE-DDS
#   See the PX4 docs for the exact apt source; the project expects the `gz`
#   binaries and the MicroXRCEAgent on PATH.
which gz          # e.g. /usr/bin/gz
which MicroXRCEAgent
```

### 3.2 Fetch the PX4 submodule and build the SITL target

```bash
git submodule update --init --recursive          # populates external/PX4-Autopilot
cd external/PX4-Autopilot
make px4_sitl gz_x500_mono_cam_down              # the target used by the demo
cd -
```

### 3.3 Build the ROS 2 workspace

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
cd ..
```

### 3.4 Shared DDS environment

Every component (Gazebo, PX4 SITL, the ROS 2 graph) must share the **same DDS
partition and IP**, or the topics will not connect. These are already exported by
the launch files and the shell scripts:

```bash
export GZ_PARTITION=uav_ugv_sim
export GZ_IP=127.0.0.1
```

### 3.5 Launch the simulation

**Option A — one launch file (recommended for development):**

```bash
ros2 launch mission_manager simple_goal_demo.launch.py
```

This starts Gazebo (`gz sim` on [simple_goal_demo.sdf](../simulation/worlds/simple_goal_demo.sdf)),
the `MicroXRCEAgent` (`udp4 -p 8888`), PX4 SITL, and the mission nodes
(`goal_manager`, `uav_goal_controller`, `ugv_goal_controller`,
`traversability_mapper`, `local_astar_planner`) plus RViz.

Useful launch arguments (defaults shown):

| Argument | Default | Meaning |
|---|---|---|
| `goal_x` | `4.0` | Goal east coordinate (m). |
| `goal_y` | `0.0` | Goal north coordinate (m). |
| `uav_altitude` | `4.0` | UAV target altitude (m). |
| `grid_resolution` | `0.25` | Camera occupancy-grid cell size (m). |
| `obstacle_luminance_threshold` | `32.0` | Dark-pixel threshold for the camera occupancy heuristic. |
| `ugv_odom_to_map_x` | `-4.0` | X offset from Gazebo UGV odometry to the `map` frame. |
| `ugv_odom_to_map_y` | `0.0` | Y offset from Gazebo UGV odometry to the `map` frame. |
| `start_simulation` | `true` | Start Gazebo, PX4 SITL, and the DDS agent. |
| `start_rviz` | `true` | Start RViz. |

**Option B — the orchestration script (opens one terminal per component):**

```bash
chmod +x scripts/*.sh
./scripts/start_sim.sh          # everything, including the 3D Gazebo window
./scripts/start_sim.sh --no-gui # everything except the 3D window
./scripts/start_sim.sh stop     # shut every simulation process down
```

Under the hood, `px4_goal_demo.sh` runs:

```bash
PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD=simple_goal_demo \
PX4_GZ_MODEL_POSE="0,0,0.3,0,0,0" \
make px4_sitl gz_x500_mono_cam_down
```

`world_goal_demo.sh` runs:

```bash
export GZ_SIM_RESOURCE_PATH="$ROOT/simulation/models:${GZ_SIM_RESOURCE_PATH:-}"
gz sim -v4 -r --render-engine-server ogre --render-engine-gui ogre \
    "$ROOT/simulation/worlds/simple_goal_demo.sdf"
```

> **Note:** changing `PX4_GZ_MODEL_POSE` sets where the UAV spawns; changing
> `PX4_GZ_WORLD` (and the `WORLD` in `world_goal_demo.sh`) selects the world file.
> These must stay consistent, or the bridge will look for sensor topics that no
> world is producing.

---

## 4. Creating Custom Simulations

A "custom simulation" in this project means one of three things, depending on what
you are trying to do. Pick the level that matches your goal, then follow the
steps. The three levels, from least to most invasive, are:

1. **Reuse** an existing PX4 airframe/model and just relaunch it with different
   pose/world parameters (no code changes).
2. **Extend** a world `.sdf` or a PX4 model `.sdf` by adding/editing a
   `<sensor>` and/or a `<plugin>` element.
3. **Add a new ROS 2 node** that produces or consumes simulated sensor data.

### 4.1 Level 1 — Reuse an existing airframe (no file edits)

The PX4 model you load is chosen by the `make px4_sitl <target>` argument, which
maps to a model directory under
[Tools/simulation/gz/models](../external/PX4-Autopilot/Tools/simulation/gz/models).

**Step 1.** Pick a target. The demo uses `gz_x500_mono_cam_down`; the available
UAV targets add different sensor suites:

| Target | Adds |
|---|---|
| `gz_x500` | Base IMU, magnetometer, air pressure, navsat. |
| `gz_x500_mono_cam_down` | Base + downward mono camera (used by `traversability_mapper`). |
| `gz_x500_lidar_down` | Base + downward `gpu_lidar` (`<ray>`). |
| `gz_x500_lidar_front` | Base + forward `gpu_lidar`. |
| `gz_x500_flow` | Base + optical flow. |
| `gz_x500_depth` | Base + depth camera. |

**Step 2.** Point the bridge at the right world and spawn pose, then build/run:

```bash
cd external/PX4-Autopilot
PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD=simple_goal_demo \
PX4_GZ_MODEL_POSE="0,0,0.3,0,0,0" \
make px4_sitl gz_x500_lidar_down
```

**Step 3.** Confirm the new sensor's topic exists:

```bash
ros2 topic list | grep -E "obstacle_distance|distance_sensor|lidar"
```

### 4.2 Level 2 — Add or edit a Gazebo `<sensor>` (most common)

This is the right level when you want to add a sensor that PX4's bridge already
knows how to forward (IMU, magnetometer, air pressure, navsat, camera, LiDAR,
optical flow, depth) or to change a sensor's rate/noise.

**Step 1 — choose where the sensor lives.**

- If the sensor belongs to the **UAV**, edit the PX4 model file, e.g.
  [x500_base/model.sdf](../external/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf).
  The `<link name="base_link">` block is where IMU, magnetometer, air pressure, and
  navsat are defined.
- If the sensor belongs to the **UGV** or to the **environment**, edit the world
  file [uav_ugv.sdf](../simulation/worlds/uav_ugv.sdf) or
  [simple_goal_demo.sdf](../simulation/worlds/simple_goal_demo.sdf) inside the
  relevant `<model>`.

**Step 2 — add the `<sensor>` element.** Here is a complete, valid template you can
drop into a `<link>`. It shows the fields every sensor needs plus a `<noise>` block
(covered fully in §6).

```xml
<link name="base_link">
  <sensor name="my_range_sensor" type="ray">
    <gz_frame_id>base_link</gz_frame_id>
    <pose>0 0 0 0 0 0</pose>              <!-- relative to the link -->
    <always_on>1</always_on>
    <update_rate>50</update_rate>         <!-- Hz -->
    <topic>/my_range</topic>              <!-- optional; see below -->
    <ray>
      <scan>
        <horizontal><samples>360</samples><resolution>1</resolution>
          <min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle>
        </horizontal>
        <vertical><samples>1</samples><resolution>1</resolution>
          <min_angle>0</min_angle><max_angle>0</max_angle>
        </vertical>
      </scan>
      <range>
        <min>0.1</min><max>100.0</max><resolution>0.01</resolution>
      </range>
    </ray>
    <visualize>true</visualize>
  </sensor>
</link>
```

The sensor type selects the inner config block: `imu` → `<imu>`,
`magnetometer` → `<magnetometer>`, `air_pressure` → `<air_pressure>`,
`navsat` → `<navsat>`, `camera` → `<camera>`, `ray`/`gpu_lidar` → `<ray>`,
`optical_flow` → `<optical_flow>`, `depth_camera` → `<depth_camera>`.

**Step 3 — register the sensor's system plugin** so Gazebo actually instantiates it.
`simple_goal_demo.sdf` already registers the standard sensor systems at world scope:

```xml
<plugin filename="gz-sim-imu-system"         name="gz::sim::systems::Imu"/>
<plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
<plugin filename="gz-sim-air-speed-system"    name="gz::sim::systems::AirSpeed"/>
<plugin filename="gz-sim-navsat-system"       name="gz::sim::systems::NavSat"/>
<plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
<plugin filename="gz-sim-sensors-system"      name="gz::sim::systems::Sensors">
  ...
</plugin>
```

If you add a sensor type that has **no** world-scope plugin yet, add its plugin too.
For example, a new `ray`/LiDAR sensor needs the sensors system to be present:

```xml
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
  <sensors>ray</sensors>
</plugin>
```

**Step 4 — know what topic the sensor publishes.** PX4's `gz_bridge`
([GZBridge.cpp](../external/PX4-Autopilot/src/modules/simulation/gz_bridge/GZBridge.cpp))
subscribes to Gazebo topics of the form

```
/world/<world>/model/<model>/link/<link>/sensor/<sensor>/<data>
```

For the demo (`world=simple_goal_demo`), examples are:

| Gazebo topic (bridged by PX4) | uORB / ROS 2 topic |
|---|---|
| `…/imu_sensor/imu` | `/fmu/out/imu` |
| `…/magnetometer_sensor/magnetometer` | `/fmu/out/magnetometer` |
| `…/air_pressure_sensor/air_pressure` | `/fmu/out/air_pressure` |
| `…/lidar_sensor_link/sensor/lidar/scan` | `/fmu/out/distance_sensor`, `/fmu/out/obstacle_distance` |
| `…/lidar_2d_v2/scan` | `/fmu/out/obstacle_distance` |

**Step 5 — relaunch and verify** (see §7). Rebuild the model target if you edited a
PX4 model, then restart the launch.

### 4.3 Level 2b — Add a new Gazebo model (a brand-new robot/sensor rig)

Use this when the sensor does not fit any existing PX4 model.

**Step 1.** Create a model directory:

```
simulation/models/my_sensor_rig/
  model.sdf
  model.urdf      (or model.mjs / description)
```

**Step 2.** In `model.sdf`, declare the `<model>`, its `<link>`(s), and the
`<sensor>` element(s) exactly like §4.2 Step 2. Point the model at an existing
world by passing it to `gz sim` via `GZ_SIM_RESOURCE_PATH` (already set to
`$ROOT/simulation/models` by `world_goal_demo.sh`):

```bash
export GZ_SIM_RESOURCE_PATH="$ROOT/simulation/models:${GZ_SIM_RESOURCE_PATH:-}"
gz sim -r my_sensor_rig.sdf
```

**Step 3.** If the sensor data must reach PX4/uORB, either (a) have PX4's bridge
subscribe to it (only works for the sensors the bridge knows), or (b) publish it
straight to a ROS 2 topic with your own bridge/node (§4.4).

### 4.4 Level 3 — Add a new ROS 2 node that simulates sensor data

Use this when you want full control over the data (synthetic fault data, scripted
trajectories, custom fault injection for the fault-detection tooling). This is the
most flexible option and does not touch Gazebo at all.

**Step 1.** Create the package (if it does not exist):

```bash
cd ros2_ws/src
ros2 pkg create --build-type ament_python --dependencies \
    rclpy std-msgs geometry-msgs sensor-msgs uav_ugv_msgs my_fault_sensor
```

**Step 2.** Write a node that publishes a sensor message. Example: publish a
synthetic `sensor_msgs/Imu` with a configurable Gaussian noise level (see §6 for the
noise detail):

```python
# ros2_ws/src/my_fault_sensor/my_fault_sensor/node.py
import math, numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class SyntheticImu(Node):
    def __init__(self):
        super().__init__("synthetic_imu")
        self.rate = self.declare_parameter("rate", 250).get_parameter_value().double_value
        self.accel_stddev = self.declare_parameter("accel_stddev", 0.00637).get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / self.rate, self.publish)
        self.pub = self.create_publisher(Imu, "/sim/imu", 10)
        self.t = 0.0
    def publish(self):
        dt = 1.0 / self.rate
        self.t += dt
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        # ideal signal + Gaussian noise
        msg.linear_acceleration.z = 9.81 + np.random.normal(0.0, self.accel_stddev)
        msg.angular_velocity.x = np.random.normal(0.0, 0.0008726646)
        self.pub.publish(msg)
        self.get_logger().info(f"published accel_z={msg.linear_acceleration.z:.4f}")

def main():
    rclpy.init()
    rclpy.spin(SyntheticImu())
    rclpy.shutdown()
```

**Step 3.** Wire it into the launch file
([simple_goal_demo.launch.py](../ros2_ws/src/mission_manager/launch/simple_goal_demo.launch.py))
so it starts with the rest of the graph:

```python
Node(
    package="my_fault_sensor",
    executable="node",
    name="synthetic_imu",
    parameters=[{"accel_stddev": 0.00637, "rate": 250}],
    output="screen",
),
```

**Step 4.** Build and run, then verify (§7).

---

## 5. Measuring the Distance Between the UAV and the UGV

**Current state of the project:** there is **no dedicated inter-robot distance
sensor**. The two robots do not sense each other with a range/LiDAR sensor, and no
node currently publishes their separation distance. What *does* exist is a pose for
each robot:

| Robot | Pose topic | Message | Frame / convention |
|---|---|---|---|
| UAV | `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` | NED (north/east/down), from PX4 EKF. |
| UGV | `/ugv_pose` | `geometry_msgs/PoseStamped` | `frame_id = "map"` (ENU). |

The UGV pose comes from the differential-drive odometry on `/simple_ugv/odom`
(`gz::msgs::Odometry`) that
[ugv_control/goal_controller.cpp](../ros2_ws/src/ugv_control/src/goal_controller.cpp)
re-publishes to `/ugv_pose` after applying the `odom_to_map_x/y` offsets.

Because the poses are both available, the **recommended** way to get the
UAV↔UGV distance is to compute it from those poses (Approach A). Approach B adds a
real sensor so the distance is produced *in-simulation* instead of by a post-hoc
node.

### 5.1 Approach A — Euclidean distance from the two poses (recommended)

This needs no new sensor and works today. The only subtlety is the frame: the UAV
is in NED, the UGV is in ENU, so you must put both in the same frame before taking
the difference.

**Step 1.** Create a small node:

```python
# ros2_ws/src/uav_ugv_msgs or a new package; example in my_fault_sensor/node_dist.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry

class UavUgvDistance(Node):
    def __init__(self):
        super().__init__("uav_ugv_distance")
        self.uav = None   # px4 VehicleOdometry (NED)
        self.ugv = None   # PoseStamped (ENU, frame_id "map")
        self.pub = self.create_publisher(Float64, "/uav_ugv/distance", 10)
        self.sub_uav = self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self._uav_cb, 10)
        self.sub_ugv = self.create_subscription(
            PoseStamped, "/ugv_pose", self._ugv_cb, 10)
        self.create_timer(0.05, self._publish)

    def _uav_cb(self, msg):
        # NED -> ENU: x=east, y=north, z=up => z = -down
        self.uav = (msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                   -msg.pose.pose.position.z)

    def _ugv_cb(self, msg):
        p = msg.pose.position
        self.ugv = (p.x, p.y, p.z)   # already ENU in "map"

    def _publish(self):
        if self.uav is None or self.ugv is None:
            return
        dx = self.uav[0] - self.ugv[0]
        dy = self.uav[1] - self.ugv[1]
        dz = self.uav[2] - self.ugv[2]
        d = (dx*dx + dy*dy + dz*dz) ** 0.5
        self.pub.publish(Float64(data=d))

def main():
    rclpy.init()
    rclpy.spin(UavUgvDistance())
    rclpy.shutdown()
```

**Step 2.** Launch it (add the `Node(...)` to the launch file as in §4.4 Step 3, or
run it standalone):

```bash
ros2 run my_fault_sensor uav_ugv_distance
```

**Step 3.** Read the distance:

```bash
ros2 topic echo /uav_ugv/distance
# -> data: 3.742 ...
```

> **Why convert NED→ENU?** PX4 reports position in NED, so "down" is positive. The
> `map` frame is ENU, so "up" is positive. Converting the UAV's z to `-z` before the
> difference keeps the vertical term correct. If both poses were already in the same
> frame, you could skip the sign flip.

### 5.2 Approach B — A real range/LiDAR sensor between the robots

Use this when the fault-detection tooling must ingest a *sensor-measured* distance
(the measurement itself should be noisy, have a range limit, and possibly fail).

**Step 1.** Add a `ray` (LiDAR) sensor to one robot, aimed at the other. For the
UAV, edit
[x500_lidar_down/model.sdf](../external/PX4-Autopilot/Tools/simulation/gz/models/x500_lidar_down/model.sdf).
The existing downward LiDAR is a good template:

```xml
<link name="lidar_sensor_link">
  <pose relative_to="base_link">0 0 -0.05 0 1.57 0</pose>
  <sensor name="lidar" type="gpu_lidar">
    <gz_frame_id>lidar_sensor_link</gz_frame_id>
    <pose>0 0 0 3.14 0 0</pose>
    <update_rate>50</update_rate>
    <ray>
      <scan>
        <horizontal><samples>1</samples><resolution>1</resolution>
          <min_angle>0</min_angle><max_angle>0</max_angle></horizontal>
        <vertical><samples>1</samples><resolution>1</resolution>
          <min_angle>0</min_angle><max_angle>0</max_angle></vertical>
      </scan>
      <range><min>0.1</min><max>100.0</max><resolution>0.01</resolution></range>
    </ray>
    <always_on>1</always_on>
    <visualize>true</visualize>
  </sensor>
</link>
```

**Step 2.** To point it *at the UGV* instead of straight down, set the sensor
`<pose>` so its boresight axis points at the target. A downward sensor uses
`pitch=+90°` (`0 1.57 0` on the link, plus `roll=180°` on the sensor). Aim it at the
ground/UGV by keeping the downward orientation, or tilt it toward the UGV's position.
The single-ray config (`samples=1`) gives one range reading per update — a direct
"distance to the nearest thing along the boresight."

**Step 3.** Make sure the sensors system plugin is registered (§4.2 Step 3) and that
PX4's bridge subscribes to the scan topic. The bridge subscribes to
`/world/<world>/model/<model>/link/lidar_sensor_link/sensor/lidar/scan` and republishes
it as uORB `distance_sensor_s` / `obstacle_distance_s`, which Micro XRCE-DDS exposes as:

```
/fmu/out/distance_sensor
/fmu/out/obstacle_distance
```

**Step 4.** Read the measured distance (this is the *sensor's* distance, so it will
include any noise from §6):

```bash
ros2 topic echo /fmu/out/obstacle_distance
ros2 topic hz /fmu/out/obstacle_distance
```

**Step 5 (optional).** If you want a single "distance between the two robots" value
rather than a per-sector obstacle array, either pick the single-ray sector or, more
robustly, combine the LiDAR measurement with the pose-based distance from Approach A
in a small fusion node.

### 5.3 Which approach?

- **Approach A (pose-based)** — simplest, always available, exact (no sensor error).
  Best for the fault-detection tooling when you need a ground-truth reference distance.
- **Approach B (sensor-based)** — the measurement is *noisy and bounded*, so it is
  the right choice when the tooling must detect sensor faults or operate under
  realistic measurement conditions. It is also how you would feed a real distance
  sensor into the pipeline.

---

## 6. Applying Gaussian Noise to Sensor Data

There are three ways to inject Gaussian noise, from "native to the simulator" to
"fully in your own code." Start with Method 1 and move down only when you need more
control.

### 6.1 Method 1 — Gazebo SDF `<noise type="gaussian">` (native)

This is how every UAV sensor in the project already gets its noise. It is the
simplest and the correct place to change the noise on a real sensor.

**Where the noise lives.** Inside a `<sensor>`, on the specific field you want to
corrupt. Example from
[x500_base/model.sdf](../external/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf)
(the IMU, modeled after the IIM42653):

```xml
<sensor name="imu_sensor" type="imu">
  <gz_frame_id>base_link</gz_frame_id>
  <update_rate>250</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.0008726646</stddev>     <!-- 0.05 deg/s -> rad/s -->
        </noise>
      </x>
      <!-- y and z are identical -->
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian"><mean>0.0</mean><stddev>0.00637</stddev></noise>
      </x>                                   <!-- 0.65 mg-rms -> m/s^2 -->
      <y>
        <noise type="gaussian"><mean>0.0</mean><stddev>0.00637</stddev></noise>
      </y>
      <z>
        <noise type="gaussian"><mean>0.0</mean><stddev>0.00686</stddev></noise>
      </z>                                   <!-- 0.70 mg-rms -> m/s^2 -->
    </linear_acceleration>
  </imu>
</sensor>
```

**Key facts to internalize:**

- The noise block is `<noise type="gaussian"><mean>…</mean><stddev>…</stddev></noise>`.
- `mean` is the **bias** (0 = zero-mean, symmetric). `stddev` is the **spread** of the
  Gaussian. Set `stddev` to 0 to remove noise; increase it to make the sensor noisier.
- For **vector** sensors (IMU, magnetometer) noise is specified **per axis** — a
  separate `<noise>` block under each of `<x>`, `<y>`, `<z>`. For **scalar** sensors
  (air pressure, LiDAR range) the block sits directly under the value element:

  ```xml
  <air_pressure>
    <pressure>
      <noise type="gaussian"><mean>0</mean><stddev>3</stddev></noise>   <!-- 3 Pa RMS -->
    </pressure>
  </air_pressure>
  ```

- **Units matter.** `stddev` is in the sensor's output units. The IMU gyro is in
  rad/s and the accel in m/s², which is why the values look like `0.0008726646` and
  `0.00637` (0.05 °/s and 0.65 mg-rms converted to SI).

**Step-by-step to add or change Gaussian noise on a sensor:**

1. Open the model file that owns the sensor (for the UAV:
   [x500_base/model.sdf](../external/PX4-Autopilot/Tools/simulation/gz/models/x500_base/model.sdf)).
2. Find the `<sensor>` by its `name` and `type`.
3. Locate the field to corrupt (e.g. `<linear_acceleration>/<z>`).
4. Add or edit its `<noise type="gaussian">` block, setting `mean` and `stddev`.
5. Rebuild the PX4 target (`make px4_sitl gz_x500_mono_cam_down`) so the model
   changes take effect, then relaunch.
6. Verify the noise (§6.4).

### 6.2 Method 2 — Gaussian noise inside a ROS 2 node

Use this when the noise is part of the *fault* you are injecting (e.g. a drifting
bias, a multiplicative scale error, sensor dropout) or when you are synthesizing a
signal that never touches Gazebo. Use `numpy` (or C++ `<random>`) to add the noise
as you republish.

**Python example** (extend the §4.4 node):

```python
import numpy as np
from sensor_msgs.msg import Imu

# inside publish():
ideal_z = 9.81
noise = np.random.normal(loc=self.bias, scale=self.accel_stddev)  # bias is the mean
msg.linear_acceleration.z = ideal_z + noise
```

**C++ example:**

```cpp
#include <random>
#include <cmath>
// in the callback / timer:
std::normal_distribution<double> noise(mean_, stddev_);
double ideal = 9.81;
msg->linear_acceleration.z = ideal + noise(rng_);
// rng_ is a thread_local std::mt19937
```

**When to prefer this over Method 1:**
- You want a **multiplicative** fault (`ideal * (1 + noise)`) or a **time-varying**
  bias — Gazebo's `<noise>` only adds a zero-mean Gaussian.
- You are generating the sensor value from scratch (a scripted / synthetic signal).
- The fault should live in the fault-detection tooling, not in the physical model.

### 6.3 Method 3 — PX4 `sensor_simulator` (reference only)

The PX4 submodule ships a C++ sensor simulator at
[external/PX4-Autopilot/src/modules/ekf2/test/sensor_simulator](../external/PX4-Autopilot/src/modules/ekf2/test/sensor_simulator).
It replays/feeds the EKF2 estimator with simulated sensor streams. **It is not wired
into the running UAV/UGV demo** — the demo gets its sensor noise from the Gazebo
`<noise>` elements (Method 1). Reach for `sensor_simulator` only when you are
developing or testing the EKF2 in isolation, not when driving the live simulation.

### 6.4 Tuning and verifying the noise

**Set realistic values.** Match `stddev` to a real part:

| Sensor | Field | Typical `stddev` (SI) | Real part |
|---|---|---|---|
| IMU gyro | `angular_velocity.x/y/z` | `0.0008726646` | IIM42653, 0.05 °/s |
| IMU accel | `linear_acceleration.x/y` | `0.00637` | 0.65 mg-rms |
| IMU accel | `linear_acceleration.z` | `0.00686` | 0.70 mg-rms |
| Magnetometer | `x/y/z` | `0.0001` | IIS2MDC, 3 µT |
| Air pressure | `pressure` | `3` (Pa) | BMP390 |

**Confirm the noise is present** by collecting samples and checking the standard
deviation. One-liner over `ros2 topic echo`:

```bash
# Collect 1000 IMU samples, print accel_z, then compute std-dev in Python.
ros2 topic echo /fmu/out/imu --once 1000 > /tmp/imu.txt 2>/dev/null || true
python3 - <<'PY'
import re, statistics
zs = [float(m) for m in re.findall(r"linear_acceleration_z: ([-\d.eE]+)", open("/tmp/imu.txt").read())]
if zs:
    print(f"n={len(zs)}  mean={statistics.fmean(zs):.4f}  stdev={statistics.pstdev(zs):.5f}")
else:
    print("no samples captured — check that the topic is publishing")
PY
```

If `stdev` ≈ the `stddev` you set in the SDF, the noise is applied. If it is 0, the
noise element is missing, in the wrong units, or the model was not rebuilt.

---

## 7. Verification / How to Test

Run these while the simulation is up (`ros2 launch mission_manager simple_goal_demo.launch.py`
or `./scripts/start_sim.sh`).

**List and inspect topics**

```bash
ros2 topic list                          # see /fmu/out/... and /ugv_pose, /simple_ugv/odom, ...
ros2 topic info /fmu/out/vehicle_odometry
ros2 topic info /ugv_pose
ros2 topic info /fmu/out/obstacle_distance
```

**Confirm a sensor is publishing at the right rate**

```bash
ros2 topic hz /fmu/out/imu                 # ~250 Hz
ros2 topic hz /fmu/out/air_pressure        # ~50 Hz
ros2 topic hz /fmu/out/obstacle_distance   # LiDAR ~50 Hz (only with a lidar target)
ros2 topic hz /ugv_pose
```

**Inspect values**

```bash
ros2 topic echo /fmu/out/vehicle_odometry --once
ros2 topic echo /ugv_pose --once
ros2 topic echo /fmu/out/obstacle_distance --once
```

**Confirm distance** (after adding §5.1):

```bash
ros2 topic echo /uav_ugv/distance
```

**Confirm noise** — see the one-liner in §6.4.

**Gazebo-side topics** (if you are on the Gazebo DDS partition directly):

```bash
gz topic -l
gz topic -e /world/simple_goal_demo/model/x500_mono_cam_down_0/link/base_link/sensor/imu_sensor/imu
```

---

## 8. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Topic not appearing at all | DDS partition/IP mismatch, or the bridge/agent is not running. | Ensure `GZ_PARTITION=uav_ugv_sim` and `GZ_IP=127.0.0.1` are exported in **every** component; confirm `MicroXRCEAgent` is up; rebuild the model. |
| Sensor topic missing | The sensor's system plugin is not registered. | Add the `<plugin>` in §4.2 Step 3 (e.g. `gz-sim-sensors-system` with `<sensors>ray</sensors>`). |
| LiDAR/range not publishing | Wrong `make px4_sitl` target (e.g. `gz_x500` has no LiDAR). | Use `gz_x500_lidar_down` / `gz_x500_lidar_front`. |
| Distance looks wrong / sign flipped | NED vs ENU mismatch. | Invert the UAV's z (§5.1) so both poses are ENU. |
| Noise not visible | `stddev` too small vs signal, wrong units, or model not rebuilt. | Check units (§6.4), then `make px4_sitl <target>` and relaunch. |
| Camera topic name looks odd | The full Gazebo path includes world+model+link. | Use the exact `/world/…/link/camera_link/sensor/imager/image` path used by `traversability_mapper`. |
| World/bridge disagree | `PX4_GZ_WORLD` and the `WORLD` in `world_goal_demo.sh` differ. | Keep them identical (e.g. `simple_goal_demo`). |

---

*This document was written from the current state of the repository. When the
simulation is updated (new sensors, new models, changed topics), re-verify the topic
names and noise values against the files referenced above.*
