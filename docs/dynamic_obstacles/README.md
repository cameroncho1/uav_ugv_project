# Dynamic obstacle scenarios

[Documentation index](../README.md) | [Navigation](../navigation.md) | [Requirements](../verification/requirements.md) | [Completion plan](../project/completion_plan.md)

A scheduled box appears after the UGV begins driving. The UAV camera supplies the map update, the existing planner computes a new route, and the UGV follows it. Scenario code changes the simulated world; it does not publish maps, routes, poses, or vehicle commands.

[Measured example and reproducible figures](../project/results/dynamic_obstacles/README.md).

The first example uses one static dark box in the small, flat camera-demo world. The infrastructure supports multiple boxes, arbitrary upright box poses and dimensions, and ordered spawn/remove operations. Moving obstacles, meshes, terrain, and automatic route-relative placement are outside this version.

## Components

| Component | Function |
| --- | --- |
| [Scenario definitions and schedule](../../ros2_ws/src/simulation_support/simulation_support/obstacle_scenarios.py) | Strict YAML validation, box/SDF definitions, simulation-time scheduling, exactly-once dispatch, and footprint sampling; no ROS/Gazebo imports |
| [Gazebo adapter](../../ros2_ws/src/simulation_support/simulation_support/obstacle_world.py) | Bounded creation/removal requests and scene name/pose confirmation |
| [Scenario node](../../ros2_ws/src/simulation_support/scripts/obstacle_scenario) | First-drive trigger, serial mutations, durable operation log, and acquisition-stamped lifecycle/geometry snapshots |
| [Hazard assessment](../../ros2_ws/src/fault_management/scripts/hazard_assessment.py) | Numerical criteria for a qualifying observed hazard; no ROS/Gazebo imports |
| [Observer node](../../ros2_ws/src/fault_management/scripts/hazard_observer.py) | Associates actual camera-grid updates with applied obstacles, the prior route, and independent truth for scoring |
| [Trial command](../../scripts/run_dynamic_obstacle_trial.py) | Fresh outputs, source/input hashes, bounded headless execution, profile binding, and R1/R2/R5/R6 evaluation |
| [Saved-evidence report](../../scripts/report_dynamic_obstacle_trial.py) | Timeline CSV and measured trajectory/route figures; replay needs no ROS |

The existing mapper, planner, supervisor, and path follower retain their measurement interfaces. The planner already recomputes routes from fresh camera grids. Truth and scenario geometry remain in simulation/evaluation components.

## Scenario format

The [single-box example](../../ros2_ws/src/simulation_support/config/scenarios/single_box.yaml) contains:

```yaml
schema_version: 1
id: single_box_v1
world: simple_goal_demo
frame: map
trigger: first_drive_command
obstacles:
  - id: route_box
    shape: box
    size: [0.6, 0.6, 0.4]
    pose: [0.5, 0.0, 0.2, 0.0]
operations:
  - id: add_route_box
    at_s: 0.5
    action: spawn
    obstacle: route_box
```

- `size` is x/y/z extent in meters; `pose` is ENU map x/y/z/yaw, with yaw in radians. Boxes rest on the z=0 ground plane and have matching collision/visual extents.
- `at_s` is elapsed simulation time from the first observed nonzero controller command, preserving that command's acquisition time. It is not wall time or a claim of measured physical motion.
- Obstacle and operation IDs are unique. Operations are ordered by time; equal-time operations execute serially and record their actual request/confirmation times.
- `spawn` requires an absent obstacle, and `remove` requires an active obstacle. A removed box can appear again through another uniquely named operation.
- Unknown fields, duplicate keys, unsupported shapes/frames, nonfinite values, invalid dimensions, and invalid lifecycles fail before world mutation.
- Gazebo names use `scenario_` plus the obstacle ID. Existing entities are never replaced or silently renamed. A timed-out mutation is not retried because its effect may already exist.

The scenario runs once per world. New navigation goals do not restart its schedule. Paused simulation time cannot advance scheduled events; clock rewind aborts the scenario rather than replaying mutations. Configuration parameters are read-only after startup.

## Observation and evaluation

The event sequence distinguishes three facts:

1. **Requested:** a mutation is submitted to Gazebo.
2. **Confirmed:** the named model appears at its requested pose, or disappears after removal. Static/collision geometry comes from the submitted SDF; scene confirmation checks entity identity and pose.
3. **Observed:** a later acquisition-stamped camera map contains at least three occupied cell centers inside the box's ground footprint.

A qualifying R5 trigger additionally requires a fresh, previously free baseline map; a fresh preappearance route intersecting the obstacle's inflated footprint; the same active goal revision; and fresh independent truth showing sufficient distance. The distance test uses a conservative request-to-observation latency bound and accounts for motion since the previous truth sample. Unknown, stale, off-route, or too-close observations remain unqualified and cannot satisfy the example's acceptance check.

R5 measures a fresh clear route or a sustained zero-command response within its declared bound, starting at grid acquisition time. It does not equate zero commands with physical braking. R1 scores sampled disk clearance, R2 independently scores physical/estimated arrival and quiet dwell, and R6 scores localization against acquisition-matched truth.

`/simulation/obstacle_scenario` provides native-sequenced geometry snapshots. Scoring conservatively includes a box from the creation request until confirmed removal. The adapter merges these snapshots with static world collision geometry, preserves their timestamps and sequence gaps, and expires missing updates. When this provider is declared but missing, import does not fall back to an empty/static world and report successful clearance.

The [dynamic setup](../../verification/setups/camera_dynamic_obstacles.yaml) and [example profile](../../verification/profiles/dynamic_obstacles.yaml) are explicit. The profile is provisional: its bounds are declared assumptions for the example, not calibration for arbitrary worlds or loads. Observer speed/radius/clearance come from the navigation configuration, and its response bound comes from the selected verification profile. Footprint-radius disagreement fails before launch.

## Run and reproduce

Linux: source ROS 2 Humble and the project's existing dependencies, install the [numerical verification package](../../verification/README.md), then build:

The scenario adapter also requires the OSRF Jammy packages `python3-gz-transport13` and `python3-gz-msgs10`. The [project rosdep rules](../../simulation/rosdep.yaml) include both keys; refresh the registered rules with the [simulation setup procedure](../simulation_setup.md) before dependency installation on a new machine.

```bash
cd ros2_ws
colcon build --parallel-workers 2 --packages-select simulation_support fault_management mission_manager
source install/setup.bash
colcon test --packages-select simulation_support fault_management
colcon test-result --verbose
cd ..
python scripts/test_obstacle_lifecycle.py verification/outputs/obstacle-lifecycle
python scripts/run_dynamic_obstacle_trial.py verification/outputs/obstacle-example
python scripts/report_dynamic_obstacle_trial.py verification/outputs/obstacle-example \
  verification/outputs/obstacle-report
```

Use new output directories. The trial uses the existing server-only camera launcher, coordinates access to PX4's port, isolates ROS/Gazebo namespaces, and stops owned processes. It records raw bags, normalized traces, input/source hashes, operation logs, observation evidence, and evaluated results. An unobserved/unqualified scheduled spawn, incomplete scenario, missing evidence, or failed requirement prevents success.

The report export contains `evidence.json`, `timeline.csv`, and PNG/SVG/PDF figures. Reproduce its tables/figures without ROS or raw bags:

```bash
python scripts/report_dynamic_obstacle_trial.py SAVED_REPORT NEW_REPORT
```

Saved plots may subsample truth for display; full normalized traces determine requirement verdicts. Source hashes accompany the export. Raw recordings remain separate from compact committed examples.

## Topic contracts

- `/simulation/obstacle_scenario` is versioned lifecycle telemetry: native sequence, simulation stamp, pending operation, active/possible geometry, confirmed active operations, completion, and error status. Geometry is an evaluation input.
- `/evaluation/hazards` contains normalized `hazard_observed` events keyed by operation ID and goal revision. Event time is camera-grid acquisition time; qualification details remain in the payload.
- Standard ROS remapping can bind these nodes to another navigation graph. Start both nodes with `use_sim_time:=true`; pass the same `scenario_file` and a fresh `output_directory`. The observer's input topics can be remapped independently of the Gazebo adapter.

## Extend the example

- **More obstacles:** add definitions and ordered operations to YAML; the same scheduler, adapter, observer, and monitors consume them.
- **Larger maps or curved paths:** keep ENU map coordinates and acquisition stamps, supply the appropriate camera/navigation setup, and choose a separate profile and scenario with enough observation distance. Eligibility checks inspect route segments rather than assuming a straight route.
- **Other worlds:** the scenario node addresses the YAML world name. Launch it alongside the world's existing navigation graph. The bundled camera trial uses `simple_goal_demo` and its world variants because that is the existing PX4/camera launch contract.
- **Other shapes:** extend the validated shape definition, SDF generation, independent collision geometry, and map-footprint sampling together. Unsupported shapes fail explicitly.
- **Moving obstacles:** add timestamped pose/geometry updates and motion-aware evaluation. Static spawn/remove snapshots do not represent motion between events.
- **Other sensors or simulators:** retain the normalized geometry/event contracts and implement the relevant mutation and observation adapters. Evaluation thresholds and capabilities remain setup-specific.

Completion-plan W2/W3 coverage begins with this one-box example. Four-layout coverage, hazard-placement sweeps, ten-run response characterization, and larger scenario matrices remain separate work. The measured example supports a course report's architecture, methods, and results sections without claiming general obstacle avoidance.

## API references

- [Gazebo Harmonic entity creation](https://gazebosim.org/api/sim/8/entity_creation.html): UserCommands creation/removal services.
- [Gazebo Transport 13 Python support](https://gazebosim.org/api/transport/13/python.html): typed service requests through the Python transport adapter.
