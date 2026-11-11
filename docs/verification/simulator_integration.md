# Simulator recording and verification

ROS 2 Humble and Gazebo Harmonic run in Linux. The numerical evaluator imports without ROS; `import-bag` loads ROS serialization only when called. Current storage support is Humble `sqlite3`, including split bags.

## Build and test

From the repository root, after building the existing simulation dependencies:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
python3 -m venv --system-site-packages verification/.venv
source verification/.venv/bin/activate
python -m pip install --upgrade pip setuptools wheel
python -m pip install -r verification/requirements-tested.txt
python -m pip install --no-build-isolation -e './verification[test]'
cd ros2_ws
colcon build --packages-select uav_ugv_msgs ugv_control simulation_support \
  fault_management mission_manager --symlink-install \
  --cmake-args -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python" \
  -DPYTHON_EXECUTABLE="$VIRTUAL_ENV/bin/python"
source install/setup.bash
colcon test --packages-select uav_ugv_msgs ugv_control simulation_support \
  fault_management mission_manager
colcon test-result --all
cd ..
```

Install `python3.10-venv` if `venv` reports missing `ensurepip`. Use pytest 8.4.2: Humble's installed `launch_testing` pytest plugin is incompatible with pytest 9. Required Python and real CDR bag tests are registered through `fault_management`; they do not skip when ROS imports are missing.

## Recording

```bash
INITIAL_GPS_ENABLED=true GOAL_X=3.0 GOAL_Y=0.5 \
  bash scripts/run_goal_navigation_trial.sh verification/outputs/camera

INITIAL_GPS_ENABLED=true GPS_OUTAGE_AFTER_MOTION=3 \
  bash scripts/run_goal_navigation_trial.sh verification/outputs/outage

python scripts/test_navigation_guards.py verification/outputs/guards/result.json
python scripts/test_actuator_watchdog.py verification/outputs/watchdog
python scripts/test_actuator_watchdog.py verification/outputs/slow --clock-mode slow
python scripts/test_actuator_watchdog.py verification/outputs/frozen --clock-mode frozen
```

Use a new output directory for each attempt. Run full PX4 trials serially; ROS/Gazebo domains do not isolate XRCE port 8888. `WORLD_FILE` selects an absolute SDF path; current camera variants retain internal world name `simple_goal_demo`. `GOAL_X`, `GOAL_Y`, and `UAV_ASSISTANCE` feed launch, scoring, and recording. `TRIAL_TIMEOUT` is a simulation-time deadline; `WALL_TIMEOUT` is an independent wall-time watchdog.

The runner records numerical topics automatically. Manual launches enable `evaluation_trace:=true`; the truth publisher also needs `-p evaluation_trace:=true`. The watchdog diagnostic bridge runs separately from command transport. The recorder uses sensor-compatible best-effort subscriptions and transient-local goal, route, grid, and applied-fault subscriptions. Camera images are excluded.

Each recording directory contains the bag, driver events, QoS configuration, logs, source hashes, supplied run settings, dependency declaration, terminal status, and normalized numerical data. Recorder failure prevents completed status. Camera normalization starts the mission at the first control tick that publishes a nonzero command. Startup records remain in the files and source checksums. Controlled fixtures evaluate their entire recorded interval.

## Import and evaluate

The runner already normalizes its bag. To import another completed camera recording:

```bash
python -m uav_ugv_verification import-bag PATH_TO_BAG \
  --setup verification/setups/camera_navigation.yaml \
  --world "$PWD/simulation/worlds/simple_goal_demo.sdf" \
  --status completed --output verification/outputs/imported
python -m uav_ugv_verification bind-profile verification/outputs/imported \
  --template verification/profiles/m2_camera.yaml \
  --output verification/outputs/camera_profile.yaml
python -m uav_ugv_verification evaluate verification/outputs/imported \
  --profile verification/outputs/camera_profile.yaml \
  --output verification/outputs/camera_result
```

For controlled recordings, select `controlled_navigation.yaml` and `m2_guard.yaml` or `m2_watchdog.yaml`. Include the driver's `--events PATH_TO_EVENTS_JSONL` for R4. `bind-profile` replaces only `steady:recording` with the single recorded boot clock; bounds remain unchanged. It refuses missing or ambiguous steady clocks. Explicit `--start-ns`/`--end-ns` override the simulation mission interval and remain in provenance.

Storage message IDs preserve receipt order across `/clock` resets. Acquisition sorting uses a temporary on-disk SQLite table. Unselected image blobs are neither loaded nor deserialized. Native telemetry counters retain gaps; ordinary header messages receive canonical sequences after acquisition sorting. Merged events retain producer counters and expose missing producer emissions. Imports refuse overwrite, incompatible message types, unsupported storage, and mixed Linux boot clocks. Source bag, setup, world, event, recorder, and adapter hashes identify the transformation. Re-importing one bag preserves its source-run identity. Source snapshot hashes separate simulator code cohorts even when a runtime mirror has no Git metadata. Missing first-drive evidence retains the full interval and records the missing boundary; historical data cannot manufacture consumer telemetry.

## Runtime signals

| Topic | Recorded observation | Normalized stream |
| --- | --- | --- |
| `/evaluation/control` | Actual guard validity/reason, acquisition stamp, consumer receipt, control tick, command publication time | `guard_state`, `requested_command` |
| `/evaluation/watchdog` | Accepted/rejected actuator input, associated output, expiry, publication success | `actuator_input`, `actuator_output` |
| `/evaluation/truth` | Independent Gazebo position differences, acquisition/receipt stamps, advancing simulation, sampling uncertainty | `measured_motion` |
| `/evaluation/faults` | Initial GPS state and applied callback transitions | `events` |
| `/evaluation/ugv_truth`, pose, goal, route, grid, relative observation | Acquisition-stamped numerical messages | Corresponding canonical streams |
| `/evaluation/hazards`, `/evaluation/detector` | Optional canonical JSON envelopes in `std_msgs/String` | `events`, `detector_decision` |

Steady clocks use `steady:<Linux boot ID>`. Controller evaluation, ROS publication, actuator receipt, and actuator output remain separate timestamps. A bag storage timestamp never substitutes for them. Earlier telemetry without command publication time has invalid steady-time command evidence; acquisition-time data remains available.

The watchdog publishes diagnostics after releasing its command-state lock. Diagnostics use an independent bridge, so command-transport loss remains observable. `queued=true` means Gazebo transport accepted publication; physical truth establishes settling separately. Motion is an interval-average planar speed/yaw rate. Sampling uncertainty can make a deadline inconclusive. Frozen simulation cannot establish physical settling even when repeated truth messages show unchanged positions.

## Setup extension

- Add a setup YAML mapping topics to supported message semantics. Different wire formats require a translator in the adapter; R1–R8 consume the same canonical contract.
- Supply independent collision geometry and the shared map/reference-point transform. The SDF extractor composes model/link/collision planar poses for static upright boxes and cylinders. Explicitly exclude ground and the evaluated vehicle. Unknown obstacle includes, moving collisions, relative frames, tilted shapes, and unsupported geometry are rejected.
- Supply consumer telemetry for timing claims. Historical unstamped commands cannot establish receipt or watchdog expiry. Missing topics remain visible in provenance and dependent checks remain inconclusive.
- Supply actual hazard/detector events when those producers exist. Canonical JSON has `schema_version: 1`, `stream`, and the [normalized record fields](interfaces.md); timestamps describe actual observation/emission. Labels and planned schedules cannot manufacture decisions.

The [M2 evidence index](../project/results/verification/m2/README.md) records calibration, measured conditions, unsuccessful pilots, and unavailable capabilities. M3 uses the frozen profiles for new experiment runs; these pilots are calibration data.
