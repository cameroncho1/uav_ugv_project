#!/usr/bin/env bash
# Run the goal demo and score physical arrival without sending route commands.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT="${1:-$ROOT/localization_results/goal_navigation}"
if [[ -e "$OUTPUT/results.json" || -e "$OUTPUT/verification" ]]; then
  echo "Trial output already contains results or a recording; choose a new directory." >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
PX4="$ROOT/external/PX4-Autopilot"
source "$PX4/build/px4_sitl_default/rootfs/gz_env.sh"
export GZ_PARTITION="${GZ_PARTITION:-uav_ugv_sim}" GZ_IP=127.0.0.1
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROOT/ros2_ws/install/ugv_control/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$ROOT/simulation/models:$GZ_SIM_RESOURCE_PATH"
export PX4_PARAM_UXRCE_DDS_DOM_ID="${ROS_DOMAIN_ID:-0}"
python3 "$ROOT/scripts/check_sim_dependencies.py"
source "$ROOT/scripts/process_cleanup.sh"
ulimit -c 0
mkdir -p "$OUTPUT"
pids=()
WORLD="${WORLD_FILE:-$ROOT/simulation/worlds/simple_goal_demo.sdf}"
export WORLD_FILE="$WORLD" GOAL_X="${GOAL_X:-4.0}" GOAL_Y="${GOAL_Y:-0.0}"
export UAV_ASSISTANCE="${UAV_ASSISTANCE:-true}"
record_pid=""
recorder_status=0
cleanup() {
  if [[ -n "$record_pid" ]]; then
    kill -INT -- "-$record_pid" 2>/dev/null || true
    wait "$record_pid" || recorder_status=$?
    record_pid=""
  fi
  stop_process_groups "${pids[@]}"
  pids=()
  return "$recorder_status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
setsid gz sim -s -r --render-engine-server "${RENDER_ENGINE:-ogre}" \
  "$WORLD" >"$OUTPUT/world.log" 2>&1 & pids+=("$!")
deadline=$((SECONDS + 30))
until [[ "$(timeout 3s gz service -s /world/simple_goal_demo/control \
  --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 500 \
  --req 'pause: false' 2>/dev/null || true)" == *'data: true'* ]]; do
  kill -0 "${pids[0]}" || exit 2
  (( SECONDS < deadline )) || { echo 'World readiness timed out' >&2; exit 2; }
  sleep .25
done
setsid MicroXRCEAgent udp4 -p 8888 >"$OUTPUT/dds.log" 2>&1 & pids+=("$!")
setsid "$ROOT/scripts/px4_goal_demo.sh" >"$OUTPUT/px4.log" 2>&1 & pids+=("$!")
# Let the native XRCE session establish its writers before starting the ROS graph.
# This is a transport barrier; actual ROS observations are checked below.
printf '%s\n' '{"ready":false,"stage":"px4_entities"}' >"$OUTPUT/readiness.json"
px4_pid="${pids[-1]}"
deadline=$((SECONDS + 45))
until grep -Fq 'successfully created rt/fmu/out/vehicle_odometry' "$OUTPUT/px4.log"; do
  kill -0 "$px4_pid" || { echo 'PX4 exited before entity readiness' >&2; exit 2; }
  (( SECONDS < deadline )) || { echo 'PX4 entity readiness timed out' >&2; exit 2; }
  sleep .25
done
setsid python3 "$ROOT/scripts/verification_recording.py" "$OUTPUT/verification" --world "$WORLD" \
  --setup "${RECORDING_SETUP:-camera_navigation}" --result "$OUTPUT/results.json" &
record_pid=$!
if [[ -n "${OBSTACLE_SCENARIO:-}" ]]; then
  mkdir -p "$OUTPUT/hazards"
  observer_args=(--ros-args -p use_sim_time:=true -p scenario_file:="$OBSTACLE_SCENARIO"
                 -p output_directory:="$OUTPUT/hazards")
  if [[ -n "${HAZARD_OBSERVER_PARAMS:-}" ]]; then
    observer_args+=(--params-file "$HAZARD_OBSERVER_PARAMS")
  fi
  setsid ros2 run simulation_support obstacle_scenario --ros-args \
    -p use_sim_time:=true -p scenario_file:="$OBSTACLE_SCENARIO" \
    -p output_directory:="$OUTPUT/hazards" >"$OUTPUT/hazards/scenario.log" 2>&1 & pids+=("$!")
  setsid ros2 run fault_management hazard_observer.py "${observer_args[@]}" \
    >"$OUTPUT/hazards/observer.log" 2>&1 & pids+=("$!")
fi
setsid ros2 launch mission_manager simple_goal_demo.launch.py \
  start_simulation:=false start_rviz:=false \
  world_file:="$WORLD" evaluation_trace:=true goal_x:="$GOAL_X" goal_y:="$GOAL_Y" \
  uav_assistance:="$UAV_ASSISTANCE" \
  initial_gps_enabled:="${INITIAL_GPS_ENABLED:-false}" \
  uav_altitude:="${UAV_ALTITUDE:-4.5}" \
  uav_heading:="${UAV_HEADING:-true}" \
  seed:="${SEED:-513}" shared_position_stddev:="${SHARED_POSITION_STDDEV:-0.0}" \
  systematic_position_stddev:="${SYSTEMATIC_POSITION_STDDEV:-${SHARED_POSITION_STDDEV:-0.0}}" \
  >"$OUTPUT/launch.log" 2>&1 & pids+=("$!")
setsid ros2 run simulation_support ugv_truth --ros-args \
  -p world:=simple_goal_demo -p use_sim_time:=true -p evaluation_trace:=true >"$OUTPUT/truth.log" 2>&1 & pids+=("$!")
python3 "$ROOT/scripts/verification_readiness.py" --timeout 60 \
  /clock=rosgraph_msgs/msg/Clock /evaluation/control=std_msgs/msg/String \
  /evaluation/ugv_truth=geometry_msgs/msg/PoseStamped \
  /fmu/out/vehicle_odometry=px4_msgs/msg/VehicleOdometry >"$OUTPUT/readiness.json"
kill -0 "$record_pid" || { echo 'Recorder exited before readiness' >&2; exit 2; }
score_args=(--timeout "${TRIAL_TIMEOUT:-600}" --seed "${SEED:-513}"
  --wall-timeout "${WALL_TIMEOUT:-900}" --goal-x "$GOAL_X" --goal-y "$GOAL_Y" --uav-assistance "$UAV_ASSISTANCE"
  --gps-outage-after-motion "${GPS_OUTAGE_AFTER_MOTION:--1}"
  --uav-heading "${UAV_HEADING:-true}"
  --uav-altitude "${UAV_ALTITUDE:-4.5}"
  --shared-position-stddev "${SHARED_POSITION_STDDEV:-0.0}"
  --systematic-position-stddev "${SYSTEMATIC_POSITION_STDDEV:-${SHARED_POSITION_STDDEV:-0.0}}")
if [[ "${INITIAL_GPS_ENABLED:-false}" == true ]]; then score_args+=(--initial-gps-enabled); fi
if [[ "${EXPECT_HOLD:-false}" == true ]]; then score_args+=(--expect-hold); fi
setsid python3 "$ROOT/scripts/score_goal_navigation.py" "$OUTPUT/results.json" "${score_args[@]}" &
score_pid=$!
pids+=("$score_pid")
wait "$score_pid"
"$PX4/build/px4_sitl_default/bin/px4-commander" land || true
