#!/usr/bin/env bash
# Run the goal demo and score physical arrival without sending route commands.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT="${1:-$ROOT/localization_results/goal_navigation}"
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
mkdir -p "$OUTPUT"
pids=()
cleanup() {
  stop_process_groups "${pids[@]}"
  pids=()
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
setsid gz sim -s -r --render-engine-server "${RENDER_ENGINE:-ogre}" \
  "$ROOT/simulation/worlds/simple_goal_demo.sdf" >"$OUTPUT/world.log" 2>&1 & pids+=("$!")
sleep 3
setsid MicroXRCEAgent udp4 -p 8888 >"$OUTPUT/dds.log" 2>&1 & pids+=("$!")
setsid "$ROOT/scripts/px4_goal_demo.sh" >"$OUTPUT/px4.log" 2>&1 & pids+=("$!")
sleep 15
setsid ros2 launch mission_manager simple_goal_demo.launch.py \
  start_simulation:=false start_rviz:=false \
  initial_gps_enabled:="${INITIAL_GPS_ENABLED:-false}" \
  uav_altitude:="${UAV_ALTITUDE:-4.5}" \
  seed:="${SEED:-513}" shared_position_stddev:="${SHARED_POSITION_STDDEV:-0.0}" \
  systematic_position_stddev:="${SYSTEMATIC_POSITION_STDDEV:-${SHARED_POSITION_STDDEV:-0.0}}" \
  >"$OUTPUT/launch.log" 2>&1 & pids+=("$!")
setsid ros2 run simulation_support ugv_truth --ros-args \
  -p world:=simple_goal_demo -p use_sim_time:=true >"$OUTPUT/truth.log" 2>&1 & pids+=("$!")
score_args=(--timeout "${TRIAL_TIMEOUT:-600}" --seed "${SEED:-513}"
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
