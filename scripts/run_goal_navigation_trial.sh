#!/usr/bin/env bash
# Run the goal demo and score physical arrival without sending route commands.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT="${1:-$ROOT/localization_results/goal_navigation}"
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
PX4="$ROOT/external/PX4-Autopilot"
source "$PX4/build/px4_sitl_default/rootfs/gz_env.sh"
export GZ_PARTITION=uav_ugv_sim GZ_IP=127.0.0.1
export GZ_SIM_RESOURCE_PATH="$ROOT/simulation/models:$GZ_SIM_RESOURCE_PATH"
export PX4_PARAM_UXRCE_DDS_DOM_ID="${ROS_DOMAIN_ID:-0}"
python3 "$ROOT/scripts/check_sim_dependencies.py"
mkdir -p "$OUTPUT"
pids=()
cleanup() {
  for pid in "${pids[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
  sleep 3
  for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  sleep 3
  # Rendering shutdown can stall; remove only this runner's process groups.
  for pid in "${pids[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM
setsid gz sim -s -r --render-engine-server "${RENDER_ENGINE:-ogre}" \
  "$ROOT/simulation/worlds/simple_goal_demo.sdf" >"$OUTPUT/world.log" 2>&1 & pids+=("$!")
sleep 3
setsid MicroXRCEAgent udp4 -p 8888 >"$OUTPUT/dds.log" 2>&1 & pids+=("$!")
setsid "$ROOT/scripts/px4_goal_demo.sh" >"$OUTPUT/px4.log" 2>&1 & pids+=("$!")
sleep 15
setsid ros2 launch mission_manager simple_goal_demo.launch.py \
  start_simulation:=false start_rviz:=false \
  initial_gps_enabled:="${INITIAL_GPS_ENABLED:-false}" \
  >"$OUTPUT/launch.log" 2>&1 & pids+=("$!")
setsid ros2 run cooperative_localization ugv_truth --ros-args \
  -p world:=simple_goal_demo -p use_sim_time:=true >"$OUTPUT/truth.log" 2>&1 & pids+=("$!")
python3 "$ROOT/scripts/score_goal_navigation.py" "$OUTPUT/results.json" \
  --timeout "${TRIAL_TIMEOUT:-300}"
"$PX4/build/px4_sitl_default/bin/px4-commander" land || true
