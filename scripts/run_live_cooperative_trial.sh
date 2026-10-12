#!/usr/bin/env bash
# Compare localization while a PX4 UAV flies; select simulated or PX4 anchors.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT="${1:-$ROOT/localization_results/live_px4}"
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u
export GZ_PARTITION="${GZ_PARTITION:-uav_ugv_sim}" GZ_IP=127.0.0.1
export PX4_DDS_SYNCT=0
export PX4_PARAM_UXRCE_DDS_DOM_ID="${ROS_DOMAIN_ID:-0}"
PX4_BIN="$ROOT/external/PX4-Autopilot/build/px4_sitl_default/bin"
relative_mode="${RELATIVE_MODE:-camera}"
case "$relative_mode" in
  simulated) world=uav_ugv; uav_model=x500_0; px4_script=px4.sh ;;
  camera) world=simple_goal_demo; uav_model=cooperative_camera_uav_0; px4_script=px4_goal_demo.sh ;;
  *) echo "RELATIVE_MODE must be simulated or camera" >&2; exit 1 ;;
esac
uav_altitude="${UAV_ALTITUDE:-4.0}"
if [[ "$relative_mode" == camera ]]; then uav_altitude="${UAV_ALTITUDE:-4.5}"; fi
if [[ "$relative_mode" == camera && "${ANCHOR_MODE:-px4}" != px4 ]]; then
  echo "Camera trials require ANCHOR_MODE=px4" >&2
  exit 1
fi
case "${ANCHOR_MODE:-px4}" in
  simulated) publish_anchor=true; odometry_topic=/simulation/uav_odometry; airborne=false; settle=0.0 ;;
  px4) publish_anchor=false; odometry_topic=/fmu/out/vehicle_odometry; airborne=true; settle=10.0 ;;
  *) echo "ANCHOR_MODE must be simulated or px4" >&2; exit 1 ;;
esac
gps_case="${GPS_CASE:-startup_off}"
case "$gps_case" in
  on|off|startup_off) ;;
  *) echo "GPS_CASE must be on, off, or startup_off" >&2; exit 1 ;;
esac
initial_gps_enabled=true
if [[ "$gps_case" == startup_off ]]; then initial_gps_enabled=false; fi
python3 "$ROOT/scripts/check_sim_dependencies.py"
mkdir -p "$OUTPUT"
pids=()
cleanup() {
  for pid in "${pids[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
  sleep 3
  for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  sleep 3
  for pid in "${pids[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM

if [[ "$relative_mode" == camera ]]; then
  # Upstream setup reads optional environment variables without defaults.
  set +u
  source "$ROOT/external/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh"
  set -u
  export GZ_SIM_RESOURCE_PATH="$ROOT/simulation/models:$GZ_SIM_RESOURCE_PATH"
  setsid gz sim -s -r --render-engine-server "${RENDER_ENGINE:-ogre}" \
    "$ROOT/simulation/worlds/simple_goal_demo.sdf" >"$OUTPUT/world.log" 2>&1 & pids+=("$!")
else
  setsid "$ROOT/scripts/world.sh" >"$OUTPUT/world.log" 2>&1 & pids+=("$!")
fi
sleep 3
setsid MicroXRCEAgent udp4 -p 8888 >"$OUTPUT/dds.log" 2>&1 & pids+=("$!")
# PX4's interactive shell needs a terminal, including when its log is redirected.
printf -v px4_command '%q' "$ROOT/scripts/$px4_script"
setsid script -q -c "$px4_command" /dev/null >"$OUTPUT/px4.log" 2>&1 & pids+=("$!")
sleep 20
setsid ros2 launch cooperative_localization ugv_cooperative_evaluation.launch.py \
  seed:="${SEED:-513}" initial_gps_enabled:="$initial_gps_enabled" \
  world:="$world" relative_source:="$relative_mode" \
  uav_model:="$uav_model" publish_anchor:="$publish_anchor" \
  uav_odometry_topic:="$odometry_topic" \
  uav_require_airborne:="$airborne" uav_initial_settle_time:="$settle" \
  >"$OUTPUT/launch.log" 2>&1 & pids+=("$!")
sleep 4
setsid ros2 run uav_control offboard_goal_controller --ros-args -p use_sim_time:=true \
  -p uav_altitude:="$uav_altitude" \
  >"$OUTPUT/control.log" 2>&1 & pids+=("$!")
timeout "${ANCHOR_TIMEOUT:-240}" ros2 topic echo --no-daemon --once /uav/localization/pose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  >"$OUTPUT/anchor.txt"
"$PX4_BIN/px4-param" show UXRCE_DDS_SYNCT >"$OUTPUT/time_parameter.txt"
timeout "${TRIAL_TIMEOUT:-300}" ros2 run cooperative_localization localization_trial.py \
  --gps "$gps_case" --seed "${SEED:-513}" \
  --anchor-source "${ANCHOR_MODE:-px4}" --observer-model "$uav_model" \
  --relative-source "$relative_mode" \
  --uav-altitude "$uav_altitude" \
  --comparison-topic /evaluation/baseline_pose --output "$OUTPUT/trial_live"
"$PX4_BIN/px4-commander" land || true
sleep 5
