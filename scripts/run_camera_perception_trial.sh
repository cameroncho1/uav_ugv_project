#!/usr/bin/env bash
# Test pixels and actual visual occlusion with the owned metric camera fixture.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT="${1:-$ROOT/localization_results/camera_fixture}"
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
source "$ROOT/external/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh"
export GZ_PARTITION="${GZ_PARTITION:-uav_camera_fixture}" GZ_IP=127.0.0.1
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROOT/ros2_ws/install/ugv_control/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$ROOT/simulation/models:$GZ_SIM_RESOURCE_PATH"
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
  "$ROOT/simulation/worlds/fiducial_camera_test.sdf" >"$OUTPUT/world.log" 2>&1 & pids+=("$!")
sleep 5
setsid ros2 launch uav_sensing uav_camera_sensor.launch.py \
  world:=fiducial_fixture uav_model:=observer >"$OUTPUT/sensor.log" 2>&1 & pids+=("$!")
setsid ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
  >"$OUTPUT/clock.log" 2>&1 & pids+=("$!")
setsid python3 "$ROOT/scripts/test_camera_perception.py" "$OUTPUT/results.json" &
score_pid=$!
pids+=("$score_pid")
wait "$score_pid"
