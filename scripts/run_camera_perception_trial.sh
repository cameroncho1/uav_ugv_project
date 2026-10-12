#!/usr/bin/env bash
# Test pixels and actual visual occlusion with the owned metric camera fixture.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT="${1:-$ROOT/localization_results/camera_fixture}"
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
source "$ROOT/external/PX4-Autopilot/build/px4_sitl_default/rootfs/gz_env.sh"
export GZ_PARTITION="${GZ_PARTITION:-uav_camera_fixture}" GZ_IP=127.0.0.1
export GZ_SIM_RESOURCE_PATH="$ROOT/simulation/models:$GZ_SIM_RESOURCE_PATH"
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
setsid gz sim -s -r --render-engine-server "${RENDER_ENGINE:-ogre}" \
  "$ROOT/simulation/worlds/fiducial_camera_test.sdf" >"$OUTPUT/world.log" 2>&1 & pids+=("$!")
sleep 5
setsid ros2 launch cooperative_localization uav_camera_sensor.launch.py \
  world:=fiducial_fixture uav_model:=observer >"$OUTPUT/sensor.log" 2>&1 & pids+=("$!")
setsid ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
  >"$OUTPUT/clock.log" 2>&1 & pids+=("$!")
python3 "$ROOT/scripts/test_camera_perception.py" "$OUTPUT/results.json"
