#!/usr/bin/env bash
# The ML chapter of docs/demo.md: straight drive, live BiLSTM, persistence and supervisor recovery.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
exec ros2 launch mission_manager simple_goal_demo.launch.py \
  world_file:="$ROOT/simulation/worlds/simple_goal_demo.sdf" \
  goal_x:=4.0 goal_y:=0.0 evaluation_trace:=true \
  fault_detection:=true fault_hold:=true dashboard:=true dashboard_view:=ml \
  uav_dropout_control:=true start_rviz:=false "$@"
