#!/usr/bin/env bash
# One-command presentation demo: the 13 m "complex" course, a hazard box that appears
# mid-run, the live sensor-fault detector, and a dashboard with live sensor controls.
# Add fault_hold:=true to let detector decisions hold the UGV (the learned detector raises
# brief false alarms in nominal runs, which stops the UGV).
# Script and talking points: docs/demo.md. Run from an Ubuntu desktop session.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
echo "Dashboard: http://localhost:8765"
exec ros2 launch mission_manager simple_goal_demo.launch.py \
  world_file:="$ROOT/simulation/worlds/simple_goal_demo_complex.sdf" \
  goal_x:=9.0 goal_y:=3.0 \
  evaluation_trace:=true \
  obstacle_scenario:="$ROOT/ros2_ws/src/simulation_support/config/scenarios/complex_course_box.yaml" \
  fault_detection:=true dashboard:=true start_rviz:=false \
  "$@"
