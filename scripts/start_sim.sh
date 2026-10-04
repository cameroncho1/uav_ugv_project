#!/usr/bin/env bash
#
# start_sim.sh - open every terminal needed for the UAV-UGV simulation.
#
# Usage:
#   ./start_sim.sh            start everything, including the 3D Gazebo window
#   ./start_sim.sh --no-gui   start everything except the 3D window
#   ./start_sim.sh stop       shut down every simulation process
#
# Run this from a terminal on the Ubuntu desktop (not over SSH), because it
# opens graphical terminal tabs and the Gazebo window.

set -u

# ---------------------------------------------------------------- settings
PROJECT="${PROJECT:-$HOME/uav_ugv_project}"
WORLD="${WORLD:-uav_ugv}"
export GZ_PARTITION="${GZ_PARTITION:-uav_ugv_sim}"
export GZ_IP="${GZ_IP:-127.0.0.1}"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$PROJECT/ros2_ws/install/setup.bash"

# ---------------------------------------------------------------- stop mode
stop_all() {
  echo "Stopping simulation processes..."
  pkill -INT -x px4 2>/dev/null
  pkill -INT -f MicroXRCEAgent 2>/dev/null
  sleep 1
  pkill -INT -f "gz sim" 2>/dev/null
  sleep 2
  # Anything that ignored the polite request gets a firm one.
  pkill -KILL -x px4 2>/dev/null
  pkill -KILL -f MicroXRCEAgent 2>/dev/null
  pkill -KILL -f "gz sim" 2>/dev/null
  echo "Done. The terminal tabs stay open so you can read their output; close them by hand."
}

if [[ "${1:-}" == "stop" ]]; then
  stop_all
  exit 0
fi

WITH_GUI=1
[[ "${1:-}" == "--no-gui" ]] && WITH_GUI=0

# ---------------------------------------------------------------- sanity checks
if ! command -v gnome-terminal >/dev/null; then
  echo "Error: gnome-terminal not found. Run this on the Ubuntu desktop." >&2
  exit 1
fi
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "Error: no graphical display. Run this from the desktop Terminal app, not over SSH." >&2
  exit 1
fi
if [[ ! -d "$PROJECT/scripts" ]]; then
  echo "Error: $PROJECT/scripts not found. Set PROJECT=/path/to/uav_ugv_project." >&2
  exit 1
fi
if pgrep -x px4 >/dev/null || pgrep -f "gz sim" >/dev/null; then
  echo "A simulation already seems to be running."
  echo "Run '$0 stop' first, then start again."
  exit 1
fi

chmod +x "$PROJECT"/scripts/*.sh 2>/dev/null

# ---------------------------------------------------------------- helpers
# Open a new terminal tab that runs a command and then stays open as a
# normal shell, so error messages remain readable after a crash or Ctrl+C.
open_tab() {
  local title="$1" cmd="$2"
  gnome-terminal --tab --title="$title" -- bash -c "
    export GZ_PARTITION='$GZ_PARTITION' GZ_IP='$GZ_IP'
    cd '$PROJECT'
    $cmd
    echo
    echo '[$title has exited. This tab is now a normal shell.]'
    exec bash"
}

# Wait until a pattern shows up in the Gazebo topic list.
wait_for_topic() {
  local pattern="$1" timeout="$2" label="$3"
  printf "Waiting for %s" "$label"
  for ((i = 0; i < timeout; i++)); do
    if gz topic -l 2>/dev/null | grep -q "$pattern"; then
      echo " ready."
      return 0
    fi
    printf "."
    sleep 1
  done
  echo " not seen after ${timeout}s, continuing anyway."
  return 1
}

# ---------------------------------------------------------------- launch
echo "Starting UAV-UGV simulation (partition: $GZ_PARTITION)"

# 1. Gazebo physics server
open_tab "1 Gazebo world" "./scripts/world.sh"
wait_for_topic "/world/$WORLD/" 60 "Gazebo world"

# 2. PX4 autopilot. Stays in the foreground so the pxh> prompt is usable.
open_tab "2 PX4" "./scripts/px4.sh"
wait_for_topic "x500" 90 "PX4 to spawn the drone"

# 3. DDS bridge between PX4 and ROS 2
open_tab "3 DDS agent" "./scripts/dds.sh"

# 4. ROS 2 shell with the workspace already sourced
ROS_RC="$(mktemp /tmp/uavugv_rosrc.XXXXXX)"
cat > "$ROS_RC" <<EOF
[ -f ~/.bashrc ] && source ~/.bashrc
source "$ROS_SETUP"
[ -f "$WS_SETUP" ] && source "$WS_SETUP" || echo "Workspace not built yet: run colcon build in ros2_ws"
export GZ_PARTITION="$GZ_PARTITION" GZ_IP="$GZ_IP"
cd "$PROJECT/ros2_ws"
echo "ROS 2 shell ready. Try:"
echo "  ros2 topic list | grep /fmu"
echo "  ros2 launch mission_manager simple_goal_demo.launch.py start_simulation:=false"
EOF
gnome-terminal --tab --title="4 ROS 2" -- bash --rcfile "$ROS_RC"

# 5. Gazebo 3D window (optional)
if [[ $WITH_GUI -eq 1 ]]; then
  sleep 3
  open_tab "5 Gazebo GUI" "./scripts/gui.sh"
fi

echo
echo "All terminals launched."
echo "  Fly:   in the PX4 tab type  commander takeoff"
echo "  Stop:  $0 stop"
