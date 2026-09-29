#!/usr/bin/env bash
#
# start_sim_tmux.sh - start every process needed for the UAV-UGV simulation
# inside one tmux session, one tmux window per process.
#
# Works anywhere tmux is installed (Ubuntu VM, WSL2, SSH). It does not need
# gnome-terminal. start_sim.sh falls back to this script automatically when
# gnome-terminal is missing.
#
# Usage:
#   ./start_sim_tmux.sh              start everything, including the 3D Gazebo window
#   ./start_sim_tmux.sh --no-gui     start everything except the 3D window
#   ./start_sim_tmux.sh --no-attach  start in the background, do not open tmux
#   ./start_sim_tmux.sh stop         shut down every simulation process
#
# tmux basics (prefix is Ctrl+b):
#   Ctrl+b then n / p    next / previous window
#   Ctrl+b then 0-9      jump to window by number
#   Ctrl+b then d        detach (everything keeps running); reattach with:
#                        tmux attach -t uavugv
# Mouse support is on, so you can also click the window names at the bottom.

set -u

# ---------------------------------------------------------------- settings
PROJECT="${PROJECT:-$HOME/uav_ugv_project}"
WORLD="${WORLD:-uav_ugv}"
SESSION="${SESSION:-uavugv}"
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
  tmux kill-session -t "$SESSION" 2>/dev/null
  echo "Done. tmux session '$SESSION' closed."
}

WITH_GUI=1
ATTACH=1
MODE=start
for arg in "$@"; do
  case "$arg" in
    stop)        MODE=stop ;;
    --no-gui)    WITH_GUI=0 ;;
    --no-attach) ATTACH=0 ;;
    *) echo "Unknown option: $arg" >&2; exit 1 ;;
  esac
done

if [[ "$MODE" == "stop" ]]; then
  stop_all
  exit 0
fi

# ---------------------------------------------------------------- sanity checks
if ! command -v tmux >/dev/null; then
  echo "Error: tmux not found. Install it with: sudo apt install tmux" >&2
  exit 1
fi
if [[ ! -d "$PROJECT/scripts" ]]; then
  echo "Error: $PROJECT/scripts not found. Set PROJECT=/path/to/uav_ugv_project." >&2
  exit 1
fi
if pgrep -x px4 >/dev/null || pgrep -f "gz sim" >/dev/null || tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "A simulation already seems to be running."
  echo "Run '$0 stop' first, then start again."
  exit 1
fi
if [[ $WITH_GUI -eq 1 && -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "No graphical display found, skipping the Gazebo window (same as --no-gui)."
  WITH_GUI=0
fi

chmod +x "$PROJECT"/scripts/*.sh 2>/dev/null

# ---------------------------------------------------------------- helpers
# Open a tmux window that runs a command and then stays open as a normal
# shell, so error messages remain readable after a crash or Ctrl+C.
# ROS is sourced first because some tools (the DDS agent) need its libraries.
open_win() {
  local title="$1" cmd="$2"
  local inner="
    export GZ_PARTITION='$GZ_PARTITION' GZ_IP='$GZ_IP'
    [ -f '$ROS_SETUP' ] && source '$ROS_SETUP'
    [ -f '$WS_SETUP' ] && source '$WS_SETUP'
    cd '$PROJECT'
    $cmd
    echo
    echo '[$title has exited. This window is now a normal shell.]'
    exec bash"
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux new-window -d -t "$SESSION:" -n "$title" bash -c "$inner"
  else
    tmux new-session -d -s "$SESSION" -n "$title" -x 200 -y 50 bash -c "$inner"
    tmux set-option -t "$SESSION" mouse on
  fi
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
echo "Starting UAV-UGV simulation in tmux session '$SESSION' (partition: $GZ_PARTITION)"

# 1. Gazebo physics server
open_win "1-world" "./scripts/world.sh"
wait_for_topic "/world/$WORLD/" 60 "Gazebo world"

# 2. PX4 autopilot. Stays in the foreground so the pxh> prompt is usable.
open_win "2-px4" "./scripts/px4.sh"
wait_for_topic "x500" 90 "PX4 to spawn the drone"

# 3. DDS bridge between PX4 and ROS 2
open_win "3-dds" "./scripts/dds.sh"

# 4. ROS 2 shell with the workspace already sourced
open_win "4-ros2" "cd '$PROJECT/ros2_ws'
    echo 'ROS 2 shell ready. Try:'
    echo '  ros2 topic list | grep /fmu'
    echo '  ros2 run uav_control odometry_listener'"

# 5. Gazebo 3D window (optional)
if [[ $WITH_GUI -eq 1 ]]; then
  sleep 3
  open_win "5-gui" "./scripts/gui.sh"
fi

echo
echo "All windows launched in tmux session '$SESSION'."
echo "  Fly:   in the 2-px4 window type  commander takeoff"
echo "  Stop:  $0 stop"

if [[ $ATTACH -eq 1 ]]; then
  tmux select-window -t "$SESSION:4-ros2"
  if [[ -n "${TMUX:-}" ]]; then
    tmux switch-client -t "$SESSION"
  else
    exec tmux attach -t "$SESSION"
  fi
else
  echo "  View:  tmux attach -t $SESSION"
fi
