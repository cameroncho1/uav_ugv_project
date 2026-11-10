#!/usr/bin/env bash
set -e

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PX4="$ROOT/external/PX4-Autopilot"
WORLD="$ROOT/simulation/worlds/simple_goal_demo.sdf" #swap this for whichever variant to be tested (incl spare, dense, narrow, corridor)

python3 "$ROOT/scripts/check_sim_dependencies.py"

if [[ -z "${DISPLAY:-}" ]]; then
	echo "Start this demo from a terminal in the Ubuntu desktop session (DISPLAY is unset)." >&2
	exit 1
fi

source "$PX4/build/px4_sitl_default/rootfs/gz_env.sh"

export GZ_PARTITION="${GZ_PARTITION:-uav_ugv_sim}"
export GZ_IP=127.0.0.1
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROOT/ros2_ws/install/ugv_control/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$ROOT/simulation/models:${GZ_SIM_RESOURCE_PATH:-}"

# Start the server with the world file, then the GUI as a separate client. The combined
# `gz sim WORLD` form makes the GUI hand the world to the server in a single message; when
# that message is lost both processes wait for each other forever (about one demo launch
# in ten on the ARM64 VM). A standalone GUI keeps asking the server until it answers.
gz sim -v4 -s -r --render-engine-server ogre "$WORLD" &
server=$!
gui=

# Stop both processes within 3 s, inside ros2 launch's 5 s window before it escalates.
# Further signals are ignored so the cleanup cannot be cut short, and SIGTERM is used
# because background jobs of a script ignore SIGINT. The Gazebo server does not always
# finish its own shutdown, so anything still running after 3 s is killed.
stop() {
	trap '' INT TERM
	local pids=("$server")
	[ -n "$gui" ] && pids+=("$gui")
	kill -TERM "${pids[@]}" 2>/dev/null || true
	for _ in $(seq 30); do
		kill -0 "${pids[@]}" 2>/dev/null || return 0
		sleep 0.1
	done
	kill -KILL "${pids[@]}" 2>/dev/null || true
}
trap 'stop; exit 0' INT TERM

# Start the GUI only after the server has created the camera sensor (PX4's script spawns
# the camera UAV), so the two processes never set up OpenGL at the same time, matching the
# order of the combined command. Without a camera within 120 s the GUI starts anyway.
deadline=$((SECONDS + 120))
while [ "$SECONDS" -lt "$deadline" ] && kill -0 "$server" 2>/dev/null; do
	gz topic -l 2>/dev/null | grep -q '/sensor/camera/image$' && break
	sleep 0.5
done
gz sim -v4 -g --render-engine-gui ogre &
gui=$!

# Closing the GUI or losing the server stops both, as the combined command did. The first
# exit status is kept (set -e must not skip the cleanup when a process crashes).
status=0
wait -n "$server" "$gui" || status=$?
stop
exit "$status"
