#!/usr/bin/env bash
set -e

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PX4="$ROOT/external/PX4-Autopilot"
WORLD="$ROOT/simulation/worlds/simple_goal_demo.sdf"

if [[ -z "${DISPLAY:-}" ]]; then
	echo "Start this demo from a terminal in the Ubuntu desktop session (DISPLAY is unset)." >&2
	exit 1
fi

source "$PX4/build/px4_sitl_default/rootfs/gz_env.sh"

export GZ_PARTITION=uav_ugv_sim
export GZ_IP=127.0.0.1
export GZ_SIM_RESOURCE_PATH="$ROOT/simulation/models:${GZ_SIM_RESOURCE_PATH:-}"

exec gz sim -v4 -r --render-engine-server ogre --render-engine-gui ogre "$WORLD"