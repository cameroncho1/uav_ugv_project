#!/usr/bin/env bash
set -e

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PX4="$ROOT/external/PX4-Autopilot"

python3 "$ROOT/scripts/check_sim_dependencies.py"

source "$PX4/build/px4_sitl_default/rootfs/gz_env.sh"

export GZ_SIM_RESOURCE_PATH="$ROOT/simulation/models:$GZ_SIM_RESOURCE_PATH"

export GZ_PARTITION="${GZ_PARTITION:-uav_ugv_sim}"
export GZ_IP=127.0.0.1

gz sim -v4 -s -r "$ROOT/simulation/worlds/uav_ugv.sdf"
