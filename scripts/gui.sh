#!/usr/bin/env bash
set -e

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PX4="$ROOT/external/PX4-Autopilot"

source "$PX4/build/px4_sitl_default/rootfs/gz_env.sh"

export GZ_PARTITION=uav_ugv_sim
export GZ_IP=127.0.0.1

gz sim -g --render-engine-gui ogre