#!/usr/bin/env bash
set -e

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PX4="$ROOT/external/PX4-Autopilot"

cd "$PX4"

export GZ_PARTITION=uav_ugv_sim
export GZ_IP=127.0.0.1

PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD=uav_ugv \
PX4_GZ_MODEL_POSE="2,-2,0.3,0,0,0" \
PX4_PARAM_NAV_DLL_ACT=0 \
make px4_sitl gz_x500