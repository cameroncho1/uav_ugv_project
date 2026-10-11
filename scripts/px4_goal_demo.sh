#!/usr/bin/env bash
set -e

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PX4="$ROOT/external/PX4-Autopilot"

cd "$PX4"
export GZ_PARTITION=uav_ugv_sim
export GZ_IP=127.0.0.1

# The airframe overwrites the environment default during startup. Apply this
# after commander starts so a demo without QGroundControl passes preflight.
(
  px4_param="$PX4/build/px4_sitl_default/bin/px4-param"
  for _ in $(seq 1 120); do
    sleep 2
    if "$PX4/build/px4_sitl_default/bin/px4-commander" status 2>/dev/null | grep -q "navigation mode"; then
      "$px4_param" set NAV_DLL_ACT 0 >/dev/null 2>&1 && break
    fi
  done
) &
trap 'kill %1 2>/dev/null || true' EXIT

PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD=simple_goal_demo \
PX4_GZ_MODEL_POSE="0,0,0.3,0,0,0" \
PX4_PARAM_UXRCE_DDS_SYNCT=0 \
make px4_sitl gz_x500_mono_cam_down
