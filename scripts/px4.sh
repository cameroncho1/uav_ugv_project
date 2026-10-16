#!/usr/bin/env bash
set -e

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PX4="$ROOT/external/PX4-Autopilot"

cd "$PX4"

export GZ_PARTITION="${GZ_PARTITION:-uav_ugv_sim}"
export GZ_IP=127.0.0.1
export PX4_PARAM_UXRCE_DDS_DOM_ID="${ROS_DOMAIN_ID:-0}"

# Disable the "No connection to the GCS" preflight check (no QGroundControl here).
# PX4_PARAM_NAV_DLL_ACT=0 does not work: 0 is PX4's built-in default, so the
# value is dropped when the x500 airframe later runs "param set-default
# NAV_DLL_ACT 2". Set it from a helper once PX4 has finished booting instead.
(
  px4_param="$PX4/build/px4_sitl_default/bin/px4-param"
  for _ in $(seq 1 120); do
    sleep 2
    # commander starts after the airframe defaults are applied
    if "$PX4/build/px4_sitl_default/bin/px4-commander" status 2>/dev/null | grep -q "navigation mode"; then
      "$px4_param" set NAV_DLL_ACT 0 >/dev/null 2>&1 && break
    fi
  done
) &
trap 'kill %1 2>/dev/null || true' EXIT

PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD=uav_ugv \
PX4_GZ_MODEL_POSE="2,-2,0.3,0,0,0" \
PX4_PARAM_UXRCE_DDS_SYNCT="${PX4_DDS_SYNCT:-1}" \
make px4_sitl gz_x500
