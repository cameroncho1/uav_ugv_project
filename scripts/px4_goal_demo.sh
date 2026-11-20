#!/usr/bin/env bash
set -e

# ROS launch redirects stdin. Give PX4's console a terminal to avoid an EOF loop.
if [[ ! -t 0 ]]; then
  printf -v launch_command '%q' "$0"
  exec script -q -c "$launch_command" /dev/null
fi

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PX4="$ROOT/external/PX4-Autopilot"

source "$PX4/build/px4_sitl_default/rootfs/gz_env.sh"
export GZ_SIM_RESOURCE_PATH="$ROOT/simulation/models:$GZ_SIM_RESOURCE_PATH"

cd "$PX4"
export GZ_PARTITION="${GZ_PARTITION:-uav_ugv_sim}"
export GZ_IP=127.0.0.1
export PX4_PARAM_UXRCE_DDS_DOM_ID="${ROS_DOMAIN_ID:-0}"

# The owned camera model reuses the aligned upstream flight model. Create it
# through the world API, then let PX4 attach without changing its submodule.
created=false
for _ in $(seq 1 60); do
  if gz service -s /world/simple_goal_demo/create \
    --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 2000 \
    --req "name: 'cooperative_camera_uav_0', allow_renaming: false, \
      sdf_filename: '$ROOT/simulation/models/cooperative_camera_uav/model.sdf', \
      pose: {position: {x: 0, y: 0, z: 0.3}}" | grep -q 'data: true'; then
    created=true
    break
  fi
  sleep 1
done
if [[ "$created" != true ]]; then
  echo "Could not create cooperative camera UAV in a fresh goal-demo world" >&2
  exit 1
fi

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
PX4_GZ_MODEL_NAME=cooperative_camera_uav_0 \
PX4_PARAM_UXRCE_DDS_SYNCT=0 \
make px4_sitl gz_x500_mono_cam_down
