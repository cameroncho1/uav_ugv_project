#!/usr/bin/env bash
# Record clean raw UGV sensor runs: one headless world and one seeded maneuver schedule per seed.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT="${1:-$ROOT/datasets/ugv_clean}"
SEEDS="${SEEDS:-$(seq 1000 1009)}"
DURATION="${DURATION:-120}"
WORLD="${WORLD:-ugv_dataset}"
# Gazebo steps at 250 Hz in real time; a higher update rate runs faster than real time.
UPDATE_RATE="${UPDATE_RATE:-250}"
export WORLD
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u
export GZ_PARTITION="${GZ_PARTITION:-ugv_dataset}" GZ_IP=127.0.0.1

world_pid=""
launch_pid=""
cleanup() {
  local groups=()
  [[ -n "$launch_pid" ]] && groups+=("$launch_pid")
  [[ -n "$world_pid" ]] && groups+=("$world_pid")
  stop_process_groups "${groups[@]}"
  launch_pid=""
  world_pid=""
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
source "$ROOT/scripts/process_cleanup.sh"
mkdir -p "$OUTPUT"
git -C "$ROOT" rev-parse HEAD > "$OUTPUT/git_commit.txt"
git -C "$ROOT" status --short > "$OUTPUT/git_status.txt"

for seed in $SEEDS; do
  run="$OUTPUT/run_$seed"
  if [[ -f "$run/metadata.json" ]]; then
    echo "Skipping seed=$seed (already recorded)"
    continue
  fi
  echo "Recording seed=$seed duration=${DURATION}s world=$WORLD"
  mkdir -p "$run"
  setsid "$ROOT/scripts/world.sh" -z "$UPDATE_RATE" >"$run/world.log" 2>&1 &
  world_pid=$!
  sleep 3
  if ! kill -0 "$world_pid" 2>/dev/null; then
    cat "$run/world.log" >&2
    exit 1
  fi
  setsid timeout "$(( DURATION * 2 + 120 ))" ros2 launch fault_management ugv_dataset.launch.py \
    world:="$WORLD" seed:="$seed" duration:="$DURATION" output:="$run" \
    >"$run/launch.log" 2>&1 &
  launch_pid=$!
  wait "$launch_pid" || true
  launch_pid=""
  cleanup
  if [[ ! -f "$run/metadata.json" ]]; then
    tail -20 "$run/launch.log" >&2
    echo "Seed $seed did not finish; see $run/launch.log" >&2
    exit 1
  fi
  sleep 2
done

python3 - "$OUTPUT" <<'PY'
import json
import pathlib
import sys

for run in sorted(pathlib.Path(sys.argv[1]).glob("run_*/metadata.json")):
    print(run.parent.name, json.loads(run.read_text())["counts"])
PY
