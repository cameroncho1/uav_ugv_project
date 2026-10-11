#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT="${1:-$ROOT/localization_results}"
SEEDS="${SEEDS:-513 514 515}"
export SEEDS
ESTIMATE_TOPIC="${ESTIMATE_TOPIC:-/ugv/localization/pose}"
EVALUATION_LAUNCH="${EVALUATION_LAUNCH:-ugv_evaluation.launch.py}"
GPS_CASES="${GPS_CASES:-on off startup_off}"
export GPS_CASES
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u
export GZ_PARTITION=uav_ugv_sim GZ_IP=127.0.0.1

world_pid=""
launch_pid=""
cleanup() {
  if [[ -n "$launch_pid" ]]; then kill -INT -- "-$launch_pid" 2>/dev/null || true; wait "$launch_pid" 2>/dev/null || true; fi
  if [[ -n "$world_pid" ]]; then kill -INT -- "-$world_pid" 2>/dev/null || true; wait "$world_pid" 2>/dev/null || true; fi
  launch_pid=""
  world_pid=""
}
trap cleanup EXIT INT TERM
mkdir -p "$OUTPUT"

for seed in $SEEDS; do
  for gps in $GPS_CASES; do
    echo "Running seed=$seed gps=$gps"
    setsid "$ROOT/scripts/world.sh" >"$OUTPUT/world_${seed}_${gps}.log" 2>&1 &
    world_pid=$!
    sleep 3
    initial_gps_enabled=true
    if [[ "$gps" == startup_off ]]; then initial_gps_enabled=false; fi
    setsid ros2 launch cooperative_localization "$EVALUATION_LAUNCH" seed:="$seed" \
      initial_gps_enabled:="$initial_gps_enabled" \
      >"$OUTPUT/launch_${seed}_${gps}.log" 2>&1 &
    launch_pid=$!
    sleep 4
    timeout 150 ros2 run cooperative_localization localization_trial.py \
      --gps "$gps" --seed "$seed" --estimate-topic "$ESTIMATE_TOPIC" \
      --output "$OUTPUT/trial_${seed}_${gps}"
    cleanup
    sleep 2
  done
done

python3 - "$OUTPUT" <<'PY'
import csv
import json
import os
import pathlib
import sys

output = pathlib.Path(sys.argv[1])
rows = []
for seed in os.environ["SEEDS"].split():
    for gps in os.environ["GPS_CASES"].split():
        trial = json.loads((output / f"trial_{seed}_{gps}.json").read_text())
        rows.append({"seed": trial["seed"], "gps": trial["scenario"],
                     "samples": trial["after_10s"]["samples"],
                     "mean_error_m": trial["after_10s"]["mean_error_m"],
                     "rmse_m": trial["after_10s"]["rmse_m"],
                     "p95_error_m": trial["after_10s"]["p95_error_m"],
                     "max_error_m": trial["after_10s"]["max_error_m"],
                     "mean_heading_error_deg": trial["after_10s"]["mean_heading_error_deg"]})
with (output / "summary.csv").open("w", newline="") as file:
    writer = csv.DictWriter(file, fieldnames=rows[0])
    writer.writeheader()
    writer.writerows(rows)
print((output / "summary.csv").read_text())
PY
