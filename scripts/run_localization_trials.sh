#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT="${1:-$ROOT/localization_results}"
SEEDS="${SEEDS:-513 514 515}"
export SEEDS
ESTIMATE_TOPIC="${ESTIMATE_TOPIC:-/ugv/localization/pose}"
EVALUATION_LAUNCH="${EVALUATION_LAUNCH:-ugv_evaluation.launch.py}"
GPS_CASES="${GPS_CASES:-on off startup_off}"
COMPARISON_TOPIC="${COMPARISON_TOPIC:-}"
ROUTE="${ROUTE:-short}"
DURATION="${DURATION:-31}"
MEASUREMENT_PROFILE="${MEASUREMENT_PROFILE:-nominal}"
export GPS_CASES
source /opt/ros/humble/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
set -u
export GZ_PARTITION="${GZ_PARTITION:-uav_ugv_sim}" GZ_IP=127.0.0.1

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
    if ! kill -0 "$world_pid" 2>/dev/null; then
      cat "$OUTPUT/world_${seed}_${gps}.log" >&2
      exit 1
    fi
    initial_gps_enabled=true
    if [[ "$gps" == startup_off ]]; then initial_gps_enabled=false; fi
    launch_args=(seed:="$seed" initial_gps_enabled:="$initial_gps_enabled"
                 shared_position_stddev:="${SHARED_POSITION_STDDEV:-0.0}"
                 systematic_position_stddev:="${SYSTEMATIC_POSITION_STDDEV:-${SHARED_POSITION_STDDEV:-0.0}}"
                 measurement_profile:="$MEASUREMENT_PROFILE")
    if [[ -n "$COMPARISON_TOPIC" ]]; then
      launch_args+=(outlier_every:="${OUTLIER_EVERY:-0}"
                    dropout_start:="${DROPOUT_START:--1.0}" dropout_end:="${DROPOUT_END:--1.0}")
    fi
    setsid ros2 launch cooperative_localization "$EVALUATION_LAUNCH" "${launch_args[@]}" \
      >"$OUTPUT/launch_${seed}_${gps}.log" 2>&1 &
    launch_pid=$!
    sleep 4
    if ! kill -0 "$launch_pid" 2>/dev/null; then
      cat "$OUTPUT/launch_${seed}_${gps}.log" >&2
      exit 1
    fi
    timeout "${TRIAL_TIMEOUT:-300}" ros2 run cooperative_localization localization_trial.py \
      --gps "$gps" --seed "$seed" --estimate-topic "$ESTIMATE_TOPIC" \
      --comparison-topic "$COMPARISON_TOPIC" --route "$ROUTE" --duration "$DURATION" \
      --measurement-profile "$MEASUREMENT_PROFILE" \
      --anchor-source "${ANCHOR_SOURCE:-none}" \
      --shared-position-stddev "${SHARED_POSITION_STDDEV:-0.0}" \
      --systematic-position-stddev "${SYSTEMATIC_POSITION_STDDEV:-${SHARED_POSITION_STDDEV:-0.0}}" \
      --outlier-every "${OUTLIER_EVERY:-0}" \
      --dropout-start "${DROPOUT_START:--1.0}" --dropout-end "${DROPOUT_END:--1.0}" \
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
        strategies = {"assisted" if trial.get("comparison_topic") else "baseline": trial["after_10s"]}
        if "baseline_after_10s" in trial:
            strategies["baseline"] = trial["baseline_after_10s"]
        for strategy, result in strategies.items():
            rows.append({"seed": trial["seed"], "gps": trial["scenario"], "strategy": strategy,
                         **result})
with (output / "summary.csv").open("w", newline="") as file:
    writer = csv.DictWriter(file, fieldnames=rows[0])
    writer.writeheader()
    writer.writerows(rows)
print((output / "summary.csv").read_text())
PY
