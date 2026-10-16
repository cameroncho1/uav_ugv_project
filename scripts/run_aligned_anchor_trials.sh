#!/usr/bin/env bash
# Repeat actual PX4 anchor trials with a fresh world for each seed and GPS case.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT="${1:-$ROOT/localization_results/aligned_px4}"
for seed in ${SEEDS:-513 514 515}; do
  for gps in ${GPS_CASES:-on off startup_off}; do
    echo "Running aligned PX4: seed=$seed gps=$gps"
    SEED="$seed" GPS_CASE="$gps" ANCHOR_MODE=px4 RELATIVE_MODE=simulated \
      "$ROOT/scripts/run_live_cooperative_trial.sh" "$OUTPUT/seed_${seed}_${gps}"
  done
done
