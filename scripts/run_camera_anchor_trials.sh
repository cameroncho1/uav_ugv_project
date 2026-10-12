#!/usr/bin/env bash
# Live PX4, rendered camera measurements, paired filters, and fresh worlds.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT="${1:-$ROOT/localization_results/camera_anchors}"
export ANCHOR_MODE=px4 RELATIVE_MODE=camera
for seed in ${SEEDS:-513 514 515}; do
  for gps in ${GPS_CASES:-on off startup_off}; do
    SEED="$seed" GPS_CASE="$gps" "$ROOT/scripts/run_live_cooperative_trial.sh" \
      "$OUTPUT/seed_${seed}_${gps}"
  done
done
