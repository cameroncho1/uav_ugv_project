#!/usr/bin/env bash
# Paired filters share one world, one route, and one noisy sensor stream.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export EVALUATION_LAUNCH=ugv_cooperative_evaluation.launch.py
export COMPARISON_TOPIC=/evaluation/baseline_pose
export ANCHOR_SOURCE=simulated
exec "$ROOT/scripts/run_localization_trials.sh" "${1:-$ROOT/cooperative_results}"
