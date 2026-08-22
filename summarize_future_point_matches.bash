#!/bin/bash

set -euo pipefail

BASE_DIR="${1:-examples/exp/detailed_archive/run_1}"
CONFIG="${2:-_config_vehwidth=9}"
OUT_FILE="${3:-}"

# Prefer repo-local virtualenv Python when available.
PYTHON_BIN="python3"
if [[ -x ".venv/bin/python" ]]; then
  PYTHON_BIN=".venv/bin/python"
fi

CMD=(
  "$PYTHON_BIN" morty/future_point_match_summary.py
  --base-dir "$BASE_DIR"
  --config "$CONFIG"
  --mode min_or_x
  --compress-increasing
  --compress-use-any-match
)

if [[ -n "$OUT_FILE" ]]; then
  CMD+=(--out "$OUT_FILE")
  "${CMD[@]}"
  echo "Wrote summary to: $OUT_FILE"
else
  "${CMD[@]}"
fi
