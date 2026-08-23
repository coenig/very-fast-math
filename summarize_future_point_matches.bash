#!/bin/bash

set -euo pipefail

# Usage: ./summarize_future_point_matches.bash [RUN] [ARCHIVE_DIR] [OUT_FILE]
#   RUN         Run folder to summarize, e.g. "run_1" or just "1" (default: run_0).
#   ARCHIVE_DIR Directory containing the run_<n> folders (default: examples/exp/detailed_archive).
#   OUT_FILE    Optional file to write the summary to (default: print to stdout).
RUN="${1:-run_0}"
ARCHIVE_DIR="${2:-examples/exp/detailed_archive}"
OUT_FILE="${3:-}"

# Allow passing just a number (e.g. "1" -> "run_1").
if [[ "$RUN" =~ ^[0-9]+$ ]]; then
  RUN="run_$RUN"
fi

RUN_DIR="$ARCHIVE_DIR/$RUN"

# If the run directory is missing but a ZIP archive exists, extract it.
# The archive stores iteration_* at its root, so extract into the run dir itself.
if [[ ! -d "$RUN_DIR" ]]; then
  ZIP_SRC=""
  for cand in "$RUN_DIR.zip" "$RUN_DIR.zip.zip"; do
    if [[ -f "$cand" ]]; then
      ZIP_SRC="$cand"
      break
    fi
  done
  if [[ -n "$ZIP_SRC" ]]; then
    echo "Run directory '$RUN_DIR' not found; extracting from '$ZIP_SRC'..."
    mkdir -p "$RUN_DIR"
    unzip -q -o "$ZIP_SRC" -d "$RUN_DIR"
  fi
fi

if [[ ! -d "$RUN_DIR" ]]; then
  echo "ERROR: run directory not found and no ZIP available to extract: $RUN_DIR" >&2
  exit 1
fi

# Prefer repo-local virtualenv Python when available.
PYTHON_BIN="python3"
if [[ -x ".venv/bin/python" ]]; then
  PYTHON_BIN=".venv/bin/python"
fi

CMD=(
  "$PYTHON_BIN" morty/future_point_match_summary.py
  --base-dir "$RUN_DIR"
  --all-configs
  --single-block
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
