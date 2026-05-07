#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
AERIAL_MOTION_DIR="$(cd "$PACKAGE_DIR/.." && pwd)"
OUTPUT_BASE_DIR="$PACKAGE_DIR/data/exp_setup"

log() {
  printf '[save_exp_setup] %s\n' "$*" >&2
}

die() {
  log "ERROR: $*"
  exit 1
}

SOURCE_FILES=(
  "$PACKAGE_DIR/launch/copilot_planner.launch"
  "$PACKAGE_DIR/config/copilot_planner.yaml"
  "$AERIAL_MOTION_DIR/mono_planner/launch/waypoint_conditioned_planner.launch"
)

for source_file in "${SOURCE_FILES[@]}"; do
  [[ -f "$source_file" ]] || die "Source file not found: $source_file"
done

mkdir -p "$OUTPUT_BASE_DIR"

DATE_PREFIX="$(date +%Y%m%d)"
OUTPUT_DIR=""

for suffix in $(seq -f '%03g' 1 999); do
  candidate="$OUTPUT_BASE_DIR/${DATE_PREFIX}_${suffix}"
  if mkdir "$candidate" 2>/dev/null; then
    OUTPUT_DIR="$candidate"
    break
  fi
done

[[ -n "$OUTPUT_DIR" ]] || die "Unable to create a new output directory for $DATE_PREFIX"

TIMESTAMP_SUFFIX="$(basename "$OUTPUT_DIR")"

for source_file in "${SOURCE_FILES[@]}"; do
  source_name="$(basename "$source_file")"
  name_without_ext="${source_name%.*}"
  extension="${source_name##*.}"
  target_name="${name_without_ext}_${TIMESTAMP_SUFFIX}.${extension}"

  cp -p "$source_file" "$OUTPUT_DIR/$target_name"
done

log "Saved experiment setup to: $OUTPUT_DIR"
