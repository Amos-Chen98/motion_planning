#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

CONFIG_FILE="${MULTILINK_COPILOT_SYNC_CONFIG_FILE:-$SCRIPT_DIR/sync_data.local.env}"
LOCAL_DATA_DIR="$PACKAGE_DIR/data"
LOCAL_BACKUP_DIR="$HOME/.local/share/multilink_copilot_sync_backup/data_local"
FILTER_FILE="$SCRIPT_DIR/sync_data.filters"
RCLONE_BIN="${RCLONE_BIN:-$HOME/.local/bin/rclone}"
CHECK_FILENAME="RCLONE_TEST"
MIN_RCLONE_VERSION="1.71.0"
REMOTE_DATA_DIR="${MULTILINK_COPILOT_SYNC_REMOTE_DATA_DIR:-}"
REMOTE_BACKUP_DIR="${MULTILINK_COPILOT_SYNC_REMOTE_BACKUP_DIR:-}"

log() {
  printf '[sync_data] %s\n' "$*" >&2
}

die() {
  log "ERROR: $*"
  exit 1
}

usage() {
  cat <<EOF
Usage: $(basename "$0") <init|sync|resync|check>

Commands:
  init    Initialize bisync state, create remote directories, create access sentinels, and run the first bisync --resync
  sync    Run the normal manual bidirectional sync
  resync  Rebuild the bisync baseline with --resync
  check   Run rclone check with the shared filter file

This script expects a user-installed rclone with bisync support at:
  $RCLONE_BIN

Remote paths must be provided either by:
  $CONFIG_FILE
or by exporting:
  MULTILINK_COPILOT_SYNC_REMOTE_DATA_DIR
  MULTILINK_COPILOT_SYNC_REMOTE_BACKUP_DIR
EOF
}

version_ge() {
  local left="$1"
  local right="$2"
  [[ "$(printf '%s\n%s\n' "$right" "$left" | sort -V | head -n 1)" == "$right" ]]
}

require_rclone() {
  local version_line version

  [[ -x "$RCLONE_BIN" ]] || die "Expected rclone at $RCLONE_BIN. Install rclone >= $MIN_RCLONE_VERSION there first."

  version_line="$("$RCLONE_BIN" version | head -n 1)"
  version="$(sed -E 's/^rclone v([0-9]+\.[0-9]+\.[0-9]+).*/\1/' <<<"$version_line")"
  [[ -n "$version" && "$version" != "$version_line" ]] || die "Unable to parse rclone version from: $version_line"
  version_ge "$version" "$MIN_RCLONE_VERSION" || die "rclone $version is too old; need >= $MIN_RCLONE_VERSION."

  "$RCLONE_BIN" help bisync >/dev/null 2>&1 || die "This rclone build does not support bisync."
}

require_local_paths() {
  [[ -d "$LOCAL_DATA_DIR" ]] || die "Local data directory not found: $LOCAL_DATA_DIR"
  [[ -f "$FILTER_FILE" ]] || die "Filter file not found: $FILTER_FILE"
}

load_sync_config() {
  if [[ -f "$CONFIG_FILE" ]]; then
    # shellcheck disable=SC1090
    source "$CONFIG_FILE"
  fi

  REMOTE_DATA_DIR="${MULTILINK_COPILOT_SYNC_REMOTE_DATA_DIR:-${REMOTE_DATA_DIR:-}}"
  REMOTE_BACKUP_DIR="${MULTILINK_COPILOT_SYNC_REMOTE_BACKUP_DIR:-${REMOTE_BACKUP_DIR:-}}"

  [[ -n "$REMOTE_DATA_DIR" ]] || die "Remote data dir is not configured. Set MULTILINK_COPILOT_SYNC_REMOTE_DATA_DIR in $CONFIG_FILE or the environment."
  [[ -n "$REMOTE_BACKUP_DIR" ]] || die "Remote backup dir is not configured. Set MULTILINK_COPILOT_SYNC_REMOTE_BACKUP_DIR in $CONFIG_FILE or the environment."
}

require_remote_data_dir() {
  "$RCLONE_BIN" lsf "$REMOTE_DATA_DIR" >/dev/null 2>&1 || die "Remote data directory not found: $REMOTE_DATA_DIR. Run init first."
}

ensure_no_active_writes() {
  local active_files=()

  while IFS= read -r -d '' file; do
    active_files+=("$file")
  done < <(find "$LOCAL_DATA_DIR" -type f -name '*.active' -print0)

  if ((${#active_files[@]} > 0)); then
    printf '[sync_data] Active write marker detected:\n' >&2
    printf '  %s\n' "${active_files[@]}" >&2
    die "Refusing to sync while *.active files are present. Retry after the producer has finished writing."
  fi
}

ensure_support_dirs() {
  mkdir -p "$LOCAL_BACKUP_DIR"
  "$RCLONE_BIN" mkdir "$REMOTE_BACKUP_DIR"
}

create_access_sentinels() {
  local local_sentinel="$LOCAL_DATA_DIR/$CHECK_FILENAME"
  local sentinel_text

  sentinel_text="multilink_copilot data sync access sentinel"
  printf '%s\n' "$sentinel_text" >"$local_sentinel"
  printf '%s\n' "$sentinel_text" | "$RCLONE_BIN" rcat "$REMOTE_DATA_DIR/$CHECK_FILENAME"
}

run_bisync() {
  local mode="$1"
  local args=(
    bisync
    "$LOCAL_DATA_DIR"
    "$REMOTE_DATA_DIR"
    --check-access
    --check-filename
    "$CHECK_FILENAME"
    --create-empty-src-dirs
    --disable
    ListR
    --conflict-resolve
    newer
    --conflict-loser
    num
    --conflict-suffix
    conflict
    --backup-dir1
    "$LOCAL_BACKUP_DIR"
    --backup-dir2
    "$REMOTE_BACKUP_DIR"
    --filters-file
    "$FILTER_FILE"
    --resilient
    --recover
    -v
  )

  if [[ "$mode" == "resync" ]]; then
    args+=(--resync)
  fi

  "$RCLONE_BIN" "${args[@]}"
}

run_check() {
  "$RCLONE_BIN" check "$LOCAL_DATA_DIR" "$REMOTE_DATA_DIR" --filter-from "$FILTER_FILE" -v
}

main() {
  local command="${1:-}"

  case "$command" in
    init|sync|resync|check) ;;
    ""|-h|--help)
      usage
      exit 0
      ;;
    *)
      usage
      die "Unknown command: $command"
      ;;
  esac

  require_rclone
  require_local_paths
  load_sync_config
  ensure_no_active_writes

  case "$command" in
    init)
      ensure_support_dirs
      "$RCLONE_BIN" mkdir "$REMOTE_DATA_DIR"
      create_access_sentinels
      run_bisync resync
      ;;
    sync)
      ensure_support_dirs
      require_remote_data_dir
      run_bisync sync
      ;;
    resync)
      ensure_support_dirs
      require_remote_data_dir
      run_bisync resync
      ;;
    check)
      require_remote_data_dir
      run_check
      ;;
  esac
}

main "$@"
