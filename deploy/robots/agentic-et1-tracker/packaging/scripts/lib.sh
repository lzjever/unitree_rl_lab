#!/usr/bin/env bash
set -euo pipefail

ET1_RELEASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"

detect_prefix() {
  local release_dir="$1"
  local parent
  parent="$(basename "$(dirname "$release_dir")")"
  if [[ "$parent" == "releases" ]]; then
    cd "$release_dir/../.." && pwd -P
  else
    printf '%s\n' "$release_dir"
  fi
}

ET1_PREFIX="${ET1_PREFIX:-$(detect_prefix "$ET1_RELEASE_DIR")}"
ET1_CONFIG="${ET1_TRACKER_CONFIG:-$ET1_PREFIX/shared/config/config.robot.yaml}"
ET1_URL="${ET1_TRACKER_URL:-http://127.0.0.1:8083}"
ET1_PID_FILE="${ET1_PID_FILE:-$ET1_PREFIX/shared/run/agentic-et1-tracker.pid}"
ET1_LOG_FILE="${ET1_LOG_FILE:-$ET1_PREFIX/shared/logs/agentic-et1-tracker.log}"
ET1_BIN="$ET1_RELEASE_DIR/bin/agentic-et1-tracker"
ET1_CLI="$ET1_RELEASE_DIR/bin/et1-action"
ET1_LIB_DIR="$ET1_RELEASE_DIR/lib"

die() {
  printf 'error: %s\n' "$*" >&2
  exit 1
}

is_running_pid() {
  local pid="$1"
  [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null
}

read_pid() {
  [[ -f "$ET1_PID_FILE" ]] || return 1
  local pid
  pid="$(tr -d '[:space:]' < "$ET1_PID_FILE")"
  [[ -n "$pid" ]] || return 1
  printf '%s\n' "$pid"
}

prepend_release_lib_path() {
  export LD_LIBRARY_PATH="$ET1_LIB_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
}
