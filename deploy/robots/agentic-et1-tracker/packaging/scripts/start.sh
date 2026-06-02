#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)/lib.sh"

usage() {
  cat <<'USAGE'
Usage: start.sh [--config PATH] [--foreground]

Starts agentic-et1-tracker from this release. Defaults to the installed
prefix config at config/config.robot.yaml.
USAGE
}

config="$ET1_CONFIG"
foreground=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --config)
      [[ $# -ge 2 ]] || die "--config requires a path"
      config="$2"
      shift 2
      ;;
    --foreground)
      foreground=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown argument: $1"
      ;;
  esac
done

[[ -x "$ET1_BIN" ]] || die "missing executable: $ET1_BIN"
[[ -f "$config" ]] || die "missing config: $config"

mkdir -p "$(dirname "$ET1_PID_FILE")" "$(dirname "$ET1_LOG_FILE")"

if pid="$(read_pid 2>/dev/null)" && is_running_pid "$pid"; then
  printf 'agentic-et1-tracker already running pid=%s\n' "$pid"
  exit 0
fi

if [[ "$foreground" -eq 1 ]]; then
  prepend_release_lib_path
  exec "$ET1_BIN" --config "$config"
fi

prepend_release_lib_path
nohup "$ET1_BIN" --config "$config" >>"$ET1_LOG_FILE" 2>&1 &
pid="$!"
tmp_pid="$ET1_PID_FILE.tmp.$$"
printf '%s\n' "$pid" > "$tmp_pid"
mv -f "$tmp_pid" "$ET1_PID_FILE"

sleep 0.2
if ! is_running_pid "$pid"; then
  rm -f "$ET1_PID_FILE"
  tail -n 40 "$ET1_LOG_FILE" >&2 || true
  die "agentic-et1-tracker exited during startup"
fi

printf 'started agentic-et1-tracker pid=%s config=%s log=%s\n' "$pid" "$config" "$ET1_LOG_FILE"
