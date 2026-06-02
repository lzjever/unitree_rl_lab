#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)/lib.sh"

usage() {
  cat <<'USAGE'
Usage: stop.sh [--timeout SECONDS] [--kill]

Stops the pid recorded under the installed prefix run directory.
USAGE
}

timeout_s=10
send_kill=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --timeout)
      [[ $# -ge 2 ]] || die "--timeout requires seconds"
      timeout_s="$2"
      shift 2
      ;;
    --kill)
      send_kill=1
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

pid="$(read_pid 2>/dev/null || true)"
if [[ -z "$pid" ]]; then
  printf 'agentic-et1-tracker is not running: no pid file\n'
  exit 0
fi

if ! is_running_pid "$pid"; then
  rm -f "$ET1_PID_FILE"
  printf 'agentic-et1-tracker is not running: removed stale pid=%s\n' "$pid"
  exit 0
fi

kill "$pid"
deadline=$((SECONDS + timeout_s))
while is_running_pid "$pid"; do
  if (( SECONDS >= deadline )); then
    if [[ "$send_kill" -eq 1 ]]; then
      kill -KILL "$pid" 2>/dev/null || true
      break
    fi
    die "timed out waiting for pid=$pid; rerun with --kill to force"
  fi
  sleep 0.2
done

rm -f "$ET1_PID_FILE"
printf 'stopped agentic-et1-tracker pid=%s\n' "$pid"
