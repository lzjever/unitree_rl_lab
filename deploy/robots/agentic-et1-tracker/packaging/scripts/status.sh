#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)/lib.sh"

usage() {
  cat <<'USAGE'
Usage: status.sh [--url URL] [--process-only] [--full] [--id RUN_ID]

Prints local process state, then queries the bundled et1-trk2motion CLI unless
--process-only is used.
USAGE
}

url="$ET1_URL"
process_only=0
cli_args=(status)
while [[ $# -gt 0 ]]; do
  case "$1" in
    --url)
      [[ $# -ge 2 ]] || die "--url requires a URL"
      url="$2"
      shift 2
      ;;
    --process-only)
      process_only=1
      shift
      ;;
    --full)
      cli_args+=(--full)
      shift
      ;;
    --id)
      [[ $# -ge 2 ]] || die "--id requires a run id"
      cli_args+=(--id "$2")
      shift 2
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
if [[ -n "$pid" ]] && is_running_pid "$pid"; then
  printf 'process: running pid=%s\n' "$pid"
else
  printf 'process: stopped\n'
fi

if [[ "$process_only" -eq 1 ]]; then
  exit 0
fi

[[ -x "$ET1_CLI" ]] || die "missing bundled CLI: $ET1_CLI"
ET1_TRACKER_URL="$url" "$ET1_CLI" "${cli_args[@]}"
