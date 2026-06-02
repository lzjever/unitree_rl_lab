#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)/lib.sh"

usage() {
  cat <<'USAGE'
Usage: selftest.sh [--online] [--url URL]

Runs offline release layout checks. With --online, also queries /status through
the bundled et1-trk2motion CLI.
USAGE
}

online=0
url="$ET1_URL"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --online)
      online=1
      shift
      ;;
    --url)
      [[ $# -ge 2 ]] || die "--url requires a URL"
      url="$2"
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

check_file() {
  [[ -f "$1" ]] || die "missing file: $1"
  printf 'ok file %s\n' "$1"
}

check_exec() {
  [[ -x "$1" ]] || die "missing executable: $1"
  printf 'ok exec %s\n' "$1"
}

check_exec "$ET1_BIN"
check_exec "$ET1_CLI"
check_file "$ET1_RELEASE_DIR/config/config.robot.yaml.template"
check_file "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/policy/general_tracker/exported/self_collision_footmesh_15k.onnx"
check_file "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/policy/general_tracker/params/deploy.yaml"
check_file "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/policy/velocity/v0/exported/policy.onnx"
check_file "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/policy/velocity/v0/params/deploy.yaml"
check_file "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/posture/fixstand/v0/fixstand.yaml"
check_file "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/posture/passive/v0/passive.yaml"

prepend_release_lib_path
"$ET1_BIN" --help >/dev/null 2>&1
"$ET1_CLI" --help >/dev/null 2>&1
printf 'ok help commands\n'

if [[ -f "$ET1_CONFIG" ]]; then
  if grep -q '@PREFIX@' "$ET1_CONFIG"; then
    die "installed config still contains @PREFIX@: $ET1_CONFIG"
  fi
  printf 'ok installed config %s\n' "$ET1_CONFIG"
else
  printf 'skip installed config %s\n' "$ET1_CONFIG"
fi

if [[ "$online" -eq 1 ]]; then
  ET1_TRACKER_URL="$url" "$ET1_CLI" status
fi

printf 'selftest ok release=%s prefix=%s\n' "$ET1_RELEASE_DIR" "$ET1_PREFIX"
