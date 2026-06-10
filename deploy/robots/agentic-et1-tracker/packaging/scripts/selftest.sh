#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)/lib.sh"

usage() {
  cat <<'USAGE'
Usage: selftest.sh [--online] [--url URL]

Runs offline release layout checks. With --online, also queries /status through
the bundled et1-action CLI.
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

check_sha256() {
  local path="$1"
  local expected="$2"
  local actual
  check_file "$path"
  actual="$(sha256sum "$path" | awk '{print $1}')"
  [[ "$actual" == "$expected" ]] || die "sha256 mismatch: $path expected $expected got $actual"
  printf 'ok sha256 %s\n' "$path"
}

check_contains() {
  local path="$1"
  local needle="$2"
  check_file "$path"
  grep -q "$needle" "$path" || die "missing '$needle' in $path"
  printf 'ok contains %s %s\n' "$path" "$needle"
}

check_exec "$ET1_BIN"
check_exec "$ET1_CLI"
check_file "$ET1_RELEASE_DIR/config/config.robot.yaml.template"
check_contains "$ET1_RELEASE_DIR/config/config.robot.yaml.template" \
  "GeneralTrackerCLNFootstate"
check_contains "$ET1_RELEASE_DIR/config/config.robot.yaml.template" \
  "multi_policy_footstate3.onnx"
check_contains "$ET1_RELEASE_DIR/config/config.robot.yaml.template" \
  "deploy_fut_multi_footstate.yaml"
[[ ! -d "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/policy/general_tracker" ]] || \
  die "legacy policy directory must not be present: $ET1_RELEASE_DIR/share/agentic-et1-tracker/config/policy/general_tracker"
check_file "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/policy/general_tracker_cln/ASSET_MANIFEST.yaml"
check_sha256 "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/policy/general_tracker_cln/exported/multi_policy_footstate3.onnx" \
  "3afdd52f115dc01b042cd3f1be40c90a2affcae5987e5cb5e52442a2115b37d7"
check_sha256 "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/policy/general_tracker_cln/params/deploy_fut_multi_footstate.yaml" \
  "89734594308d6e036d348f80e6fa2fa7e224d1e9a4da4ac2b53bb68de91095fc"
check_sha256 "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/policy/general_tracker_cln/exported/multi_policy_v17c2_70k.onnx" \
  "d4f37c972eb5e98e37a1d425302a70729343009a1564974921965c5faea0d911"
check_sha256 "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/policy/general_tracker_cln/params/deploy.yaml" \
  "de8ba00c0b79590b2ccc0a7d84fcc0db4a8869ad9111e54e46c9427b89ffaf84"
check_file "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/policy/velocity/v0/exported/policy.onnx"
check_file "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/policy/velocity/v0/params/deploy.yaml"
check_file "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/posture/fixstand/v0/fixstand.yaml"
check_file "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/posture/passive/v0/passive.yaml"
check_file "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/reference/standby/v0/ASSET_MANIFEST.yaml"
check_file "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/reference/standby/v0/README.md"
check_sha256 "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/reference/standby/v0/standby_ref.trk" \
  "6ca49404e1ee1008f6226a2f7c00e990f0447ae6c826657246b7a29fbb525741"

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
