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

check_not_lfs_pointer() {
  local path="$1"
  check_file "$path"
  if head -n 1 "$path" | grep -q '^version https://git-lfs.github.com/spec/v1$'; then
    die "git-lfs pointer detected: $path"
  fi
  printf 'ok blob %s\n' "$path"
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
check_file "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/policy/loco_lower/et1_low/README.md"
check_file "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/policy/loco_lower/et1_low/ASSET_MANIFEST.yaml"
check_contains "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/policy/loco_lower/et1_low/ASSET_MANIFEST.yaml" \
  "profile: LocoLowerEt1Low"
check_contains "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/policy/loco_lower/et1_low/ASSET_MANIFEST.yaml" \
  "runtime_use: \"used by /execute_loco_upper lower locomotion runtime; simulation-only handoff, not true-robot GA\""
check_not_lfs_pointer "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/policy/loco_lower/et1_low/exported/policy.onnx"
check_not_lfs_pointer "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/policy/loco_lower/et1_low/params/deploy_lowobs10k.yaml"
check_sha256 "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/policy/loco_lower/et1_low/exported/policy.onnx" \
  "c76686a5b952a10eded30b87673cf098d23d469f596ad6289bbc05b81bdb5203"
check_sha256 "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/policy/loco_lower/et1_low/params/deploy_lowobs10k.yaml" \
  "29cab8fe979f6f8c1647c555916b250a3061664427ff62b0aac7cf09aef87aef"
check_file "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/posture/fixstand/v0/fixstand.yaml"
check_file "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/posture/passive/v0/passive.yaml"
check_file "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/limits/et1_upper_body/v0/README.md"
check_file "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/limits/et1_upper_body/v0/ASSET_MANIFEST.yaml"
check_contains "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/limits/et1_upper_body/v0/ASSET_MANIFEST.yaml" \
  "profile: Et1UpperBodyLimits"
check_contains "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/limits/et1_upper_body/v0/ASSET_MANIFEST.yaml" \
  "runtime_contract:"
check_not_lfs_pointer "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/limits/et1_upper_body/v0/limits.yaml"
check_not_lfs_pointer "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/limits/et1_upper_body/v0/joint_map.yaml"
check_sha256 "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/limits/et1_upper_body/v0/limits.yaml" \
  "d50716e222211c6ca476da70cc46cd2c386470577eb16b267c7bc9bc51d95d99"
check_sha256 "$ET1_RELEASE_DIR/share/agentic-et1-tracker/config/limits/et1_upper_body/v0/joint_map.yaml" \
  "f0e93b20f38352c071f09f68c01c9a907e709c4e37c01b449ac3433ce96bc0c0"
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
