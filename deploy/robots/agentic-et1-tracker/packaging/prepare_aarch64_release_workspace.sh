#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
TRACKER_DIR="$(cd "$SCRIPT_DIR/.." && pwd -P)"
REPO_ROOT="$(git -C "$TRACKER_DIR" rev-parse --show-toplevel)"
WORK_ROOT="$(cd "$REPO_ROOT/.." && pwd -P)"
MARKER=".agentic-et1-aarch64-release-workspace"

usage() {
  cat <<'USAGE'
Usage: prepare_aarch64_release_workspace.sh [options]

Creates a real git clone release workspace from the current commit. This avoids
git worktree paths and host absolute symlinks that are not visible in Docker.

Options:
  --workspace-root DIR    Parent dir for the prepared workspace. Default: /tmp.
  --workspace DIR         Exact workspace dir. Default: auto under workspace-root.
  --ref REF               Commit/ref to clone. Default: HEAD.
  --keep-existing         Reuse an empty or marked release workspace after deleting its contents.
  -h, --help              Show help.
USAGE
}

workspace_root="/tmp"
workspace=""
ref="HEAD"
keep_existing=0
cleanup_on_err=0

cleanup_workspace_on_err() {
  if [[ "$cleanup_on_err" -eq 1 && -n "$workspace" && -f "$workspace/$MARKER" ]]; then
    find "$workspace" -mindepth 1 -maxdepth 1 -exec rm -rf {} + || true
  fi
}
trap cleanup_workspace_on_err ERR EXIT

workspace_is_empty() {
  [[ -d "$1" ]] || return 1
  [[ -z "$(find "$1" -mindepth 1 -maxdepth 1 -print -quit)" ]]
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace-root)
      [[ $# -ge 2 ]] || { printf 'error: --workspace-root requires a value\n' >&2; exit 1; }
      workspace_root="$2"
      shift 2
      ;;
    --workspace)
      [[ $# -ge 2 ]] || { printf 'error: --workspace requires a value\n' >&2; exit 1; }
      workspace="$2"
      shift 2
      ;;
    --ref)
      [[ $# -ge 2 ]] || { printf 'error: --ref requires a value\n' >&2; exit 1; }
      ref="$2"
      shift 2
      ;;
    --keep-existing)
      keep_existing=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf 'error: unknown argument: %s\n' "$1" >&2
      exit 1
      ;;
  esac
done

commit="$(git -C "$REPO_ROOT" rev-parse "$ref")"
if [[ -z "$workspace" ]]; then
  workspace="$workspace_root/agentic-et1-aarch64-release-$commit"
fi

if [[ -e "$workspace" ]]; then
  if [[ ! -d "$workspace" ]]; then
    printf 'error: workspace path exists but is not a directory: %s\n' "$workspace" >&2
    exit 1
  fi
  if [[ "$keep_existing" -eq 1 ]]; then
    if [[ -f "$workspace/$MARKER" ]]; then
      find "$workspace" -mindepth 1 -maxdepth 1 -exec rm -rf {} +
    elif workspace_is_empty "$workspace"; then
      :
    else
      printf 'error: refusing to reuse non-empty workspace without marker: %s\n' "$workspace" >&2
      printf 'hint: choose an empty directory or an existing agentic-et1 aarch64 release workspace\n' >&2
      exit 1
    fi
  else
    printf 'error: workspace already exists: %s\n' "$workspace" >&2
    printf 'hint: pass --keep-existing or choose another --workspace\n' >&2
    exit 1
  fi
fi

mkdir -p "$workspace"
printf 'agentic-et1-aarch64-release-workspace\ncommit=%s\n' "$commit" > "$workspace/$MARKER"
cleanup_on_err=1
git clone --no-local "$REPO_ROOT" "$workspace/unitree_rl_lab" >/dev/null
git -C "$workspace/unitree_rl_lab" checkout --detach "$commit" >/dev/null

sdk_src="$WORK_ROOT/third_party/unitree_sdk2_install_aarch64"
sdk_dst="$workspace/third_party/unitree_sdk2_install_aarch64"
[[ -d "$sdk_src" ]] || { printf 'error: missing %s\n' "$sdk_src" >&2; exit 1; }
mkdir -p "$workspace/third_party"
cp -aL "$sdk_src" "$sdk_dst"

cleanup_on_err=0
printf '%s\n' "$workspace"
