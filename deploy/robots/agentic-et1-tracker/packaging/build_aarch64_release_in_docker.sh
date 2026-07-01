#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
TRACKER_DIR="$(cd "$SCRIPT_DIR/.." && pwd -P)"
REPO_ROOT="$(git -C "$TRACKER_DIR" rev-parse --show-toplevel)"

DEFAULT_IMAGE="agentic-et1-tracker-aarch64-builder:ubuntu20.04-yaml-cpp0.8.0"
YAML_CPP_ROOT="/opt/agentic-et1/aarch64/yaml-cpp"

usage() {
  cat <<'USAGE'
Usage: build_aarch64_release_in_docker.sh [options]

Builds the aarch64 robot release in the fixed builder image. The host workspace
is cloned into a temporary release workspace; the container does not apt install
packages or rebuild yaml-cpp.

Options:
  --version VERSION        Release version. Default: timestamp-gitsha; build_release.sh appends -aarch64 to the package name.
  --out-dir DIR           Host output dir. Default: packaging/dist. Legacy /work/unitree_rl_lab/... paths are accepted.
  --jobs N                Parallel jobs. Default: nproc in container.
  --image IMAGE           Builder image. Default: agentic-et1-tracker-aarch64-builder:ubuntu20.04-yaml-cpp0.8.0.
  --yaml-cpp-tag TAG      yaml-cpp git tag for --bootstrap-image. Default: 0.8.0.
  --bootstrap-image       Build the builder image first if it is missing.
  --workspace-root DIR    Parent dir for the temporary release workspace. Default: /tmp.
  --workspace DIR         Exact temporary release workspace dir.
  --keep-workspace        Keep the temporary release workspace after build.
  -h, --help              Show help.
USAGE
}

version=""
out_dir="$SCRIPT_DIR/dist"
jobs=""
image="$DEFAULT_IMAGE"
yaml_cpp_tag="0.8.0"
bootstrap_image=0
workspace_root="/tmp"
workspace=""
keep_workspace=0

cleanup_release_workspace() {
  [[ "$keep_workspace" -eq 0 ]] || return 0
  [[ -n "${release_workspace:-}" ]] || return 0
  [[ "$release_workspace" != "/" ]] || return 0
  if [[ -f "$release_workspace/.agentic-et1-aarch64-release-workspace" ]]; then
    rm -rf "$release_workspace"
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --version)
      [[ $# -ge 2 ]] || { printf 'error: --version requires a value\n' >&2; exit 1; }
      version="$2"
      shift 2
      ;;
    --out-dir)
      [[ $# -ge 2 ]] || { printf 'error: --out-dir requires a value\n' >&2; exit 1; }
      out_dir="$2"
      shift 2
      ;;
    --jobs)
      [[ $# -ge 2 ]] || { printf 'error: --jobs requires a value\n' >&2; exit 1; }
      jobs="$2"
      shift 2
      ;;
    --image)
      [[ $# -ge 2 ]] || { printf 'error: --image requires a value\n' >&2; exit 1; }
      image="$2"
      shift 2
      ;;
    --yaml-cpp-tag)
      [[ $# -ge 2 ]] || { printf 'error: --yaml-cpp-tag requires a value\n' >&2; exit 1; }
      yaml_cpp_tag="$2"
      shift 2
      ;;
    --bootstrap-image)
      bootstrap_image=1
      shift
      ;;
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
    --keep-workspace)
      keep_workspace=1
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

if [[ -z "$version" ]]; then
  git_sha="$(git -C "$REPO_ROOT" rev-parse --short HEAD 2>/dev/null || printf 'nogit')"
  version="$(date -u +%Y%m%d%H%M%S)-$git_sha"
fi

docker_cmd=(docker)
if ! docker ps >/dev/null 2>&1; then
  if sudo -n docker ps >/dev/null 2>&1; then
    docker_cmd=(sudo docker)
  else
    printf 'error: docker daemon is not available to this user\n' >&2
    exit 1
  fi
fi

if ! "${docker_cmd[@]}" image inspect "$image" >/dev/null 2>&1; then
  if [[ "$bootstrap_image" -eq 1 ]]; then
    "$SCRIPT_DIR/build_aarch64_builder_image.sh" --image "$image" --yaml-cpp-tag "$yaml_cpp_tag"
  else
    printf 'error: missing builder image: %s\n' "$image" >&2
    printf 'hint: run %s --bootstrap-image or build_aarch64_builder_image.sh\n' "$0" >&2
    exit 1
  fi
fi

prepare_args=(--workspace-root "$workspace_root" --keep-existing)
if [[ -n "$workspace" ]]; then
  prepare_args=(--workspace "$workspace" --keep-existing)
fi
release_workspace="$("$SCRIPT_DIR/prepare_aarch64_release_workspace.sh" "${prepare_args[@]}")"
if [[ "$keep_workspace" -eq 0 ]]; then
  trap cleanup_release_workspace EXIT
else
  printf 'keeping release workspace %s\n' "$release_workspace"
fi

workspace_tracker_dir="$release_workspace/unitree_rl_lab/deploy/robots/agentic-et1-tracker"
[[ -d "$workspace_tracker_dir" ]] || {
  printf 'error: prepared workspace is missing tracker dir: %s\n' "$workspace_tracker_dir" >&2
  exit 1
}
rm -rf "$workspace_tracker_dir/third_party/ruckig"
mkdir -p "$workspace_tracker_dir/third_party" "$workspace_tracker_dir/packaging/scripts"
cp -a "$TRACKER_DIR/third_party/ruckig" "$workspace_tracker_dir/third_party/ruckig"
cp -a "$TRACKER_DIR/third_party/THIRD_PARTY_MANIFEST.yaml" "$workspace_tracker_dir/third_party/THIRD_PARTY_MANIFEST.yaml"
cp -a "$TRACKER_DIR/CMakeLists.txt" "$workspace_tracker_dir/CMakeLists.txt"
cp -a "$SCRIPT_DIR/build_release.sh" "$workspace_tracker_dir/packaging/build_release.sh"
cp -a "$SCRIPT_DIR/README.release.md" "$workspace_tracker_dir/packaging/README.release.md"
cp -a "$SCRIPT_DIR/scripts/selftest.sh" "$workspace_tracker_dir/packaging/scripts/selftest.sh"

if [[ "$out_dir" == /work/unitree_rl_lab/* ]]; then
  out_dir="$REPO_ROOT/${out_dir#/work/unitree_rl_lab/}"
fi
mkdir -p "$out_dir"
host_out_dir="$(cd "$out_dir" && pwd -P)"

container_script="$(mktemp)"
trap 'rm -f "$container_script"; cleanup_release_workspace' EXIT
cat > "$container_script" <<'EOS'
set -euo pipefail

chown_release_out() {
  chown -R "$HOST_UID:$HOST_GID" /release-out || true
}
trap chown_release_out EXIT

export GIT_CONFIG_GLOBAL=/tmp/agentic-et1-gitconfig
git config --global --add safe.directory /work/unitree_rl_lab

jobs="${ET1_JOBS:-$(nproc 2>/dev/null || printf '4')}"
repo="/work/unitree_rl_lab"
toolchain="$repo/deploy/robots/agentic-et1-tracker/packaging/toolchains/aarch64-linux-gnu.cmake"
unitree_sdk2="/work/third_party/unitree_sdk2_install_aarch64"
onnxruntime="$repo/deploy/thirdparty/onnxruntime-linux-aarch64-1.26.0"
skill_dir="$repo/deploy/robots/agentic-et1-tracker/packaging/skills/et1-action"

AGENTIC_ET1_YAML_CPP_AARCH64_ROOT="$ET1_YAML_CPP_ROOT" \
AGENTIC_ET1_UNITREE_SDK2_AARCH64_ROOT="$unitree_sdk2" \
AGENTIC_ET1_ONNXRUNTIME_AARCH64_ROOT="$onnxruntime" \
"$repo/deploy/robots/agentic-et1-tracker/packaging/build_release.sh" \
  --version "$ET1_VERSION" \
  --target-arch aarch64 \
  --cmake-toolchain "$toolchain" \
  --unitree-sdk2-root "$unitree_sdk2" \
  --onnxruntime-root "$onnxruntime" \
  --cmake-prefix-path "$ET1_YAML_CPP_ROOT" \
  --skill-dir "$skill_dir" \
  --out-dir /release-out \
  --build-dir /tmp/agentic-et1-tracker-aarch64-release \
  --jobs "$jobs" \
  --cmake-arg "-Dyaml-cpp_DIR=$ET1_YAML_CPP_ROOT/lib/cmake/yaml-cpp"

pkg="/release-out/agentic-et1-tracker-${ET1_VERSION}-aarch64.tar.gz"
stage="/tmp/agentic-et1-tracker-aarch64-release/stage/agentic-et1-tracker-${ET1_VERSION}-aarch64"
file "$stage/bin/agentic-et1-tracker" "$stage/lib/"*.so* | tee "/release-out/agentic-et1-tracker-${ET1_VERSION}-aarch64.file.txt"
readelf -d "$stage/bin/agentic-et1-tracker" | egrep 'NEEDED|RPATH|RUNPATH' | tee "/release-out/agentic-et1-tracker-${ET1_VERSION}-aarch64.readelf.txt"
(cd /release-out && sha256sum "$(basename "$pkg")" > "$(basename "$pkg").sha256")

chown -R "$HOST_UID:$HOST_GID" /release-out
EOS

"${docker_cmd[@]}" run --rm \
  --network none \
  -e HOST_UID="$(id -u)" \
  -e HOST_GID="$(id -g)" \
  -e ET1_VERSION="$version" \
  -e ET1_JOBS="$jobs" \
  -e ET1_YAML_CPP_ROOT="$YAML_CPP_ROOT" \
  -v "$release_workspace:/work" \
  -v "$host_out_dir:/release-out" \
  -v "$container_script:/tmp/build-aarch64.sh:ro" \
  "$image" \
  bash /tmp/build-aarch64.sh

printf 'aarch64 release written under %s\n' "$host_out_dir"
