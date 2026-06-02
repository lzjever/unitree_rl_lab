#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
TRACKER_DIR="$(cd "$SCRIPT_DIR/.." && pwd -P)"
REPO_ROOT="$(git -C "$TRACKER_DIR" rev-parse --show-toplevel)"
WORK_ROOT="$(cd "$REPO_ROOT/.." && pwd -P)"

usage() {
  cat <<'USAGE'
Usage: build_aarch64_release_in_docker.sh [options]

Builds the aarch64 robot release inside a temporary Ubuntu 20.04 container.
The host is not apt-modified; outputs are written under this workspace.

Options:
  --version VERSION        Release version. Default: timestamp-gitsha-aarch64.
  --out-dir DIR           Output dir. Default: packaging/dist.
  --jobs N                Parallel jobs. Default: nproc in container.
  --image IMAGE           Builder base image. Default: ubuntu:20.04.
  --yaml-cpp-tag TAG      yaml-cpp git tag. Default: 0.8.0.
  -h, --help              Show help.
USAGE
}

version=""
out_dir="/work/unitree_rl_lab/deploy/robots/agentic-et1-tracker/packaging/dist"
jobs=""
image="ubuntu:20.04"
yaml_cpp_tag="0.8.0"

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
  version="$(date -u +%Y%m%d%H%M%S)-$git_sha-aarch64"
fi

docker_cmd=(docker)
if ! docker ps >/dev/null 2>&1; then
  if sudo -n docker ps >/dev/null 2>&1; then
    docker_cmd=(sudo docker)
  else
    printf 'error: docker daemon is not available to this user; run with Docker access or rootless Podman\n' >&2
    exit 1
  fi
fi

container_script="$(mktemp)"
trap 'rm -f "$container_script"' EXIT
cat > "$container_script" <<'EOS'
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends \
  ca-certificates \
  cmake \
  file \
  g++ \
  gcc \
  g++-aarch64-linux-gnu \
  gcc-aarch64-linux-gnu \
  git \
  libc6-dev-arm64-cross \
  make \
  xz-utils

export GIT_CONFIG_GLOBAL=/tmp/agentic-et1-gitconfig
git config --global --add safe.directory /work/unitree_rl_lab

jobs="${ET1_JOBS:-$(nproc 2>/dev/null || printf '4')}"
yaml_src="/work/third_party/yaml-cpp-src-${ET1_YAML_CPP_TAG}"
yaml_install="/work/third_party/yaml-cpp_install_aarch64"
toolchain="/work/unitree_rl_lab/deploy/robots/agentic-et1-tracker/packaging/toolchains/aarch64-linux-gnu.cmake"

if [[ ! -d "$yaml_src/.git" ]]; then
  rm -rf "$yaml_src"
  git clone --depth 1 --branch "$ET1_YAML_CPP_TAG" https://github.com/jbeder/yaml-cpp.git "$yaml_src"
fi

rm -rf /tmp/yaml-cpp-aarch64-build
cmake -S "$yaml_src" -B /tmp/yaml-cpp-aarch64-build \
  -DCMAKE_TOOLCHAIN_FILE="$toolchain" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$yaml_install" \
  -DYAML_CPP_BUILD_TESTS=OFF \
  -DYAML_CPP_BUILD_TOOLS=OFF \
  -DYAML_BUILD_SHARED_LIBS=ON
cmake --build /tmp/yaml-cpp-aarch64-build --parallel "$jobs"
cmake --install /tmp/yaml-cpp-aarch64-build

/work/unitree_rl_lab/deploy/robots/agentic-et1-tracker/packaging/build_release.sh \
  --version "$ET1_VERSION" \
  --target-arch aarch64 \
  --cmake-toolchain "$toolchain" \
  --unitree-sdk2-root /work/third_party/unitree_sdk2_install_aarch64 \
  --onnxruntime-root /work/unitree_rl_lab/deploy/thirdparty/onnxruntime-linux-aarch64-1.26.0 \
  --cmake-prefix-path "$yaml_install" \
  --skill-dir /work/unitree_rl_lab/deploy/robots/agentic-et1-tracker/packaging/skills/et1-trk2motion \
  --out-dir "$ET1_OUT_DIR" \
  --build-dir /tmp/agentic-et1-tracker-aarch64-release \
  --jobs "$jobs"

pkg="$ET1_OUT_DIR/agentic-et1-tracker-${ET1_VERSION}-aarch64.tar.gz"
stage="/tmp/agentic-et1-tracker-aarch64-release/stage/agentic-et1-tracker-${ET1_VERSION}-aarch64"
file "$stage/bin/agentic-et1-tracker" "$stage/lib/"*.so* | tee "$ET1_OUT_DIR/agentic-et1-tracker-${ET1_VERSION}-aarch64.file.txt"
readelf -d "$stage/bin/agentic-et1-tracker" | egrep 'NEEDED|RPATH|RUNPATH' | tee "$ET1_OUT_DIR/agentic-et1-tracker-${ET1_VERSION}-aarch64.readelf.txt"
(cd "$ET1_OUT_DIR" && sha256sum "$(basename "$pkg")" > "$(basename "$pkg").sha256")

chown -R "$HOST_UID:$HOST_GID" \
  /work/third_party/yaml-cpp-src-"$ET1_YAML_CPP_TAG" \
  /work/third_party/yaml-cpp_install_aarch64 \
  "$ET1_OUT_DIR"
EOS

"${docker_cmd[@]}" run --rm \
  -e HOST_UID="$(id -u)" \
  -e HOST_GID="$(id -g)" \
  -e ET1_VERSION="$version" \
  -e ET1_OUT_DIR="$out_dir" \
  -e ET1_JOBS="$jobs" \
  -e ET1_YAML_CPP_TAG="$yaml_cpp_tag" \
  -v "$WORK_ROOT:/work" \
  -v "$container_script:/tmp/build-aarch64.sh:ro" \
  "$image" \
  bash /tmp/build-aarch64.sh

host_out_dir="$out_dir"
if [[ "$host_out_dir" == /work/* ]]; then
  host_out_dir="$WORK_ROOT/${host_out_dir#/work/}"
fi
printf 'aarch64 release written under %s\n' "$host_out_dir"
