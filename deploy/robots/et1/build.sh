#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<EOF
Usage:
  ./build.sh [x86|arm64] [--no-clean] [--jobs N]

Examples:
  ./build.sh
  ./build.sh x86
  ./build.sh arm64
  ./build.sh arm64 --no-clean
  ./build.sh arm64 --jobs 4
EOF
}

target=""
clean=1
jobs="${BUILD_JOBS:-8}"

while [[ $# -gt 0 ]]; do
  arg="$1"
  case "$arg" in
    x86|x86_64)
      target="x86-release"
      shift
      ;;
    arm64|aarch64|arm)
      target="arm64-release"
      shift
      ;;
    --clean|clean)
      clean=1
      shift
      ;;
    --no-clean|no-clean)
      clean=0
      shift
      ;;
    -j|--jobs)
      if [[ $# -lt 2 || ! "$2" =~ ^[0-9]+$ || "$2" -lt 1 ]]; then
        echo "Invalid jobs value. Usage: --jobs N" >&2
        exit 1
      fi
      jobs="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ -z "$target" ]]; then
  machine="$(uname -m)"
  case "$machine" in
    x86_64|amd64)
      target="x86"
      ;;
    aarch64|arm64)
      target="arm64"
      ;;
    *)
      echo "Unsupported machine architecture: $machine" >&2
      echo "Please specify x86 or arm64 explicitly." >&2
      exit 1
      ;;
  esac
fi

case "$target" in
  x86-release|x86)
    target="x86-release"
    onnxruntime_root="$SCRIPT_DIR/../../thirdparty/onnxruntime-linux-x64-1.22.0"
    ;;
  arm64-release|arm64)
    target="arm64-release"
    onnxruntime_root="$SCRIPT_DIR/../../thirdparty/onnxruntime-linux-aarch64-1.26.0"
    ;;
  *)
    echo "Unsupported build target: $target" >&2
    exit 1
    ;;
esac

build_dir="$SCRIPT_DIR/build"

if [[ "$clean" -eq 1 ]]; then
  rm -rf "$build_dir"
fi

cmake \
  -S "$SCRIPT_DIR" \
  -B "$build_dir" \
  -DCMAKE_BUILD_TYPE=Release \
  -DONNXRUNTIME_ROOT="$onnxruntime_root"

cmake --build "$build_dir" --parallel "$jobs"

echo "Built: $build_dir/et1_ctrl"
