#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"

usage() {
  cat <<'USAGE'
Usage: build_aarch64_builder_image.sh [options]

Builds the fixed aarch64 release builder image. The image contains the Ubuntu
20.04 aarch64 cross toolchain and yaml-cpp cross-installed under
/opt/agentic-et1/aarch64/yaml-cpp.

Options:
  --image IMAGE           Image tag. Default: agentic-et1-tracker-aarch64-builder:ubuntu20.04-yaml-cpp0.8.0
  --yaml-cpp-tag TAG      yaml-cpp git tag. Default: 0.8.0.
  --save-output FILE      Save the image with docker save after build.
  -h, --help              Show help.
USAGE
}

image="agentic-et1-tracker-aarch64-builder:ubuntu20.04-yaml-cpp0.8.0"
yaml_cpp_tag="0.8.0"
save_output=""

while [[ $# -gt 0 ]]; do
  case "$1" in
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
    --save-output)
      [[ $# -ge 2 ]] || { printf 'error: --save-output requires a value\n' >&2; exit 1; }
      save_output="$2"
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

docker_cmd=(docker)
docker_uses_sudo=0
if ! docker ps >/dev/null 2>&1; then
  if sudo -n docker ps >/dev/null 2>&1; then
    docker_cmd=(sudo docker)
    docker_uses_sudo=1
  else
    printf 'error: docker daemon is not available to this user\n' >&2
    exit 1
  fi
fi

"${docker_cmd[@]}" build \
  --build-arg "YAML_CPP_TAG=$yaml_cpp_tag" \
  -f "$SCRIPT_DIR/Dockerfile.aarch64-builder" \
  -t "$image" \
  "$SCRIPT_DIR"

if [[ -n "$save_output" ]]; then
  mkdir -p "$(dirname "$save_output")"
  "${docker_cmd[@]}" save "$image" -o "$save_output"
  if [[ "$docker_uses_sudo" -eq 1 && "$(id -u)" -ne 0 ]]; then
    if ! sudo -n chown "$(id -u):$(id -g)" "$save_output"; then
      printf 'error: saved image exists but failed to chown %s to %s:%s\n' \
        "$save_output" "$(id -u)" "$(id -g)" >&2
      exit 1
    fi
  fi
  printf 'saved %s to %s\n' "$image" "$save_output"
fi

printf 'built %s\n' "$image"
