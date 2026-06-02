#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
TRACKER_DIR="$(cd "$SCRIPT_DIR/.." && pwd -P)"
REPO_ROOT="$(git -C "$TRACKER_DIR" rev-parse --show-toplevel)"

usage() {
  cat <<'USAGE'
Usage: build_release.sh [options]

Options:
  --version VERSION             Release version. Defaults to timestamp-gitsha.
  --target-arch ARCH            Target arch, default: host arch.
  --build-dir DIR               CMake/staging work dir, default under /tmp.
  --out-dir DIR                 Output dir, default: packaging/dist.
  --build-type TYPE             CMake build type, default: Release.
  --jobs N                      Parallel build jobs, default: nproc.
  --cmake-toolchain FILE        CMake toolchain file for cross builds.
  --cmake-prefix-path PATH      Extra CMAKE_PREFIX_PATH.
  --onnxruntime-root DIR        ONNX Runtime root.
  --unitree-sdk2-root DIR       Unitree SDK2 install root.
  --skill-dir DIR               et1-trk2motion skill dir.
  --cmake-arg ARG               Extra CMake argument, repeatable.
  -h, --help                    Show help.
USAGE
}

host_arch="$(uname -m)"
target_arch="$host_arch"
version=""
build_dir=""
out_dir="$SCRIPT_DIR/dist"
build_type="Release"
jobs="$(nproc 2>/dev/null || printf '4')"
cmake_toolchain=""
cmake_prefix_path=""
onnxruntime_root=""
unitree_sdk2_root=""
skill_dir="${ET1_TRK2MOTION_SKILL_DIR:-$SCRIPT_DIR/skills/et1-trk2motion}"
if [[ ! -d "$skill_dir" && -d /home/galbot/.agents/skills/et1-trk2motion ]]; then
  skill_dir="/home/galbot/.agents/skills/et1-trk2motion"
fi
extra_cmake_args=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --version)
      [[ $# -ge 2 ]] || { printf 'error: --version requires a value\n' >&2; exit 1; }
      version="$2"
      shift 2
      ;;
    --target-arch)
      [[ $# -ge 2 ]] || { printf 'error: --target-arch requires a value\n' >&2; exit 1; }
      target_arch="$2"
      shift 2
      ;;
    --build-dir)
      [[ $# -ge 2 ]] || { printf 'error: --build-dir requires a value\n' >&2; exit 1; }
      build_dir="$2"
      shift 2
      ;;
    --out-dir)
      [[ $# -ge 2 ]] || { printf 'error: --out-dir requires a value\n' >&2; exit 1; }
      out_dir="$2"
      shift 2
      ;;
    --build-type)
      [[ $# -ge 2 ]] || { printf 'error: --build-type requires a value\n' >&2; exit 1; }
      build_type="$2"
      shift 2
      ;;
    --jobs)
      [[ $# -ge 2 ]] || { printf 'error: --jobs requires a value\n' >&2; exit 1; }
      jobs="$2"
      shift 2
      ;;
    --cmake-toolchain)
      [[ $# -ge 2 ]] || { printf 'error: --cmake-toolchain requires a value\n' >&2; exit 1; }
      cmake_toolchain="$2"
      shift 2
      ;;
    --cmake-prefix-path)
      [[ $# -ge 2 ]] || { printf 'error: --cmake-prefix-path requires a value\n' >&2; exit 1; }
      cmake_prefix_path="$2"
      shift 2
      ;;
    --onnxruntime-root)
      [[ $# -ge 2 ]] || { printf 'error: --onnxruntime-root requires a value\n' >&2; exit 1; }
      onnxruntime_root="$2"
      shift 2
      ;;
    --unitree-sdk2-root)
      [[ $# -ge 2 ]] || { printf 'error: --unitree-sdk2-root requires a value\n' >&2; exit 1; }
      unitree_sdk2_root="$2"
      shift 2
      ;;
    --skill-dir)
      [[ $# -ge 2 ]] || { printf 'error: --skill-dir requires a value\n' >&2; exit 1; }
      skill_dir="$2"
      shift 2
      ;;
    --cmake-arg)
      [[ $# -ge 2 ]] || { printf 'error: --cmake-arg requires a value\n' >&2; exit 1; }
      extra_cmake_args+=("$2")
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

case "$target_arch" in
  amd64) target_arch="x86_64" ;;
  arm64) target_arch="aarch64" ;;
esac

if [[ -z "$version" ]]; then
  git_sha="$(git -C "$REPO_ROOT" rev-parse --short HEAD 2>/dev/null || printf 'nogit')"
  version="$(date -u +%Y%m%d%H%M%S)-$git_sha"
fi

if [[ "$target_arch" != "$host_arch" && -z "$cmake_toolchain" ]]; then
  printf 'error: target arch %s differs from host %s; pass --cmake-toolchain\n' \
    "$target_arch" "$host_arch" >&2
  exit 1
fi

if [[ -z "$build_dir" ]]; then
  build_dir="/tmp/agentic-et1-tracker-release-$version-$target_arch"
fi

if [[ -z "$onnxruntime_root" ]]; then
  case "$target_arch" in
    x86_64)
      onnxruntime_root="$REPO_ROOT/deploy/thirdparty/onnxruntime-linux-x64-1.22.0"
      ;;
    aarch64)
      onnxruntime_root="$REPO_ROOT/deploy/thirdparty/onnxruntime-linux-aarch64-1.26.0"
      ;;
  esac
fi

if [[ -z "$unitree_sdk2_root" ]]; then
  candidate="$(cd "$TRACKER_DIR/../../../.." && pwd -P)/third_party/unitree_sdk2_install"
  if [[ -d "$candidate" ]]; then
    unitree_sdk2_root="$candidate"
  fi
fi

effective_cmake_prefix_path="$cmake_prefix_path"
if [[ -n "$unitree_sdk2_root" ]]; then
  if [[ -n "$effective_cmake_prefix_path" ]]; then
    effective_cmake_prefix_path="$unitree_sdk2_root;$effective_cmake_prefix_path"
  else
    effective_cmake_prefix_path="$unitree_sdk2_root"
  fi
fi

[[ -d "$onnxruntime_root" ]] || { printf 'error: missing ONNX Runtime root: %s\n' "$onnxruntime_root" >&2; exit 1; }
[[ -d "$skill_dir" ]] || { printf 'error: missing skill dir: %s\n' "$skill_dir" >&2; exit 1; }

cmake_build="$build_dir/cmake"
stage_dir="$build_dir/stage"
package_name="agentic-et1-tracker-$version-$target_arch"
package_root="$stage_dir/$package_name"
tarball="$out_dir/$package_name.tar.gz"

rm -rf "$stage_dir"
mkdir -p "$cmake_build" "$package_root" "$out_dir"

cmake_args=(
  -S "$TRACKER_DIR"
  -B "$cmake_build"
  -DCMAKE_BUILD_TYPE="$build_type"
  -DAGENTIC_ET1_BUILD_TESTS=OFF
  -DAGENTIC_ET1_BUILD_ONNX=ON
  -DAGENTIC_ET1_BUILD_ROBOT=ON
  -DONNXRUNTIME_ROOT="$onnxruntime_root"
  -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON
  "-DCMAKE_INSTALL_RPATH=\$ORIGIN/../lib"
)

if [[ -n "$cmake_toolchain" ]]; then
  cmake_args+=("-DCMAKE_TOOLCHAIN_FILE=$cmake_toolchain")
fi
if [[ -n "$effective_cmake_prefix_path" ]]; then
  cmake_args+=("-DCMAKE_PREFIX_PATH=$effective_cmake_prefix_path")
fi
cmake_args+=("${extra_cmake_args[@]}")

printf 'configuring release build in %s\n' "$cmake_build"
if [[ -n "$unitree_sdk2_root" ]]; then
  UNITREE_SDK2_PATH="$unitree_sdk2_root" cmake "${cmake_args[@]}"
else
  cmake "${cmake_args[@]}"
fi

printf 'building agentic-et1-tracker\n'
cmake --build "$cmake_build" --target agentic-et1-tracker --parallel "$jobs"

binary="$cmake_build/agentic-et1-tracker"
[[ -x "$binary" ]] || { printf 'error: missing built binary: %s\n' "$binary" >&2; exit 1; }

install -d \
  "$package_root/bin" \
  "$package_root/lib" \
  "$package_root/config" \
  "$package_root/scripts" \
  "$package_root/share/agentic-et1-tracker" \
  "$package_root/skills"

install -m 0755 "$binary" "$package_root/bin/agentic-et1-tracker"
install -m 0644 "$SCRIPT_DIR/config.robot.yaml.template" "$package_root/config/config.robot.yaml.template"
install -m 0644 "$SCRIPT_DIR/README.release.md" "$package_root/README.release.md"
printf '%s\n' "$version" > "$package_root/VERSION"

cp -a "$TRACKER_DIR/config" "$package_root/share/agentic-et1-tracker/"
cp -a "$SCRIPT_DIR/scripts/." "$package_root/scripts/"
cp -a "$skill_dir" "$package_root/skills/et1-trk2motion"
chmod +x "$package_root"/scripts/*.sh
chmod +x "$package_root/skills/et1-trk2motion/scripts/et1-trk2motion"

make_bin_wrapper() {
  local name="$1"
  local target="$2"
  cat > "$package_root/bin/$name" <<EOF
#!/usr/bin/env bash
set -euo pipefail
release_dir="\$(cd "\$(dirname "\${BASH_SOURCE[0]}")/.." && pwd -P)"
exec "\$release_dir/$target" "\$@"
EOF
  chmod 0755 "$package_root/bin/$name"
}

make_bin_wrapper start-tracker scripts/start.sh
make_bin_wrapper stop-tracker scripts/stop.sh
make_bin_wrapper status-tracker scripts/status.sh
make_bin_wrapper selftest scripts/selftest.sh
make_bin_wrapper install-release scripts/install.sh
make_bin_wrapper et1-trk2motion skills/et1-trk2motion/scripts/et1-trk2motion

copy_shared_libs_from_dir() {
  local dir="$1"
  [[ -d "$dir" ]] || return 0
  find "$dir" -maxdepth 1 \( -type f -o -type l \) \( -name '*.so' -o -name '*.so.*' \) -print0 |
    while IFS= read -r -d '' lib; do
      cp -a "$lib" "$package_root/lib/"
    done
}

copy_shared_libs_from_dir "$onnxruntime_root/lib"
copy_shared_libs_from_dir "$onnxruntime_root/lib64"
if [[ -n "$unitree_sdk2_root" ]]; then
  copy_shared_libs_from_dir "$unitree_sdk2_root/lib"
  copy_shared_libs_from_dir "$unitree_sdk2_root/lib/$target_arch"
fi
if [[ -n "$cmake_prefix_path" ]]; then
  IFS=';' read -r -a cmake_prefix_entries <<< "$cmake_prefix_path"
  for prefix_entry in "${cmake_prefix_entries[@]}"; do
    [[ -n "$prefix_entry" ]] || continue
    copy_shared_libs_from_dir "$prefix_entry/lib"
    copy_shared_libs_from_dir "$prefix_entry/lib64"
    copy_shared_libs_from_dir "$prefix_entry/lib/$target_arch"
    copy_shared_libs_from_dir "$prefix_entry/lib/aarch64-linux-gnu"
    copy_shared_libs_from_dir "$prefix_entry/lib/x86_64-linux-gnu"
  done
fi

if [[ "$target_arch" == "$host_arch" ]] && command -v ldd >/dev/null 2>&1; then
  ldd "$package_root/bin/agentic-et1-tracker" |
    awk '/=> \// {print $3}' |
    while IFS= read -r lib; do
      name="$(basename "$lib")"
      case "$name" in
        ld-linux*|libc.so*|libm.so*|libpthread.so*|librt.so*|libdl.so*|libgcc_s.so*|libstdc++.so*)
          continue
          ;;
      esac
      [[ -e "$package_root/lib/$name" ]] && continue
      cp -L "$lib" "$package_root/lib/$name"
    done
fi

if command -v readelf >/dev/null 2>&1; then
  if ! readelf -d "$package_root/bin/agentic-et1-tracker" | grep -q '\$ORIGIN/../lib'; then
    printf 'warning: binary RUNPATH does not show $ORIGIN/../lib\n' >&2
  fi
fi

(cd "$package_root" &&
  find . -type f ! -name manifest.sha256 -print0 |
  sort -z |
  xargs -0 sha256sum > manifest.sha256)

rm -f "$tarball"
(cd "$stage_dir" && tar -czf "$tarball" "$package_name")

printf 'wrote %s\n' "$tarball"
