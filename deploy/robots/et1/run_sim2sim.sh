#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "$script_dir/../../.." && pwd)"

mujoco_dir="${UNITREE_MUJOCO_DIR:-$(cd "$repo_dir/.." && pwd)/unitree_mujoco}"
network="lo"
robot="et1_v2"
scene="scene.xml"
sim_control_port="8090"
contact_timeout="8.0"
build_et1=1
extra_ctrl_args=()

usage() {
  cat <<EOF
Usage:
  ./run_sim2sim.sh [options] [-- extra et1_ctrl args]

Options:
  --mujoco-dir PATH       unitree_mujoco repository path
                           default: $mujoco_dir
  --network IFACE         DDS network interface for both processes
                           default: $network
  --robot NAME            unitree_mujoco robot name
                           default: $robot
  --scene FILE            unitree_mujoco scene file
                           default: $scene
  --sim-control-port N    UDP port used by MuJoCo sim control
                           default: $sim_control_port
  --contact-timeout SEC   et1_ctrl --sim-auto contact timeout
                           default: $contact_timeout
  --no-build              do not build et1_ctrl if missing
  -h, --help              show this help

Examples:
  ./run_sim2sim.sh
  ./run_sim2sim.sh --network lo -- --log
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mujoco-dir)
      mujoco_dir="$2"
      shift 2
      ;;
    --network|-n)
      network="$2"
      shift 2
      ;;
    --robot|-r)
      robot="$2"
      shift 2
      ;;
    --scene|-s)
      scene="$2"
      shift 2
      ;;
    --sim-control-port)
      sim_control_port="$2"
      shift 2
      ;;
    --contact-timeout)
      contact_timeout="$2"
      shift 2
      ;;
    --no-build)
      build_et1=0
      shift
      ;;
    --)
      shift
      extra_ctrl_args+=("$@")
      break
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      extra_ctrl_args+=("$1")
      shift
      ;;
  esac
done

mujoco_bin="$mujoco_dir/simulate/build/unitree_mujoco"
et1_bin="$script_dir/build/et1_ctrl"
log_dir="$script_dir/logs"
mkdir -p "$log_dir"

if [[ ! -x "$mujoco_bin" ]]; then
  echo "Missing executable: $mujoco_bin" >&2
  echo "Build unitree_mujoco first, for example:" >&2
  echo "  cmake -S \"$mujoco_dir/simulate\" -B \"$mujoco_dir/simulate/build\"" >&2
  echo "  cmake --build \"$mujoco_dir/simulate/build\" --parallel" >&2
  exit 1
fi

if [[ ! -x "$et1_bin" ]]; then
  if [[ "$build_et1" -eq 1 ]]; then
    "$script_dir/build.sh" --no-clean
  else
    echo "Missing executable: $et1_bin" >&2
    echo "Run \"$script_dir/build.sh\" or omit --no-build." >&2
    exit 1
  fi
fi

timestamp="$(date +%Y%m%d_%H%M%S)"
mujoco_log="$log_dir/unitree_mujoco_${timestamp}.log"
ctrl_log="$log_dir/et1_ctrl_${timestamp}.log"
mujoco_pid=""

cleanup() {
  local status=$?
  if [[ -n "${mujoco_pid:-}" ]] && kill -0 "$mujoco_pid" 2>/dev/null; then
    echo
    echo "Stopping unitree_mujoco (pid $mujoco_pid)..."
    kill "$mujoco_pid" 2>/dev/null || true
    wait "$mujoco_pid" 2>/dev/null || true
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

wait_for_sim_control() {
  local port="$1"
  local pid="$2"
  local timeout_sec=15

  if ! command -v python3 >/dev/null 2>&1; then
    sleep 2
    return 0
  fi

  python3 - "$port" "$pid" "$timeout_sec" <<'PY'
import os
import socket
import sys
import time

port = int(sys.argv[1])
pid = int(sys.argv[2])
deadline = time.monotonic() + float(sys.argv[3])

while time.monotonic() < deadline:
    try:
        os.kill(pid, 0)
    except OSError:
        sys.exit(2)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.25)
    try:
        sock.sendto(b"status", ("127.0.0.1", port))
        data, _ = sock.recvfrom(256)
        if data.startswith(b"ok"):
            sys.exit(0)
    except OSError:
        pass
    finally:
        sock.close()
    time.sleep(0.25)

sys.exit(1)
PY
}

echo "Starting unitree_mujoco..."
echo "  binary:  $mujoco_bin"
echo "  robot:   $robot"
echo "  scene:   $scene"
echo "  network: $network"
echo "  log:     $mujoco_log"

(
  cd "$mujoco_dir/simulate/build"
  exec "$mujoco_bin" \
    --network "$network" \
    --robot "$robot" \
    --scene "$scene" \
    --sim_control_port "$sim_control_port"
) >"$mujoco_log" 2>&1 &
mujoco_pid=$!

if ! wait_for_sim_control "$sim_control_port" "$mujoco_pid"; then
  echo "unitree_mujoco did not expose sim control on 127.0.0.1:$sim_control_port." >&2
  echo "Last MuJoCo log lines:" >&2
  tail -80 "$mujoco_log" >&2 || true
  exit 1
fi

echo "Starting et1_ctrl..."
echo "  binary:  $et1_bin"
echo "  network: $network"
echo "  log:     $ctrl_log"
echo

(
  cd "$script_dir"
  exec "$et1_bin" \
    --network "$network" \
    --sim-auto \
    --sim-auto-port "$sim_control_port" \
    --sim-auto-contact-timeout "$contact_timeout" \
    "${extra_ctrl_args[@]}"
) 2>&1 | tee "$ctrl_log"
