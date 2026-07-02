#!/usr/bin/env python3
"""Manual product E2E and MuJoCo visual gates for agentic-et1-tracker.

This is intentionally outside CTest, release selftest, and packaging gates.
By default it connects to an already running tracker and MuJoCo session.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import signal
import struct
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Callable


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_EXISTING_MOTION_DIR = Path("/home/galbot/works/agent-test/generated")
MAGIC = b"ET1TRK1\0"
FLOAT32 = 1
INT64 = 5
JOINT_DIM = 26
BODY_COUNT = 27
REQUIRED_ARRAYS = [
    ("joint_pos", FLOAT32, (JOINT_DIM,)),
    ("joint_vel", FLOAT32, (JOINT_DIM,)),
    ("body_pos_w", FLOAT32, (BODY_COUNT, 3)),
    ("body_quat_w", FLOAT32, (BODY_COUNT, 4)),
    ("body_lin_vel_w", FLOAT32, (BODY_COUNT, 3)),
    ("body_ang_vel_w", FLOAT32, (BODY_COUNT, 3)),
    ("left_foot_contact_state", INT64, ()),
    ("right_foot_contact_state", INT64, ()),
    ("ref_com_rel_navi", FLOAT32, (3,)),
    ("ref_com_vel_navi", FLOAT32, (3,)),
]


class GateError(RuntimeError):
    pass


class HttpError(GateError):
    def __init__(self, method: str, path: str, status: int, body: Any):
        super().__init__(f"{method} {path} returned HTTP {status}: {body}")
        self.status = status
        self.body = body


def log(message: str) -> None:
    print(message, flush=True)


def fail(message: str) -> None:
    raise GateError(message)


def require(condition: bool, message: str) -> None:
    if not condition:
        fail(message)


def now_stamp() -> str:
    return time.strftime("%Y%m%d-%H%M%S")


def mkdir(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    return path


def http_json(url: str, method: str, path: str, body: Any = None,
              expected: int | tuple[int, ...] = 200) -> Any:
    expected_tuple = (expected,) if isinstance(expected, int) else expected
    data = None
    headers = {}
    if body is not None:
        data = json.dumps(body, separators=(",", ":")).encode("utf-8")
        headers["Content-Type"] = "application/json"
    elif method == "POST":
        data = b""
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url.rstrip("/") + path, data=data, method=method,
                                 headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            payload = resp.read().decode("utf-8")
            parsed = json.loads(payload) if payload else None
            if resp.status not in expected_tuple:
                raise HttpError(method, path, resp.status, parsed)
            return parsed
    except urllib.error.HTTPError as err:
        payload_text = err.read().decode("utf-8", errors="replace")
        try:
            parsed = json.loads(payload_text)
        except json.JSONDecodeError:
            parsed = payload_text
        if err.code not in expected_tuple:
            raise HttpError(method, path, err.code, parsed)
        return parsed
    except urllib.error.URLError as err:
        raise GateError(f"{method} {path} failed to connect to {url}: {err}") from err


def get(url: str, path: str, expected: int | tuple[int, ...] = 200) -> Any:
    return http_json(url, "GET", path, expected=expected)


def post(url: str, path: str, body: Any | None = None,
         expected: int | tuple[int, ...] = 200) -> Any:
    return http_json(url, "POST", path, body=body, expected=expected)


def poll(desc: str, timeout_s: float, fn: Callable[[], Any],
         pred: Callable[[Any], bool], interval_s: float = 0.05) -> Any:
    deadline = time.monotonic() + timeout_s
    latest = None
    while time.monotonic() < deadline:
        latest = fn()
        if pred(latest):
            return latest
        time.sleep(interval_s)
    fail(f"timed out waiting for {desc}; latest={json.dumps(latest, sort_keys=True)}")


def element_count(shape: tuple[int, ...]) -> int:
    count = 1
    for dim in shape:
        count *= dim
    return count


def write_scalar(out, fmt: str, value: Any) -> None:
    out.write(struct.pack(fmt, value))


def write_array(out, name: str, dtype: int, shape: tuple[int, ...],
                frames: int, variant: float) -> None:
    full_shape = (frames,) + shape
    write_scalar(out, "<I", len(name))
    out.write(name.encode("ascii"))
    write_scalar(out, "<I", dtype)
    write_scalar(out, "<I", len(full_shape))
    for dim in full_shape:
        write_scalar(out, "<I", dim)

    count = element_count(full_shape)
    if dtype == FLOAT32:
        write_scalar(out, "<Q", count * 4)
        for i in range(count):
            write_scalar(out, "<f", variant + float(i % 97) * 0.001)
    elif dtype == INT64:
        write_scalar(out, "<Q", count * 8)
        for i in range(count):
            write_scalar(out, "<q", 1 if (i + int(variant * 10)) % 2 == 0 else 2)
    else:
        raise AssertionError(dtype)


def write_trk(path: Path, frames: int, variant: float = 0.0) -> Path:
    with path.open("wb") as out:
        out.write(MAGIC)
        write_scalar(out, "<I", 1)
        write_scalar(out, "<I", len(REQUIRED_ARRAYS))
        for name, dtype, trailing_shape in REQUIRED_ARRAYS:
            if name == "body_quat_w":
                write_scalar(out, "<I", len(name))
                out.write(name.encode("ascii"))
                write_scalar(out, "<I", dtype)
                full_shape = (frames,) + trailing_shape
                write_scalar(out, "<I", len(full_shape))
                for dim in full_shape:
                    write_scalar(out, "<I", dim)
                count = element_count(full_shape)
                write_scalar(out, "<Q", count * 4)
                for frame in range(frames):
                    for _body in range(BODY_COUNT):
                        write_scalar(out, "<f", 1.0)
                        write_scalar(out, "<f", 0.0)
                        write_scalar(out, "<f", 0.0)
                        write_scalar(out, "<f", 0.0)
            else:
                write_array(out, name, dtype, trailing_shape, frames, variant)
    return path


def make_fixtures(motion_dir: Path) -> dict[str, Path]:
    mkdir(motion_dir)
    fixtures = {
        "idle_a": write_trk(motion_dir / "manual_gate_idle_a.trk", 80, 0.1),
        "idle_b": write_trk(motion_dir / "manual_gate_idle_b.trk", 90, 0.2),
        "short": write_trk(motion_dir / "manual_gate_short.trk", 35, 0.3),
        "long_a": write_trk(motion_dir / "manual_gate_long_a.trk", 260, 0.4),
        "long_b": write_trk(motion_dir / "manual_gate_long_b.trk", 220, 0.5),
        "long_c": write_trk(motion_dir / "manual_gate_long_c.trk", 220, 0.6),
        "transition_a": write_trk(motion_dir / "manual_gate_transition_a.trk", 12, 0.7),
        "transition_b": write_trk(motion_dir / "manual_gate_transition_b.trk", 500, 0.8),
        "transition_c": write_trk(motion_dir / "manual_gate_transition_c.trk", 500, 0.9),
        "loco": write_trk(motion_dir / "manual_gate_loco.trk", 60, 1.0),
    }
    return fixtures


def wait_ready(url: str, timeout_s: float = 15.0) -> Any:
    return poll("tracker ready", timeout_s, lambda: get(url, "/status"),
                lambda s: s.get("ready") is True)


def wait_standby(url: str, timeout_s: float = 8.0) -> Any:
    return poll("ctrl=standby_velocity", timeout_s, lambda: get(url, "/status"),
                lambda s: s.get("ctrl") == "standby_velocity" and
                s.get("ready") is True)


def recover_to_standby(url: str, passive_password: str) -> Any:
    status = get(url, "/status")
    if status.get("ctrl") == "passive":
        post(url, "/fixstand")
        poll("ctrl=fixstand after passive", 8, lambda: get(url, "/status"),
             lambda s: s.get("ctrl") == "fixstand")
    if status.get("ctrl") not in ("standby_velocity", "fixstand"):
        post(url, "/stop")
        poll("stable after stop", 8, lambda: get(url, "/status"),
             lambda s: s.get("ctrl") in ("standby_velocity", "fixstand", "passive"))
        status = get(url, "/status")
        if status.get("ctrl") == "passive":
            post(url, "/fixstand")
            poll("ctrl=fixstand after stop/passive", 8, lambda: get(url, "/status"),
                 lambda s: s.get("ctrl") == "fixstand")
    status = get(url, "/status")
    if status.get("ctrl") == "fixstand":
        post(url, "/standby_velocity")
    return wait_standby(url)


def execute(url: str, path: Path, mode: str = "queue", hold: bool = False) -> str:
    body = {"path": str(path), "mode": mode}
    if hold:
        body["hold"] = True
    out = post(url, "/execute", body)
    require(out.get("ok") is True and isinstance(out.get("id"), str),
            f"/execute did not return an id: {out}")
    return out["id"]


def execute_response(url: str,
                     path: Path,
                     mode: str = "queue",
                     hold: bool = False,
                     expected: int | tuple[int, ...] = 200) -> Any:
    body = {"path": str(path), "mode": mode}
    if hold:
        body["hold"] = True
    return post(url, "/execute", body, expected=expected)


def run_status(url: str, run_id: str) -> Any:
    return get(url, f"/status?id={run_id}")


def run_e2e(args: argparse.Namespace, fixtures: dict[str, Path]) -> dict[str, Any]:
    url = args.url
    results: dict[str, Any] = {}
    wait_ready(url, args.ready_timeout)

    log("[e2e] startup_control_recovery")
    post(url, "/fixstand")
    fixstand = poll("ctrl=fixstand", 8, lambda: get(url, "/status"),
                    lambda s: s.get("ctrl") == "fixstand")
    require(fixstand["active"]["kind"] == "none", "fixstand should not expose user active")
    post(url, "/standby_velocity")
    standby = wait_standby(url)
    require(standby["active"]["kind"] == "none", "standby should have no active user")
    results["startup_control_recovery"] = "PASS"

    log("[e2e] normal stop vs passive sink idle semantics")
    post(url, "/idle", {"paths": [str(fixtures["idle_a"])]})
    idle_set = poll("idle enabled", 8, lambda: get(url, "/status"),
                    lambda s: s["idle"]["enabled"] is True and s["idle"]["n"] == 1)
    require(idle_set["ctrl"] in ("standby_velocity", "running"),
            f"unexpected ctrl after idle set: {idle_set['ctrl']}")
    post(url, "/standby_velocity")
    standby_idle = wait_standby(url)
    require(standby_idle["idle"]["enabled"] is True and standby_idle["idle"]["n"] == 1,
            "standby_velocity should retain idle config")
    post(url, "/stop")
    stopped = poll("stop clears idle", 8, lambda: get(url, "/status"),
                   lambda s: s["idle"]["enabled"] is False and s["idle"]["n"] == 0)
    require(stopped["ctrl"] == "standby_velocity",
            f"/stop from standby should stay standby_velocity, got {stopped['ctrl']}")
    recover_to_standby(url, args.passive_password)
    post(url, "/idle", {"paths": [str(fixtures["idle_a"])]})
    poll("idle reset before passive", 8, lambda: get(url, "/status"),
         lambda s: s["idle"]["enabled"] is True)
    post(url, "/passive", {"password": args.passive_password})
    passive = poll("passive clears idle", 8, lambda: get(url, "/status"),
                   lambda s: s["ctrl"] == "passive" and s["idle"]["enabled"] is False)
    require(passive["idle"]["n"] == 0, "passive should clear idle pool")
    post(url, "/fixstand")
    poll("fixstand after passive", 8, lambda: get(url, "/status"),
         lambda s: s["ctrl"] == "fixstand")
    post(url, "/standby_velocity")
    wait_standby(url)
    results["normal_stop_vs_passive_sink"] = "PASS"

    log("[e2e] idle_set_preempt_resume_clear")
    post(url, "/idle", {"paths": [str(fixtures["idle_a"]), str(fixtures["idle_b"])]})
    poll("idle active or configured", 8, lambda: get(url, "/status"),
         lambda s: s["idle"]["enabled"] is True and s["idle"]["n"] == 2)
    user_id = execute(url, fixtures["short"], mode="queue")
    user = poll("user preempts idle", 8, lambda: run_status(url, user_id),
                lambda s: s["state"] in ("running", "done", "holding"))
    require(user["id"] == user_id, "user status id mismatch")
    poll("user terminal", 12, lambda: run_status(url, user_id),
         lambda s: s["state"] in ("done", "stopped", "failed"))
    resumed = poll("idle resumes after user terminal", 10, lambda: get(url, "/status"),
                   lambda s: s["idle"]["enabled"] is True and s["active"]["kind"] == "idle")
    require(resumed["idle"]["active"] is True, "idle should resume active playback")
    post(url, "/idle", {"paths": []})
    cleared = poll("idle clear", 8, lambda: get(url, "/status"),
                   lambda s: s["idle"]["enabled"] is False and s["idle"]["n"] == 0)
    require(cleared["active"]["kind"] in ("none", "transition"),
            f"unexpected active after idle clear: {cleared['active']}")
    recover_to_standby(url, args.passive_password)
    results["idle_set_preempt_resume_clear"] = "PASS"

    log("[e2e] queue_interrupt_status_contract")
    a_id = execute(url, fixtures["long_a"], mode="queue")
    poll("A running", 10, lambda: run_status(url, a_id),
         lambda s: s["state"] in ("running", "holding"))
    b_id = execute(url, fixtures["long_b"], mode="queue")
    b_queued = poll("B queued", 5, lambda: run_status(url, b_id),
                    lambda s: s["state"] == "queued")
    require(b_queued["queue_pos"] is not None, "B should expose queue_pos while queued")
    c_id = execute(url, fixtures["long_c"], mode="interrupt")
    c_active = poll("C active after interrupt", 10, lambda: run_status(url, c_id),
                    lambda s: s["state"] in ("running", "holding", "done"))
    require(c_active["id"] == c_id, "C status id mismatch")
    a_terminal = poll("A stopped by interrupt", 10, lambda: run_status(url, a_id),
                      lambda s: s["state"] == "stopped")
    b_terminal = poll("B canceled by interrupt", 10, lambda: run_status(url, b_id),
                      lambda s: s["state"] == "canceled")
    require(a_terminal["stop_reason"] == "interrupt",
            f"A stop_reason should be interrupt: {a_terminal}")
    require(b_terminal["stop_reason"] == "interrupt",
            f"B stop_reason should be interrupt: {b_terminal}")
    post(url, "/stop")
    wait_standby(url)
    results["queue_interrupt_status_contract"] = {
        "result": "PASS",
        "A": {"id": a_id, "state": a_terminal["state"]},
        "B": {"id": b_id, "state": b_terminal["state"]},
        "C": {"id": c_id, "state": c_active["state"]},
    }

    log("[e2e] transition_interrupt_and_stopping_conflict")
    recover_to_standby(url, args.passive_password)
    ta_id = execute(url, fixtures["transition_a"], mode="queue")
    tb_id = execute(url, fixtures["transition_b"], mode="queue")
    transition = poll("active user transition", args.transition_timeout,
                      lambda: get(url, "/status"),
                      lambda s: s["transition"]["active"] is True and
                      s["transition"]["target"] == "user")
    require(transition["transition"]["target_id"] == tb_id,
            f"transition target should be B id {tb_id}: {transition['transition']}")
    before_interrupt_queue = transition["queue"]
    interrupt_out = execute_response(url, fixtures["transition_c"], mode="interrupt")
    tc_id = interrupt_out["id"]
    require(interrupt_out.get("ok") is True and interrupt_out.get("state") == "queued",
            f"interrupt during transition should be accepted: {interrupt_out}")
    c_running = poll("interrupt target running", args.transition_timeout + 4.0,
                     lambda: run_status(url, tc_id),
                     lambda s: s["state"] in ("running", "holding"))
    after_interrupt_status = get(url, "/status")
    require(after_interrupt_status["queue"]["ids"] == [],
            f"interrupt during transition should not leave queued ids: {after_interrupt_status['queue']}")
    old_target = poll("old transition target terminal", 5,
                      lambda: run_status(url, tb_id),
                      lambda s: s["state"] in ("canceled", "stopped"))
    stopping_queue = after_interrupt_status["queue"]
    post(url, "/standby_velocity")
    stopping = poll("ctrl=stopping after transition standby", args.stopping_timeout,
                    lambda: get(url, "/status"),
                    lambda s: s["ctrl"] == "stopping")
    stopping_queue = stopping["queue"]
    conflict = execute_response(url, fixtures["transition_b"], mode="interrupt", expected=409)
    require(conflict["error"]["code"] == "CONTROL_STATE_CONFLICT",
            f"execute during stopping should return CONTROL_STATE_CONFLICT: {conflict}")
    require(conflict["next"] == "status",
            f"execute during stopping should advise status: {conflict}")
    require("id" not in conflict, f"409 response should not allocate/expose an id: {conflict}")
    after_conflict = get(url, "/status")
    require(after_conflict["queue"] == stopping_queue,
            f"execute during stopping should not change queue: before={stopping_queue} after={after_conflict['queue']}")
    poll("transition A terminal", 10, lambda: run_status(url, ta_id),
         lambda s: s["state"] in ("done", "stopped", "canceled"))
    poll("transition B terminal", 10, lambda: run_status(url, tb_id),
         lambda s: s["state"] in ("stopped", "canceled", "done"))
    poll("transition C terminal", 10, lambda: run_status(url, tc_id),
         lambda s: s["state"] in ("stopped", "canceled", "done"))
    wait_standby(url)
    results["transition_interrupt_and_stopping_conflict"] = {
        "result": "PASS",
        "initial_target": tb_id,
        "interrupt_target": tc_id,
        "pre_interrupt_queue": before_interrupt_queue,
        "stopping_queue": stopping_queue,
    }

    log("[e2e] loco_upper bounded blackbox")
    health = get(url, "/health")
    loco_cap = health.get("cap", {}).get("loco_upper", {})
    if not loco_cap.get("enabled", False):
        if args.require_loco:
            fail("loco_upper is disabled; pass --enable-loco-temp with --start-tracker or use a loco config")
        results["loco_upper_bounded_blackbox"] = "SKIP disabled by config"
    else:
        require(loco_cap.get("ready") is True, f"loco_upper enabled but not ready: {loco_cap}")
        out = post(url, "/execute_loco_upper",
                   {"path": str(fixtures["loco"]), "mode": "queue", "max_radius_m": args.loco_radius})
        loco_id = out["id"]
        status = poll("loco status", 10, lambda: run_status(url, loco_id),
                      lambda s: s.get("executor") == "loco_upper" and "loco" in s)
        loco = status["loco"]
        for field in ("max_radius_m", "distance_m", "phase", "radius_clamped",
                      "radius_limit_reached", "upper_clamped"):
            require(field in loco, f"missing loco status field {field}: {status}")
        post(url, "/stop")
        wait_standby(url)
        results["loco_upper_bounded_blackbox"] = {"result": "PASS", "id": loco_id}

    return results


def parse_png_size(path: Path) -> tuple[int, int] | None:
    with path.open("rb") as f:
        header = f.read(24)
    if len(header) >= 24 and header[:8] == b"\x89PNG\r\n\x1a\n" and header[12:16] == b"IHDR":
        return struct.unpack(">II", header[16:24])
    return None


def parse_jpeg_size(path: Path) -> tuple[int, int] | None:
    data = path.read_bytes()
    if not data.startswith(b"\xff\xd8"):
        return None
    i = 2
    while i + 9 < len(data):
        if data[i] != 0xFF:
            i += 1
            continue
        marker = data[i + 1]
        i += 2
        if marker in (0xD8, 0xD9):
            continue
        if i + 2 > len(data):
            return None
        length = struct.unpack(">H", data[i:i + 2])[0]
        if marker in range(0xC0, 0xC4):
            if i + 7 <= len(data):
                height, width = struct.unpack(">HH", data[i + 3:i + 7])
                return width, height
        i += length
    return None


def image_size(path: Path) -> tuple[int, int] | None:
    return parse_png_size(path) or parse_jpeg_size(path)


def mujoco_processes(pattern: str) -> list[str]:
    try:
        out = subprocess.check_output(["ps", "-eo", "pid=,comm=,args="], text=True)
    except subprocess.CalledProcessError:
        return []
    regex = re.compile(pattern, re.IGNORECASE)
    rows = []
    for line in out.splitlines():
        if regex.search(line) and "manual_gate.py" not in line:
            rows.append(line.strip())
    return rows


def capture_screenshot(path: Path, command_template: str | None) -> None:
    if command_template:
        command = command_template.format(path=str(path))
        subprocess.check_call(command, shell=True)
        return
    if shutil.which("xdotool") and shutil.which("import"):
        search = subprocess.run(["xdotool", "search", "--name", "MuJoCo"],
                                text=True,
                                stdout=subprocess.PIPE,
                                stderr=subprocess.DEVNULL,
                                check=False)
        window_ids = [line.strip() for line in search.stdout.splitlines() if line.strip()]
        if window_ids:
            window_id = window_ids[-1]
            subprocess.run(["xdotool", "windowactivate", "--sync", window_id],
                           stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL,
                           check=False)
            time.sleep(0.5)
            subprocess.check_call(["import", "-window", window_id, str(path)])
            return
    candidates = [
        ("gnome-screenshot", ["gnome-screenshot", "-f", str(path)]),
        ("scrot", ["scrot", str(path)]),
        ("import", ["import", "-window", "root", str(path)]),
    ]
    for name, command in candidates:
        if shutil.which(name):
            subprocess.check_call(command)
            return
    fail("no screenshot command found; pass --screenshot-command '... {path} ...'")


def run_visual(args: argparse.Namespace, fixtures: dict[str, Path]) -> dict[str, Any]:
    url = args.url
    artifacts = mkdir(args.artifacts_dir)
    screenshot = artifacts / f"mujoco_visual_gate_{now_stamp()}.png"
    rows = mujoco_processes(args.mujoco_process_regex)
    if not rows:
        fail(f"no MuJoCo process matched /{args.mujoco_process_regex}/")
    log("[visual] MuJoCo process candidates:")
    for row in rows[:5]:
        log(f"  {row}")

    if args.visual_run_action:
        wait_ready(url, args.ready_timeout)
        recover_to_standby(url, args.passive_password)
        run_id = execute(url, fixtures["short"], mode="queue")
        poll("visual action active/done", 8, lambda: run_status(url, run_id),
             lambda s: s["state"] in ("running", "done", "holding"))
        time.sleep(args.visual_settle_s)

    capture_screenshot(screenshot, args.screenshot_command)
    require(screenshot.exists(), f"screenshot was not created: {screenshot}")
    size_bytes = screenshot.stat().st_size
    dims = image_size(screenshot)
    require(size_bytes >= args.min_screenshot_bytes,
            f"screenshot too small: {size_bytes} bytes")
    require(dims is not None, "screenshot is not a PNG/JPEG with readable dimensions")
    require(dims[0] >= args.min_screenshot_width and dims[1] >= args.min_screenshot_height,
            f"screenshot dimensions too small: {dims}")
    result = {
        "result": "PASS",
        "screenshot": str(screenshot),
        "bytes": size_bytes,
        "dimensions": {"width": dims[0], "height": dims[1]},
        "checklist": [
            "MuJoCo process was present",
            "action was submitted before capture" if args.visual_run_action else "action submission skipped by flag",
            "screenshot file exists",
            "screenshot dimensions and byte size passed minimum checks",
            "operator still must visually inspect posture/no-fall/no-snap",
        ],
    }
    log(f"[visual] screenshot={screenshot} bytes={size_bytes} dimensions={dims[0]}x{dims[1]}")
    return result


def write_temp_config(args: argparse.Namespace, temp_dir: Path, motion_dir: Path) -> Path:
    root = ROOT
    loco = "true" if args.enable_loco_temp else "false"
    text = f"""agentic_et1_tracker:
  bind: "127.0.0.1"
  port: {args.port}
  network: "{args.network}"
  domain_id: {args.domain_id}
  lowcmd_startup_preflight_ms: 200
  release_motion_mode_on_startup: false
  mode_machine: 0
  lock_path: "{temp_dir / 'tracker.lock'}"
  motion_dirs:
    - "{motion_dir}"
  queue_limit: 8
  recent_limit: 64
  hz: {args.temp_hz}
  max_track_duration_s: 120
  stop_hold_s: 0.0
  transition_duration_s: {args.temp_transition_duration_s}
  transition_min_frames: 2
  transition_duration_dt_tolerance_s: 1.0e-9
  user_bridge_reduced_startup_hold_s: 0.10
  transition_root_yaw_residual_limit_rad: 0.05
  transition_contact_guard: "same_nonzero_contact"
  transition_max_velocity: 250.0
  transition_max_acceleration: 10000.0
  transition_max_jerk: 1000000.0
  idle_mode: "hold_current"
  reference:
    enabled: true
  loco_upper:
    enabled: {loco}
    policy_dir: "{root / 'config/policy/loco_lower/et1_low'}"
    policy_file: "policy.onnx"
    deploy: "{root / 'config/policy/loco_lower/et1_low/params/deploy_lowobs10k.yaml'}"
    default_radius_m: 0.8
    max_radius_m: 2.0
    radius_tolerance_m: 0.05
    max_hold_s: 10.0
    strict_pose: false
    limits: "{root / 'config/limits/et1_upper_body/v0/limits.yaml'}"
    joint_map: "{root / 'config/limits/et1_upper_body/v0/joint_map.yaml'}"
  policy:
    profile: "GeneralTrackerDR3"
    policy_dir: "{root / 'config/policy/general_tracker_dr3'}"
    policy_file: "DR3-all.onnx"
    deploy: "{root / 'config/policy/general_tracker_dr3/params/deploy_fut_obs.yaml'}"
    fps: 50
  control:
    startup_control: "FixStand"
    velocity_policy_dir: "{root / 'config/policy/velocity/v0'}"
    velocity_policy_file: "policy.onnx"
    velocity_deploy: "{root / 'config/policy/velocity/v0/params/deploy.yaml'}"
    fixstand_config: "{root / 'config/posture/fixstand/v0/fixstand.yaml'}"
    passive_config: "{root / 'config/posture/passive/v0/passive.yaml'}"
    standby_reference: "{root / 'config/reference/standby/v0/standby_ref.trk'}"
"""
    path = temp_dir / "manual_gate.config.yaml"
    path.write_text(text)
    return path


def start_process(command: list[str], log_path: Path) -> subprocess.Popen:
    log_file = log_path.open("ab")
    proc = subprocess.Popen(command, stdout=log_file, stderr=subprocess.STDOUT,
                            preexec_fn=os.setsid)
    log(f"[process] started pid={proc.pid} command={' '.join(command)} log={log_path}")
    return proc


def stop_process(proc: subprocess.Popen | None, name: str) -> None:
    if proc is None or proc.poll() is not None:
        return
    log(f"[process] stopping {name} pid={proc.pid}")
    os.killpg(proc.pid, signal.SIGTERM)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait(timeout=5)


def choose_motion_dir(args: argparse.Namespace, temp_dir: Path) -> Path:
    if args.motion_dir:
        return mkdir(args.motion_dir.resolve())
    if args.start_tracker:
        return mkdir(temp_dir / "motions")
    if DEFAULT_EXISTING_MOTION_DIR.exists():
        return mkdir(DEFAULT_EXISTING_MOTION_DIR)
    return mkdir(temp_dir / "motions")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Manual agentic-et1-tracker E2E and MuJoCo visual gate")
    parser.add_argument("gate", choices=("all", "e2e", "visual"),
                        help="gate to run; all runs E2E then visual")
    parser.add_argument("--url", default=None,
                        help="tracker URL; default existing http://127.0.0.1:8083 or started port")
    parser.add_argument("--motion-dir", type=Path,
                        help="directory allowed by tracker motion_dirs for generated fixtures")
    parser.add_argument("--artifacts-dir", type=Path,
                        default=Path("/tmp/agentic-et1-manual-gate"),
                        help="artifact directory for screenshots and reports")
    parser.add_argument("--passive-password", default="galaxy")
    parser.add_argument("--ready-timeout", type=float, default=20.0)
    parser.add_argument("--transition-timeout", type=float, default=8.0)
    parser.add_argument("--stopping-timeout", type=float, default=8.0)
    parser.add_argument("--require-loco", action="store_true",
                        help="fail instead of skip when loco_upper is disabled")
    parser.add_argument("--loco-radius", type=float, default=0.8)

    parser.add_argument("--start-tracker", action="store_true",
                        help="start tracker explicitly and stop only that process at exit")
    parser.add_argument("--binary", type=Path, default=ROOT / "build/agentic-et1-tracker")
    parser.add_argument("--config", type=Path,
                        help="exact config to use with --start-tracker; otherwise a temp sim config is generated")
    parser.add_argument("--port", type=int, default=18083,
                        help="port for generated --start-tracker config")
    parser.add_argument("--network", default="lo")
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument("--enable-loco-temp", action="store_true",
                        help="enable loco_upper in the generated temp config")
    parser.add_argument("--temp-transition-duration-s", type=float, default=2.0,
                        help="transition_duration_s used only in generated --start-tracker config")
    parser.add_argument("--temp-hz", type=float, default=10.0,
                        help="runtime hz used only in generated --start-tracker config")

    parser.add_argument("--start-mujoco-cmd",
                        help="optional explicit shell command to start MuJoCo; stopped at exit")
    parser.add_argument("--mujoco-process-regex",
                        default=r"(unitree_mujoco|mujoco|simulate)")
    parser.add_argument("--screenshot-command",
                        help="shell command to capture a screenshot; use {path} for output path")
    parser.add_argument("--visual-run-action", action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument("--visual-settle-s", type=float, default=1.0)
    parser.add_argument("--min-screenshot-bytes", type=int, default=20_000)
    parser.add_argument("--min-screenshot-width", type=int, default=640)
    parser.add_argument("--min-screenshot-height", type=int, default=360)
    args = parser.parse_args()
    if args.url is None:
        args.url = f"http://127.0.0.1:{args.port}" if args.start_tracker else "http://127.0.0.1:8083"
    return args


def main() -> int:
    args = parse_args()
    tracker_proc = None
    mujoco_proc = None
    temp_root_obj = tempfile.TemporaryDirectory(prefix="agentic-et1-manual-gate-")
    temp_root = Path(temp_root_obj.name)
    report: dict[str, Any] = {"url": args.url, "gate": args.gate, "started": {}}
    try:
        motion_dir = choose_motion_dir(args, temp_root)
        fixtures = make_fixtures(motion_dir)
        report["motion_dir"] = str(motion_dir)

        if args.start_mujoco_cmd:
            mujoco_proc = start_process(["bash", "-lc", args.start_mujoco_cmd],
                                        mkdir(args.artifacts_dir) / f"mujoco_{now_stamp()}.log")
            report["started"]["mujoco_pid"] = mujoco_proc.pid
            time.sleep(2.0)

        if args.start_tracker:
            binary = args.binary.resolve()
            require(binary.exists() and os.access(binary, os.X_OK),
                    f"tracker binary is not executable: {binary}")
            config = args.config.resolve() if args.config else write_temp_config(args, temp_root, motion_dir)
            tracker_proc = start_process([str(binary), "--config", str(config)],
                                         mkdir(args.artifacts_dir) / f"tracker_{now_stamp()}.log")
            report["started"]["tracker_pid"] = tracker_proc.pid
            report["started"]["tracker_config"] = str(config)
            time.sleep(0.5)

        if args.gate in ("all", "e2e"):
            report["e2e"] = run_e2e(args, fixtures)
        if args.gate in ("all", "visual"):
            report["visual"] = run_visual(args, fixtures)

        report_path = mkdir(args.artifacts_dir) / f"manual_gate_report_{now_stamp()}.json"
        report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
        log(f"[result] PASS report={report_path}")
        return 0
    except GateError as err:
        report["error"] = str(err)
        report_path = mkdir(args.artifacts_dir) / f"manual_gate_report_{now_stamp()}_FAILED.json"
        report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
        print(f"[result] FAIL {err}", file=sys.stderr)
        print(f"[result] report={report_path}", file=sys.stderr)
        return 1
    finally:
        stop_process(tracker_proc, "tracker")
        stop_process(mujoco_proc, "MuJoCo")
        temp_root_obj.cleanup()


if __name__ == "__main__":
    sys.exit(main())
