#!/usr/bin/env python3
"""Manual product E2E and MuJoCo visual gates for agentic-et1-tracker.

This is intentionally outside CTest, release selftest, and packaging gates.
By default it connects to an already running tracker and MuJoCo session.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import shutil
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Callable, NamedTuple


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_EXISTING_MOTION_DIR = Path("/home/galbot/works/agent-test/generated")
E2E_SAFE_SOURCE_TRK = ROOT / "config/reference/standby/v0/standby_ref.trk"
MAGIC = b"ET1TRK1\0"
FLOAT32 = 1
FLOAT64 = 2
BOOL = 3
INT32 = 4
INT64 = 5
UINT8 = 6
INT8 = 7
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
TRK_DTYPE_SIZES = {
    FLOAT32: 4,
    FLOAT64: 8,
    BOOL: 1,
    INT32: 4,
    INT64: 8,
    UINT8: 1,
    INT8: 1,
}
FLOAT_DTYPES = (FLOAT32, FLOAT64)
CONTACT_DTYPES = (INT32, INT64, UINT8, INT8)
FIXTURE_KEYS = (
    "idle_a",
    "idle_b",
    "short",
    "long_a",
    "long_b",
    "long_c",
    "transition_a",
    "transition_b",
    "transition_c",
    "loco",
)
E2E_SAFE_FIXTURE_FRAMES = {
    "idle_a": 50,
    "idle_b": 50,
    "short": 35,
    "long_a": 80,
    "long_b": 80,
    "long_c": 80,
    "transition_a": 80,
    "transition_b": 80,
    "transition_c": 80,
    "loco": 35,
}
E2E_SAFE_FIXTURE_MAX_FRAMES = max(E2E_SAFE_FIXTURE_FRAMES.values())
NAMED_MANUAL_GATE_FIXTURES = {
    f"manual_gate_e2e_safe_{key}.trk": key for key in FIXTURE_KEYS
}
LEGACY_MANUAL_GATE_FIXTURES = {
    f"manual_gate_{key}.trk": key for key in FIXTURE_KEYS
}
PHYSICAL_SAFE_FIXTURE_SOURCES = ("existing", "e2e_safe")
STANDBY_CTRLS = ("standby", "standby_velocity")
UNSTABLE_CTRL_STATES = ("passive", "fault", "stopping", "urgent_stopping")
USER_PREEMPTED_IDLE_RUN_STATES = ("running", "done", "holding")
TERMINAL_RUN_STATES = ("done", "stopped", "failed", "canceled")
FAILED_TERMINAL_RUN_STATES = ("failed", "canceled")
HOLD_SETTLE_S = 0.25
VISUAL_CAMERA_TRACK_BODY = "pelvis_link"


class TrkCandidate(NamedTuple):
    path: Path
    frames: int
    mtime_ns: int


class TrkArray(NamedTuple):
    name: str
    dtype: int
    shape: tuple[int, ...]
    data: bytes


class GateError(RuntimeError):
    def __init__(self, message: str, report: dict[str, Any] | None = None):
        super().__init__(message)
        self.report = report


class HttpError(GateError):
    def __init__(self, method: str, path: str, status: int, body: Any):
        super().__init__(f"{method} {path} returned HTTP {status}: {body}")
        self.status = status
        self.body = body


def log(message: str) -> None:
    print(message, flush=True)


def fail(message: str, report: dict[str, Any] | None = None) -> None:
    raise GateError(message, report)


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


def parse_sim_control_status(raw: str) -> dict[str, Any]:
    parsed: dict[str, Any] = {"available": True, "raw": raw, "ok": raw.startswith("ok")}
    for token in raw.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        if key in ("left_contact", "right_contact", "both"):
            normalized = value.lower()
            if normalized in ("0", "1"):
                parsed[key] = normalized == "1"
                continue
            if normalized in ("true", "false"):
                parsed[key] = normalized == "true"
                continue
        if key == "camera_trackbodyid":
            try:
                parsed[key] = int(value)
            except ValueError:
                parsed[key] = value
            continue
        try:
            parsed[key] = float(value)
        except ValueError:
            parsed[key] = value
    return parsed


def sim_control_command(port: int, timeout_ms: int, command: str) -> dict[str, Any]:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout_ms / 1000.0)
    try:
        sock.sendto(command.encode("ascii"), ("127.0.0.1", port))
        data, _ = sock.recvfrom(1024)
        return parse_sim_control_status(data.decode("utf-8", errors="replace"))
    except OSError as err:
        return {"available": False, "error": str(err)}
    finally:
        sock.close()


def sim_control_status(port: int, timeout_ms: int) -> dict[str, Any]:
    return sim_control_command(port, timeout_ms, "status")


def sim_control_camera_align(port: int, timeout_ms: int) -> dict[str, Any]:
    return sim_control_command(port, timeout_ms, "camera_align")


def sim_control_compact(sim_status: dict[str, Any]) -> dict[str, Any]:
    keys = (
        "available",
        "ok",
        "left_contact",
        "right_contact",
        "both",
        "root_z",
        "raw",
        "error",
    )
    return {key: sim_status.get(key) for key in keys if key in sim_status}


def compact_status(status: Any) -> Any:
    if not isinstance(status, dict):
        return status
    keys = ("ok", "ready", "ctrl", "robot", "block", "err", "active", "queue",
            "transition", "idle", "pose")
    return {key: status.get(key) for key in keys if key in status}


def is_user_transition_to(status: Any, target_id: str) -> bool:
    if not isinstance(status, dict):
        return False
    transition = status.get("transition")
    if not isinstance(transition, dict):
        return False
    return (transition.get("active") is True and
            transition.get("target") == "user" and
            transition.get("target_id") == target_id)


def user_preempted_idle(run_status: Any, status: Any, target_id: str) -> bool:
    if not isinstance(run_status, dict) or run_status.get("id") != target_id:
        return False
    state = run_status.get("state")
    if state in USER_PREEMPTED_IDLE_RUN_STATES:
        return True
    if state != "queued" or not is_user_transition_to(status, target_id):
        return False
    transition = status.get("transition") if isinstance(status, dict) else None
    return isinstance(transition, dict) and transition.get("target_state") == "queued"


def active_kind(status: dict[str, Any]) -> str | None:
    active = status.get("active")
    if isinstance(active, dict):
        return active.get("kind")
    return None


def is_standby_ctrl(status: dict[str, Any]) -> bool:
    return status.get("ctrl") in STANDBY_CTRLS


def is_idle_background(status: dict[str, Any]) -> bool:
    idle = status.get("idle")
    if not isinstance(idle, dict):
        return False
    return idle.get("enabled") is True and idle.get("active") is True and active_kind(status) == "idle"


def is_standby_or_idle_background(status: dict[str, Any]) -> bool:
    return (is_standby_ctrl(status) and active_kind(status) == "none") or is_idle_background(status)


def generic_stopping_reason(status: dict[str, Any], label: str) -> str | None:
    if status.get("ctrl") == "stopping":
        return f"{label}: ctrl is generic stopping; {compact_status_line(status)}"
    if status.get("state") == "stopping":
        return f"{label}: state is generic stopping; {compact_status_line(status)}"
    exec_state = None
    exec_status = status.get("exec")
    if isinstance(exec_status, dict):
        exec_state = exec_status.get("state")
    if exec_state == "stopping":
        return f"{label}: exec.state is generic stopping; {compact_status_line(status)}"
    return None


def idle_config_reason(status: dict[str, Any],
                       label: str,
                       *,
                       enabled: bool,
                       n: int | None = None) -> str | None:
    idle = status.get("idle")
    if not isinstance(idle, dict):
        return f"{label}: idle status is missing; {compact_status_line(status)}"
    if idle.get("enabled") is not enabled:
        return f"{label}: idle.enabled is not {enabled}; {compact_status_line(status)}"
    if n is not None and idle.get("n") != n:
        return f"{label}: idle.n is not {n}; {compact_status_line(status)}"
    return None


def compact_status_line(status: dict[str, Any]) -> str:
    queue = status.get("queue") or {}
    active = status.get("active") or {}
    err = status.get("err")
    err_code = err.get("code") if isinstance(err, dict) else err
    pose = status.get("pose") or {}
    p = pose.get("p") if isinstance(pose, dict) else None
    root_z = p[2] if isinstance(p, list) and len(p) >= 3 else None
    return (
        f"ready={status.get('ready')} ctrl={status.get('ctrl')} "
        f"robot={status.get('robot')} block={status.get('block')} err={err_code} "
        f"active={active.get('kind')} queue_n={queue.get('n')} root_z={root_z}"
    )


def status_clear_reason(status: dict[str, Any],
                        *,
                        label: str,
                        require_ready: bool = True,
                        require_ctrl: str | None = None,
                        require_ctrls: tuple[str, ...] | None = None,
                        require_active_none: bool = False,
                        require_queue_empty: bool = False,
                        forbid_ctrls: tuple[str, ...] = (),
                        forbid_robot_fault: bool = True) -> str | None:
    if status.get("block") is not None:
        return f"{label}: block is not null ({status.get('block')}); {compact_status_line(status)}"
    if status.get("err") is not None:
        return f"{label}: err is not null ({status.get('err')}); {compact_status_line(status)}"
    if require_ready and status.get("ready") is not True:
        return f"{label}: ready is not true; {compact_status_line(status)}"
    if forbid_robot_fault and status.get("robot") == "fault":
        return f"{label}: robot is fault; {compact_status_line(status)}"
    if status.get("ctrl") in forbid_ctrls:
        return f"{label}: ctrl is {status.get('ctrl')}; {compact_status_line(status)}"
    if require_ctrl is not None and status.get("ctrl") != require_ctrl:
        return f"{label}: ctrl is not {require_ctrl}; {compact_status_line(status)}"
    if require_ctrls is not None and status.get("ctrl") not in require_ctrls:
        return f"{label}: ctrl is not one of {require_ctrls}; {compact_status_line(status)}"
    if require_active_none and (status.get("active") or {}).get("kind") != "none":
        return f"{label}: active.kind is not none; {compact_status_line(status)}"
    if require_queue_empty and (status.get("queue") or {}).get("n") != 0:
        return f"{label}: queue is not empty; {compact_status_line(status)}"
    return None


def sim_root_z_reason(label: str,
                      sim_status: dict[str, Any],
                      min_root_z: float) -> str | None:
    if not sim_status.get("available"):
        return None
    root_z = sim_status.get("root_z")
    if isinstance(root_z, bool) or not isinstance(root_z, (int, float)):
        return f"{label}: sim-control root_z is not numeric ({root_z})"
    if root_z < min_root_z:
        return f"{label}: sim-control root_z {root_z:.3f} < {min_root_z:.3f}"
    return None


def sim_control_required_reason(label: str, sim_status: dict[str, Any]) -> str | None:
    if not sim_status.get("available"):
        error = sim_status.get("error")
        suffix = f" ({error})" if error else ""
        return f"{label}: sim-control unavailable{suffix}"
    if sim_status.get("ok") is not True:
        raw = sim_status.get("raw")
        suffix = f"; raw={raw}" if raw is not None else ""
        return f"{label}: sim-control status is not ok{suffix}"
    return None


def sim_both_contact_reason(label: str,
                            sim_status: dict[str, Any],
                            *,
                            require_available: bool = False) -> str | None:
    if not sim_status.get("available"):
        if require_available:
            return sim_control_required_reason(label, sim_status)
        return None
    reason = sim_control_required_reason(label, sim_status)
    if reason is not None:
        return reason
    if sim_status.get("both") is not True:
        return f"{label}: both contact is not true; sim_control={sim_control_compact(sim_status)}"
    return None


def contact_recovery_update(sim_status: dict[str, Any],
                            now: float,
                            consecutive_contact: int,
                            contact_started_at: float | None) -> tuple[int, float | None]:
    if sim_status.get("both") is True:
        consecutive_contact += 1
        if contact_started_at is None:
            contact_started_at = now
        return consecutive_contact, contact_started_at
    return 0, None


def contact_recovery_s(now: float, contact_started_at: float | None) -> float:
    return (now - contact_started_at) if contact_started_at is not None else 0.0


def contact_recovered(now: float,
                      contact_started_at: float | None,
                      consecutive_contact: int,
                      samples_required: int,
                      s_required: float) -> bool:
    return (consecutive_contact >= samples_required and
            contact_recovery_s(now, contact_started_at) >= s_required)


def status_pose_root_z(status: dict[str, Any]) -> Any:
    pose = status.get("pose")
    if not isinstance(pose, dict):
        return None
    p = pose.get("p")
    if not isinstance(p, list) or len(p) < 3:
        return None
    return p[2]


def status_root_z_reason(label: str,
                         status: dict[str, Any],
                         min_root_z: float) -> str | None:
    root_z = status_pose_root_z(status)
    if isinstance(root_z, bool) or not isinstance(root_z, (int, float)):
        return f"{label}: status pose root_z is not numeric ({root_z})"
    if root_z < min_root_z:
        return f"{label}: status pose root_z {root_z:.3f} < {min_root_z:.3f}"
    return None


def physical_root_z_reason(label: str,
                           status: dict[str, Any],
                           sim_status: dict[str, Any],
                           min_root_z: float) -> str | None:
    if sim_status.get("available"):
        return sim_root_z_reason(label, sim_status, min_root_z)
    return status_root_z_reason(label, status, min_root_z)


def visual_camera_reason(sim_status: dict[str, Any],
                         expected_body: str = VISUAL_CAMERA_TRACK_BODY) -> str | None:
    reason = sim_control_required_reason("visual camera", sim_status)
    if reason is not None:
        return reason
    camera_fields = ("camera_type", "camera_track_body", "camera_trackbodyid")
    missing = [field for field in camera_fields if field not in sim_status]
    if missing:
        return f"visual camera: sim-control missing camera fields {missing}"
    if sim_status.get("camera_type") != "tracking":
        return f"visual camera: camera_type is {sim_status.get('camera_type')}, expected tracking"
    if sim_status.get("camera_track_body") != expected_body:
        return (
            f"visual camera: camera_track_body is {sim_status.get('camera_track_body')}, "
            f"expected {expected_body}"
        )
    trackbodyid = sim_status.get("camera_trackbodyid")
    if isinstance(trackbodyid, bool) or not isinstance(trackbodyid, (int, float)):
        return f"visual camera: camera_trackbodyid is not numeric ({trackbodyid})"
    if trackbodyid < 0:
        return f"visual camera: camera_trackbodyid {trackbodyid} < 0"
    return None


def final_settle_reason(status: dict[str, Any],
                        sim_status: dict[str, Any],
                        min_root_z: float) -> str | None:
    reason = status_clear_reason(
        status,
        label="final settle",
        require_ready=True,
        require_ctrls=STANDBY_CTRLS,
        require_active_none=True,
        require_queue_empty=True,
    )
    if reason is not None:
        return reason
    return sim_root_z_reason("final settle", sim_status, min_root_z)


def visual_stability_reason(status: dict[str, Any],
                            sim_status: dict[str, Any],
                            min_root_z: float) -> str | None:
    reason = status_clear_reason(
        status,
        label="visual stability",
        require_ready=True,
        forbid_ctrls=UNSTABLE_CTRL_STATES,
    )
    if reason is not None:
        return reason
    reason = sim_control_required_reason("visual stability", sim_status)
    if reason is not None:
        return reason
    reason = sim_root_z_reason("visual stability", sim_status, min_root_z)
    if reason is not None:
        return reason
    return visual_camera_reason(sim_status)


def runtime_evidence(args: argparse.Namespace, url: str) -> dict[str, Any]:
    evidence: dict[str, Any] = {}
    try:
        evidence["status"] = compact_status(get(url, "/status"))
    except GateError as err:
        evidence["status_error"] = str(err)
    evidence["sim_control"] = sim_control_status(args.sim_control_port,
                                                 args.sim_control_timeout_ms)
    return evidence


def check_status_checkpoint(args: argparse.Namespace,
                            url: str,
                            label: str,
                            *,
                            require_ctrl: str | None = None,
                            require_ctrls: tuple[str, ...] | None = None,
                            require_active_none: bool = False,
                            require_queue_empty: bool = False,
                            forbid_ctrls: tuple[str, ...] = ()) -> dict[str, Any]:
    status = get(url, "/status")
    reason = status_clear_reason(
        status,
        label=label,
        require_ctrl=require_ctrl,
        require_ctrls=require_ctrls,
        require_active_none=require_active_none,
        require_queue_empty=require_queue_empty,
        forbid_ctrls=forbid_ctrls,
    )
    if reason is not None:
        fail(reason, {"failure_evidence": runtime_evidence(args, url)})
    return compact_status(status)


def final_settle_check(args: argparse.Namespace, url: str) -> dict[str, Any]:
    if args.final_settle_s > 0:
        time.sleep(args.final_settle_s)
    status = get(url, "/status")
    sim_status = sim_control_status(args.sim_control_port, args.sim_control_timeout_ms)
    fixture_source = getattr(args, "fixture_source_resolved", None)
    evidence = {
        "settle_s": args.final_settle_s,
        "status": compact_status(status),
        "sim_control": sim_status,
        "fixture_source": fixture_source,
    }
    reason = final_settle_reason(status, sim_status, args.min_root_z)
    if reason is not None:
        fail(reason, {"final_settle": evidence})
    if not sim_status.get("available"):
        evidence["result"] = "SKIP"
        evidence["reason"] = "sim-control unavailable; physical root_z was not checked"
        return evidence
    if fixture_source not in PHYSICAL_SAFE_FIXTURE_SOURCES:
        fail(
            "sim-control is available but final settle used a non-physical-safe "
            f"fixture source ({fixture_source}); refusing synthetic/non-physical-safe "
            "TRK fixtures",
            {"final_settle": evidence},
        )
    evidence["result"] = "PASS"
    return evidence


def visual_stability_check(args: argparse.Namespace, url: str) -> dict[str, Any]:
    status = get(url, "/status")
    sim_status = sim_control_status(args.sim_control_port, args.sim_control_timeout_ms)
    evidence = {
        "status": compact_status(status),
        "sim_control": sim_status,
        "min_root_z": args.min_root_z,
    }
    reason = visual_stability_reason(status, sim_status, args.min_root_z)
    if reason is not None:
        fail(reason, {"visual_stability": evidence})
    evidence["result"] = "PASS"
    evidence["sim_control_result"] = "PASS"
    return evidence


def visual_camera_align_check(args: argparse.Namespace) -> dict[str, Any]:
    sim_status = sim_control_camera_align(args.sim_control_port, args.sim_control_timeout_ms)
    evidence: dict[str, Any] = {"sim_control": sim_status}
    reason = visual_camera_reason(sim_status)
    if reason is not None:
        fail(reason, {"visual_camera_align": evidence})
    evidence["result"] = "PASS"
    return evidence


def fail_recent(label: str,
                reason: str,
                samples: list[dict[str, Any]],
                **extra: Any) -> None:
    report = {
        "result": "FAIL",
        "samples": samples[-8:],
    }
    report.update(extra)
    fail(reason, {label: report})


def sim_control_checked_command(args: argparse.Namespace,
                                command: str,
                                label: str,
                                samples: list[dict[str, Any]]) -> dict[str, Any]:
    sim_status = sim_control_command(
        args.sim_control_port,
        args.sim_control_timeout_ms,
        command,
    )
    samples.append({
        "command": command,
        "sim_control": sim_status,
    })
    reason = sim_control_required_reason(label, sim_status)
    if reason is not None:
        fail_recent(label, reason, samples)
    return sim_status


def standby_stability_reason(label: str,
                             status: dict[str, Any],
                             sim_status: dict[str, Any],
                             min_root_z: float,
                             *,
                             require_sim_contact: bool) -> str | None:
    reason = standby_safety_reason(label, status, sim_status, min_root_z)
    if reason is not None:
        return reason
    return sim_both_contact_reason(
        label,
        sim_status,
        require_available=require_sim_contact,
    )


def standby_safety_reason(label: str,
                          status: dict[str, Any],
                          sim_status: dict[str, Any],
                          min_root_z: float) -> str | None:
    reason = status_clear_reason(
        status,
        label=label,
        require_ready=True,
        require_ctrls=STANDBY_CTRLS,
        require_active_none=True,
        require_queue_empty=True,
        forbid_ctrls=UNSTABLE_CTRL_STATES,
    )
    if reason is not None:
        return reason
    reason = generic_stopping_reason(status, label)
    if reason is not None:
        return reason
    reason = physical_root_z_reason(label, status, sim_status, min_root_z)
    if reason is not None:
        return reason
    return None


def mujoco_landing_settle_check(args: argparse.Namespace, url: str) -> dict[str, Any]:
    label = "mujoco_landing_settle"
    samples: list[dict[str, Any]] = []
    post(url, "/fixstand")
    fixstand_status = poll(
        "ctrl=fixstand before mujoco landing settle",
        args.standby_timeout,
        lambda: get(url, "/status"),
        lambda s: s.get("ctrl") == "fixstand" and s.get("ready") is True,
    )
    samples.append({
        "phase": "fixstand",
        "status": compact_status(fixstand_status),
    })
    if args.mujoco_land_hold:
        sim_control_checked_command(args, "hold", label, samples)

    contact_samples_required = max(1, int(args.mujoco_land_contact_samples))
    contact_s_required = max(0.0, float(args.mujoco_land_contact_s))
    interval_s = max(0.0, float(args.mujoco_land_interval_s))
    deadline = time.monotonic() + max(0.0, float(args.mujoco_land_timeout_s))
    consecutive_contact = 0
    contact_started_at: float | None = None
    lower_count = 0
    latest_sim: dict[str, Any] | None = None

    while time.monotonic() <= deadline:
        latest_sim = sim_control_checked_command(args, "lower", label, samples)
        lower_count += 1
        reason = sim_root_z_reason(label, latest_sim, args.min_root_z)
        if reason is not None:
            fail_recent(label, reason, samples, lower_count=lower_count)
        now = time.monotonic()
        consecutive_contact, contact_started_at = contact_recovery_update(
            latest_sim,
            now,
            consecutive_contact,
            contact_started_at,
        )
        if contact_recovered(
            now,
            contact_started_at,
            consecutive_contact,
            contact_samples_required,
            contact_s_required,
        ):
            break
        if interval_s > 0:
            time.sleep(interval_s)
    else:
        fail_recent(
            label,
            (
                f"{label}: timed out waiting for both contact "
                f"({contact_samples_required} consecutive samples, "
                f"{contact_s_required:.3f}s); latest={sim_control_compact(latest_sim or {})}"
            ),
            samples,
            lower_count=lower_count,
        )

    post(url, "/standby")
    standby_status = wait_standby(url, args.standby_timeout)
    samples.append({
        "phase": "standby_before_release",
        "status": compact_status(standby_status),
    })
    standby_sim = sim_control_checked_command(args, "status", label, samples)
    reason = standby_stability_reason(
        label,
        standby_status,
        standby_sim,
        args.min_root_z,
        require_sim_contact=True,
    )
    if reason is not None:
        fail_recent(
            label,
            reason,
            samples,
            lower_count=lower_count,
            standby_status=compact_status(standby_status),
            standby_sim_control=standby_sim,
        )

    sim_control_checked_command(args, "release", label, samples)

    release_check_s = max(0.0, float(args.mujoco_land_release_check_s))
    release_deadline = time.monotonic() + release_check_s
    release_check: dict[str, Any] | None = None
    release_contact_samples_required = contact_samples_required if release_check_s > 0 else 1
    release_contact_s_required = contact_s_required if release_check_s > 0 else 0.0
    release_consecutive_contact = 0
    release_contact_started_at: float | None = None
    release_contact_latest_at = time.monotonic()
    release_sample_count = 0
    first = True
    while first or time.monotonic() < release_deadline:
        first = False
        release_sim = sim_control_checked_command(args, "status", label, samples)
        release_status = get(url, "/status")
        release_contact_latest_at = time.monotonic()
        compact_release_status = compact_status(release_status)
        samples.append({
            "phase": "post_release",
            "status": compact_release_status,
        })
        release_sample_count += 1
        release_check = {
            "command": "status",
            "sim_control": release_sim,
            "status": compact_release_status,
        }
        reason = standby_safety_reason(
            label,
            release_status,
            release_sim,
            args.min_root_z,
        )
        if reason is not None:
            fail_recent(
                label,
                reason,
                samples,
                lower_count=lower_count,
                release_check=release_check,
            )
        release_consecutive_contact, release_contact_started_at = contact_recovery_update(
            release_sim,
            release_contact_latest_at,
            release_consecutive_contact,
            release_contact_started_at,
        )
        if release_check_s <= 0:
            break
        if interval_s > 0:
            time.sleep(interval_s)

    release_contact_s = contact_recovery_s(release_contact_latest_at,
                                           release_contact_started_at)
    release_contact = {
        "samples_required": release_contact_samples_required,
        "s_required": release_contact_s_required,
        "consecutive": release_consecutive_contact,
        "continuous_s": release_contact_s,
        "sample_count": release_sample_count,
    }
    if not contact_recovered(
        release_contact_latest_at,
        release_contact_started_at,
        release_consecutive_contact,
        release_contact_samples_required,
        release_contact_s_required,
    ):
        fail_recent(
            label,
            (
                f"{label}: post-release both contact did not recover "
                f"({release_contact_samples_required} consecutive samples, "
                f"{release_contact_s_required:.3f}s); "
                f"latest={sim_control_compact((release_check or {}).get('sim_control', {}))}"
            ),
            samples,
            lower_count=lower_count,
            release_check=release_check,
            release_contact=release_contact,
        )

    return {
        "result": "PASS",
        "hold": args.mujoco_land_hold,
        "lower_count": lower_count,
        "contact_samples_required": contact_samples_required,
        "contact_s_required": contact_s_required,
        "release_check_s": args.mujoco_land_release_check_s,
        "release_contact": release_contact,
        "release_check": release_check,
        "standby_status": compact_status(standby_status),
        "standby_sim_control": standby_sim,
        "samples": samples[-8:],
    }


def standby_soak_reason(status: dict[str, Any],
                        sim_status: dict[str, Any],
                        min_root_z: float) -> str | None:
    return standby_stability_reason(
        "standby_soak",
        status,
        sim_status,
        min_root_z,
        require_sim_contact=False,
    )


def standby_soak_check(args: argparse.Namespace, url: str) -> dict[str, Any]:
    label = "standby_soak"
    post(url, "/standby")
    wait_standby(url, args.standby_timeout)

    duration_s = max(0.0, float(args.standby_soak_s))
    interval_s = max(0.0, float(args.standby_soak_interval_s))
    deadline = time.monotonic() + duration_s
    samples: list[dict[str, Any]] = []
    first = True
    while first or time.monotonic() < deadline:
        first = False
        status = get(url, "/status")
        sim_status = sim_control_status(args.sim_control_port, args.sim_control_timeout_ms)
        sample = {
            "status": compact_status(status),
            "sim_control": sim_status,
        }
        samples.append(sample)
        reason = standby_soak_reason(status, sim_status, args.min_root_z)
        if reason is not None:
            fail_recent(
                label,
                reason,
                samples,
                duration_s=duration_s,
                sample_count=len(samples),
            )
        if duration_s <= 0:
            break
        if interval_s > 0:
            time.sleep(interval_s)

    return {
        "result": "PASS",
        "duration_s": duration_s,
        "sample_count": len(samples),
        "samples": samples[-8:],
    }


def standby_handoff_reason(status: dict[str, Any],
                           sim_status: dict[str, Any],
                           label: str,
                           *,
                           idle_n: int | None = None,
                           require_idle_enabled: bool = False,
                           final: bool = False,
                           min_root_z: float = 0.2) -> str | None:
    reason = status_clear_reason(
        status,
        label=label,
        require_ready=True,
        forbid_ctrls=UNSTABLE_CTRL_STATES,
    )
    if reason is not None:
        return reason
    reason = generic_stopping_reason(status, label)
    if reason is not None:
        return reason
    if require_idle_enabled:
        reason = idle_config_reason(status, label, enabled=True, n=idle_n)
        if reason is not None:
            return reason
    reason = sim_root_z_reason(label, sim_status, min_root_z)
    if reason is not None:
        return reason
    if final and not is_standby_or_idle_background(status):
        return f"{label}: final status is neither standby nor idle background; {compact_status_line(status)}"
    return None


def standby_handoff_check(args: argparse.Namespace,
                          url: str,
                          label: str,
                          *,
                          idle_n: int | None = None,
                          require_idle_enabled: bool = False,
                          timeout_s: float = 10.0) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    samples: list[dict[str, Any]] = []
    latest_status = None
    latest_sim = None
    while time.monotonic() < deadline:
        latest_status = get(url, "/status")
        latest_sim = sim_control_status(args.sim_control_port, args.sim_control_timeout_ms)
        samples.append({
            "status": compact_status(latest_status),
            "sim_control": latest_sim,
        })
        reason = standby_handoff_reason(
            latest_status,
            latest_sim,
            label,
            idle_n=idle_n,
            require_idle_enabled=require_idle_enabled,
            min_root_z=args.min_root_z,
        )
        if reason is not None:
            fail(reason, {label: {
                "result": "FAIL",
                "samples": samples[-8:],
            }})
        if is_standby_or_idle_background(latest_status):
            final_reason = standby_handoff_reason(
                latest_status,
                latest_sim,
                label,
                idle_n=idle_n,
                require_idle_enabled=require_idle_enabled,
                final=True,
                min_root_z=args.min_root_z,
            )
            if final_reason is not None:
                fail(final_reason, {label: {
                    "result": "FAIL",
                    "samples": samples[-8:],
                }})
            result = {
                "result": "PASS",
                "status": compact_status(latest_status),
                "sim_control": latest_sim,
                "samples": samples[-8:],
            }
            if latest_sim and latest_sim.get("available"):
                result["sim_control_result"] = "PASS"
            else:
                result["sim_control_result"] = "SKIP"
                result["sim_control_reason"] = "sim-control unavailable; physical root_z was not checked"
            return result
        time.sleep(0.05)
    fail(
        f"{label}: timed out waiting for standby or idle background; "
        f"latest={json.dumps(compact_status(latest_status), sort_keys=True)}",
        {label: {
            "result": "FAIL",
            "latest_status": compact_status(latest_status),
            "latest_sim_control": latest_sim,
            "samples": samples[-8:],
        }},
    )


def urgent_stop_idle_clear_check(args: argparse.Namespace,
                                 url: str,
                                 label: str,
                                 *,
                                 timeout_s: float = 10.0) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    samples: list[dict[str, Any]] = []
    latest_status = None
    latest_sim = None
    while time.monotonic() < deadline:
        latest_status = get(url, "/status")
        latest_sim = sim_control_status(args.sim_control_port, args.sim_control_timeout_ms)
        samples.append({
            "status": compact_status(latest_status),
            "sim_control": latest_sim,
        })
        reason = generic_stopping_reason(latest_status, label)
        if reason is not None:
            fail(reason, {label: {
                "result": "FAIL",
                "samples": samples[-8:],
            }})
        reason = sim_root_z_reason(label, latest_sim, args.min_root_z)
        if reason is not None:
            fail(reason, {label: {
                "result": "FAIL",
                "samples": samples[-8:],
            }})
        idle = latest_status.get("idle") if isinstance(latest_status.get("idle"), dict) else {}
        idle_cleared = (
            idle.get("enabled") is False and
            idle.get("n") == 0 and
            active_kind(latest_status) not in ("user", "transition")
        )
        stable_reason = status_clear_reason(
            latest_status,
            label=label,
            require_ready=True,
            forbid_ctrls=UNSTABLE_CTRL_STATES,
        )
        if idle_cleared and stable_reason is None:
            result = {
                "result": "PASS",
                "status": compact_status(latest_status),
                "sim_control": latest_sim,
                "samples": samples[-8:],
            }
            if latest_sim and latest_sim.get("available"):
                result["sim_control_result"] = "PASS"
            else:
                result["sim_control_result"] = "SKIP"
                result["sim_control_reason"] = "sim-control unavailable; physical root_z was not checked"
            return result
        time.sleep(0.05)
    fail(
        f"{label}: timed out waiting for urgent_stop to clear idle and reach stable status; "
        f"latest={json.dumps(compact_status(latest_status), sort_keys=True)}",
        {label: {
            "result": "FAIL",
            "latest_status": compact_status(latest_status),
            "latest_sim_control": latest_sim,
            "samples": samples[-8:],
        }},
    )


def visual_action_terminal_check(args: argparse.Namespace,
                                 url: str,
                                 run_id: str,
                                 track: Path) -> dict[str, Any]:
    timeout_s = trk_terminal_timeout_s(args, track, min_timeout_s=12.0)
    terminal = poll(
        "visual action terminal",
        timeout_s,
        lambda: run_status(url, run_id),
        lambda s: s.get("state") in TERMINAL_RUN_STATES,
    )
    state = terminal.get("state")
    evidence = {
        "id": run_id,
        "state": state,
        "run": terminal,
        "timeout_s": timeout_s,
    }
    if state in FAILED_TERMINAL_RUN_STATES:
        fail(f"visual action terminal state is {state}: {terminal}",
             {"visual_action_terminal": evidence})
    evidence["result"] = "PASS"
    return evidence


def visual_action_standby_check(args: argparse.Namespace,
                                url: str,
                                label: str = "visual action standby") -> dict[str, Any]:
    deadline = time.monotonic() + args.standby_timeout
    samples: list[dict[str, Any]] = []
    latest_status = None
    while time.monotonic() < deadline:
        latest_status = get(url, "/status")
        samples.append(compact_status(latest_status))
        reason = status_clear_reason(
            latest_status,
            label=label,
            require_ready=True,
            require_ctrls=STANDBY_CTRLS,
            require_active_none=True,
            require_queue_empty=True,
            forbid_ctrls=UNSTABLE_CTRL_STATES,
        )
        if reason is None:
            return {
                "result": "PASS",
                "status": compact_status(latest_status),
                "samples": samples[-8:],
            }
        if (latest_status.get("ctrl") in UNSTABLE_CTRL_STATES or
                latest_status.get("block") is not None or
                latest_status.get("err") is not None or
                latest_status.get("robot") == "fault"):
            fail(reason, {label: {
                "result": "FAIL",
                "samples": samples[-8:],
            }})
        time.sleep(0.05)
    fail(
        f"{label}: timed out waiting for standby with no active run and empty queue; "
        f"latest={json.dumps(compact_status(latest_status), sort_keys=True)}",
        {label: {
            "result": "FAIL",
            "latest_status": compact_status(latest_status),
            "samples": samples[-8:],
        }},
    )


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


def read_exact(infile, size: int) -> bytes | None:
    data = infile.read(size)
    if len(data) != size:
        return None
    return data


def read_uint32(infile) -> int | None:
    data = read_exact(infile, 4)
    if data is None:
        return None
    return struct.unpack("<I", data)[0]


def read_uint64(infile) -> int | None:
    data = read_exact(infile, 8)
    if data is None:
        return None
    return struct.unpack("<Q", data)[0]


def read_trk_arrays(path: Path) -> list[TrkArray] | None:
    try:
        with path.open("rb") as infile:
            if read_exact(infile, len(MAGIC)) != MAGIC:
                return None
            version = read_uint32(infile)
            array_count = read_uint32(infile)
            if version != 1 or array_count is None or array_count == 0 or array_count > 64:
                return None
            arrays: list[TrkArray] = []
            for _index in range(array_count):
                name_len = read_uint32(infile)
                if name_len is None or name_len == 0 or name_len > 128:
                    return None
                name_data = read_exact(infile, name_len)
                if name_data is None or b"\0" in name_data:
                    return None
                try:
                    name = name_data.decode("ascii")
                except UnicodeDecodeError:
                    return None
                dtype = read_uint32(infile)
                ndim = read_uint32(infile)
                if dtype not in TRK_DTYPE_SIZES or ndim is None or ndim > 4:
                    return None
                dims = []
                for _dim in range(ndim):
                    dim = read_uint32(infile)
                    if dim is None:
                        return None
                    dims.append(dim)
                shape = tuple(dims)
                byte_count = read_uint64(infile)
                if byte_count is None:
                    return None
                expected_bytes = trk_element_count(shape) * TRK_DTYPE_SIZES[dtype]
                if byte_count != expected_bytes:
                    return None
                data = read_exact(infile, byte_count)
                if data is None:
                    return None
                arrays.append(TrkArray(name=name, dtype=dtype, shape=shape, data=data))
            return arrays
    except OSError:
        return None


def trk_element_count(shape: tuple[int, ...]) -> int:
    count = 1
    for dim in shape:
        count *= dim
    return count


def required_dtype_allowed(name: str, dtype: int) -> bool:
    if name in ("left_foot_contact_state", "right_foot_contact_state"):
        return dtype in CONTACT_DTYPES
    return dtype in FLOAT_DTYPES


def trk_summary(path: Path) -> TrkCandidate | None:
    try:
        stat = path.stat()
        with path.open("rb") as infile:
            if read_exact(infile, len(MAGIC)) != MAGIC:
                return None
            version = read_uint32(infile)
            array_count = read_uint32(infile)
            if version != 1 or array_count is None or array_count == 0 or array_count > 64:
                return None
            found: dict[str, tuple[int, tuple[int, ...]]] = {}
            for _index in range(array_count):
                name_len = read_uint32(infile)
                if name_len is None or name_len == 0 or name_len > 128:
                    return None
                name_data = read_exact(infile, name_len)
                if name_data is None or b"\0" in name_data:
                    return None
                try:
                    name = name_data.decode("ascii")
                except UnicodeDecodeError:
                    return None
                dtype = read_uint32(infile)
                ndim = read_uint32(infile)
                if dtype not in TRK_DTYPE_SIZES or ndim is None or ndim > 4:
                    return None
                dims = []
                for _dim in range(ndim):
                    dim = read_uint32(infile)
                    if dim is None:
                        return None
                    dims.append(dim)
                shape = tuple(dims)
                byte_count = read_uint64(infile)
                if byte_count is None:
                    return None
                expected_bytes = trk_element_count(shape) * TRK_DTYPE_SIZES[dtype]
                if byte_count != expected_bytes:
                    return None
                found[name] = (dtype, shape)
                infile.seek(byte_count, os.SEEK_CUR)
    except OSError:
        return None

    joint = found.get("joint_pos")
    if joint is None or len(joint[1]) != 2 or joint[1][1] != JOINT_DIM:
        return None
    frames = joint[1][0]
    if frames <= 0:
        return None
    for name, _synthetic_dtype, trailing_shape in REQUIRED_ARRAYS:
        current = found.get(name)
        if current is None:
            return None
        dtype, shape = current
        if not required_dtype_allowed(name, dtype):
            return None
        if len(shape) != 1 + len(trailing_shape) or shape[0] != frames:
            return None
        if shape[1:] != trailing_shape:
            return None
    return TrkCandidate(path=path.resolve(), frames=frames, mtime_ns=stat.st_mtime_ns)


def repeat_frame_data(data: bytes, source_frames: int, target_frames: int) -> bytes:
    if source_frames <= 0 or target_frames <= 0:
        raise ValueError("source_frames and target_frames must be positive")
    if len(data) % source_frames != 0:
        raise ValueError("TRK array byte count is not divisible by source frames")
    frame_bytes = len(data) // source_frames
    if target_frames <= source_frames:
        return data[:target_frames * frame_bytes]
    last_frame = data[(source_frames - 1) * frame_bytes:]
    return data + last_frame * (target_frames - source_frames)


def write_repeated_trk(source: Path, target: Path, frames: int) -> Path:
    source_summary = trk_summary(source)
    arrays = read_trk_arrays(source)
    if source_summary is None or arrays is None:
        fail(f"e2e-safe fixture source is not a valid TRK: {source}")
    source_frames = source_summary.frames
    mkdir(target.parent)
    with target.open("wb") as out:
        out.write(MAGIC)
        write_scalar(out, "<I", 1)
        write_scalar(out, "<I", len(arrays))
        for array in arrays:
            shape = array.shape
            data = array.data
            if shape and shape[0] == source_frames:
                shape = (frames,) + shape[1:]
                data = repeat_frame_data(data, source_frames, frames)
            write_scalar(out, "<I", len(array.name))
            out.write(array.name.encode("ascii"))
            write_scalar(out, "<I", array.dtype)
            write_scalar(out, "<I", len(shape))
            for dim in shape:
                write_scalar(out, "<I", dim)
            write_scalar(out, "<Q", len(data))
            out.write(data)
    return target


def trk_terminal_timeout_s(args: argparse.Namespace,
                           path: Path,
                           min_timeout_s: float) -> float:
    if not getattr(args, "start_tracker", False):
        return min_timeout_s
    summary = trk_summary(path)
    if summary is None:
        return min_timeout_s
    hz = getattr(args, "temp_hz", None)
    transition_s = getattr(args, "temp_transition_duration_s", 0.0)
    if not isinstance(hz, (int, float)) or not math.isfinite(hz) or hz <= 0.0:
        return min_timeout_s
    if not isinstance(transition_s, (int, float)) or not math.isfinite(transition_s):
        transition_s = 0.0
    transition_s = max(0.0, transition_s)
    return max(min_timeout_s, summary.frames / hz + transition_s + 4.0)


def is_obvious_synthetic_trk(path: Path, motion_dir: Path) -> bool:
    try:
        relative = path.relative_to(motion_dir)
        parts = [part.lower() for part in relative.parts]
    except ValueError:
        parts = [part.lower() for part in path.parts]
    name = path.name.lower()
    for part in parts[:-1]:
        if part in ("tmp", "temp", "synthetic", "fixture", "fixtures"):
            return True
        if part.startswith("tmp") or part.endswith("_tmp"):
            return True
        if "manual_gate" in part and name not in NAMED_MANUAL_GATE_FIXTURES:
            return True
    if name in NAMED_MANUAL_GATE_FIXTURES:
        return False
    if name in LEGACY_MANUAL_GATE_FIXTURES:
        return True
    if name.startswith("manual_gate_") or name.startswith("synthetic_"):
        return True
    if ("manual_gate" in name or "_synthetic" in name or name.startswith("fixture_") or
            "tmp" in name):
        return True
    return False


def find_existing_trk_candidates(motion_dir: Path) -> list[TrkCandidate]:
    if not motion_dir.exists():
        return []
    candidates: list[TrkCandidate] = []
    for path in motion_dir.rglob("*.trk"):
        if not path.is_file() or is_obvious_synthetic_trk(path, motion_dir):
            continue
        candidate = trk_summary(path)
        if candidate is not None:
            candidates.append(candidate)
    candidates.sort(key=lambda item: (-item.mtime_ns, str(item.path)))
    return candidates


def named_manual_gate_fixture_candidates(candidates: list[TrkCandidate]) -> dict[str, TrkCandidate]:
    named: dict[str, TrkCandidate] = {}
    for candidate in sorted(candidates, key=lambda item: str(item.path)):
        key = NAMED_MANUAL_GATE_FIXTURES.get(candidate.path.name.lower())
        if key is not None:
            named[key] = candidate
    return named


def existing_fixture_paths(candidates: list[TrkCandidate]) -> dict[str, Path] | None:
    named = existing_fixture_candidates(candidates)
    if named is not None:
        return {key: named[key].path for key in FIXTURE_KEYS}
    return None


def existing_fixture_candidates(candidates: list[TrkCandidate]) -> dict[str, TrkCandidate] | None:
    named = named_manual_gate_fixture_candidates(candidates)
    if len(named) != len(FIXTURE_KEYS):
        return None
    for key in FIXTURE_KEYS:
        if named[key].frames != E2E_SAFE_FIXTURE_FRAMES[key]:
            return None
    return {key: named[key] for key in FIXTURE_KEYS}


def existing_fixture_frame_mismatches(candidates: list[TrkCandidate]) -> dict[str, dict[str, Any]]:
    named = named_manual_gate_fixture_candidates(candidates)
    mismatches: dict[str, dict[str, Any]] = {}
    for key, candidate in named.items():
        expected = E2E_SAFE_FIXTURE_FRAMES[key]
        if candidate.frames != expected:
            mismatches[key] = {
                "path": str(candidate.path),
                "frames": candidate.frames,
                "expected_frames": expected,
            }
    return mismatches


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as infile:
        for chunk in iter(lambda: infile.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def fixture_path_report(fixtures: dict[str, Path],
                        candidates: list[TrkCandidate] | None = None,
                        provenance: str | None = None) -> dict[str, Any]:
    candidate_by_path = {
        str(candidate.path.resolve()): candidate for candidate in candidates or []
    }
    report: dict[str, Any] = {}
    for key, path in fixtures.items():
        resolved = path.resolve()
        candidate = candidate_by_path.get(str(resolved))
        entry: dict[str, Any] = {"path": str(path)}
        if candidate is not None:
            entry["frames"] = candidate.frames
        if provenance is not None:
            entry["provenance"] = provenance
        try:
            entry["sha256"] = file_sha256(resolved)
        except OSError:
            pass
        report[key] = entry
    return report


def e2e_safe_fixture_candidates(fixtures: dict[str, Path]) -> list[TrkCandidate]:
    return [
        TrkCandidate(
            path=path.resolve(),
            frames=E2E_SAFE_FIXTURE_FRAMES[key],
            mtime_ns=path.stat().st_mtime_ns,
        )
        for key, path in fixtures.items()
    ]


def motion_fixtures_will_run(args: argparse.Namespace) -> bool:
    if args.gate in ("all", "e2e"):
        return True
    return args.gate == "visual" and args.visual_run_action


def current_sim_status(args: argparse.Namespace) -> dict[str, Any]:
    return sim_control_status(args.sim_control_port, args.sim_control_timeout_ms)


def resolve_fixtures(args: argparse.Namespace,
                     motion_dir: Path) -> tuple[dict[str, Path], dict[str, Any]]:
    requested = args.fixture_source
    candidates: list[TrkCandidate] = []
    if requested in ("auto", "existing"):
        candidates = find_existing_trk_candidates(motion_dir)

    if requested == "auto" and motion_fixtures_will_run(args):
        fixtures = make_e2e_safe_fixtures(motion_dir)
        log(f"[fixtures] source=e2e_safe motion_dir={motion_dir}")
        return fixtures, {
            "source": "e2e_safe",
            "requested": requested,
            "motion_dir": str(motion_dir),
            "candidate_count": len(candidates),
            "source_trk": str(E2E_SAFE_SOURCE_TRK),
            "selected": fixture_path_report(
                fixtures,
                e2e_safe_fixture_candidates(fixtures),
                provenance="app_standby_ref_derived",
            ),
            "note": "reference-derived e2e-safe fixtures keep manual_gate semantic checks low risk",
        }

    if requested == "existing":
        fixtures_by_key = existing_fixture_candidates(candidates)
        if fixtures_by_key is not None:
            fixtures = {key: fixtures_by_key[key].path for key in FIXTURE_KEYS}
            log(f"[fixtures] source=existing candidates={len(candidates)} motion_dir={motion_dir}")
            return fixtures, {
                "source": "existing",
                "requested": requested,
                "motion_dir": str(motion_dir),
                "candidate_count": len(candidates),
                "selected": fixture_path_report(
                    fixtures,
                    list(fixtures_by_key.values()),
                    provenance="existing_named_e2e_safe",
                ),
            }
        if candidates:
            frame_mismatches = existing_fixture_frame_mismatches(candidates)
            if frame_mismatches:
                fail(
                    f"--fixture-source existing found manual_gate_e2e_safe_*.trk with "
                    f"wrong expected e2e-safe fixture frames under {motion_dir}: "
                    f"{json.dumps(frame_mismatches, sort_keys=True)}"
                )
            else:
                fail(
                    f"--fixture-source existing found no complete e2e-safe fixture set "
                    f"under {motion_dir}; expected manual_gate_e2e_safe_*.trk. "
                    "Arbitrary generated and legacy manual_gate_*.trk files are not "
                    "stable product gate fixtures"
                )
        fail(
            f"--fixture-source existing found no usable generated TRK under {motion_dir}; "
            "expected manual_gate_e2e_safe_*.trk; legacy manual_gate_*.trk/tmp/"
            "synthetic-looking files are ignored"
        )

    if requested in ("auto", "synthetic") and motion_fixtures_will_run(args):
        sim_status = current_sim_status(args)
        if sim_status.get("available"):
            reason = (
                "no usable generated TRK was found"
                if requested == "auto"
                else "--fixture-source synthetic was requested"
            )
            fail(
                f"sim-control is available but {reason}; "
                "refusing to synthesize manual_gate_*.trk for a physical MuJoCo gate",
                {"fixture_selection": {
                    "source": "none",
                    "requested": requested,
                    "motion_dir": str(motion_dir),
                    "candidate_count": 0,
                    "sim_control": sim_status,
                }},
            )

    fixtures = make_fixtures(motion_dir)
    log(f"[fixtures] source=synthetic motion_dir={motion_dir}")
    return fixtures, {
        "source": "synthetic",
        "requested": requested,
        "motion_dir": str(motion_dir),
        "candidate_count": len(candidates),
        "selected": fixture_path_report(fixtures),
        "warning": "synthetic manual_gate_*.trk fixtures are for HTTP/status contracts only",
    }


def make_e2e_safe_fixtures(motion_dir: Path) -> dict[str, Path]:
    return {
        key: write_repeated_trk(
            E2E_SAFE_SOURCE_TRK,
            motion_dir / f"manual_gate_e2e_safe_{key}.trk",
            E2E_SAFE_FIXTURE_FRAMES[key],
        ).resolve()
        for key in FIXTURE_KEYS
    }


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
    return poll("ctrl=standby", timeout_s, lambda: get(url, "/status"),
                lambda s: s.get("ctrl") in STANDBY_CTRLS and
                s.get("ready") is True)


def recover_to_standby(url: str, passive_password: str) -> Any:
    status = get(url, "/status")
    if status.get("ctrl") == "passive":
        post(url, "/fixstand")
        poll("ctrl=fixstand after passive", 8, lambda: get(url, "/status"),
             lambda s: s.get("ctrl") == "fixstand")
    if status.get("ctrl") not in (*STANDBY_CTRLS, "fixstand"):
        post(url, "/standby")
        poll("stable after standby", 8, lambda: get(url, "/status"),
             lambda s: s.get("ctrl") in (*STANDBY_CTRLS, "fixstand", "passive"))
        status = get(url, "/status")
        if status.get("ctrl") == "passive":
            post(url, "/fixstand")
            poll("ctrl=fixstand after standby/passive", 8, lambda: get(url, "/status"),
                 lambda s: s.get("ctrl") == "fixstand")
    status = get(url, "/status")
    if status.get("ctrl") == "fixstand":
        post(url, "/standby")
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


def refuse_synthetic_physical_fixture(args: argparse.Namespace, url: str) -> None:
    if getattr(args, "fixture_source_resolved", None) != "synthetic":
        return
    sim_status = current_sim_status(args)
    if not sim_status.get("available"):
        return
    evidence: dict[str, Any] = {
        "sim_control": sim_status,
        "fixtures": getattr(args, "fixture_report", {}),
    }
    try:
        evidence["status"] = compact_status(get(url, "/status"))
    except GateError as err:
        evidence["status_error"] = str(err)
    fail(
        "sim-control is available but manual_gate resolved synthetic fixtures; "
        "refusing to run manual_gate_*.trk as physical stability evidence",
        {"failure_evidence": evidence},
    )


def queue_interrupt_status_contract_check(args: argparse.Namespace,
                                          url: str,
                                          fixtures: dict[str, Path]) -> dict[str, Any]:
    a_id = execute(url, fixtures["long_a"], mode="queue", hold=True)
    poll("A running", 10, lambda: run_status(url, a_id),
         lambda s: s["state"] in ("running", "holding"))
    b_id = execute(url, fixtures["long_b"], mode="queue")
    b_queued = poll("B queued", 5, lambda: run_status(url, b_id),
                    lambda s: s["state"] == "queued")
    require(b_queued["queue_pos"] is not None, "B should expose queue_pos while queued")
    c_id = execute(url, fixtures["long_c"], mode="interrupt", hold=True)
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
    post(url, "/standby")
    wait_standby(url)
    return {
        "result": "PASS",
        "A": {"id": a_id, "state": a_terminal["state"], "hold": True},
        "B": {"id": b_id, "state": b_terminal["state"], "hold": False},
        "C": {"id": c_id, "state": c_active["state"], "hold": True},
        "checkpoint": check_status_checkpoint(
            args, url, "queue_interrupt_status_contract",
            require_ctrls=STANDBY_CTRLS,
            require_active_none=True,
            require_queue_empty=True,
        ),
    }


def run_e2e(args: argparse.Namespace, fixtures: dict[str, Path]) -> dict[str, Any]:
    url = args.url
    results: dict[str, Any] = {}
    try:
        refuse_synthetic_physical_fixture(args, url)
        wait_ready(url, args.ready_timeout)

        log("[e2e] startup_control_recovery")
        post(url, "/fixstand")
        fixstand = poll("ctrl=fixstand", 8, lambda: get(url, "/status"),
                        lambda s: s.get("ctrl") == "fixstand")
        require(fixstand["active"]["kind"] == "none", "fixstand should not expose user active")
        post(url, "/standby")
        standby = wait_standby(url)
        require(standby["active"]["kind"] == "none", "standby should have no active user")
        results["startup_control_recovery"] = {
            "result": "PASS",
            "checkpoint": check_status_checkpoint(
                args, url, "startup_control_recovery",
                require_ctrls=STANDBY_CTRLS,
                require_active_none=True,
                require_queue_empty=True,
            ),
        }

        log("[e2e] standby vs urgent_stop/passive idle semantics")
        post(url, "/idle", {"paths": [str(fixtures["idle_a"])]})
        idle_set = poll("idle enabled", 8, lambda: get(url, "/status"),
                        lambda s: s["idle"]["enabled"] is True and s["idle"]["n"] == 1)
        require(idle_set["ctrl"] in (*STANDBY_CTRLS, "running"),
                f"unexpected ctrl after idle set: {idle_set['ctrl']}")
        post(url, "/standby")
        standby_idle_result = standby_handoff_check(
            args,
            url,
            "standby_vs_urgent_stop_passive_sink_standby_idle",
            idle_n=1,
            require_idle_enabled=True,
            timeout_s=args.standby_timeout,
        )
        standby_idle = get(url, "/status")
        require(standby_idle["idle"]["enabled"] is True and standby_idle["idle"]["n"] == 1,
                "/standby should retain idle config")
        post(url, "/urgent_stop")
        urgent_stop_idle_clear = urgent_stop_idle_clear_check(
            args,
            url,
            "standby_vs_urgent_stop_passive_sink_urgent_stop",
            timeout_s=args.standby_timeout,
        )
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
        post(url, "/standby")
        wait_standby(url)
        results["standby_vs_urgent_stop_passive_sink"] = {
            "result": "PASS",
            "standby_idle": standby_idle_result,
            "urgent_stop_idle_clear": urgent_stop_idle_clear,
            "checkpoint": check_status_checkpoint(
                args, url, "standby_vs_urgent_stop_passive_sink",
                require_ctrls=STANDBY_CTRLS,
                require_active_none=True,
                require_queue_empty=True,
            ),
        }

        log("[e2e] idle_set_preempt_resume_clear")
        post(url, "/idle", {"paths": [str(fixtures["idle_a"]), str(fixtures["idle_b"])]})
        poll("idle active or configured", 8, lambda: get(url, "/status"),
             lambda s: s["idle"]["enabled"] is True and s["idle"]["n"] == 2)
        user_id = execute(url, fixtures["short"], mode="queue")
        user_preempt = poll(
            "user preempts idle",
            args.transition_timeout,
            lambda: {
                "run": run_status(url, user_id),
                "status": get(url, "/status"),
            },
            lambda s: user_preempted_idle(s["run"], s["status"], user_id),
        )
        user = user_preempt["run"]
        require(user["id"] == user_id, "user status id mismatch")
        user_terminal_timeout = trk_terminal_timeout_s(args, fixtures["short"], 12.0)
        poll("user terminal", user_terminal_timeout, lambda: run_status(url, user_id),
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
        results["idle_set_preempt_resume_clear"] = {
            "result": "PASS",
            "checkpoint": check_status_checkpoint(
                args, url, "idle_set_preempt_resume_clear",
                require_ctrls=STANDBY_CTRLS,
                require_active_none=True,
                require_queue_empty=True,
            ),
        }

        log("[e2e] active_user_idle_enabled_standby_handoff")
        post(url, "/idle", {"paths": [str(fixtures["idle_a"]), str(fixtures["idle_b"])]})
        poll("idle enabled for active user standby", 8, lambda: get(url, "/status"),
             lambda s: s["idle"]["enabled"] is True and s["idle"]["n"] == 2)
        active_idle_user_id = execute(url, fixtures["long_a"], mode="interrupt", hold=True)
        active_idle_user = poll("active user with idle enabled", 10, lambda: get(url, "/status"),
                                lambda s: active_kind(s) == "user" and
                                ((s.get("exec") or {}).get("id") == active_idle_user_id or
                                 (s.get("active") or {}).get("id") == active_idle_user_id))
        require(active_idle_user["idle"]["enabled"] is True and active_idle_user["idle"]["n"] == 2,
                "active user standby hard gate requires idle config to be enabled before /standby")
        post(url, "/standby")
        active_user_standby = standby_handoff_check(
            args,
            url,
            "active_user_idle_enabled_standby_handoff",
            idle_n=2,
            require_idle_enabled=True,
            timeout_s=args.standby_timeout,
        )
        active_user_terminal = poll("active user terminal after standby", 10,
                                    lambda: run_status(url, active_idle_user_id),
                                    lambda s: s["state"] in ("stopped", "done", "holding", "canceled"))
        require(active_user_terminal["state"] != "canceled",
                f"/standby should stop or complete active user, not cancel it: {active_user_terminal}")
        results["active_user_idle_enabled_standby_handoff"] = {
            "result": "PASS",
            "id": active_idle_user_id,
            "before": compact_status(active_idle_user),
            "terminal": {"state": active_user_terminal["state"]},
            "handoff": active_user_standby,
        }
        post(url, "/idle", {"paths": []})
        recover_to_standby(url, args.passive_password)

        log("[e2e] active_user_idle_enabled_urgent_stop_clears_idle")
        post(url, "/idle", {"paths": [str(fixtures["idle_a"]), str(fixtures["idle_b"])]})
        poll("idle enabled for active user urgent_stop", 8, lambda: get(url, "/status"),
             lambda s: s["idle"]["enabled"] is True and s["idle"]["n"] == 2)
        urgent_user_id = execute(url, fixtures["long_b"], mode="interrupt", hold=True)
        urgent_user = poll("active user before urgent_stop", 10, lambda: get(url, "/status"),
                           lambda s: active_kind(s) == "user" and
                           ((s.get("exec") or {}).get("id") == urgent_user_id or
                            (s.get("active") or {}).get("id") == urgent_user_id))
        require(urgent_user["idle"]["enabled"] is True and urgent_user["idle"]["n"] == 2,
                "active user urgent_stop gate requires idle config to be enabled before /urgent_stop")
        post(url, "/urgent_stop")
        urgent_stop_clear = urgent_stop_idle_clear_check(
            args,
            url,
            "active_user_idle_enabled_urgent_stop_clears_idle",
            timeout_s=args.standby_timeout,
        )
        urgent_terminal = poll("active user terminal after urgent_stop", 10,
                               lambda: run_status(url, urgent_user_id),
                               lambda s: s["state"] in ("stopped", "done", "failed", "canceled"))
        results["active_user_idle_enabled_urgent_stop_clears_idle"] = {
            "result": "PASS",
            "id": urgent_user_id,
            "before": compact_status(urgent_user),
            "terminal": {"state": urgent_terminal["state"]},
            "urgent_stop": urgent_stop_clear,
        }
        recover_to_standby(url, args.passive_password)

        log("[e2e] queue_interrupt_status_contract")
        results["queue_interrupt_status_contract"] = (
            queue_interrupt_status_contract_check(args, url, fixtures)
        )

        log("[e2e] short_hold_settle_standby")
        hold_id = execute(url, fixtures["short"], mode="queue", hold=True)
        held = poll("hold action holding", 12, lambda: run_status(url, hold_id),
                    lambda s: s["state"] == "holding")
        time.sleep(HOLD_SETTLE_S)
        hold_settle = check_status_checkpoint(
            args,
            url,
            "short_hold_settle_standby hold settle",
            forbid_ctrls=UNSTABLE_CTRL_STATES,
        )
        post(url, "/standby")
        wait_standby(url)
        results["short_hold_settle_standby"] = {
            "result": "PASS",
            "id": hold_id,
            "holding": {"state": held["state"]},
            "settle_s": HOLD_SETTLE_S,
            "hold_settle": hold_settle,
            "checkpoint": check_status_checkpoint(
                args, url, "short_hold_settle_standby",
                require_ctrls=STANDBY_CTRLS,
                require_active_none=True,
                require_queue_empty=True,
            ),
        }

        log("[e2e] transition_interrupt_and_standby_handoff")
        recover_to_standby(url, args.passive_password)
        ta_id = execute(url, fixtures["transition_a"], mode="queue")
        tb_id = execute(url, fixtures["transition_b"], mode="queue")
        transition = poll("active user transition", args.transition_timeout,
                          lambda: get(url, "/status"),
                          lambda s: is_user_transition_to(s, tb_id))
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
        post(url, "/standby")
        standby_after_transition = standby_handoff_check(
            args,
            url,
            "transition_interrupt_and_standby_handoff",
            timeout_s=args.standby_timeout,
        )
        stopped_after_standby = poll("interrupt target stopped after standby",
                                     5,
                                     lambda: run_status(url, tc_id),
                                     lambda s: s["state"] in ("stopped", "canceled", "done"))
        require(stopped_after_standby["state"] != "canceled",
                f"/standby should stop or complete interrupt target, not cancel it: {stopped_after_standby}")
        after_fast_standby = get(url, "/status")
        require(after_fast_standby["queue"]["ids"] == [],
                f"/standby should not leave queued ids: {after_fast_standby['queue']}")
        poll("transition A terminal", 10, lambda: run_status(url, ta_id),
             lambda s: s["state"] in ("done", "stopped", "canceled"))
        poll("transition B terminal", 10, lambda: run_status(url, tb_id),
             lambda s: s["state"] in ("stopped", "canceled", "done"))
        poll("transition C terminal", 10, lambda: run_status(url, tc_id),
             lambda s: s["state"] in ("stopped", "canceled", "done"))
        wait_standby(url)
        results["transition_interrupt_and_standby_handoff"] = {
            "result": "PASS",
            "initial_target": tb_id,
            "interrupt_target": tc_id,
            "pre_interrupt_queue": before_interrupt_queue,
            "interrupt_running": {"state": c_running["state"]},
            "old_target": {"state": old_target["state"]},
            "handoff": standby_after_transition,
            "checkpoint": check_status_checkpoint(
                args, url, "transition_interrupt_and_standby_handoff",
                require_ctrls=STANDBY_CTRLS,
                require_active_none=True,
                require_queue_empty=True,
            ),
        }

        log("[e2e] loco_upper bounded blackbox")
        health = get(url, "/health")
        loco_cap = health.get("cap", {}).get("loco_upper", {})
        if not loco_cap.get("enabled", False):
            if args.require_loco:
                fail("loco_upper is disabled; pass --enable-loco-temp with --start-tracker or use a loco config")
            results["loco_upper_bounded_blackbox"] = {
                "result": "SKIP",
                "reason": "disabled by config",
                "checkpoint": check_status_checkpoint(
                    args, url, "loco_upper_bounded_blackbox_skip",
                    require_ctrls=STANDBY_CTRLS,
                    require_active_none=True,
                    require_queue_empty=True,
                ),
            }
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
            active_loco = poll(
                "active loco before urgent_stop",
                10,
                lambda: get(url, "/status"),
                lambda s: active_kind(s) == "user" and
                ((s.get("active") or {}).get("id") == loco_id or
                 (s.get("exec") or {}).get("id") == loco_id) and
                (s.get("exec") or {}).get("executor") == "loco_upper",
            )
            post(url, "/urgent_stop")
            urgent_stop_clear = urgent_stop_idle_clear_check(
                args,
                url,
                "loco_upper_bounded_blackbox_urgent_stop",
                timeout_s=args.standby_timeout,
            )
            terminal = poll("loco terminal after urgent_stop",
                            10,
                            lambda: run_status(url, loco_id),
                            lambda s: s["state"] in ("stopped", "done", "failed", "canceled"))
            results["loco_upper_bounded_blackbox"] = {
                "result": "PASS",
                "id": loco_id,
                "active": compact_status(active_loco),
                "terminal": {"state": terminal["state"]},
                "urgent_stop": urgent_stop_clear,
                "checkpoint": check_status_checkpoint(
                    args, url, "loco_upper_bounded_blackbox",
                    require_active_none=True,
                    require_queue_empty=True,
                ),
            }

        log("[e2e] final settle")
        results["final_settle"] = final_settle_check(args, url)

        return results
    except GateError as err:
        results["failure"] = {"reason": str(err)}
        if err.report:
            results["failure"].update(err.report)
        else:
            results["failure"]["evidence"] = runtime_evidence(args, url)
        raise GateError(str(err), {"e2e": results}) from err


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
    result: dict[str, Any] = {
        "pre_status": runtime_evidence(args, url),
    }
    rows = mujoco_processes(args.mujoco_process_regex)
    if not rows:
        fail(f"no MuJoCo process matched /{args.mujoco_process_regex}/")
    log("[visual] MuJoCo process candidates:")
    for row in rows[:5]:
        log(f"  {row}")

    if args.visual_run_action:
        refuse_synthetic_physical_fixture(args, url)
        wait_ready(url, args.ready_timeout)
        recover_to_standby(url, args.passive_password)
        run_id = execute(url, fixtures["short"], mode="queue")
        poll("visual action active/done", 8, lambda: run_status(url, run_id),
             lambda s: s["state"] in ("running", "done", "holding"))
        time.sleep(args.visual_settle_s)
        result["action"] = {"submitted": True, "id": run_id}
    else:
        result["action"] = {
            "submitted": False,
            "reason": "visual_run_action=false; action submission skipped before capture",
        }

    result["camera_align"] = visual_camera_align_check(args)
    if result["camera_align"].get("result") == "PASS":
        time.sleep(0.2)
    capture_screenshot(screenshot, args.screenshot_command)
    require(screenshot.exists(), f"screenshot was not created: {screenshot}")
    size_bytes = screenshot.stat().st_size
    dims = image_size(screenshot)
    require(size_bytes >= args.min_screenshot_bytes,
            f"screenshot too small: {size_bytes} bytes")
    require(dims is not None, "screenshot is not a PNG/JPEG with readable dimensions")
    require(dims[0] >= args.min_screenshot_width and dims[1] >= args.min_screenshot_height,
            f"screenshot dimensions too small: {dims}")
    stability = visual_stability_check(args, url)
    if args.visual_run_action:
        result["action"]["terminal"] = visual_action_terminal_check(
            args, url, result["action"]["id"], fixtures["short"])
        result["action"]["standby"] = visual_action_standby_check(args, url)
        result["final_settle"] = final_settle_check(args, url)
    result.update({
        "result": "PASS",
        "post_status": {
            "status": stability["status"],
            "sim_control": stability["sim_control"],
        },
        "stability": stability,
        "screenshot": str(screenshot),
        "bytes": size_bytes,
        "dimensions": {"width": dims[0], "height": dims[1]},
        "checklist": [
            "MuJoCo process was present",
            "action was submitted before capture" if args.visual_run_action else "action submission skipped by flag",
            "camera_align was applied through sim-control",
            "screenshot file exists",
            "screenshot dimensions and byte size passed minimum checks",
            "post-capture tracker, sim-control stability, and camera tracking checks passed",
            "action reached a non-failed terminal state" if args.visual_run_action else "no action terminal wait required",
            "tracker returned to standby with no active run and empty queue" if args.visual_run_action else "no action standby wait required",
            "final settle check passed" if args.visual_run_action else "final settle skipped without visual action",
            "operator still must visually inspect posture/no-fall/no-snap",
        ],
    })
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
    if args.fixture_source in ("auto", "existing") and DEFAULT_EXISTING_MOTION_DIR.exists():
        return mkdir(DEFAULT_EXISTING_MOTION_DIR)
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
                        help="directory allowed by tracker motion_dirs for TRK fixtures")
    parser.add_argument("--fixture-source", choices=("auto", "existing", "synthetic"),
                        default="auto",
                        help="auto uses e2e-safe reference-derived TRK; existing requires manual_gate_e2e_safe_*.trk; synthetic is HTTP/status-only")
    parser.add_argument("--artifacts-dir", type=Path,
                        default=Path("/tmp/agentic-et1-manual-gate"),
                        help="artifact directory for screenshots and reports")
    parser.add_argument("--passive-password", default="galaxy")
    parser.add_argument("--ready-timeout", type=float, default=20.0)
    parser.add_argument("--transition-timeout", type=float, default=8.0)
    parser.add_argument("--standby-timeout", type=float, default=8.0)
    parser.add_argument("--require-loco", action="store_true",
                        help="fail instead of skip when loco_upper is disabled")
    parser.add_argument("--loco-radius", type=float, default=0.8)
    parser.add_argument("--final-settle-s", type=float, default=3.0,
                        help="extra wait before final E2E stability check")
    parser.add_argument("--min-root-z", type=float, default=0.2,
                        help="minimum sim-control root_z when MuJoCo status is available")

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
    parser.add_argument("--temp-transition-duration-s", type=float, default=0.7,
                        help="transition_duration_s used only in generated --start-tracker config")
    parser.add_argument("--temp-hz", type=float, default=1000.0,
                        help="runtime hz used only in generated --start-tracker config")

    parser.add_argument("--start-mujoco-cmd",
                        help="optional explicit shell command to start MuJoCo; stopped at exit")
    parser.add_argument("--sim-control-port", type=int, default=8090,
                        help="UDP sim-control port for optional MuJoCo status evidence")
    parser.add_argument("--sim-control-timeout-ms", type=int, default=200,
                        help="timeout for optional MuJoCo sim-control status")
    parser.add_argument("--mujoco-land-settle", action="store_true",
                        help="opt-in MuJoCo sling landing helper before the selected gate")
    parser.add_argument("--mujoco-land-hold", action=argparse.BooleanOptionalAction,
                        default=True,
                        help="send sim-control hold before lower when --mujoco-land-settle is enabled")
    parser.add_argument("--mujoco-land-timeout-s", type=float, default=20.0,
                        help="timeout for opt-in MuJoCo lower/contact landing helper")
    parser.add_argument("--mujoco-land-contact-samples", type=int, default=5,
                        help="consecutive both-contact samples required before release and after release recovery")
    parser.add_argument("--mujoco-land-contact-s", type=float, default=0.5,
                        help="continuous both-contact duration required before release and after release recovery")
    parser.add_argument("--mujoco-land-interval-s", type=float, default=0.1,
                        help="lower/status sample interval for opt-in MuJoCo landing helper")
    parser.add_argument("--mujoco-land-release-check-s", type=float, default=2.0,
                        help="post-release both-contact/root_z check duration")
    parser.add_argument("--mujoco-process-regex",
                        default=r"(unitree_mujoco|mujoco|simulate)")
    parser.add_argument("--screenshot-command",
                        help="shell command to capture a screenshot; use {path} for output path")
    parser.add_argument("--visual-run-action", action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument("--visual-settle-s", type=float, default=1.0)
    parser.add_argument("--standby-soak-s", type=float, default=0.0,
                        help="opt-in post-gate standby soak duration; 0 disables it")
    parser.add_argument("--standby-soak-interval-s", type=float, default=0.5,
                        help="status sample interval for opt-in standby soak")
    parser.add_argument("--min-screenshot-bytes", type=int, default=20_000)
    parser.add_argument("--min-screenshot-width", type=int, default=640)
    parser.add_argument("--min-screenshot-height", type=int, default=360)
    args = parser.parse_args()
    if args.mujoco_land_contact_samples < 1:
        parser.error("--mujoco-land-contact-samples must be >= 1")
    for field in (
        "mujoco_land_timeout_s",
        "mujoco_land_contact_s",
        "mujoco_land_interval_s",
        "mujoco_land_release_check_s",
        "standby_soak_s",
        "standby_soak_interval_s",
    ):
        if getattr(args, field) < 0:
            parser.error(f"--{field.replace('_', '-')} must be >= 0")
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
        report["motion_dir"] = str(motion_dir)

        if args.start_mujoco_cmd:
            mujoco_proc = start_process(["bash", "-lc", args.start_mujoco_cmd],
                                        mkdir(args.artifacts_dir) / f"mujoco_{now_stamp()}.log")
            report["started"]["mujoco_pid"] = mujoco_proc.pid
            time.sleep(2.0)

        fixtures, fixture_report = resolve_fixtures(args, motion_dir)
        args.fixture_source_resolved = fixture_report["source"]
        args.fixture_report = fixture_report
        report["fixtures"] = fixture_report

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

        if args.mujoco_land_settle:
            log("[manual] mujoco_landing_settle")
            report["mujoco_landing_settle"] = mujoco_landing_settle_check(args, args.url)

        if args.gate in ("all", "e2e"):
            report["e2e"] = run_e2e(args, fixtures)
        if args.gate in ("all", "visual"):
            report["visual"] = run_visual(args, fixtures)
        if args.standby_soak_s > 0:
            log("[manual] standby_soak")
            report["standby_soak"] = standby_soak_check(args, args.url)

        report_path = mkdir(args.artifacts_dir) / f"manual_gate_report_{now_stamp()}.json"
        report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
        log(f"[result] PASS report={report_path}")
        return 0
    except GateError as err:
        if err.report:
            report.update(err.report)
        report["failure_evidence"] = runtime_evidence(args, args.url)
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
