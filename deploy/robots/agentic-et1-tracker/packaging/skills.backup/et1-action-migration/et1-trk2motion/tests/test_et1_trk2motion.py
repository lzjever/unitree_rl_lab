import json
import os
import subprocess
import sys
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "et1-trk2motion"
TRK = "/tmp/et1-test.trk"
CONTRACT_NEXT_TOKENS = {"status", "retry", "wait_robot", "fix", "fixstand", "standby_velocity", "stop", "manual"}


class TrackerState:
    def __init__(self):
        self.ctrl = "standby_velocity"
        self.ready = True
        self.err = None
        self.records = []
        self.top_status_queue = []
        self.next_id = 1
        self.run_status = {}
        self.missing_execute_id = False
        self.execute_idle_response = False
        self.execute_failure = None
        self.control_failures = {}
        self.large_health = False
        self.large_control = False
        self.active = {"kind": "none", "id": None}
        self.exec = {"id": None, "state": None, "frame": 0, "frames": 0, "progress": 0}
        self.queue = {"ids": [], "n": 0, "limit": 8}
        self.transition = {
            "active": False,
            "target": None,
            "target_id": None,
            "frame": 0,
            "frames": 0,
            "time_s": 0,
            "duration_s": 0,
            "progress": 0,
        }
        self.idle = {
            "enabled": False,
            "n": 0,
            "active": False,
            "current": None,
            "frame": 0,
            "frames": 0,
            "time_s": 0,
            "duration_s": 0,
            "progress": 0,
        }

    def top_status(self):
        if self.top_status_queue:
            return self.top_status_queue.pop(0)
        return {
            "ok": True,
            "ctrl": self.ctrl,
            "ready": self.ready,
            "err": self.err,
            "active": dict(self.active),
            "transition": dict(self.transition),
            "idle": dict(self.idle),
            "exec": dict(self.exec),
            "queue": dict(self.queue),
            "pose": list(range(64)),
        }

    def queued_top_status(self, ctrl, ready, err=None):
        doc = self.top_status()
        doc.update({"ctrl": ctrl, "ready": ready, "err": err})
        return doc

    def status_for(self, run_id):
        if run_id not in self.run_status:
            return {"ok": False, "error": {"code": "unknown_id", "message": run_id}}
        queue = self.run_status[run_id]
        if len(queue) > 1:
            return queue.pop(0)
        return queue[0]

    def response(self, obj, large=False):
        out = dict(obj)
        if large:
            out["pose"] = list(range(128))
            out["samples"] = [{"i": i, "v": list(range(16))} for i in range(16)]
        return out


class Handler(BaseHTTPRequestHandler):
    server_version = "FakeET1/1"

    def log_message(self, fmt, *args):
        pass

    def send_json(self, obj, code=200):
        data = json.dumps(obj, separators=(",", ":")).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def body(self):
        size = int(self.headers.get("Content-Length", "0"))
        if not size:
            return None
        return json.loads(self.rfile.read(size).decode())

    def do_GET(self):
        state = self.server.state
        state.records.append(("GET", self.path, None))
        parsed = urlsplit(self.path)
        if parsed.path == "/health":
            self.send_json(state.response({"ok": True}, state.large_health))
        elif parsed.path == "/status":
            query = parse_qs(parsed.query)
            run_id = query.get("id", [None])[0]
            if run_id == "gone404":
                self.send_json({"ok": False, "error": {"code": "not_found", "message": run_id}}, 404)
                return
            self.send_json(state.status_for(run_id) if run_id else state.top_status())
        else:
            self.send_json({"ok": False, "error": {"code": "not_found", "message": self.path}}, 404)

    def do_POST(self):
        state = self.server.state
        payload = self.body()
        state.records.append(("POST", self.path, payload))
        if self.path == "/fixstand":
            if self.path in state.control_failures:
                self.send_json(state.response(state.control_failures[self.path], state.large_control))
                return
            state.ctrl = "fixstand"
            state.ready = False
            self.send_json(state.response({"ok": True}, state.large_control))
        elif self.path == "/passive":
            if self.path in state.control_failures:
                self.send_json(state.response(state.control_failures[self.path], state.large_control))
                return
            state.ctrl = "passive"
            state.ready = False
            self.send_json(state.response({"ok": True}, state.large_control))
        elif self.path == "/standby_velocity":
            if self.path in state.control_failures:
                self.send_json(state.response(state.control_failures[self.path], state.large_control))
                return
            state.ctrl = "standby_velocity"
            state.ready = True
            self.send_json(state.response({"ok": True}, state.large_control))
        elif self.path == "/stop":
            if self.path in state.control_failures:
                self.send_json(state.response(state.control_failures[self.path], state.large_control))
                return
            self.send_json(state.response({"ok": True}, state.large_control))
        elif self.path == "/idle":
            paths = payload.get("paths") if isinstance(payload, dict) else None
            state.idle.update(
                {
                    "enabled": bool(paths),
                    "n": len(paths or []),
                    "active": False,
                    "current": None,
                    "frame": 0,
                    "frames": 0,
                    "progress": 0,
                }
            )
            self.send_json({"ok": True, "idle": {key: state.idle[key] for key in ("enabled", "n", "active")}})
        elif self.path == "/execute":
            if state.execute_failure:
                self.send_json(state.response(state.execute_failure))
                return
            if state.execute_idle_response:
                state.active = {"kind": "idle", "id": None}
                state.idle.update({"enabled": True, "n": 1, "active": True, "current": 0, "frame": 3, "frames": 12, "progress": 0.25})
                self.send_json({"ok": True, "active": dict(state.active), "idle": dict(state.idle)})
                return
            if state.missing_execute_id:
                self.send_json({"ok": True})
                return
            run_id = f"r{state.next_id}"
            state.next_id += 1
            state.run_status.setdefault(run_id, [{"ok": True, "id": run_id, "state": "done"}])
            self.send_json({"ok": True, "id": run_id})
        elif self.path == "/echo":
            self.send_json({"ok": True, "body": payload})
        else:
            self.send_json({"ok": False, "error": {"code": "not_found", "message": self.path}}, 404)


class ServerCase(unittest.TestCase):
    def setUp(self):
        self.state = TrackerState()
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.server.state = self.state
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.url = f"http://127.0.0.1:{self.server.server_port}"

    def tearDown(self):
        self.server.shutdown()
        self.thread.join(timeout=2)
        self.server.server_close()

    def cli(self, *args, env_extra=None):
        env = os.environ.copy()
        env.pop("ET1_PASSIVE_PASSWORD", None)
        env["ET1_TRACKER_URL"] = self.url
        if env_extra:
            env.update(env_extra)
        proc = subprocess.run(
            [sys.executable, str(SCRIPT), *args],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=env,
            check=False,
        )
        lines = proc.stdout.splitlines()
        self.assertEqual(lines, [proc.stdout.rstrip("\n")])
        self.assertEqual(proc.stderr, "")
        return proc, json.loads(proc.stdout)

    def test_state_short_output_omits_pose(self):
        proc, out = self.cli("state")
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(
            out,
            {
                "ok": True,
                "ctrl": "standby_velocity",
                "ready": True,
                "err": None,
                "active": {"kind": "none", "id": None},
                "transition": {
                    "active": False,
                    "target": None,
                    "target_id": None,
                    "frame": 0,
                    "frames": 0,
                    "progress": 0,
                },
                "idle": {
                    "enabled": False,
                    "n": 0,
                    "active": False,
                    "current": None,
                    "frame": 0,
                    "frames": 0,
                    "progress": 0,
                },
                "exec": {"id": None, "state": None, "frame": 0, "frames": 0, "progress": 0},
                "queue": {"ids": [], "n": 0, "limit": 8},
            },
        )

    def test_state_status_short_output_preserves_contract_next_tokens(self):
        for command in ("state", "status"):
            for token in sorted(CONTRACT_NEXT_TOKENS):
                with self.subTest(command=command, token=token):
                    self.state.top_status_queue = [dict(self.state.top_status(), next=token)]
                    proc, out = self.cli(command)
                    self.assertEqual(proc.returncode, 0)
                    self.assertEqual(out["next"], token)

        self.state.top_status_queue = [dict(self.state.top_status(), next="stand by velocity")]
        proc, out = self.cli("state")
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(out["next"], "status")

    def test_control_failure_with_top_fields_preserves_contract_next_tokens(self):
        cases = (
            (("fixstand",), "/fixstand", "fixstand"),
            (("standby",), "/standby_velocity", "standby_velocity"),
            (("stop", "--urgent"), "/stop", "stop"),
        )
        for command, path, token in cases:
            with self.subTest(command=" ".join(command), token=token):
                self.state.control_failures = {
                    path: {"ok": False, "error": {"code": "CONTROL_STATE_CONFLICT", "message": path}, "next": token}
                }
                proc, out = self.cli(*command)
                self.assertNotEqual(proc.returncode, 0)
                self.assertEqual(out["next"], token)
                self.assertIn(out["next"], CONTRACT_NEXT_TOKENS)
                self.assertEqual(out["ctrl"], self.state.ctrl)
                self.assertEqual(out["ready"], self.state.ready)

        self.state.control_failures = {
            "/stop": {"ok": False, "error": {"code": "ROBOT_NOT_READY", "message": "not ready"}}
        }
        self.state.top_status_queue = [dict(self.state.top_status(), next="wait_robot")]
        proc, out = self.cli("stop", "--urgent")
        self.assertNotEqual(proc.returncode, 0)
        self.assertEqual(out["next"], "wait_robot")

    def test_stop_without_urgent_is_rejected_before_http(self):
        proc, out = self.cli("stop")
        self.assertNotEqual(proc.returncode, 0)
        self.assertEqual(out["error"]["code"], "stop_requires_urgent")
        self.assertEqual(out["next"], "standby_velocity")
        self.assertEqual(self.state.records, [])

    def test_stop_with_urgent_posts_stop(self):
        proc, out = self.cli("stop", "--urgent")
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(out["ok"], True)
        self.assertEqual([record for record in self.state.records if record[1] == "/stop"], [("POST", "/stop", None)])

    def test_ready_from_passive_is_read_only_and_requires_fixstand(self):
        self.state.ctrl = "passive"
        self.state.ready = False
        proc, out = self.cli("ready")
        self.assertNotEqual(proc.returncode, 0)
        self.assertEqual(out["error"]["code"], "not_ready")
        self.assertEqual(out["next"], "fixstand")
        self.assertEqual(out["ctrl"], "passive")
        self.assertEqual(out["ready"], False)
        self.assertEqual([(m, p) for m, p, _ in self.state.records], [("GET", "/status")])

    def test_ready_from_fixstand_is_read_only_and_requires_standby(self):
        self.state.ctrl = "fixstand"
        self.state.ready = True
        proc, out = self.cli("ready")
        self.assertNotEqual(proc.returncode, 0)
        self.assertEqual(out["error"]["code"], "not_ready")
        self.assertEqual(out["next"], "standby_velocity")
        self.assertEqual(out["ctrl"], "fixstand")
        self.assertEqual(out["ready"], True)
        self.assertEqual([(m, p) for m, p, _ in self.state.records], [("GET", "/status")])

    def test_ready_loop_timeout_uses_contract_next_and_compact_detail(self):
        self.state.ctrl = "standby_velocity"
        self.state.ready = False
        self.state.top_status_queue = [self.state.queued_top_status("standby_velocity", False) for _ in range(32)]
        proc, out = self.cli("ready", "--timeout", "0.08", "--poll", "0.01")
        self.assertNotEqual(proc.returncode, 0)
        self.assertFalse(out["ok"])
        self.assertEqual(out["error"]["code"], "ready_loop")
        self.assertEqual(out["next"], "status")
        self.assertIn(out["next"], CONTRACT_NEXT_TOKENS)
        self.assertEqual(out["ctrl"], "standby_velocity")
        self.assertEqual(out["ready"], False)
        self.assertIsNone(out["err"])
        self.assertEqual(out["last"], {"ctrl": "standby_velocity", "ready": False, "err": None})
        self.assertNotIn("POST", [method for method, _, _ in self.state.records])
        compact_out = json.dumps(out, separators=(",", ":"))
        for field in ("pose", "active", "transition", "idle"):
            self.assertNotIn(field, compact_out)

    def test_run_wait_defaults_to_execute_without_ready(self):
        proc, out = self.cli("run", TRK, "--wait", "--timeout", "2", "--poll", "0.01")
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(out["state"], "done")
        self.assertEqual(out["ctrl"], "standby_velocity")
        self.assertEqual(
            [(m, p) for m, p, _ in self.state.records],
            [("POST", "/execute"), ("GET", "/status?id=r1"), ("GET", "/status")],
        )

    def test_run_from_passive_returns_execute_error_without_recovering(self):
        self.state.ctrl = "passive"
        self.state.ready = False
        self.state.execute_failure = {
            "ok": False,
            "error": {"code": "CONTROL_STATE_CONFLICT", "message": "passive cannot execute"},
            "next": "fixstand",
        }
        proc, out = self.cli("run", TRK)
        self.assertNotEqual(proc.returncode, 0)
        self.assertEqual(out["error"]["code"], "CONTROL_STATE_CONFLICT")
        self.assertEqual(out["next"], "fixstand")
        self.assertEqual(out["ctrl"], "passive")
        self.assertEqual(out["ready"], False)
        self.assertEqual(
            [(m, p) for m, p, _ in self.state.records],
            [("POST", "/execute"), ("GET", "/status")],
        )
        self.assertNotIn("/fixstand", [p for _, p, _ in self.state.records])
        self.assertNotIn("/standby_velocity", [p for _, p, _ in self.state.records])

    def test_run_hold_sends_hold_true_only_when_requested(self):
        proc, out = self.cli("run", TRK, "--hold", "--recover", "off")
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(out["id"], "r1")
        self.assertEqual(
            [(m, p, b) for m, p, b in self.state.records],
            [("POST", "/execute", {"path": TRK, "mode": "queue", "hold": True})],
        )

    def test_run_without_hold_does_not_send_hold_false(self):
        proc, out = self.cli("run", TRK, "--recover", "off")
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(out["id"], "r1")
        self.assertEqual(
            [(m, p, b) for m, p, b in self.state.records],
            [("POST", "/execute", {"path": TRK, "mode": "queue"})],
        )

    def test_run_explicit_interrupt_mode(self):
        proc, out = self.cli("run", TRK, "--mode", "interrupt")
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(out["id"], "r1")
        self.assertEqual(
            [(m, p, b) for m, p, b in self.state.records],
            [("POST", "/execute", {"path": TRK, "mode": "interrupt"})],
        )

    def test_exec_defaults_to_queue_and_explicit_interrupt_overrides(self):
        proc, out = self.cli("exec", TRK)
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(out["id"], "r1")
        self.assertEqual(
            [(m, p, b) for m, p, b in self.state.records],
            [("POST", "/execute", {"path": TRK, "mode": "queue"})],
        )

        self.state.records.clear()
        proc, out = self.cli("exec", TRK, "--mode", "interrupt")
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(out["id"], "r2")
        self.assertEqual(
            [(m, p, b) for m, p, b in self.state.records],
            [("POST", "/execute", {"path": TRK, "mode": "interrupt"})],
        )

    def test_repeat_defaults_to_queue(self):
        proc, out = self.cli("repeat", TRK, "-n", "2", "--timeout", "2", "--poll", "0.01")
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(out["done"], 2)
        self.assertEqual(
            [(m, p, b) for m, p, b in self.state.records if m == "POST" and p == "/execute"],
            [
                ("POST", "/execute", {"path": TRK, "mode": "queue"}),
                ("POST", "/execute", {"path": TRK, "mode": "queue"}),
            ],
        )

    def test_run_hold_wait_returns_compact_holding_status(self):
        self.state.run_status["r1"] = [
            {
                "ok": True,
                "id": "r1",
                "state": "holding",
                "frame": 119,
                "frames": 120,
                "time_s": 2.38,
                "duration_s": 2.4,
                "progress": 1,
                "err": None,
                "path": TRK,
                "pose": list(range(64)),
            }
        ]
        proc, out = self.cli("run", TRK, "--hold", "--wait", "--recover", "off", "--timeout", "1", "--poll", "0.01")
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(
            out,
            {
                "ok": True,
                "id": "r1",
                "state": "holding",
                "frame": 119,
                "frames": 120,
                "progress": 1,
                "err": None,
                "ctrl": "standby_velocity",
                "ready": True,
            },
        )
        self.assertEqual(
            [(m, p) for m, p, _ in self.state.records],
            [("POST", "/execute"), ("GET", "/status?id=r1"), ("GET", "/status")],
        )

    def test_repeat_summary_not_verbose(self):
        proc, out = self.cli("repeat", TRK, "-n", "3", "--timeout", "2", "--poll", "0.01")
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(out["done"], 3)
        self.assertNotIn("runs", out)
        self.assertEqual([p for m, p, _ in self.state.records if m == "POST" and p == "/execute"], ["/execute"] * 3)

    def test_repeat_from_fixstand_returns_execute_error_without_recovering(self):
        self.state.ctrl = "fixstand"
        self.state.ready = True
        self.state.execute_failure = {
            "ok": False,
            "error": {"code": "CONTROL_STATE_CONFLICT", "message": "fixstand cannot execute"},
            "next": "standby_velocity",
        }
        proc, out = self.cli("repeat", TRK, "-n", "3", "--timeout", "1", "--poll", "0.01")
        self.assertNotEqual(proc.returncode, 0)
        self.assertEqual(out["error"]["code"], "CONTROL_STATE_CONFLICT")
        self.assertEqual(out["next"], "standby_velocity")
        self.assertEqual(out["ctrl"], "fixstand")
        self.assertEqual(out["ready"], True)
        self.assertEqual(out["i"], 1)
        self.assertEqual(out["done"], 0)
        self.assertEqual(out["n"], 3)
        self.assertEqual(
            [(m, p) for m, p, _ in self.state.records],
            [("POST", "/execute"), ("GET", "/status")],
        )
        self.assertNotIn("/fixstand", [p for _, p, _ in self.state.records])
        self.assertNotIn("/standby_velocity", [p for _, p, _ in self.state.records])

    def test_recover_auto_is_not_available(self):
        proc, out = self.cli("run", TRK, "--recover", "auto")
        self.assertNotEqual(proc.returncode, 0)
        self.assertEqual(out["error"]["code"], "invalid_args")
        self.assertEqual(self.state.records, [])

    def test_raw(self):
        proc, out = self.cli("raw", "POST", "/echo", '{"x":1}')
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(out, {"ok": True, "body": {"x": 1}})

    def test_idle_set_and_clear(self):
        proc, out = self.cli("idle", "set", TRK, "/tmp/et1-idle-b.trk")
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(out, {"ok": True, "idle": {"enabled": True, "n": 2, "active": False}})

        proc, out = self.cli("idle", "clear")
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(out, {"ok": True, "idle": {"enabled": False, "n": 0, "active": False}})
        self.assertEqual(
            [(m, p, b) for m, p, b in self.state.records if m == "POST" and p == "/idle"],
            [
                ("POST", "/idle", {"paths": [TRK, "/tmp/et1-idle-b.trk"]}),
                ("POST", "/idle", {"paths": []}),
            ],
        )

    def test_idle_set_validates_paths_before_post(self):
        proc, out = self.cli("idle", "set", "relative.trk")
        self.assertNotEqual(proc.returncode, 0)
        self.assertFalse(out["ok"])
        self.assertEqual(out["error"]["code"], "bad_path")
        self.assertEqual(self.state.records, [])

    def test_wait_failed_terminal_is_nonzero(self):
        self.state.run_status["bad"] = [{"ok": True, "id": "bad", "state": "failed", "err": "boom"}]
        proc, out = self.cli("wait", "bad", "--timeout", "1", "--poll", "0.01")
        self.assertNotEqual(proc.returncode, 0)
        self.assertFalse(out["ok"])
        self.assertEqual(out["error"]["code"], "run_failed")

    def test_wait_holding_returns_success_without_timeout(self):
        self.state.run_status["held"] = [
            {
                "ok": True,
                "id": "held",
                "state": "holding",
                "frame": 119,
                "frames": 120,
                "time_s": 2.38,
                "duration_s": 2.4,
                "progress": 1,
                "err": None,
                "path": TRK,
                "pose": list(range(64)),
            }
        ]
        proc, out = self.cli("wait", "held", "--timeout", "1", "--poll", "0.01")
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(
            out,
            {
                "ok": True,
                "id": "held",
                "state": "holding",
                "frame": 119,
                "frames": 120,
                "progress": 1,
                "err": None,
            },
        )
        self.assertEqual([(m, p) for m, p, _ in self.state.records], [("GET", "/status?id=held")])

    def test_wait_unknown_id_returns_without_polling(self):
        proc, out = self.cli("wait", "missing", "--timeout", "1", "--poll", "0.01")
        self.assertNotEqual(proc.returncode, 0)
        self.assertEqual(out, {"ok": False, "error": {"code": "unknown_id", "message": "missing"}})
        self.assertEqual([(m, p) for m, p, _ in self.state.records], [("GET", "/status?id=missing")])

    def test_wait_404_json_error_returns_without_polling(self):
        proc, out = self.cli("wait", "gone404", "--timeout", "1", "--poll", "0.01")
        self.assertNotEqual(proc.returncode, 0)
        self.assertEqual(out, {"ok": False, "error": {"code": "not_found", "message": "gone404"}})
        self.assertEqual([(m, p) for m, p, _ in self.state.records], [("GET", "/status?id=gone404")])

    def test_run_wait_missing_execute_id_is_short_error(self):
        self.state.missing_execute_id = True
        proc, out = self.cli("run", TRK, "--wait", "--recover", "off", "--timeout", "1", "--poll", "0.01")
        self.assertNotEqual(proc.returncode, 0)
        self.assertFalse(out["ok"])
        self.assertEqual(out["error"]["code"], "missing_id")
        self.assertEqual(out["ctrl"], "standby_velocity")
        self.assertNotIn(("GET", "/status?id=None"), [(m, p) for m, p, _ in self.state.records])

    def test_run_wait_idle_active_response_is_not_a_run_id(self):
        self.state.execute_idle_response = True
        proc, out = self.cli("run", TRK, "--wait", "--recover", "off", "--timeout", "1", "--poll", "0.01")
        self.assertNotEqual(proc.returncode, 0)
        self.assertFalse(out["ok"])
        self.assertEqual(out["error"]["code"], "missing_id")
        self.assertNotIn(("GET", "/status?id=None"), [(m, p) for m, p, _ in self.state.records])

    def test_exec_wait_missing_execute_id_is_short_error(self):
        self.state.missing_execute_id = True
        proc, out = self.cli("exec", TRK, "--wait", "--timeout", "1", "--poll", "0.01")
        self.assertNotEqual(proc.returncode, 0)
        self.assertFalse(out["ok"])
        self.assertEqual(out["error"]["code"], "missing_id")
        self.assertEqual(out["ctrl"], "standby_velocity")
        self.assertNotIn(("GET", "/status?id=None"), [(m, p) for m, p, _ in self.state.records])

    def test_health_control_commands_omit_large_fields(self):
        self.state.large_health = True
        self.state.large_control = True
        for command in (("health",), ("fixstand",), ("standby",), ("stop", "--urgent")):
            with self.subTest(command=" ".join(command)):
                proc, out = self.cli(*command)
                self.assertEqual(proc.returncode, 0)
                self.assertEqual(out["ok"], True)
                self.assertIn("ctrl", out)
                self.assertIn("ready", out)
                self.assertIn("err", out)
                self.assertNotIn("pose", out)
                self.assertNotIn("samples", out)
                self.assertLess(len(proc.stdout), 160)

        proc, out = self.cli("passive", env_extra={"ET1_PASSIVE_PASSWORD": "large-secret"})
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(out["ok"], True)
        self.assertIn("ctrl", out)
        self.assertIn("ready", out)
        self.assertIn("err", out)
        self.assertNotIn("pose", out)
        self.assertNotIn("samples", out)
        self.assertLess(len(proc.stdout), 160)

    def test_passive_without_password_is_rejected_before_http(self):
        proc, out = self.cli("passive")
        self.assertNotEqual(proc.returncode, 0)
        self.assertEqual(out["error"]["code"], "missing_passive_password")
        self.assertEqual(out["next"], "manual")
        self.assertEqual(self.state.records, [])
        self.assertEqual(proc.stdout.count("\n"), 1)

    def test_passive_password_env_and_cli_override_without_leaking_it(self):
        proc, out = self.cli("passive", env_extra={"ET1_PASSIVE_PASSWORD": "env-secret"})
        self.assertEqual(proc.returncode, 0)
        self.assertEqual([record for record in self.state.records if record[1] == "/passive"], [("POST", "/passive", {"password": "env-secret"})])
        self.assertNotIn("env-secret", proc.stdout)
        self.assertEqual(out["ok"], True)

        self.state.records.clear()
        proc, out = self.cli("passive", "--password", "cli-secret", env_extra={"ET1_PASSIVE_PASSWORD": "env-secret"})
        self.assertEqual(proc.returncode, 0)
        self.assertEqual([record for record in self.state.records if record[1] == "/passive"], [("POST", "/passive", {"password": "cli-secret"})])
        self.assertNotIn("cli-secret", proc.stdout)
        self.assertNotIn("env-secret", proc.stdout)
        self.assertEqual(out["ok"], True)

    def test_docs_restrict_stop_and_passive_recovery(self):
        skill = (ROOT / "SKILL.md").read_text()
        state_machine = (ROOT / "references" / "state-machine.md").read_text()
        raw_http = (ROOT / "references" / "raw-http.md").read_text()
        prompt = (ROOT / "agents" / "openai.yaml").read_text()
        docs = "\n".join((skill, state_machine, raw_http, prompt))
        self.assertIn("Never infer `stop` or `/stop`", skill)
        self.assertIn("Normal requests to stand", skill)
        self.assertIn("preserve idle config", skill)
        self.assertIn("with idle preserved", skill)
        self.assertIn("normal way to leave motion and stand", skill)
        self.assertIn("recover, relax", skill)
        self.assertIn("stop --urgent", skill)
        self.assertIn("`preset-trk/idle/`", skill)
        self.assertIn("tracker-readable motion directory", skill)
        self.assertIn("configured `generated`/`motion_dirs`", skill)
        self.assertIn("idle set /path/to/idle-a.trk", skill)
        self.assertIn("This only sets the idle", skill)
        self.assertIn("After `passive`, never auto-restore", skill)
        self.assertIn("fixstand  # explicit stand configuration/control prep only", skill)
        self.assertIn("fixed-configuration control step", skill)
        self.assertIn("stand-still/velocity", skill)
        self.assertIn("configuration/进入站立构型", skill)
        self.assertIn("fixed configuration / preparation state", state_machine)
        self.assertIn("not normal stand-still", state_machine)
        self.assertIn("不要动/站着别动", state_machine)
        self.assertIn("preserves idle config", state_machine)
        self.assertIn("`idle clear` then `standby`", state_machine)
        self.assertIn("do not infer idle clearing", state_machine)
        self.assertIn("Preset idle loading", state_machine)
        self.assertIn("preset-trk/idle/*.trk", state_machine)
        self.assertIn("tracker-readable allowed/generated motion dir", state_machine)
        self.assertIn("POST /fixstand` for explicit fixed-configuration", raw_http)
        self.assertIn("`/fixstand` is not normal stand-still", raw_http)
        self.assertIn("`/standby_velocity` preserves idle", raw_http)
        self.assertIn("explicit clear-idle flow", raw_http)
        self.assertIn("transition.target==\"user\"", state_machine)
        self.assertIn("Use scripts/et1-trk2motion first for normal work", prompt)
        self.assertIn("normal stop => standby preserving idle config", prompt)
        self.assertIn("clear idle only when explicit", prompt)
        self.assertIn("preset-trk/idle/*.trk", prompt)
        self.assertIn("copy to tracker-readable generated/motion_dirs", prompt)
        self.assertIn("then idle set copied paths", prompt)
        self.assertIn("fixstand only for explicit enter stand configuration/control prep", prompt)
        self.assertNotIn("idle clear, fixstand, standby", prompt)
        self.assertNotIn("gal" + "axy", docs)

    def test_state_short_output_includes_compact_transition(self):
        self.state.ctrl = "running"
        self.state.active = {"kind": "transition", "id": None, "path": "/tmp/drop.trk"}
        self.state.transition.update(
            {
                "active": True,
                "target": "user",
                "target_id": "r2",
                "target_state": "queued",
                "frame": 8,
                "frames": 25,
                "time_s": 0.16,
                "duration_s": 0.5,
                "progress": 0.32,
            }
        )
        proc, out = self.cli("state")
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(out["active"], {"kind": "transition", "id": None})
        self.assertEqual(
            out["transition"],
            {
                "active": True,
                "target": "user",
                "target_id": "r2",
                "frame": 8,
                "frames": 25,
                "progress": 0.32,
            },
        )
        self.assertNotIn("pose", out)
        self.assertNotIn("time_s", out["transition"])

    def test_run_subcommand_timeout_poll_position(self):
        proc, out = self.cli("run", TRK, "--timeout", "2", "--poll", "0.01")
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(out["id"], "r1")

    def test_error_output_single_line_json(self):
        proc, out = self.cli("run", "relative.trk", "--recover", "off")
        self.assertNotEqual(proc.returncode, 0)
        self.assertFalse(out["ok"])
        self.assertEqual(out["error"]["code"], "bad_path")
        self.assertEqual(proc.stdout.count("\n"), 1)


if __name__ == "__main__":
    unittest.main()
