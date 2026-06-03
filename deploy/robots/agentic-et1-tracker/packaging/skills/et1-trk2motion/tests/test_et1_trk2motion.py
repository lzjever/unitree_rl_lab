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
        self.control_failures = {}
        self.large_health = False
        self.large_control = False
        self.active = {"kind": "none", "id": None}
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

    def cli(self, *args):
        env = os.environ.copy()
        env["ET1_TRACKER_URL"] = self.url
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
            ("fixstand", "/fixstand", "fixstand"),
            ("standby", "/standby_velocity", "standby_velocity"),
            ("stop", "/stop", "stop"),
        )
        for command, path, token in cases:
            with self.subTest(command=command, token=token):
                self.state.control_failures = {
                    path: {"ok": False, "error": {"code": "CONTROL_STATE_CONFLICT", "message": path}, "next": token}
                }
                proc, out = self.cli(command)
                self.assertNotEqual(proc.returncode, 0)
                self.assertEqual(out["next"], token)
                self.assertIn(out["next"], CONTRACT_NEXT_TOKENS)
                self.assertEqual(out["ctrl"], self.state.ctrl)
                self.assertEqual(out["ready"], self.state.ready)

        self.state.control_failures = {
            "/stop": {"ok": False, "error": {"code": "ROBOT_NOT_READY", "message": "not ready"}}
        }
        self.state.top_status_queue = [dict(self.state.top_status(), next="wait_robot")]
        proc, out = self.cli("stop")
        self.assertNotEqual(proc.returncode, 0)
        self.assertEqual(out["next"], "wait_robot")

    def test_ready_from_passive_fixstand_then_standby(self):
        self.state.ctrl = "passive"
        self.state.ready = False
        proc, out = self.cli("ready")
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(out["ctrl"], "standby_velocity")
        self.assertEqual(
            [(m, p) for m, p, _ in self.state.records],
            [("GET", "/status"), ("POST", "/fixstand"), ("GET", "/status"), ("POST", "/standby_velocity"), ("GET", "/status")],
        )

    def test_ready_waits_for_fixstand_after_passive_before_standby(self):
        self.state.ctrl = "passive"
        self.state.ready = False
        self.state.top_status_queue = [
            self.state.queued_top_status("passive", False),
            self.state.queued_top_status("passive", False),
            self.state.queued_top_status("passive", False),
            self.state.queued_top_status("fixstand", True),
            self.state.queued_top_status("standby_velocity", True),
        ]
        proc, out = self.cli("ready", "--timeout", "1", "--poll", "0.01")
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(out, {"ok": True, "ctrl": "standby_velocity", "ready": True, "err": None})
        self.assertEqual([p for m, p, _ in self.state.records if m == "POST"], ["/fixstand", "/standby_velocity"])

    def test_ready_waits_for_standby_after_standby_command(self):
        self.state.ctrl = "fixstand"
        self.state.ready = True
        self.state.top_status_queue = [
            self.state.queued_top_status("fixstand", True),
            self.state.queued_top_status("fixstand", True),
            self.state.queued_top_status("fixstand", True),
            self.state.queued_top_status("standby_velocity", True),
        ]
        proc, out = self.cli("ready", "--timeout", "1", "--poll", "0.01")
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(out, {"ok": True, "ctrl": "standby_velocity", "ready": True, "err": None})
        self.assertEqual([p for m, p, _ in self.state.records if m == "POST"], ["/standby_velocity"])

    def test_ready_loop_timeout_uses_contract_next_and_compact_detail(self):
        for ctrl in ("passive", "fixstand"):
            with self.subTest(ctrl=ctrl):
                self.state = TrackerState()
                self.server.state = self.state
                self.state.ctrl = ctrl
                self.state.ready = False
                self.state.top_status_queue = [self.state.queued_top_status(ctrl, False) for _ in range(32)]
                proc, out = self.cli("ready", "--timeout", "0.08", "--poll", "0.01")
                self.assertNotEqual(proc.returncode, 0)
                self.assertFalse(out["ok"])
                self.assertEqual(out["error"]["code"], "ready_loop")
                self.assertEqual(out["next"], "status")
                self.assertIn(out["next"], CONTRACT_NEXT_TOKENS)
                self.assertEqual(out["ctrl"], ctrl)
                self.assertEqual(out["ready"], False)
                self.assertIsNone(out["err"])
                self.assertEqual(out["last"], {"ctrl": ctrl, "ready": False, "err": None})
                compact_out = json.dumps(out, separators=(",", ":"))
                for field in ("pose", "active", "transition", "idle"):
                    self.assertNotIn(field, compact_out)

    def test_run_wait_auto_ready_execute_wait_status(self):
        proc, out = self.cli("run", TRK, "--wait", "--timeout", "2", "--poll", "0.01")
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(out["state"], "done")
        self.assertEqual(out["ctrl"], "standby_velocity")
        self.assertEqual(
            [(m, p) for m, p, _ in self.state.records],
            [("GET", "/status"), ("POST", "/execute"), ("GET", "/status?id=r1"), ("GET", "/status")],
        )

    def test_run_hold_sends_hold_true_only_when_requested(self):
        proc, out = self.cli("run", TRK, "--hold", "--recover", "off")
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(out["id"], "r1")
        self.assertEqual(
            [(m, p, b) for m, p, b in self.state.records],
            [("POST", "/execute", {"path": TRK, "mode": "interrupt", "hold": True})],
        )

    def test_run_without_hold_does_not_send_hold_false(self):
        proc, out = self.cli("run", TRK, "--recover", "off")
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(out["id"], "r1")
        self.assertEqual(
            [(m, p, b) for m, p, b in self.state.records],
            [("POST", "/execute", {"path": TRK, "mode": "interrupt"})],
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
        for command in ("health", "passive", "fixstand", "standby", "stop"):
            with self.subTest(command=command):
                proc, out = self.cli(command)
                self.assertEqual(proc.returncode, 0)
                self.assertEqual(out["ok"], True)
                self.assertIn("ctrl", out)
                self.assertIn("ready", out)
                self.assertIn("err", out)
                self.assertNotIn("pose", out)
                self.assertNotIn("samples", out)
                self.assertLess(len(proc.stdout), 160)

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
