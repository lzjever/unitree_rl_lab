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


class TrackerState:
    def __init__(self):
        self.ctrl = "standby_velocity"
        self.ready = True
        self.err = None
        self.records = []
        self.next_id = 1
        self.run_status = {}
        self.missing_execute_id = False
        self.large_health = False
        self.large_control = False

    def top_status(self):
        return {
            "ok": True,
            "ctrl": self.ctrl,
            "ready": self.ready,
            "err": self.err,
            "pose": list(range(64)),
        }

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
            state.ctrl = "fixstand"
            state.ready = False
            self.send_json(state.response({"ok": True}, state.large_control))
        elif self.path == "/standby_velocity":
            state.ctrl = "standby_velocity"
            state.ready = True
            self.send_json(state.response({"ok": True}, state.large_control))
        elif self.path == "/stop":
            self.send_json(state.response({"ok": True}, state.large_control))
        elif self.path == "/execute":
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
        self.assertEqual(out, {"ok": True, "ctrl": "standby_velocity", "ready": True, "err": None})

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

    def test_run_wait_auto_ready_execute_wait_status(self):
        proc, out = self.cli("run", TRK, "--wait", "--timeout", "2", "--poll", "0.01")
        self.assertEqual(proc.returncode, 0)
        self.assertEqual(out["state"], "done")
        self.assertEqual(out["ctrl"], "standby_velocity")
        self.assertEqual(
            [(m, p) for m, p, _ in self.state.records],
            [("GET", "/status"), ("POST", "/execute"), ("GET", "/status?id=r1"), ("GET", "/status")],
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

    def test_wait_failed_terminal_is_nonzero(self):
        self.state.run_status["bad"] = [{"ok": True, "id": "bad", "state": "failed", "err": "boom"}]
        proc, out = self.cli("wait", "bad", "--timeout", "1", "--poll", "0.01")
        self.assertNotEqual(proc.returncode, 0)
        self.assertFalse(out["ok"])
        self.assertEqual(out["error"]["code"], "run_failed")

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
        for command in ("health", "fixstand", "standby", "stop"):
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
