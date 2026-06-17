import json
import importlib.machinery
import importlib.util
import os
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace
from urllib.parse import parse_qs, urlsplit


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "et1-action"
PACKAGING = ROOT.parents[1]
P0_NEXT = {
    "run-text",
    "run-trk",
    "sequence-start",
    "sequence-status",
    "sequence-append",
    "sequence-replace-tail",
    "sequence-cancel",
    "sequence-interrupt",
    "standby",
    "fixstand",
    "passive",
    "motion-mode",
    "status",
    "urgent-stop",
    "idle-load",
    "cache-clear",
}
DEFAULT_FIELDS = {
    "ok",
    "cmd",
    "intent",
    "seq_id",
    "state",
    "ctrl",
    "active",
    "idle",
    "segments",
    "hold",
    "matched",
    "motion_mode",
    "executor",
    "next",
    "error",
}


class TrackerState:
    def __init__(self):
        self.records = []
        self.next_id = 1
        self.run_state = "running"
        self.status = {
            "ok": True,
            "state": "running",
            "ctrl": "standby_velocity",
            "active": {"kind": "none", "id": None},
            "idle": {"enabled": False, "active": False, "n": 0},
        }
        self.status_sequence = []

    def run_id(self):
        value = f"run{self.next_id}"
        self.next_id += 1
        return value


class Handler(BaseHTTPRequestHandler):
    server_version = "FakeET1Action/1"

    def log_message(self, fmt, *args):
        pass

    def send_json(self, obj, code=200):
        data = json.dumps(obj, separators=(",", ":")).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def body(self):
        size = int(self.headers.get("Content-Length", "0"))
        if not size:
            return None
        return json.loads(self.rfile.read(size).decode("utf-8"))

    def do_GET(self):
        state = self.server.state
        state.records.append(("GET", self.path, None))
        parsed = urlsplit(self.path)
        if parsed.path == "/status":
            query = parse_qs(parsed.query)
            run_id = query.get("id", [""])[0]
            if run_id:
                self.send_json({"ok": True, "id": run_id, "state": state.run_state, "ctrl": "standby_velocity"})
            else:
                if state.status_sequence:
                    self.send_json(state.status_sequence.pop(0))
                else:
                    self.send_json(state.status)
        else:
            self.send_json({"ok": False, "error": {"code": "not_found", "message": self.path}}, 404)

    def do_POST(self):
        state = self.server.state
        payload = self.body()
        state.records.append(("POST", self.path, payload))
        if self.path == "/execute":
            self.send_json({"ok": True, "id": state.run_id()})
        elif self.path == "/execute_loco_upper":
            self.send_json({"ok": True, "id": state.run_id()})
        elif self.path == "/standby_velocity":
            self.send_json({"ok": True, "ctrl": "standby_velocity"})
        elif self.path == "/fixstand":
            self.send_json({"ok": True, "ctrl": "fixstand"})
        elif self.path == "/passive":
            self.send_json({"ok": True, "ctrl": "passive"})
        elif self.path == "/stop":
            self.send_json({"ok": True, "stopped": True})
        elif self.path == "/idle":
            self.send_json({"ok": True, "idle": {"enabled": True, "n": len((payload or {}).get("paths", []))}})
        else:
            self.send_json({"ok": False, "error": {"code": "not_found", "message": self.path}}, 404)


class Et1ActionCase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.base = Path(self.tmp.name)
        self.log = self.base / "calls.jsonl"
        self.stage = self.base / "stage"
        self.state_dir = self.base / "state"
        self.bin = self.base / "bin"
        self.bin.mkdir()
        self.preset_src = self.base / "preset-hit.trk"
        self.preset_src.write_text("preset\n", encoding="utf-8")
        self.ready_trk = self.base / "ready.trk"
        self.ready_trk.write_text("ready\n", encoding="utf-8")
        self.idle_dir = self.base / "idle"
        self.idle_dir.mkdir()
        (self.idle_dir / "idle-a.trk").write_text("idle\n", encoding="utf-8")
        self._write_fake_internal_clis()
        self.tracker = TrackerState()
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.server.state = self.tracker
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.url = f"http://127.0.0.1:{self.server.server_port}"

    def tearDown(self):
        self.server.shutdown()
        self.thread.join(timeout=2)
        self.server.server_close()
        self.tmp.cleanup()

    def _append_log_script(self, name):
        return (
            "import json, os, sys\n"
            "from pathlib import Path\n"
            "log = Path(os.environ['CALL_LOG'])\n"
            f"row = [{name!r}, sys.argv[1:]]\n"
            "with log.open('a', encoding='utf-8') as f:\n"
            "    f.write(json.dumps(row) + '\\n')\n"
        )

    def _write_fake_internal_clis(self):
        preset = self.bin / "find_preset.py"
        preset.write_text(
            "#!/usr/bin/env python3\n"
            + self._append_log_script("preset")
            + """
import os, shutil
from pathlib import Path
mode = os.environ.get("FAKE_PRESET_MODE", "miss")
if mode == "hit":
    stage = Path(sys.argv[sys.argv.index("--stage") + 1])
    stage.mkdir(parents=True, exist_ok=True)
    out = stage / "preset-hit.trk"
    shutil.copy2(os.environ["FAKE_PRESET_SRC"], out)
    print(out)
    raise SystemExit(0)
print("No matching ET1 preset .trk found.")
raise SystemExit(0)
""",
            encoding="utf-8",
        )
        preset.chmod(0o755)

        nl = self.bin / "et1-nl2trk"
        nl.write_text(
            "#!/usr/bin/env python3\n"
            + self._append_log_script("nl2trk")
            + """
import shutil
from pathlib import Path
out = Path(sys.argv[sys.argv.index("--out") + 1])
out.parent.mkdir(parents=True, exist_ok=True)
out.write_text("generated\\n", encoding="utf-8")
if "--debug-dir" in sys.argv:
    debug_dir = Path(sys.argv[sys.argv.index("--debug-dir") + 1])
    debug_dir.mkdir(parents=True, exist_ok=True)
    (debug_dir / "action.bvh").write_text("bvh\\n", encoding="utf-8")
    shutil.copy2(out, debug_dir / "output.trk")
    (debug_dir / "nl2trk-metadata.json").write_text(json.dumps({"ok": True}) + "\\n", encoding="utf-8")
print(json.dumps({"ok": True}))
""",
            encoding="utf-8",
        )
        nl.chmod(0o755)

    def env(self, **extra):
        env = os.environ.copy()
        env.update(
            {
                "CALL_LOG": str(self.log),
                "ET1_TRACKER_URL": self.url,
                "ET1_ACTION_STATE_DIR": str(self.state_dir),
                "ET1_ACTION_PRESET_FINDER": str(self.bin / "find_preset.py"),
                "ET1_ACTION_NL2TRK_BIN": str(self.bin / "et1-nl2trk"),
                "ET1_ACTION_DISABLE_TRACKER_CONFIG_DISCOVERY": "1",
                "FAKE_PRESET_SRC": str(self.preset_src),
                "ET1_ACTION_WORKER_MAX_POLLS": "1",
                "PYTHONUNBUFFERED": "1",
            }
        )
        env.update(extra)
        return env

    def cli(self, *args, env_extra=None, check=True, cwd=None):
        proc = subprocess.run(
            [sys.executable, str(SCRIPT), *args],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=self.env(**(env_extra or {})),
            cwd=cwd,
            timeout=10,
        )
        lines = proc.stdout.splitlines()
        self.assertEqual(len(lines), 1, proc.stdout)
        out = json.loads(lines[0])
        if check and proc.returncode != 0:
            self.fail(f"rc={proc.returncode}\nstdout={proc.stdout}\nstderr={proc.stderr}")
        self.assertIn(out.get("next"), P0_NEXT)
        self.assertLessEqual(set(out), DEFAULT_FIELDS)
        return out, proc

    def calls(self):
        if not self.log.exists():
            return []
        return [json.loads(line) for line in self.log.read_text(encoding="utf-8").splitlines()]

    def wait_for_execs(self, count, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            records = [r for r in self.tracker.records if r[0] == "POST" and r[1] in {"/execute", "/execute_loco_upper"}]
            if len(records) >= count:
                return records
            time.sleep(0.05)
        return [r for r in self.tracker.records if r[0] == "POST" and r[1] in {"/execute", "/execute_loco_upper"}]

    def load_action_module(self):
        name = f"et1_action_under_test_{time.time_ns()}"
        loader = importlib.machinery.SourceFileLoader(name, str(SCRIPT))
        spec = importlib.util.spec_from_loader(name, loader)
        module = importlib.util.module_from_spec(spec)
        loader.exec_module(module)
        return module

    def write_active_sequence(self, seq_id="seq_old"):
        self.state_dir.mkdir(parents=True, exist_ok=True)
        path = self.state_dir / f"{seq_id}.json"
        path.write_text(
            json.dumps(
                {
                    "seq_id": seq_id,
                    "state": "running",
                    "segments": [
                        {
                            "segment_id": "s1",
                            "trk": str(self.ready_trk),
                            "ready_path": str(self.ready_trk),
                            "status": "running",
                            "submitted": True,
                        },
                        {
                            "segment_id": "s2",
                            "text": "old tail",
                            "status": "pending",
                            "submitted": False,
                        },
                    ],
                    "active": {"segment_id": "s1", "run_id": "run-old"},
                    "serial_id": seq_id,
                    "stage_dir": str(self.stage),
                    "op_epoch": 0,
                    "version": 0,
                    "pid": None,
                    "heartbeat_at": time.time(),
                    "last_error": None,
                    "last_cmd": "sequence-status",
                    "next": "sequence-status",
                }
            ),
            encoding="utf-8",
        )
        return path

    def test_one_line_json_and_next_is_single_p0_command(self):
        out, _ = self.cli("standby")
        self.assertTrue(out["ok"])
        self.assertEqual(out["cmd"], "standby")
        self.assertNotIn("|", out["next"])
        self.assertNotIn(",", out["next"])

    def test_standby_reports_idle_when_idle_config_takes_over(self):
        self.tracker.status = {
            "ok": True,
            "ctrl": "preparing",
            "active": {"kind": "idle", "id": None},
            "idle": {"enabled": True, "active": True, "n": 2},
        }
        out, _ = self.cli("standby")
        self.assertTrue(out["ok"])
        self.assertEqual(out["cmd"], "standby")
        self.assertEqual(out["state"], "idle")
        self.assertEqual(out["ctrl"], "preparing")
        self.assertEqual(out["active"], {"kind": "idle"})
        self.assertEqual(out["idle"], {"enabled": True, "active": True, "n": 2})
        self.assertEqual([record[1] for record in self.tracker.records], ["/standby_velocity", "/status"])

    def test_standby_waits_until_user_motion_leaves_active_state(self):
        self.tracker.status_sequence = [
            {
                "ok": True,
                "ctrl": "running",
                "active": {"kind": "user", "id": "run1"},
                "idle": {"enabled": False, "active": False, "n": 0},
            },
            {
                "ok": True,
                "ctrl": "standby_velocity",
                "active": {"kind": "none", "id": None},
                "idle": {"enabled": False, "active": False, "n": 0},
            },
        ]
        out, _ = self.cli(
            "--poll",
            "0.01",
            "standby",
            env_extra={"ET1_ACTION_STANDBY_CONFIRM_S": "1"},
        )
        self.assertTrue(out["ok"])
        self.assertEqual(out["state"], "standby")
        self.assertEqual(
            [record[1] for record in self.tracker.records],
            ["/standby_velocity", "/status", "/status"],
        )

    def test_standby_returns_clear_error_when_motion_stays_active(self):
        self.tracker.status = {
            "ok": True,
            "ctrl": "running",
            "active": {"kind": "user", "id": "run1"},
            "idle": {"enabled": False, "active": False, "n": 0},
        }
        out, proc = self.cli(
            "--poll",
            "0.01",
            "standby",
            env_extra={"ET1_ACTION_STANDBY_CONFIRM_S": "0.01"},
            check=False,
        )
        self.assertNotEqual(proc.returncode, 0)
        self.assertFalse(out["ok"])
        self.assertEqual(out["cmd"], "standby")
        self.assertEqual(out["state"], "stopping")
        self.assertEqual(out["error"]["code"], "STOP_NOT_CONFIRMED")
        self.assertEqual(out["next"], "status")

    def test_run_text_preset_hit_skips_nl2trk_and_runs_staged_path(self):
        out, _ = self.cli("run-text", "wave", "--stage-dir", str(self.stage), env_extra={"FAKE_PRESET_MODE": "hit"})
        self.assertTrue(out["ok"])
        self.assertEqual(out["motion_mode"], "fullbody")
        self.assertEqual(out["executor"], "fullbody")
        self.assertEqual([c[0] for c in self.calls()], ["preset"])
        execute = self.tracker.records[-1]
        self.assertEqual(execute[1], "/execute")
        self.assertEqual(execute[2]["mode"], "interrupt")
        self.assertTrue(Path(execute[2]["path"]).is_relative_to(self.stage))
        self.assertTrue((Path(execute[2]["path"]).parent / "prompt.txt").exists())
        self.assertTrue((Path(execute[2]["path"]).parent / "meta.json").exists())
        self.assertFalse((Path(execute[2]["path"]).parent / "action.bvh").exists())

    def test_motion_mode_switches_later_runs_to_loco_upper(self):
        out, _ = self.cli("motion-mode")
        self.assertTrue(out["ok"])
        self.assertEqual(out["motion_mode"], "fullbody")
        self.assertEqual(out["executor"], "fullbody")

        out, _ = self.cli("motion-mode", "base")
        self.assertTrue(out["ok"])
        self.assertEqual(out["motion_mode"], "base")
        self.assertEqual(out["executor"], "loco_upper")

        out, _ = self.cli("run-trk", str(self.ready_trk))
        self.assertTrue(out["ok"])
        self.assertEqual(out["motion_mode"], "base")
        self.assertEqual(out["executor"], "loco_upper")
        execute = self.tracker.records[-1]
        self.assertEqual(execute[1], "/execute_loco_upper")
        self.assertEqual(execute[2]["mode"], "interrupt")
        self.assertEqual(execute[2]["path"], str(self.ready_trk))

        out, _ = self.cli("motion-mode", "fullbody")
        self.assertTrue(out["ok"])
        self.assertEqual(out["motion_mode"], "fullbody")

        out, _ = self.cli("run-trk", str(self.ready_trk))
        self.assertTrue(out["ok"])
        self.assertEqual(out["motion_mode"], "fullbody")
        self.assertEqual(out["executor"], "fullbody")
        self.assertEqual(self.tracker.records[-1][1], "/execute")

    def test_run_text_cancels_existing_sequence_before_interrupt_execute(self):
        state_path = self.write_active_sequence()
        out, _ = self.cli(
            "run-text",
            "wave",
            "--stage-dir",
            str(self.stage),
            env_extra={"FAKE_PRESET_MODE": "hit"},
        )
        self.assertTrue(out["ok"])
        state = json.loads(state_path.read_text(encoding="utf-8"))
        self.assertEqual(state["state"], "canceled")
        self.assertIsNone(state["active"])
        self.assertEqual([seg["status"] for seg in state["segments"]], ["canceled", "canceled"])
        self.assertEqual(self.tracker.records[-1][2]["mode"], "interrupt")

    def test_standby_cancels_existing_sequence(self):
        state_path = self.write_active_sequence()
        out, _ = self.cli("standby")
        self.assertTrue(out["ok"])
        state = json.loads(state_path.read_text(encoding="utf-8"))
        self.assertEqual(state["state"], "canceled")
        self.assertIsNone(state["active"])
        self.assertEqual([seg["status"] for seg in state["segments"]], ["canceled", "canceled"])
        self.assertEqual(self.tracker.records[0][1], "/standby_velocity")

    def test_sequence_start_cancels_existing_sequence(self):
        old_path = self.write_active_sequence()
        plan = {"segments": [{"trk": str(self.ready_trk)}]}
        out, _ = self.cli(
            "sequence-start",
            "--plan-json",
            json.dumps(plan),
            "--stage-dir",
            str(self.stage),
            env_extra={"ET1_ACTION_WORKER_MAX_POLLS": "1"},
        )
        self.assertTrue(out["ok"])
        old_state = json.loads(old_path.read_text(encoding="utf-8"))
        self.assertEqual(old_state["state"], "canceled")
        execs = self.wait_for_execs(1)
        self.assertEqual(execs[-1][2]["mode"], "interrupt")

    def test_run_text_preset_miss_calls_nl2trk_then_tracker_execute(self):
        out, _ = self.cli("run-text", "slow turn", "--duration", "3", "--stage-dir", str(self.stage))
        self.assertTrue(out["ok"])
        calls = self.calls()
        self.assertEqual([c[0] for c in calls], ["preset", "nl2trk"])
        self.assertIn("--duration", calls[1][1])
        self.assertNotIn("--serial-id", calls[1][1])
        self.assertEqual(self.tracker.records[-1][1], "/execute")
        self.assertEqual(self.tracker.records[-1][2]["mode"], "interrupt")
        trk = Path(self.tracker.records[-1][2]["path"])
        artifact_dir = trk.parent
        self.assertEqual(artifact_dir.parent, self.stage)
        self.assertTrue((artifact_dir / "prompt.txt").exists())
        self.assertEqual((artifact_dir / "prompt.txt").read_text(encoding="utf-8"), "slow turn\n")
        meta = json.loads((artifact_dir / "meta.json").read_text(encoding="utf-8"))
        self.assertEqual(meta["prompt"], "slow turn")
        self.assertEqual(meta["source"], "nl2trk")
        self.assertEqual(meta["trk"], str(trk.resolve()))
        self.assertTrue((artifact_dir / "manifest.json").exists())
        self.assertFalse((artifact_dir / "action.bvh").exists())

    def test_run_text_debug_exports_bvh_trk_meta_and_prompt_and_bypasses_preset(self):
        out, _ = self.cli(
            "run-text",
            "wave",
            "--duration",
            "3",
            "--stage-dir",
            str(self.stage),
            env_extra={"ET1_ACTION_DEBUG": "true", "FAKE_PRESET_MODE": "hit"},
        )
        self.assertTrue(out["ok"])
        calls = self.calls()
        self.assertEqual([c[0] for c in calls], ["nl2trk"])
        self.assertIn("--fresh", calls[0][1])
        self.assertIn("--debug-dir", calls[0][1])
        self.assertNotIn("--serial-id", calls[0][1])
        trk = Path(self.tracker.records[-1][2]["path"])
        artifact_dir = trk.parent
        self.assertEqual(Path(calls[0][1][calls[0][1].index("--debug-dir") + 1]), artifact_dir)
        self.assertTrue((artifact_dir / "action.bvh").exists())
        self.assertTrue((artifact_dir / "output.trk").exists())
        self.assertTrue((artifact_dir / "prompt.txt").exists())
        self.assertTrue((artifact_dir / "meta.json").exists())
        meta = json.loads((artifact_dir / "meta.json").read_text(encoding="utf-8"))
        self.assertTrue(meta["debug"])

    def test_run_text_debug_off_does_not_export_bvh(self):
        out, _ = self.cli("run-text", "slow turn", "--duration", "3", "--stage-dir", str(self.stage))
        self.assertTrue(out["ok"])
        artifact_dir = Path(self.tracker.records[-1][2]["path"]).parent
        self.assertFalse((artifact_dir / "action.bvh").exists())
        self.assertNotIn("--debug-dir", self.calls()[1][1])

    def test_run_text_seed_and_diffusion_steps_bypass_preset(self):
        out, _ = self.cli(
            "run-text",
            "wave",
            "--duration",
            "4",
            "--seed",
            "123",
            "--diffusion-steps",
            "80",
            "--stage-dir",
            str(self.stage),
            env_extra={"FAKE_PRESET_MODE": "hit"},
        )
        self.assertTrue(out["ok"])
        calls = self.calls()
        self.assertEqual([c[0] for c in calls], ["nl2trk"])
        self.assertIn("--seed", calls[0][1])
        self.assertIn("123", calls[0][1])
        self.assertIn("--diffusion-steps", calls[0][1])
        self.assertIn("80", calls[0][1])
        self.assertEqual(self.tracker.records[-1][1], "/execute")

    def test_run_text_precise_hold_limb_action_bypasses_preset_and_sends_hold(self):
        out, _ = self.cli(
            "run-text",
            "A person raises the right arm up and holds the pose steadily.",
            "--duration",
            "4",
            "--stage-dir",
            str(self.stage),
            env_extra={"FAKE_PRESET_MODE": "hit"},
        )
        self.assertTrue(out["ok"])
        calls = self.calls()
        self.assertEqual([c[0] for c in calls], ["nl2trk"])
        execute = self.tracker.records[-1]
        self.assertEqual(execute[1], "/execute")
        self.assertEqual(execute[2]["mode"], "interrupt")
        self.assertTrue(execute[2]["hold"])
        self.assertTrue(out["hold"])

    def test_run_text_explicit_no_preset_and_hold(self):
        out, _ = self.cli(
            "run-text",
            "A person holds a salute pose.",
            "--hold",
            "--no-preset",
            "--stage-dir",
            str(self.stage),
            env_extra={"FAKE_PRESET_MODE": "hit"},
        )
        self.assertTrue(out["ok"])
        self.assertEqual([c[0] for c in self.calls()], ["nl2trk"])
        self.assertTrue(self.tracker.records[-1][2]["hold"])

    def test_run_trk_hold_sends_hold_to_tracker(self):
        out, _ = self.cli("run-trk", str(self.ready_trk), "--hold")
        self.assertTrue(out["ok"])
        self.assertEqual(self.tracker.records[-1][1], "/execute")
        self.assertTrue(self.tracker.records[-1][2]["hold"])
        self.assertTrue(out["hold"])

    def test_run_trk_relative_path_resolves_under_user_motion_and_adds_suffix(self):
        work = self.base / "work"
        motion = work / "generated" / "user-motion" / "sub" / "wave.trk"
        motion.parent.mkdir(parents=True)
        motion.write_text("wave\n", encoding="utf-8")

        out, _ = self.cli("run-trk", "sub/wave", cwd=work)

        self.assertTrue(out["ok"])
        self.assertEqual(self.tracker.records[-1][1], "/execute")
        self.assertEqual(self.tracker.records[-1][2]["path"], str(motion.resolve()))
        self.assertNotIn("matched", out)

    def test_run_trk_relative_best_match_uses_find_user_motion(self):
        work = self.base / "work"
        motion = work / "generated" / "user-motion" / "nested" / "friendly-wave.trk"
        motion.parent.mkdir(parents=True)
        motion.write_text("wave\n", encoding="utf-8")

        out, _ = self.cli("run-trk", "wave", cwd=work)

        self.assertTrue(out["ok"])
        self.assertEqual(self.tracker.records[-1][2]["path"], str(motion.resolve()))
        self.assertEqual(out["matched"], {"query": "wave", "file": "nested/friendly-wave.trk"})

    def test_run_trk_relative_best_match_ambiguous_is_request_invalid(self):
        work = self.base / "work"
        root = work / "generated" / "user-motion"
        (root / "a").mkdir(parents=True)
        (root / "b").mkdir(parents=True)
        (root / "a" / "friendly-wave.trk").write_text("a\n", encoding="utf-8")
        (root / "b" / "other-wave.trk").write_text("b\n", encoding="utf-8")

        out, proc = self.cli("run-trk", "wave", cwd=work, check=False)

        self.assertNotEqual(proc.returncode, 0)
        self.assertFalse(out["ok"])
        self.assertEqual(out["error"]["code"], "REQUEST_INVALID")
        self.assertEqual([r for r in self.tracker.records if r[1] == "/execute"], [])

    def test_run_trk_relative_rejects_parent_escape_and_symlink_escape(self):
        work = self.base / "work"
        root = work / "generated" / "user-motion"
        root.mkdir(parents=True)
        outside = self.base / "outside.trk"
        outside.write_text("outside\n", encoding="utf-8")
        os.symlink(outside, root / "escape.trk")

        parent, parent_proc = self.cli("run-trk", "../outside", cwd=work, check=False)
        symlink, symlink_proc = self.cli("run-trk", "escape", cwd=work, check=False)

        self.assertNotEqual(parent_proc.returncode, 0)
        self.assertEqual(parent["error"]["code"], "REQUEST_INVALID")
        self.assertNotEqual(symlink_proc.returncode, 0)
        self.assertEqual(symlink["error"]["code"], "REQUEST_INVALID")
        self.assertEqual([r for r in self.tracker.records if r[1] == "/execute"], [])

    def test_run_trk_rejects_et1trk_extension(self):
        work = self.base / "work"
        motion = work / "generated" / "user-motion" / "old.et1trk"
        motion.parent.mkdir(parents=True)
        motion.write_text("old\n", encoding="utf-8")

        out, proc = self.cli("run-trk", "old.et1trk", cwd=work, check=False)

        self.assertNotEqual(proc.returncode, 0)
        self.assertEqual(out["error"]["code"], "REQUEST_INVALID")
        self.assertEqual([r for r in self.tracker.records if r[1] == "/execute"], [])

    def test_default_stage_dir_uses_tracker_config_motion_dir(self):
        old_config = os.environ.get("ET1_TRACKER_CONFIG")
        old_disable = os.environ.get("ET1_ACTION_DISABLE_TRACKER_CONFIG_DISCOVERY")
        old_stage = os.environ.get("ET1_ACTION_STAGE_DIR")
        motion_dir = self.base / "allowed"
        config = self.base / "config.yaml"
        config.write_text(
            "agentic_et1_tracker:\n"
            "  motion_dirs:\n"
            f"    - \"{motion_dir}\"\n",
            encoding="utf-8",
        )
        os.environ["ET1_TRACKER_CONFIG"] = str(config)
        os.environ.pop("ET1_ACTION_DISABLE_TRACKER_CONFIG_DISCOVERY", None)
        os.environ.pop("ET1_ACTION_STAGE_DIR", None)
        try:
            module = self.load_action_module()
            self.assertEqual(module.default_stage_dir(), motion_dir)
            self.assertTrue(motion_dir.is_dir())
        finally:
            if old_config is None:
                os.environ.pop("ET1_TRACKER_CONFIG", None)
            else:
                os.environ["ET1_TRACKER_CONFIG"] = old_config
            if old_disable is None:
                os.environ.pop("ET1_ACTION_DISABLE_TRACKER_CONFIG_DISCOVERY", None)
            else:
                os.environ["ET1_ACTION_DISABLE_TRACKER_CONFIG_DISCOVERY"] = old_disable
            if old_stage is None:
                os.environ.pop("ET1_ACTION_STAGE_DIR", None)
            else:
                os.environ["ET1_ACTION_STAGE_DIR"] = old_stage

    def test_run_trk_uses_direct_tracker_http_not_old_skill_cli(self):
        out, _ = self.cli("run-trk", str(self.ready_trk))
        self.assertTrue(out["ok"])
        self.assertEqual(self.calls(), [])
        self.assertEqual(self.tracker.records[-1][2]["mode"], "interrupt")

    def test_standby_and_urgent_stop_guard(self):
        self.cli("standby")
        bad, proc = self.cli("urgent-stop", check=False)
        self.assertNotEqual(proc.returncode, 0)
        self.assertFalse(bad["ok"])
        self.assertEqual(bad["error"]["code"], "REQUEST_INVALID")
        self.cli("urgent-stop", "--urgent")
        paths = [record[1] for record in self.tracker.records if record[0] == "POST"]
        self.assertEqual(paths, ["/standby_velocity", "/stop"])

    def test_fixstand_is_explicit_stand_configuration(self):
        out, _ = self.cli("fixstand")
        self.assertTrue(out["ok"])
        self.assertEqual(out["cmd"], "fixstand")
        self.assertEqual(out["intent"], "explicit_stand_configuration")
        self.assertEqual(self.tracker.records[-1][1], "/fixstand")

    def test_passive_requires_password(self):
        out, proc = self.cli("passive", check=False)
        self.assertNotEqual(proc.returncode, 0)
        self.assertFalse(out["ok"])
        self.assertEqual(out["cmd"], "passive")
        self.assertEqual(out["error"]["code"], "REQUEST_INVALID")
        self.assertEqual(out["next"], "fixstand")
        self.assertEqual([r for r in self.tracker.records if r[1] == "/passive"], [])

    def test_passive_posts_password_and_cancels_existing_sequence(self):
        state_path = self.write_active_sequence()
        out, _ = self.cli("passive", "--password", "secret")
        self.assertTrue(out["ok"])
        self.assertEqual(out["cmd"], "passive")
        self.assertEqual(out["intent"], "explicit_passive")
        self.assertEqual(out["state"], "passive")
        self.assertEqual(out["next"], "fixstand")
        state = json.loads(state_path.read_text(encoding="utf-8"))
        self.assertEqual(state["state"], "canceled")
        self.assertEqual(self.tracker.records[-1][1], "/passive")
        self.assertEqual(self.tracker.records[-1][2], {"password": "secret"})

    def test_status_public_alias_queries_tracker_status(self):
        out, _ = self.cli("status")
        self.assertTrue(out["ok"])
        self.assertEqual(out["cmd"], "status")
        self.assertEqual(out["state"], "running")
        self.assertEqual(self.tracker.records[-1][1], "/status")

    def test_idle_load_stages_files_and_calls_idle_set(self):
        out, _ = self.cli("idle-load", "--preset-dir", str(self.idle_dir), "--stage-dir", str(self.stage))
        self.assertTrue(out["ok"])
        staged = list(self.stage.glob("*.trk"))
        self.assertEqual(len(staged), 1)
        idle = self.tracker.records[-1]
        self.assertEqual(idle[1], "/idle")
        self.assertEqual(Path(idle[2]["paths"][0]).parent, self.stage)

    def test_cache_clear_removes_generation_cache_only(self):
        root = self.base / "nl2trk-root"
        cache_file = root / "cache" / "keys" / "old.json"
        tmp_file = root / "tmp" / "partial"
        generated_file = root / "generated" / "kept.trk"
        cache_file.parent.mkdir(parents=True)
        tmp_file.parent.mkdir(parents=True)
        generated_file.parent.mkdir(parents=True)
        cache_file.write_text("cache\n", encoding="utf-8")
        tmp_file.write_text("tmp\n", encoding="utf-8")
        generated_file.write_text("generated\n", encoding="utf-8")

        out, _ = self.cli("cache-clear", "--root", str(root))

        self.assertTrue(out["ok"])
        self.assertEqual(out["cmd"], "cache-clear")
        self.assertEqual(out["state"], "cleared")
        self.assertFalse((root / "cache").exists())
        self.assertFalse((root / "tmp").exists())
        self.assertTrue(generated_file.exists())

    def test_sequence_start_queues_ahead_while_current_segment_runs(self):
        plan = {"segments": [{"trk": str(self.ready_trk)}, {"text": "second"}]}
        out, _ = self.cli(
            "sequence-start",
            "--plan-json",
            json.dumps(plan),
            "--stage-dir",
            str(self.stage),
            env_extra={"ET1_ACTION_WORKER_MAX_POLLS": "1"},
        )
        self.assertTrue(out["ok"])
        execs = self.wait_for_execs(2)
        self.assertEqual(len(execs), 2, self.tracker.records)
        self.assertEqual(execs[0][2]["path"], str(self.ready_trk))
        self.assertEqual(execs[0][2]["mode"], "interrupt")
        self.assertEqual(execs[1][2]["mode"], "queue")
        self.assertEqual([c[0] for c in self.calls()], ["preset", "nl2trk"])
        seq_id = out["seq_id"]
        state = json.loads((self.state_dir / f"{seq_id}.json").read_text(encoding="utf-8"))
        self.assertEqual(state["active"]["segment_id"], "s1")
        self.assertEqual(state["segments"][1]["status"], "queued")
        self.assertTrue(state["segments"][1]["submitted"])

    def test_sequence_text_segments_share_serial_artifact_root(self):
        plan = {"segments": [{"text": "first"}, {"text": "second"}]}
        out, _ = self.cli(
            "sequence-start",
            "--plan-json",
            json.dumps(plan),
            "--serial-id",
            "serial_demo",
            "--stage-dir",
            str(self.stage),
            env_extra={"ET1_ACTION_WORKER_MAX_POLLS": "1"},
        )
        self.assertTrue(out["ok"])
        execs = self.wait_for_execs(2)
        self.assertEqual(len(execs), 2, self.tracker.records)
        paths = [Path(execute[2]["path"]) for execute in execs]
        serial_root = self.stage / "serial_demo"
        self.assertEqual(paths[0].parents[1], serial_root)
        self.assertEqual(paths[1].parents[1], serial_root)
        self.assertEqual(paths[0].parent.name, "s1")
        self.assertEqual(paths[1].parent.name, "s2")
        for idx, path in enumerate(paths, 1):
            artifact_dir = path.parent
            self.assertTrue((artifact_dir / "prompt.txt").exists())
            self.assertTrue((artifact_dir / "meta.json").exists())
            meta = json.loads((artifact_dir / "meta.json").read_text(encoding="utf-8"))
            self.assertEqual(meta["serial_id"], "serial_demo")
            self.assertEqual(meta["segment_id"], f"s{idx}")
            self.assertEqual(meta["trk"], str(path.resolve()))
        manifest = json.loads((serial_root / "manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(manifest["artifact_id"], "serial_demo")
        self.assertEqual([segment["segment_id"] for segment in manifest["segments"]], ["s1", "s2"])

    def test_append_and_replace_tail_modify_only_unsubmitted_segments(self):
        plan = {"segments": [{"trk": str(self.ready_trk)}, {"text": "old tail"}]}
        out, _ = self.cli(
            "sequence-start",
            "--plan-json",
            json.dumps(plan),
            "--stage-dir",
            str(self.stage),
            env_extra={"ET1_ACTION_QUEUE_AHEAD": "1"},
        )
        seq_id = out["seq_id"]
        self.wait_for_execs(1)
        self.cli("sequence-append", seq_id, "--plan-json", json.dumps({"segments": [{"text": "append"}]}))
        self.cli("sequence-replace-tail", seq_id, "--plan-json", json.dumps({"segments": [{"text": "new tail"}]}))
        state = json.loads((self.state_dir / f"{seq_id}.json").read_text(encoding="utf-8"))
        submitted = [s for s in state["segments"] if s.get("submitted")]
        self.assertEqual(len(submitted), 1)
        self.assertEqual(state["segments"][-1]["text"], "new tail")

    def test_sequence_mutations_survive_concurrent_worker_heartbeat(self):
        old_state_dir = os.environ.get("ET1_ACTION_STATE_DIR")
        os.environ["ET1_ACTION_STATE_DIR"] = str(self.state_dir)
        try:
            module = self.load_action_module()
            cases = [
                ("append", ("sequence-append", "seq_append", "--plan-json", json.dumps({"segments": [{"text": "added"}]}))),
                ("replace", ("sequence-replace-tail", "seq_replace", "--plan-json", json.dumps({"segments": [{"text": "replacement"}]}))),
                ("cancel", ("sequence-cancel", "seq_cancel")),
            ]
            for label, argv in cases:
                seq_id = argv[1]
                def create(state, seq_id=seq_id):
                    state.update(
                        {
                            "seq_id": seq_id,
                            "state": "running",
                            "segments": [
                                {"segment_id": "s1", "trk": str(self.ready_trk), "ready_path": str(self.ready_trk), "status": "running", "submitted": True},
                                {"segment_id": "s2", "text": "old tail", "status": "pending", "submitted": False},
                            ],
                            "active": {"segment_id": "s1", "run_id": "run-live"},
                            "serial_id": seq_id,
                            "stage_dir": str(self.stage),
                            "pid": 12345,
                            "heartbeat_at": time.time(),
                            "last_error": None,
                            "last_cmd": "sequence-status",
                            "next": "sequence-status",
                        }
                    )
                module.update_state(seq_id, create, create=True)

                stop = threading.Event()
                def heartbeat():
                    while not stop.is_set():
                        def beat(state):
                            state["heartbeat_at"] = time.time()
                            state["pid"] = 54321
                            state["last_cmd"] = "sequence-status"
                        module.update_state(seq_id, beat)
                        time.sleep(0.001)

                thread = threading.Thread(target=heartbeat)
                thread.start()
                time.sleep(0.02)
                self.cli(*argv)
                time.sleep(0.02)
                stop.set()
                thread.join(timeout=2)

                state = module.load_state(seq_id)
                texts = [seg.get("text") for seg in state["segments"]]
                if label == "append":
                    self.assertIn("added", texts)
                    self.assertIn("old tail", texts)
                elif label == "replace":
                    self.assertNotIn("old tail", texts)
                    self.assertIn("replacement", texts)
                    self.assertEqual(len(state["segments"]), 2)
                else:
                    self.assertEqual(state["state"], "canceled")
                    self.assertEqual(state["segments"][1]["status"], "canceled")
        finally:
            if old_state_dir is None:
                os.environ.pop("ET1_ACTION_STATE_DIR", None)
            else:
                os.environ["ET1_ACTION_STATE_DIR"] = old_state_dir

    def test_worker_revalidates_before_execute_after_cancel_window(self):
        old_state_dir = os.environ.get("ET1_ACTION_STATE_DIR")
        old_max_polls = os.environ.get("ET1_ACTION_WORKER_MAX_POLLS")
        os.environ["ET1_ACTION_STATE_DIR"] = str(self.state_dir)
        os.environ["ET1_ACTION_WORKER_MAX_POLLS"] = "1"
        try:
            module = self.load_action_module()
            seq_id = "seq_worker_race"
            late_trk = self.stage / "late.trk"

            def create(state):
                state.update(
                    {
                        "seq_id": seq_id,
                        "state": "running",
                        "segments": [{"segment_id": "s1", "text": "slow", "status": "pending", "submitted": False}],
                        "active": None,
                        "serial_id": seq_id,
                        "stage_dir": str(self.stage),
                        "op_epoch": 0,
                        "version": 0,
                        "pid": 12345,
                        "heartbeat_at": time.time(),
                        "last_error": None,
                        "last_cmd": "sequence-status",
                        "next": "sequence-status",
                    }
                )

            module.update_state(seq_id, create, create=True)
            module.tracker_standby = lambda base, timeout: {"ok": True}

            def fake_resolve(text, duration, stage, serial_id, seed=None, diffusion_steps=None, **kwargs):
                module.sequence_cancel_cmd(SimpleNamespace(seq_id=seq_id), "http://unused", 0.1)
                late_trk.parent.mkdir(parents=True, exist_ok=True)
                late_trk.write_text("late\n", encoding="utf-8")
                return str(late_trk), "test"

            execute_calls = []
            module.resolve_text_to_trk = fake_resolve
            module.execute_trk = lambda *args: execute_calls.append(args) or {"ok": True, "id": "unexpected"}

            rc = module.worker_cmd(SimpleNamespace(seq_id=seq_id), "http://unused", 0.1, 0.001)
            state = module.load_state(seq_id)
            self.assertEqual(rc, 0)
            self.assertEqual(execute_calls, [])
            self.assertEqual(state["state"], "canceled")
            self.assertEqual(state["segments"][0]["status"], "canceled")
        finally:
            if old_state_dir is None:
                os.environ.pop("ET1_ACTION_STATE_DIR", None)
            else:
                os.environ["ET1_ACTION_STATE_DIR"] = old_state_dir
            if old_max_polls is None:
                os.environ.pop("ET1_ACTION_WORKER_MAX_POLLS", None)
            else:
                os.environ["ET1_ACTION_WORKER_MAX_POLLS"] = old_max_polls

    def test_interrupt_finish_does_not_revive_after_cancel_window(self):
        old_state_dir = os.environ.get("ET1_ACTION_STATE_DIR")
        os.environ["ET1_ACTION_STATE_DIR"] = str(self.state_dir)
        try:
            module = self.load_action_module()
            seq_id = "seq_interrupt_race"
            generated = self.stage / "interrupt-late.trk"

            def create(state):
                state.update(
                    {
                        "seq_id": seq_id,
                        "state": "running",
                        "segments": [
                            {
                                "segment_id": "s1",
                                "trk": str(self.ready_trk),
                                "ready_path": str(self.ready_trk),
                                "status": "running",
                                "submitted": True,
                            },
                            {"segment_id": "s2", "text": "tail", "status": "pending", "submitted": False},
                        ],
                        "active": {"segment_id": "s1", "run_id": "run-live"},
                        "serial_id": seq_id,
                        "stage_dir": str(self.stage),
                        "op_epoch": 0,
                        "version": 0,
                        "pid": 12345,
                        "heartbeat_at": time.time(),
                        "last_error": None,
                        "last_cmd": "sequence-status",
                        "next": "sequence-status",
                    }
                )

            module.update_state(seq_id, create, create=True)
            module.tracker_standby = lambda base, timeout: {"ok": True}

            def fake_resolve(text, duration, stage, serial_id, seed=None, diffusion_steps=None, **kwargs):
                module.sequence_cancel_cmd(SimpleNamespace(seq_id=seq_id), "http://unused", 0.1)
                generated.parent.mkdir(parents=True, exist_ok=True)
                generated.write_text("generated\n", encoding="utf-8")
                return str(generated), "test"

            execute_calls = []
            module.resolve_text_to_trk = fake_resolve
            module.execute_trk = lambda *args: execute_calls.append(args) or {"ok": True, "id": "unexpected"}

            out = module.sequence_interrupt_cmd(
                SimpleNamespace(seq_id=seq_id, trk=None, text="new generated", duration=2.0, seed=None, diffusion_steps=None),
                "http://unused",
                0.1,
                0.001,
            )
            state = module.load_state(seq_id)
            self.assertFalse(out["ok"])
            self.assertEqual(out["error"]["code"], "SEQUENCE_CHANGED")
            self.assertEqual(execute_calls, [])
            self.assertEqual(state["state"], "canceled")
            self.assertIsNone(state["active"])
            self.assertNotIn("interrupt", [seg.get("segment_id") for seg in state["segments"]])
            self.assertNotIn("new generated", [seg.get("text") for seg in state["segments"]])
        finally:
            if old_state_dir is None:
                os.environ.pop("ET1_ACTION_STATE_DIR", None)
            else:
                os.environ["ET1_ACTION_STATE_DIR"] = old_state_dir

    def test_sequence_interrupt_ready_trk_uses_interrupt_mode(self):
        plan = {"segments": [{"trk": str(self.ready_trk)}, {"text": "tail"}]}
        out, _ = self.cli("sequence-start", "--plan-json", json.dumps(plan), "--stage-dir", str(self.stage))
        seq_id = out["seq_id"]
        self.wait_for_execs(1)
        self.cli("sequence-interrupt", seq_id, "--trk", str(self.preset_src))
        execute = self.tracker.records[-1]
        self.assertEqual(execute[1], "/execute")
        self.assertEqual(execute[2]["mode"], "interrupt")

    def test_sequence_interrupt_text_standby_then_generation_then_interrupt_run(self):
        plan = {"segments": [{"trk": str(self.ready_trk)}, {"text": "tail"}]}
        out, _ = self.cli(
            "sequence-start",
            "--plan-json",
            json.dumps(plan),
            "--stage-dir",
            str(self.stage),
            env_extra={"ET1_ACTION_QUEUE_AHEAD": "1"},
        )
        seq_id = out["seq_id"]
        self.wait_for_execs(1)
        self.cli("sequence-interrupt", seq_id, "--text", "new generated", "--duration", "2", "--seed", "7", "--diffusion-steps", "42")
        post_paths = [r[1] for r in self.tracker.records if r[0] == "POST"]
        self.assertEqual(post_paths[-2:], ["/standby_velocity", "/execute"])
        self.assertEqual([c[0] for c in self.calls()], ["nl2trk"])
        self.assertIn("--seed", self.calls()[0][1])
        self.assertIn("7", self.calls()[0][1])
        self.assertIn("--diffusion-steps", self.calls()[0][1])
        self.assertIn("42", self.calls()[0][1])
        self.assertEqual(self.tracker.records[-1][2]["mode"], "interrupt")

    def test_sequence_status_detects_worker_died(self):
        seq_id = "seq_dead"
        self.state_dir.mkdir(parents=True, exist_ok=True)
        (self.state_dir / f"{seq_id}.json").write_text(
            json.dumps(
                {
                    "seq_id": seq_id,
                    "state": "running",
                    "pid": 99999999,
                    "heartbeat_at": time.time() - 3600,
                    "updated_at": time.time() - 3600,
                    "segments": [],
                    "next": "sequence-status",
                }
            ),
            encoding="utf-8",
        )
        out, _ = self.cli("sequence-status", seq_id, check=False)
        self.assertFalse(out["ok"])
        self.assertEqual(out["state"], "failed")
        self.assertEqual(out["error"]["code"], "WORKER_DIED")
        self.assertEqual(out["next"], "sequence-status")

    def test_release_packaging_exposes_only_et1_action_skill(self):
        build = (PACKAGING / "build_release.sh").read_text(encoding="utf-8")
        self.assertIn("skills/et1-action", build)
        self.assertIn('package_root/bin/et1-action', build)
        self.assertIn("ET1_ACTION_STAGE_DIR", build)
        self.assertNotIn("make_bin_wrapper et1-trk2motion", build)
        selftest = (PACKAGING / "scripts" / "selftest.sh").read_text(encoding="utf-8")
        self.assertIn('"$ET1_CLI" status', selftest)
        readme = (PACKAGING / "README.release.md").read_text(encoding="utf-8")
        self.assertIn("bin/et1-action status", readme)


if __name__ == "__main__":
    unittest.main()
