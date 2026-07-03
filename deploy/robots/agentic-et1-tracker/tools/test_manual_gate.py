#!/usr/bin/env python3

from __future__ import annotations

import unittest
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent))
import manual_gate


def status(**overrides):
    base = {
        "ready": True,
        "ctrl": "standby_velocity",
        "robot": "ok",
        "block": None,
        "err": None,
        "active": {"kind": "none", "id": None},
        "queue": {"n": 0, "ids": [], "limit": 8},
        "pose": {"p": [0.0, 0.0, 0.74]},
    }
    base.update(overrides)
    return base


def sim_status(**overrides):
    base = {
        "available": True,
        "ok": True,
        "root_z": 0.74,
        "both": True,
        "camera_type": "tracking",
        "camera_track_body": "pelvis_link",
        "camera_trackbodyid": 4,
    }
    base.update(overrides)
    return base


class ManualGateFinalSettleTests(unittest.TestCase):
    def test_user_transition_predicate_requires_target_id(self):
        current = status(
            transition={
                "active": True,
                "target": "user",
                "target_id": "00000004",
            }
        )

        self.assertTrue(manual_gate.is_user_transition_to(current, "00000004"))
        self.assertFalse(manual_gate.is_user_transition_to(current, "00000005"))

    def test_user_preempted_idle_accepts_queued_run_during_user_transition(self):
        top = status(
            active={"kind": "transition", "id": None},
            transition={
                "active": True,
                "target": "user",
                "target_id": "00000004",
                "target_state": "queued",
            },
            idle={"enabled": True, "active": True, "n": 2},
        )
        run = {"id": "00000004", "state": "queued"}

        self.assertTrue(manual_gate.user_preempted_idle(run, top, "00000004"))

    def test_user_preempted_idle_rejects_unclaimed_queued_run(self):
        top = status(
            active={"kind": "idle", "id": None},
            transition={"active": False, "target": None, "target_id": None},
            idle={"enabled": True, "active": True, "n": 2},
        )
        run = {"id": "00000004", "state": "queued"}

        self.assertFalse(manual_gate.user_preempted_idle(run, top, "00000004"))

    def test_user_preempted_idle_still_accepts_running_done_or_holding(self):
        top = status(active={"kind": "user", "id": "00000004"})

        for state in ("running", "done", "holding"):
            with self.subTest(state=state):
                self.assertTrue(
                    manual_gate.user_preempted_idle(
                        {"id": "00000004", "state": state},
                        top,
                        "00000004",
                    )
                )

    def test_final_settle_accepts_stable_standby(self):
        reason = manual_gate.final_settle_reason(
            status(),
            {"available": True, "ok": True, "root_z": 0.74, "both": True},
            min_root_z=0.2,
        )

        self.assertIsNone(reason)

    def test_final_settle_accepts_public_standby_ctrl(self):
        reason = manual_gate.final_settle_reason(
            status(ctrl="standby"),
            {"available": True, "ok": True, "root_z": 0.74, "both": True},
            min_root_z=0.2,
        )

        self.assertIsNone(reason)

    def test_final_settle_rejects_bad_orientation_status(self):
        reason = manual_gate.final_settle_reason(
            status(
                ready=False,
                ctrl="passive",
                robot="fault",
                block="bad_orientation",
                err={"code": "ROBOT_BAD_ORIENTATION"},
                pose={"p": [-2.2, -0.18, -0.709]},
            ),
            {"available": True, "ok": True, "root_z": -0.709, "both": True},
            min_root_z=0.2,
        )

        self.assertIsNotNone(reason)
        self.assertIn("block is not null", reason)
        self.assertIn("bad_orientation", reason)
        self.assertIn("root_z=-0.709", reason)

    def test_final_settle_rejects_low_sim_root_z(self):
        reason = manual_gate.final_settle_reason(
            status(),
            {"available": True, "ok": True, "root_z": -0.1, "both": True},
            min_root_z=0.2,
        )

        self.assertEqual(reason, "final settle: sim-control root_z -0.100 < 0.200")

    def test_final_settle_rejects_missing_sim_root_z_when_available(self):
        reason = manual_gate.final_settle_reason(
            status(),
            {"available": True, "ok": True},
            min_root_z=0.2,
        )

        self.assertEqual(reason, "final settle: sim-control root_z is not numeric (None)")

    def test_visual_stability_accepts_ready_non_passive_status(self):
        reason = manual_gate.visual_stability_reason(
            status(ctrl="running", active={"kind": "user", "id": "hold-id"}),
            sim_status(),
            min_root_z=0.2,
        )

        self.assertIsNone(reason)

    def test_visual_stability_rejects_passive_control(self):
        reason = manual_gate.visual_stability_reason(
            status(ctrl="passive"),
            {"available": False, "error": "timeout"},
            min_root_z=0.2,
        )

        self.assertIsNotNone(reason)
        self.assertIn("ctrl is passive", reason)

    def test_visual_stability_rejects_urgent_stopping_control(self):
        reason = manual_gate.visual_stability_reason(
            status(ctrl="urgent_stopping"),
            {"available": False, "error": "timeout"},
            min_root_z=0.2,
        )

        self.assertIsNotNone(reason)
        self.assertIn("ctrl is urgent_stopping", reason)

    def test_visual_stability_rejects_robot_fault(self):
        reason = manual_gate.visual_stability_reason(
            status(robot="fault"),
            {"available": False, "error": "timeout"},
            min_root_z=0.2,
        )

        self.assertIsNotNone(reason)
        self.assertIn("robot is fault", reason)

    def test_visual_stability_rejects_low_sim_root_z(self):
        reason = manual_gate.visual_stability_reason(
            status(),
            {"available": True, "ok": True, "root_z": 0.1},
            min_root_z=0.2,
        )

        self.assertEqual(reason, "visual stability: sim-control root_z 0.100 < 0.200")

    def test_visual_stability_rejects_sim_control_unavailable(self):
        reason = manual_gate.visual_stability_reason(
            status(),
            {"available": False, "error": "timed out"},
            min_root_z=0.2,
        )

        self.assertEqual(reason, "visual stability: sim-control unavailable (timed out)")

    def test_visual_stability_rejects_missing_camera_fields(self):
        reason = manual_gate.visual_stability_reason(
            status(),
            {"available": True, "ok": True, "root_z": 0.74},
            min_root_z=0.2,
        )

        self.assertIn("missing camera fields", reason)

    def test_visual_stability_rejects_free_camera(self):
        reason = manual_gate.visual_stability_reason(
            status(),
            sim_status(camera_type="free"),
            min_root_z=0.2,
        )

        self.assertEqual(reason, "visual camera: camera_type is free, expected tracking")

    def test_visual_stability_rejects_wrong_camera_body(self):
        reason = manual_gate.visual_stability_reason(
            status(),
            sim_status(camera_track_body="torso_link"),
            min_root_z=0.2,
        )

        self.assertEqual(
            reason,
            "visual camera: camera_track_body is torso_link, expected pelvis_link",
        )

    def test_generic_stopping_reason_rejects_exec_state(self):
        reason = manual_gate.generic_stopping_reason(
            status(exec={"state": "stopping"}),
            "standby gate",
        )

        self.assertIsNotNone(reason)
        self.assertIn("exec.state is generic stopping", reason)

    def test_standby_handoff_accepts_idle_background_with_retained_idle(self):
        reason = manual_gate.standby_handoff_reason(
            status(
                ctrl="running",
                active={"kind": "idle", "id": None},
                idle={"enabled": True, "active": True, "n": 2},
            ),
            {"available": True, "ok": True, "root_z": 0.74, "both": True},
            "active_user_idle_enabled_standby_handoff",
            idle_n=2,
            require_idle_enabled=True,
            final=True,
            min_root_z=0.2,
        )

        self.assertIsNone(reason)

    def test_standby_handoff_rejects_generic_stopping(self):
        reason = manual_gate.standby_handoff_reason(
            status(ctrl="stopping", idle={"enabled": True, "active": False, "n": 2}),
            {"available": True, "ok": True, "root_z": 0.74, "both": True},
            "active_user_idle_enabled_standby_handoff",
            idle_n=2,
            require_idle_enabled=True,
            min_root_z=0.2,
        )

        self.assertIsNotNone(reason)
        self.assertIn("ctrl is stopping", reason)

    def test_urgent_stop_idle_clear_waits_for_stable_status(self):
        old_get = manual_gate.get
        old_sim_control_status = manual_gate.sim_control_status
        samples = [
            status(
                ctrl="urgent_stopping",
                idle={"enabled": False, "active": False, "n": 0},
            ),
            status(
                ctrl="standby",
                idle={"enabled": False, "active": False, "n": 0},
            ),
        ]
        calls = []
        args = SimpleNamespace(
            min_root_z=0.2,
            sim_control_port=8090,
            sim_control_timeout_ms=200,
        )

        def fake_get(_url, _path):
            calls.append(_path)
            return samples.pop(0) if samples else status(
                ctrl="standby",
                idle={"enabled": False, "active": False, "n": 0},
            )

        try:
            manual_gate.get = fake_get
            manual_gate.sim_control_status = (
                lambda _port, _timeout_ms: {
                    "available": True,
                    "ok": True,
                    "root_z": 0.74,
                    "both": True,
                }
            )
            result = manual_gate.urgent_stop_idle_clear_check(
                args,
                "http://tracker",
                "urgent_stop_gate",
                timeout_s=1.0,
            )
        finally:
            manual_gate.get = old_get
            manual_gate.sim_control_status = old_sim_control_status

        self.assertEqual(result["result"], "PASS")
        self.assertEqual(result["status"]["ctrl"], "standby")
        self.assertEqual(len(calls), 2)

    def test_urgent_stop_idle_clear_accepts_fixstand_stable_status(self):
        old_get = manual_gate.get
        old_sim_control_status = manual_gate.sim_control_status
        args = SimpleNamespace(
            min_root_z=0.2,
            sim_control_port=8090,
            sim_control_timeout_ms=200,
        )

        try:
            manual_gate.get = lambda _url, _path: status(
                ctrl="fixstand",
                idle={"enabled": False, "active": False, "n": 0},
            )
            manual_gate.sim_control_status = (
                lambda _port, _timeout_ms: {
                    "available": True,
                    "ok": True,
                    "root_z": 0.74,
                    "both": True,
                }
            )
            result = manual_gate.urgent_stop_idle_clear_check(
                args,
                "http://tracker",
                "fixstand_urgent_stop_gate",
                timeout_s=1.0,
            )
        finally:
            manual_gate.get = old_get
            manual_gate.sim_control_status = old_sim_control_status

        self.assertEqual(result["result"], "PASS")
        self.assertEqual(result["status"]["ctrl"], "fixstand")

    def test_parse_sim_control_status(self):
        parsed = manual_gate.parse_sim_control_status(
            "ok band=0 length=1.100 left_contact=1 right_contact=0 both=0 "
            "root_x=1.700 root_y=-5.000 root_z=-0.709 "
            "camera_type=tracking camera_track_body=pelvis_link camera_trackbodyid=4"
        )

        self.assertTrue(parsed["available"])
        self.assertTrue(parsed["ok"])
        self.assertEqual(parsed["band"], 0.0)
        self.assertTrue(parsed["left_contact"])
        self.assertFalse(parsed["right_contact"])
        self.assertFalse(parsed["both"])
        self.assertEqual(parsed["root_x"], 1.7)
        self.assertEqual(parsed["root_y"], -5.0)
        self.assertEqual(parsed["root_z"], -0.709)
        self.assertEqual(parsed["camera_type"], "tracking")
        self.assertEqual(parsed["camera_track_body"], "pelvis_link")
        self.assertEqual(parsed["camera_trackbodyid"], 4)

    def test_visual_camera_accepts_tracking_pelvis(self):
        reason = manual_gate.visual_camera_reason(sim_status())

        self.assertIsNone(reason)

    def test_visual_camera_rejects_sim_control_unavailable(self):
        reason = manual_gate.visual_camera_reason({"available": False, "error": "timeout"})

        self.assertEqual(reason, "visual camera: sim-control unavailable (timeout)")

    def test_visual_camera_rejects_free_camera(self):
        reason = manual_gate.visual_camera_reason(sim_status(camera_type="free"))

        self.assertEqual(reason, "visual camera: camera_type is free, expected tracking")

    def test_visual_camera_rejects_missing_camera_fields(self):
        reason = manual_gate.visual_camera_reason(
            {"available": True, "ok": True, "root_z": 0.74}
        )

        self.assertIn("missing camera fields", reason)

    def test_visual_camera_rejects_wrong_body(self):
        reason = manual_gate.visual_camera_reason(sim_status(camera_track_body="torso_link"))

        self.assertEqual(
            reason,
            "visual camera: camera_track_body is torso_link, expected pelvis_link",
        )

    def test_visual_camera_align_check_rejects_sim_control_unavailable(self):
        old_sim_control_camera_align = manual_gate.sim_control_camera_align
        args = SimpleNamespace(sim_control_port=8090, sim_control_timeout_ms=200)
        try:
            manual_gate.sim_control_camera_align = (
                lambda _port, _timeout_ms: {"available": False, "error": "timeout"}
            )
            with self.assertRaises(manual_gate.GateError) as raised:
                manual_gate.visual_camera_align_check(args)
        finally:
            manual_gate.sim_control_camera_align = old_sim_control_camera_align

        self.assertIn("sim-control unavailable", str(raised.exception))
        self.assertIn("visual_camera_align", raised.exception.report)
        self.assertNotEqual(raised.exception.report["visual_camera_align"].get("result"), "SKIP")

    def test_final_settle_check_raises_with_report_evidence(self):
        old_get = manual_gate.get
        old_sim_control_status = manual_gate.sim_control_status
        args = SimpleNamespace(
            final_settle_s=0.0,
            min_root_z=0.2,
            sim_control_port=8090,
            sim_control_timeout_ms=200,
        )
        bad_status = status(
            ready=False,
            ctrl="passive",
            robot="fault",
            block="bad_orientation",
            err={"code": "ROBOT_BAD_ORIENTATION"},
            pose={"p": [-2.2, -0.18, -0.709]},
        )

        try:
            manual_gate.get = lambda _url, _path: bad_status
            manual_gate.sim_control_status = (
                lambda _port, _timeout_ms: {
                    "available": True,
                    "ok": True,
                    "root_z": -0.709,
                    "both": True,
                }
            )
            with self.assertRaises(manual_gate.GateError) as raised:
                manual_gate.final_settle_check(args, "http://tracker")
        finally:
            manual_gate.get = old_get
            manual_gate.sim_control_status = old_sim_control_status

        self.assertIn("bad_orientation", str(raised.exception))
        self.assertIn("final_settle", raised.exception.report)
        self.assertEqual(raised.exception.report["final_settle"]["status"]["block"],
                         "bad_orientation")
        self.assertEqual(raised.exception.report["final_settle"]["sim_control"]["root_z"],
                         -0.709)

    def test_final_settle_check_rejects_synthetic_when_sim_control_available(self):
        old_get = manual_gate.get
        old_sim_control_status = manual_gate.sim_control_status
        args = SimpleNamespace(
            final_settle_s=0.0,
            min_root_z=0.2,
            sim_control_port=8090,
            sim_control_timeout_ms=200,
            fixture_source_resolved="synthetic",
        )

        try:
            manual_gate.get = lambda _url, _path: status()
            manual_gate.sim_control_status = (
                lambda _port, _timeout_ms: {
                    "available": True,
                    "ok": True,
                    "root_z": 0.74,
                    "both": True,
                }
            )
            with self.assertRaises(manual_gate.GateError) as raised:
                manual_gate.final_settle_check(args, "http://tracker")
        finally:
            manual_gate.get = old_get
            manual_gate.sim_control_status = old_sim_control_status

        self.assertIn("did not use existing generated TRK", str(raised.exception))
        self.assertEqual(raised.exception.report["final_settle"]["fixture_source"],
                         "synthetic")
        self.assertNotEqual(raised.exception.report["final_settle"].get("result"), "SKIP")

    def test_visual_action_terminal_check_rejects_failed_terminal(self):
        old_run_status = manual_gate.run_status
        args = SimpleNamespace(start_tracker=False)
        try:
            manual_gate.run_status = (
                lambda _url, _run_id: {"id": "run-1", "state": "failed"}
            )
            with self.assertRaises(manual_gate.GateError) as raised:
                manual_gate.visual_action_terminal_check(
                    args, "http://tracker", "run-1", Path("/tmp/short.trk")
                )
        finally:
            manual_gate.run_status = old_run_status

        self.assertIn("terminal state is failed", str(raised.exception))
        self.assertIn("visual_action_terminal", raised.exception.report)

    def test_visual_action_standby_check_rejects_post_terminal_passive(self):
        old_get = manual_gate.get
        args = SimpleNamespace(standby_timeout=1.0)
        try:
            manual_gate.get = lambda _url, _path: status(
                ready=False,
                ctrl="passive",
                robot="fault",
                block="bad_orientation",
                err={"code": "ROBOT_BAD_ORIENTATION"},
            )
            with self.assertRaises(manual_gate.GateError) as raised:
                manual_gate.visual_action_standby_check(args, "http://tracker")
        finally:
            manual_gate.get = old_get

        self.assertIn("bad_orientation", str(raised.exception))
        self.assertIn("visual action standby", raised.exception.report)

    def test_run_visual_waits_for_terminal_after_screenshot_and_fails_failed_action(self):
        old_mujoco_processes = manual_gate.mujoco_processes
        old_refuse = manual_gate.refuse_synthetic_physical_fixture
        old_wait_ready = manual_gate.wait_ready
        old_recover = manual_gate.recover_to_standby
        old_execute = manual_gate.execute
        old_run_status = manual_gate.run_status
        old_camera_align = manual_gate.visual_camera_align_check
        old_capture = manual_gate.capture_screenshot
        old_get = manual_gate.get
        old_sim_control_status = manual_gate.sim_control_status
        calls = {"run_status": 0, "captured": False}

        def fake_capture(path, _command_template):
            calls["captured"] = True
            path.write_bytes(
                b"\x89PNG\r\n\x1a\n" +
                b"\x00\x00\x00\rIHDR" +
                (1).to_bytes(4, "big") +
                (1).to_bytes(4, "big") +
                b"\x08\x02\x00\x00\x00"
            )

        def fake_run_status(_url, _run_id):
            calls["run_status"] += 1
            if calls["run_status"] == 1:
                return {"id": "run-1", "state": "running"}
            return {"id": "run-1", "state": "failed"}

        with tempfile.TemporaryDirectory() as tmp:
            args = SimpleNamespace(
                url="http://tracker",
                artifacts_dir=Path(tmp),
                mujoco_process_regex="mujoco",
                visual_run_action=True,
                ready_timeout=1.0,
                passive_password="test",
                visual_settle_s=0.0,
                screenshot_command=None,
                min_screenshot_bytes=1,
                min_screenshot_width=1,
                min_screenshot_height=1,
                sim_control_port=8090,
                sim_control_timeout_ms=200,
                min_root_z=0.2,
                standby_timeout=1.0,
                start_tracker=False,
            )
            fixtures = {"short": Path(tmp) / "short.trk"}
            fixtures["short"].write_bytes(b"not-used")
            try:
                manual_gate.mujoco_processes = lambda _pattern: ["123 mujoco"]
                manual_gate.refuse_synthetic_physical_fixture = lambda _args, _url: None
                manual_gate.wait_ready = lambda _url, _timeout: status()
                manual_gate.recover_to_standby = lambda _url, _password: status()
                manual_gate.execute = lambda _url, _path, mode="queue": "run-1"
                manual_gate.run_status = fake_run_status
                manual_gate.visual_camera_align_check = (
                    lambda _args: {"result": "PASS", "sim_control": sim_status()}
                )
                manual_gate.capture_screenshot = fake_capture
                manual_gate.get = lambda _url, _path: status(
                    ctrl="running",
                    active={"kind": "user", "id": "run-1"},
                )
                manual_gate.sim_control_status = (
                    lambda _port, _timeout_ms: sim_status()
                )
                with self.assertRaises(manual_gate.GateError) as raised:
                    manual_gate.run_visual(args, fixtures)
            finally:
                manual_gate.mujoco_processes = old_mujoco_processes
                manual_gate.refuse_synthetic_physical_fixture = old_refuse
                manual_gate.wait_ready = old_wait_ready
                manual_gate.recover_to_standby = old_recover
                manual_gate.execute = old_execute
                manual_gate.run_status = old_run_status
                manual_gate.visual_camera_align_check = old_camera_align
                manual_gate.capture_screenshot = old_capture
                manual_gate.get = old_get
                manual_gate.sim_control_status = old_sim_control_status

        self.assertTrue(calls["captured"])
        self.assertGreaterEqual(calls["run_status"], 2)
        self.assertIn("terminal state is failed", str(raised.exception))


class ManualGateFixtureSelectionTests(unittest.TestCase):
    def args(self, source="auto", gate="e2e"):
        return SimpleNamespace(
            fixture_source=source,
            gate=gate,
            visual_run_action=True,
            sim_control_port=8090,
            sim_control_timeout_ms=200,
        )

    def write_named_fixtures(self, root):
        return {
            "idle_a": manual_gate.write_trk(root / "manual_gate_idle_a.trk", 80, 0.1),
            "idle_b": manual_gate.write_trk(root / "manual_gate_idle_b.trk", 90, 0.2),
            "short": manual_gate.write_trk(root / "manual_gate_short.trk", 35, 0.3),
            "long_a": manual_gate.write_trk(root / "manual_gate_long_a.trk", 260, 0.4),
            "long_b": manual_gate.write_trk(root / "manual_gate_long_b.trk", 220, 0.5),
            "long_c": manual_gate.write_trk(root / "manual_gate_long_c.trk", 220, 0.6),
            "transition_a": manual_gate.write_trk(
                root / "manual_gate_transition_a.trk", 12, 0.7
            ),
            "transition_b": manual_gate.write_trk(
                root / "manual_gate_transition_b.trk", 500, 0.8
            ),
            "transition_c": manual_gate.write_trk(
                root / "manual_gate_transition_c.trk", 500, 0.9
            ),
            "loco": manual_gate.write_trk(root / "manual_gate_loco.trk", 60, 1.0),
        }

    def test_auto_prefers_stable_named_manual_gate_fixtures(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            named = self.write_named_fixtures(root)
            manual_gate.write_trk(
                root / "et1-generated-turn_around_180_degrees-newer.trk", 20, 0.2
            )

            fixtures, report = manual_gate.resolve_fixtures(self.args("auto"), root)

        self.assertEqual(report["source"], "existing")
        self.assertGreaterEqual(report["candidate_count"], len(named))
        self.assertEqual(fixtures, {key: path.resolve() for key, path in named.items()})

    def test_obvious_tmp_and_synthetic_files_remain_filtered(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            stable = manual_gate.write_trk(root / "manual_gate_short.trk", 35, 0.3)
            real = manual_gate.write_trk(root / "et1-generated-real.trk", 20, 0.2)
            tmp_dir = root / "tmp"
            tmp_dir.mkdir()
            manual_gate.write_trk(tmp_dir / "manual_gate_long_a.trk", 260, 0.4)
            manual_gate.write_trk(root / "synthetic_generated.trk", 20, 0.5)
            manual_gate.write_trk(root / "fixture_generated.trk", 20, 0.6)

            candidates = manual_gate.find_existing_trk_candidates(root)

        paths = {candidate.path for candidate in candidates}
        self.assertEqual(paths, {stable.resolve(), real.resolve()})

    def test_existing_selection_falls_back_to_recent_generated_trk(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            existing = manual_gate.write_trk(root / "et1-generated-real.trk", 20, 0.2)

            fixtures, report = manual_gate.resolve_fixtures(self.args("existing"), root)

        self.assertEqual(report["source"], "existing")
        self.assertEqual(report["candidate_count"], 1)
        self.assertEqual(set(fixtures.values()), {existing.resolve()})

    def test_existing_source_fails_when_only_synthetic_files_exist(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manual_gate.write_trk(root / "manual_gate_short.trk", 8, 0.1)

            with self.assertRaises(manual_gate.GateError) as raised:
                manual_gate.resolve_fixtures(self.args("existing"), root)

        self.assertIn("found no usable generated TRK", str(raised.exception))

    def test_auto_refuses_synthetic_when_sim_control_is_available(self):
        old_sim_control_status = manual_gate.sim_control_status
        try:
            manual_gate.sim_control_status = (
                lambda _port, _timeout_ms: {"available": True, "ok": True, "root_z": 0.74}
            )
            with tempfile.TemporaryDirectory() as tmp:
                with self.assertRaises(manual_gate.GateError) as raised:
                    manual_gate.resolve_fixtures(self.args("auto"), Path(tmp))
        finally:
            manual_gate.sim_control_status = old_sim_control_status

        self.assertIn("refusing to synthesize", str(raised.exception))
        self.assertEqual(raised.exception.report["fixture_selection"]["candidate_count"], 0)

    def test_synthetic_source_refuses_before_writing_when_sim_control_is_available(self):
        old_sim_control_status = manual_gate.sim_control_status
        try:
            manual_gate.sim_control_status = (
                lambda _port, _timeout_ms: {"available": True, "ok": True, "root_z": 0.74}
            )
            with tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                with self.assertRaises(manual_gate.GateError):
                    manual_gate.resolve_fixtures(self.args("synthetic"), root)
                self.assertFalse((root / "manual_gate_short.trk").exists())
        finally:
            manual_gate.sim_control_status = old_sim_control_status

    def test_synthetic_source_still_writes_http_contract_fixtures(self):
        old_sim_control_status = manual_gate.sim_control_status
        try:
            manual_gate.sim_control_status = (
                lambda _port, _timeout_ms: {"available": False, "error": "timeout"}
            )
            with tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                fixtures, report = manual_gate.resolve_fixtures(self.args("synthetic"), root)

                self.assertEqual(report["source"], "synthetic")
                self.assertEqual(set(fixtures.keys()), set(manual_gate.FIXTURE_KEYS))
                self.assertTrue((root / "manual_gate_short.trk").exists())
        finally:
            manual_gate.sim_control_status = old_sim_control_status


class ManualGateTempConfigTests(unittest.TestCase):
    def parse(self, *argv):
        old_argv = sys.argv
        try:
            sys.argv = ["manual_gate.py", *argv]
            return manual_gate.parse_args()
        finally:
            sys.argv = old_argv

    def generated_config_value(self, text, key):
        prefix = f"{key}:"
        for line in text.splitlines():
            stripped = line.strip()
            if stripped.startswith(prefix):
                return stripped.split(":", 1)[1].strip()
        self.fail(f"missing {key} in generated config")

    def test_start_tracker_temp_config_defaults_match_sim_runtime(self):
        args = self.parse("e2e", "--start-tracker")

        self.assertEqual(args.temp_hz, 1000.0)
        self.assertEqual(args.temp_transition_duration_s, 0.7)
        with tempfile.TemporaryDirectory() as tmp:
            temp_dir = Path(tmp)
            motion_dir = temp_dir / "motions"
            motion_dir.mkdir()
            config = manual_gate.write_temp_config(args, temp_dir, motion_dir)
            text = config.read_text()

        self.assertEqual(float(self.generated_config_value(text, "hz")), 1000.0)
        self.assertEqual(
            float(self.generated_config_value(text, "transition_duration_s")),
            0.7,
        )

    def test_start_tracker_temp_config_allows_explicit_runtime_overrides(self):
        args = self.parse(
            "e2e",
            "--start-tracker",
            "--temp-hz",
            "50",
            "--temp-transition-duration-s",
            "1.25",
        )

        self.assertEqual(args.temp_hz, 50.0)
        self.assertEqual(args.temp_transition_duration_s, 1.25)
        with tempfile.TemporaryDirectory() as tmp:
            temp_dir = Path(tmp)
            motion_dir = temp_dir / "motions"
            motion_dir.mkdir()
            config = manual_gate.write_temp_config(args, temp_dir, motion_dir)
            text = config.read_text()

        self.assertEqual(float(self.generated_config_value(text, "hz")), 50.0)
        self.assertEqual(
            float(self.generated_config_value(text, "transition_duration_s")),
            1.25,
        )

    def test_start_tracker_run_terminal_timeout_scales_with_temp_hz(self):
        args = SimpleNamespace(
            start_tracker=True,
            temp_hz=10.0,
            temp_transition_duration_s=2.0,
        )

        with tempfile.TemporaryDirectory() as tmp:
            track = manual_gate.write_trk(Path(tmp) / "slow.trk", 200, 0.1)
            timeout_s = manual_gate.trk_terminal_timeout_s(args, track, min_timeout_s=12.0)

        self.assertGreaterEqual(timeout_s, 26.0)


if __name__ == "__main__":
    unittest.main()
