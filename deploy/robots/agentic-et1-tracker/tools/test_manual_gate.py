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

    def test_parse_sim_control_status_accepts_boolean_contact_words(self):
        parsed = manual_gate.parse_sim_control_status(
            "ok left_contact=true right_contact=false both=true root_z=0.740"
        )

        self.assertTrue(parsed["left_contact"])
        self.assertFalse(parsed["right_contact"])
        self.assertTrue(parsed["both"])
        self.assertEqual(parsed["root_z"], 0.74)

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

        self.assertIn("non-physical-safe fixture source", str(raised.exception))
        self.assertIn("synthetic", str(raised.exception))
        self.assertEqual(raised.exception.report["final_settle"]["fixture_source"],
                         "synthetic")
        self.assertNotEqual(raised.exception.report["final_settle"].get("result"), "SKIP")

    def test_final_settle_check_allows_e2e_safe_when_sim_control_available(self):
        old_get = manual_gate.get
        old_sim_control_status = manual_gate.sim_control_status
        args = SimpleNamespace(
            final_settle_s=0.0,
            min_root_z=0.2,
            sim_control_port=8090,
            sim_control_timeout_ms=200,
            fixture_source_resolved="e2e_safe",
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
            result = manual_gate.final_settle_check(args, "http://tracker")
        finally:
            manual_gate.get = old_get
            manual_gate.sim_control_status = old_sim_control_status

        self.assertEqual(result["result"], "PASS")
        self.assertEqual(result["fixture_source"], "e2e_safe")

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


class ManualGateMujocoLandingSettleTests(unittest.TestCase):
    def args(self, **overrides):
        base = {
            "sim_control_port": 8090,
            "sim_control_timeout_ms": 200,
            "min_root_z": 0.2,
            "mujoco_land_hold": True,
            "mujoco_land_timeout_s": 1.0,
            "mujoco_land_contact_samples": 2,
            "mujoco_land_contact_s": 0.0,
            "mujoco_land_interval_s": 0.0,
            "mujoco_land_release_check_s": 0.0,
            "standby_timeout": 1.0,
        }
        base.update(overrides)
        return SimpleNamespace(**base)

    def test_landing_settle_fixstand_lowers_standby_then_releases(self):
        old_command = manual_gate.sim_control_command
        old_get = manual_gate.get
        old_post = manual_gate.post
        events = []
        lower_samples = [
            {"available": True, "ok": True, "root_z": 0.90, "both": False},
            {"available": True, "ok": True, "root_z": 0.78, "both": True},
            {"available": True, "ok": True, "root_z": 0.74, "both": True},
        ]
        statuses = [
            status(ctrl="fixstand"),
            status(ctrl="standby"),
            status(ctrl="standby"),
        ]

        def fake_command(_port, _timeout_ms, command):
            events.append(f"sim:{command}")
            if command == "hold":
                return {"available": True, "ok": True, "root_z": 0.95, "both": False}
            if command == "lower":
                return lower_samples.pop(0)
            if command == "release":
                return {"available": True, "ok": True, "root_z": 0.74, "both": True}
            if command == "status":
                return {"available": True, "ok": True, "root_z": 0.74, "both": True}
            self.fail(f"unexpected command {command}")

        def fake_get(_url, path):
            events.append(f"get:{path}")
            return statuses.pop(0) if statuses else status(ctrl="standby")

        def fake_post(_url, path, body=None):
            events.append(f"post:{path}")
            return {"ok": True}

        try:
            manual_gate.sim_control_command = fake_command
            manual_gate.get = fake_get
            manual_gate.post = fake_post
            result = manual_gate.mujoco_landing_settle_check(self.args(), "http://tracker")
        finally:
            manual_gate.sim_control_command = old_command
            manual_gate.get = old_get
            manual_gate.post = old_post

        self.assertEqual(
            events,
            [
                "post:/fixstand",
                "get:/status",
                "sim:hold",
                "sim:lower",
                "sim:lower",
                "sim:lower",
                "post:/standby",
                "get:/status",
                "sim:status",
                "sim:release",
                "sim:status",
                "get:/status",
            ],
        )
        self.assertEqual(result["result"], "PASS")
        self.assertEqual(result["lower_count"], 3)
        self.assertEqual(result["standby_status"]["ctrl"], "standby")
        self.assertEqual(result["standby_sim_control"]["both"], True)
        self.assertEqual(result["release_check"]["sim_control"]["both"], True)
        self.assertEqual(result["release_check"]["status"]["ctrl"], "standby")

    def test_landing_settle_samples_fresh_status_before_release(self):
        old_command = manual_gate.sim_control_command
        old_get = manual_gate.get
        old_post = manual_gate.post
        events = []
        statuses = [
            status(ctrl="fixstand"),
            status(ctrl="standby"),
        ]

        def fake_command(_port, _timeout_ms, command):
            events.append(f"sim:{command}")
            if command == "hold":
                return {"available": True, "ok": True, "root_z": 0.95, "both": False}
            if command == "lower":
                return {"available": True, "ok": True, "root_z": 0.74, "both": True}
            if command == "status":
                return {"available": True, "ok": True, "root_z": 0.10, "both": True}
            if command == "release":
                self.fail("release must not run before fresh standby status passes")
            self.fail(f"unexpected command {command}")

        try:
            manual_gate.sim_control_command = fake_command
            manual_gate.get = (
                lambda _url, _path: statuses.pop(0) if statuses else status(ctrl="standby")
            )
            manual_gate.post = lambda _url, _path, body=None: {"ok": True}
            with self.assertRaises(manual_gate.GateError) as raised:
                manual_gate.mujoco_landing_settle_check(
                    self.args(mujoco_land_contact_samples=1),
                    "http://tracker",
                )
        finally:
            manual_gate.sim_control_command = old_command
            manual_gate.get = old_get
            manual_gate.post = old_post

        self.assertIn("sim-control root_z 0.100 < 0.200", str(raised.exception))
        self.assertIn("sim:status", events)
        self.assertNotIn("sim:release", events)

    def test_landing_settle_fresh_status_requires_contact_before_release(self):
        old_command = manual_gate.sim_control_command
        old_get = manual_gate.get
        old_post = manual_gate.post
        events = []
        statuses = [
            status(ctrl="fixstand"),
            status(ctrl="standby"),
        ]

        def fake_command(_port, _timeout_ms, command):
            events.append(f"sim:{command}")
            if command == "hold":
                return {"available": True, "ok": True, "root_z": 0.95, "both": False}
            if command == "lower":
                return {"available": True, "ok": True, "root_z": 0.74, "both": True}
            if command == "status":
                return {"available": True, "ok": True, "root_z": 0.74, "both": False}
            if command == "release":
                self.fail("release must not run before fresh standby contact passes")
            self.fail(f"unexpected command {command}")

        try:
            manual_gate.sim_control_command = fake_command
            manual_gate.get = (
                lambda _url, _path: statuses.pop(0) if statuses else status(ctrl="standby")
            )
            manual_gate.post = lambda _url, _path, body=None: {"ok": True}
            with self.assertRaises(manual_gate.GateError) as raised:
                manual_gate.mujoco_landing_settle_check(
                    self.args(mujoco_land_contact_samples=1),
                    "http://tracker",
                )
        finally:
            manual_gate.sim_control_command = old_command
            manual_gate.get = old_get
            manual_gate.post = old_post

        self.assertIn("both contact is not true", str(raised.exception))
        self.assertIn("sim:status", events)
        self.assertNotIn("sim:release", events)

    def test_landing_settle_can_skip_initial_hold(self):
        old_command = manual_gate.sim_control_command
        old_get = manual_gate.get
        old_post = manual_gate.post
        events = []
        statuses = [
            status(ctrl="fixstand"),
            status(ctrl="standby"),
            status(ctrl="standby"),
        ]

        def fake_command(_port, _timeout_ms, command):
            events.append(f"sim:{command}")
            return {"available": True, "ok": True, "root_z": 0.74, "both": True}

        def fake_get(_url, path):
            events.append(f"get:{path}")
            return statuses.pop(0) if statuses else status(ctrl="standby")

        def fake_post(_url, path, body=None):
            events.append(f"post:{path}")
            return {"ok": True}

        try:
            manual_gate.sim_control_command = fake_command
            manual_gate.get = fake_get
            manual_gate.post = fake_post
            result = manual_gate.mujoco_landing_settle_check(
                self.args(mujoco_land_hold=False, mujoco_land_contact_samples=1),
                "http://tracker",
            )
        finally:
            manual_gate.sim_control_command = old_command
            manual_gate.get = old_get
            manual_gate.post = old_post

        self.assertEqual(
            events,
            [
                "post:/fixstand",
                "get:/status",
                "sim:lower",
                "post:/standby",
                "get:/status",
                "sim:status",
                "sim:release",
                "sim:status",
                "get:/status",
            ],
        )
        self.assertEqual(result["result"], "PASS")

    def test_landing_settle_fails_when_contact_is_lost_after_release(self):
        old_command = manual_gate.sim_control_command
        old_get = manual_gate.get
        old_post = manual_gate.post
        responses = {
            "hold": [{"available": True, "ok": True, "root_z": 0.95, "both": False}],
            "lower": [{"available": True, "ok": True, "root_z": 0.74, "both": True}],
            "release": [{"available": True, "ok": True, "root_z": 0.74, "both": True}],
            "status": [
                {"available": True, "ok": True, "root_z": 0.74, "both": True},
                {"available": True, "ok": True, "root_z": 0.74, "both": False},
            ],
        }
        statuses = [
            status(ctrl="fixstand"),
            status(ctrl="standby"),
            status(ctrl="standby"),
        ]

        def fake_command(_port, _timeout_ms, command):
            return responses[command].pop(0)

        try:
            manual_gate.sim_control_command = fake_command
            manual_gate.get = (
                lambda _url, _path: statuses.pop(0) if statuses else status(ctrl="standby")
            )
            manual_gate.post = lambda _url, _path, body=None: {"ok": True}
            with self.assertRaises(manual_gate.GateError) as raised:
                manual_gate.mujoco_landing_settle_check(
                    self.args(mujoco_land_contact_samples=1),
                    "http://tracker",
                )
        finally:
            manual_gate.sim_control_command = old_command
            manual_gate.get = old_get
            manual_gate.post = old_post

        self.assertIn("post-release both contact did not recover", str(raised.exception))
        self.assertIn("mujoco_landing_settle", raised.exception.report)
        self.assertIn("samples", raised.exception.report["mujoco_landing_settle"])

    def test_landing_settle_accepts_transient_post_release_contact_loss(self):
        old_command = manual_gate.sim_control_command
        old_get = manual_gate.get
        old_post = manual_gate.post
        old_monotonic = manual_gate.time.monotonic
        old_sleep = manual_gate.time.sleep
        clock = {"t": 0.0}
        release_samples = [
            {"available": True, "ok": True, "root_z": 0.74, "both": False},
            {"available": True, "ok": True, "root_z": 0.74, "both": True},
            {"available": True, "ok": True, "root_z": 0.74, "both": True},
        ]
        seen_release_both = []
        pre_release_status = {"used": False}
        statuses = [
            status(ctrl="fixstand"),
            status(ctrl="standby"),
        ]

        def fake_monotonic():
            clock["t"] += 0.05
            return clock["t"]

        def fake_command(_port, _timeout_ms, command):
            if command == "hold":
                return {"available": True, "ok": True, "root_z": 0.95, "both": False}
            if command == "lower":
                return {"available": True, "ok": True, "root_z": 0.74, "both": True}
            if command == "release":
                return {"available": True, "ok": True, "root_z": 0.74, "both": True}
            if command == "status":
                if not pre_release_status["used"]:
                    pre_release_status["used"] = True
                    return {"available": True, "ok": True, "root_z": 0.74, "both": True}
                sample = release_samples.pop(0) if release_samples else {
                    "available": True,
                    "ok": True,
                    "root_z": 0.74,
                    "both": True,
                }
                seen_release_both.append(sample["both"])
                return sample
            self.fail(f"unexpected command {command}")

        try:
            manual_gate.sim_control_command = fake_command
            manual_gate.get = (
                lambda _url, _path: statuses.pop(0) if statuses else status(ctrl="standby")
            )
            manual_gate.post = lambda _url, _path, body=None: {"ok": True}
            manual_gate.time.monotonic = fake_monotonic
            manual_gate.time.sleep = lambda seconds: clock.__setitem__("t", clock["t"] + seconds)
            result = manual_gate.mujoco_landing_settle_check(
                self.args(
                    mujoco_land_contact_samples=2,
                    mujoco_land_release_check_s=0.50,
                ),
                "http://tracker",
            )
        finally:
            manual_gate.sim_control_command = old_command
            manual_gate.get = old_get
            manual_gate.post = old_post
            manual_gate.time.monotonic = old_monotonic
            manual_gate.time.sleep = old_sleep

        self.assertEqual(result["result"], "PASS")
        self.assertIn(False, seen_release_both)
        self.assertEqual(seen_release_both[-2:], [True, True])
        self.assertEqual(result["release_check"]["sim_control"]["both"], True)

    def test_landing_settle_fails_when_post_release_contact_never_recovers(self):
        old_command = manual_gate.sim_control_command
        old_get = manual_gate.get
        old_post = manual_gate.post
        old_monotonic = manual_gate.time.monotonic
        old_sleep = manual_gate.time.sleep
        clock = {"t": 0.0}
        pre_release_status = {"used": False}
        statuses = [
            status(ctrl="fixstand"),
            status(ctrl="standby"),
        ]

        def fake_monotonic():
            clock["t"] += 0.05
            return clock["t"]

        def fake_command(_port, _timeout_ms, command):
            if command == "hold":
                return {"available": True, "ok": True, "root_z": 0.95, "both": False}
            if command == "lower":
                return {"available": True, "ok": True, "root_z": 0.74, "both": True}
            if command == "release":
                return {"available": True, "ok": True, "root_z": 0.74, "both": True}
            if command == "status":
                if not pre_release_status["used"]:
                    pre_release_status["used"] = True
                    return {"available": True, "ok": True, "root_z": 0.74, "both": True}
                return {"available": True, "ok": True, "root_z": 0.74, "both": False}
            self.fail(f"unexpected command {command}")

        try:
            manual_gate.sim_control_command = fake_command
            manual_gate.get = (
                lambda _url, _path: statuses.pop(0) if statuses else status(ctrl="standby")
            )
            manual_gate.post = lambda _url, _path, body=None: {"ok": True}
            manual_gate.time.monotonic = fake_monotonic
            manual_gate.time.sleep = lambda seconds: clock.__setitem__("t", clock["t"] + seconds)
            with self.assertRaises(manual_gate.GateError) as raised:
                manual_gate.mujoco_landing_settle_check(
                    self.args(
                        mujoco_land_contact_samples=2,
                        mujoco_land_release_check_s=0.25,
                    ),
                    "http://tracker",
                )
        finally:
            manual_gate.sim_control_command = old_command
            manual_gate.get = old_get
            manual_gate.post = old_post
            manual_gate.time.monotonic = old_monotonic
            manual_gate.time.sleep = old_sleep

        self.assertIn("post-release both contact did not recover", str(raised.exception))
        report = raised.exception.report["mujoco_landing_settle"]
        self.assertIn("samples", report)
        self.assertTrue(any(
            sample.get("sim_control", {}).get("both") is False
            for sample in report["samples"]
        ))

    def test_landing_settle_fails_immediately_on_post_release_low_root_z(self):
        old_command = manual_gate.sim_control_command
        old_get = manual_gate.get
        old_post = manual_gate.post
        statuses = [
            status(ctrl="fixstand"),
            status(ctrl="standby"),
            status(ctrl="standby"),
        ]

        def fake_command(_port, _timeout_ms, command):
            if command == "hold":
                return {"available": True, "ok": True, "root_z": 0.95, "both": False}
            if command == "lower":
                return {"available": True, "ok": True, "root_z": 0.74, "both": True}
            if command == "release":
                return {"available": True, "ok": True, "root_z": 0.74, "both": True}
            if command == "status":
                return {"available": True, "ok": True, "root_z": 0.10, "both": True}
            self.fail(f"unexpected command {command}")

        try:
            manual_gate.sim_control_command = fake_command
            manual_gate.get = (
                lambda _url, _path: statuses.pop(0) if statuses else status(ctrl="standby")
            )
            manual_gate.post = lambda _url, _path, body=None: {"ok": True}
            with self.assertRaises(manual_gate.GateError) as raised:
                manual_gate.mujoco_landing_settle_check(
                    self.args(mujoco_land_contact_samples=1),
                    "http://tracker",
                )
        finally:
            manual_gate.sim_control_command = old_command
            manual_gate.get = old_get
            manual_gate.post = old_post

        self.assertIn("sim-control root_z 0.100 < 0.200", str(raised.exception))
        self.assertIn("samples", raised.exception.report["mujoco_landing_settle"])

    def test_landing_settle_fails_immediately_on_post_release_passive(self):
        old_command = manual_gate.sim_control_command
        old_get = manual_gate.get
        old_post = manual_gate.post
        statuses = [
            status(ctrl="fixstand"),
            status(ctrl="standby"),
            status(
                ready=False,
                ctrl="passive",
                robot="fault",
                block="bad_orientation",
                err={"code": "ROBOT_BAD_ORIENTATION"},
                pose={"p": [0.0, 0.0, 0.74]},
            ),
        ]

        def fake_command(_port, _timeout_ms, command):
            if command == "hold":
                return {"available": True, "ok": True, "root_z": 0.95, "both": False}
            if command == "lower":
                return {"available": True, "ok": True, "root_z": 0.74, "both": True}
            if command in ("release", "status"):
                return {"available": True, "ok": True, "root_z": 0.74, "both": True}
            self.fail(f"unexpected command {command}")

        try:
            manual_gate.sim_control_command = fake_command
            manual_gate.get = (
                lambda _url, _path: statuses.pop(0) if statuses else status(ctrl="standby")
            )
            manual_gate.post = lambda _url, _path, body=None: {"ok": True}
            with self.assertRaises(manual_gate.GateError) as raised:
                manual_gate.mujoco_landing_settle_check(
                    self.args(mujoco_land_contact_samples=1),
                    "http://tracker",
                )
        finally:
            manual_gate.sim_control_command = old_command
            manual_gate.get = old_get
            manual_gate.post = old_post

        self.assertIn("bad_orientation", str(raised.exception))
        self.assertIn("samples", raised.exception.report["mujoco_landing_settle"])


class ManualGateStandbySoakTests(unittest.TestCase):
    def args(self, **overrides):
        base = {
            "sim_control_port": 8090,
            "sim_control_timeout_ms": 200,
            "standby_timeout": 1.0,
            "standby_soak_s": 0.0,
            "standby_soak_interval_s": 0.0,
            "min_root_z": 0.2,
        }
        base.update(overrides)
        return SimpleNamespace(**base)

    def test_standby_soak_requests_standby_and_samples_stability(self):
        old_get = manual_gate.get
        old_post = manual_gate.post
        old_sim_control_status = manual_gate.sim_control_status
        posts = []
        statuses = [
            status(ctrl="standby"),
            status(ctrl="standby"),
        ]

        try:
            manual_gate.post = lambda _url, path, body=None: posts.append(path) or {"ok": True}
            manual_gate.get = lambda _url, _path: statuses.pop(0) if statuses else status(ctrl="standby")
            manual_gate.sim_control_status = (
                lambda _port, _timeout_ms: {
                    "available": True,
                    "ok": True,
                    "root_z": 0.74,
                    "both": True,
                }
            )
            result = manual_gate.standby_soak_check(self.args(), "http://tracker")
        finally:
            manual_gate.get = old_get
            manual_gate.post = old_post
            manual_gate.sim_control_status = old_sim_control_status

        self.assertEqual(posts, ["/standby"])
        self.assertEqual(result["result"], "PASS")
        self.assertGreaterEqual(result["sample_count"], 1)
        self.assertEqual(result["samples"][-1]["sim_control"]["both"], True)

    def test_standby_soak_fails_with_recent_sample_summary(self):
        old_get = manual_gate.get
        old_post = manual_gate.post
        old_sim_control_status = manual_gate.sim_control_status
        statuses = [
            status(ctrl="standby"),
            status(
                ready=False,
                ctrl="passive",
                robot="fault",
                block="bad_orientation",
                err={"code": "ROBOT_BAD_ORIENTATION"},
                pose={"p": [-2.2, -0.18, -0.709]},
            ),
        ]
        sim_samples = [
            {"available": True, "ok": True, "root_z": 0.74, "both": True},
            {"available": True, "ok": True, "root_z": -0.709, "both": True},
        ]

        try:
            manual_gate.post = lambda _url, _path, body=None: {"ok": True}
            manual_gate.get = lambda _url, _path: statuses.pop(0) if statuses else status(
                ready=False,
                ctrl="passive",
                robot="fault",
                block="bad_orientation",
                err={"code": "ROBOT_BAD_ORIENTATION"},
                pose={"p": [-2.2, -0.18, -0.709]},
            )
            manual_gate.sim_control_status = (
                lambda _port, _timeout_ms: sim_samples.pop(0) if sim_samples else {
                    "available": True,
                    "ok": True,
                    "root_z": -0.709,
                    "both": True,
                }
            )
            with self.assertRaises(manual_gate.GateError) as raised:
                manual_gate.standby_soak_check(
                    self.args(standby_soak_s=1.0), "http://tracker"
                )
        finally:
            manual_gate.get = old_get
            manual_gate.post = old_post
            manual_gate.sim_control_status = old_sim_control_status

        self.assertIn("bad_orientation", str(raised.exception))
        self.assertIn("standby_soak", raised.exception.report)
        self.assertIn("samples", raised.exception.report["standby_soak"])
        self.assertLessEqual(len(raised.exception.report["standby_soak"]["samples"]), 8)


class ManualGateQueueInterruptContractTests(unittest.TestCase):
    def test_held_interrupt_handoff_waits_for_holding_before_interrupt(self):
        old_get = manual_gate.get
        old_post = manual_gate.post
        saw_holding = False
        events = []
        top_statuses = [
            status(),
            status(),
            status(),
            status(
                ctrl="running",
                active={"kind": "transition", "id": None},
                transition={
                    "active": True,
                    "target": "user",
                    "target_id": "C",
                    "target_state": "queued",
                },
            ),
            status(),
            status(),
        ]
        run_statuses = {
            "A": [
                {"id": "A", "state": "running"},
                {"id": "A", "state": "holding"},
                {"id": "A", "state": "stopped", "stop_reason": "interrupt"},
            ],
            "C": [
                {"id": "C", "state": "queued"},
            ],
        }
        posts = []

        def fake_get(_url, path):
            nonlocal saw_holding
            if path == "/status":
                return top_statuses.pop(0) if top_statuses else status()
            if path.startswith("/status?id="):
                run_id = path.rsplit("=", 1)[1]
                states = run_statuses[run_id]
                current = states.pop(0) if len(states) > 1 else states[0]
                events.append(("run", run_id, current["state"]))
                if run_id == "A" and current["state"] == "holding":
                    saw_holding = True
                return current
            self.fail(f"unexpected GET path {path}")

        def fake_post(_url, path, body=None, expected=200):
            self.assertEqual(expected, 200)
            posts.append((path, body))
            if path == "/execute":
                if len([post for post in posts if post[0] == "/execute"]) == 1:
                    self.assertEqual(body["mode"], "queue")
                    self.assertIs(body["hold"], True)
                    events.append(("execute", "queue"))
                    return {"ok": True, "id": "A"}
                self.assertTrue(saw_holding, "interrupt submitted before A reached holding")
                self.assertEqual(body["mode"], "interrupt")
                self.assertNotIn("hold", body)
                events.append(("execute", "interrupt"))
                return {"ok": True, "id": "C"}
            if path == "/standby":
                return {"ok": True}
            self.fail(f"unexpected POST path {path}")

        try:
            manual_gate.get = fake_get
            manual_gate.post = fake_post
            fixtures = {
                key: Path(f"/motions/manual_gate_e2e_safe_{key}.trk")
                for key in manual_gate.FIXTURE_KEYS
            }

            result = manual_gate.held_interrupt_handoff_check(
                SimpleNamespace(passive_password="test", standby_timeout=1.0),
                "http://tracker",
                fixtures,
            )
        finally:
            manual_gate.get = old_get
            manual_gate.post = old_post

        self.assertEqual(result["result"], "PASS")
        self.assertEqual(result["A"]["state"], "stopped")
        self.assertEqual(result["A"]["stop_reason"], "interrupt")
        self.assertEqual(result["C"]["id"], "C")
        self.assertEqual(posts[-1][0], "/standby")
        self.assertLess(
            events.index(("run", "A", "holding")),
            events.index(("execute", "interrupt")),
        )

    def test_held_interrupt_handoff_rejects_failed_new_run(self):
        old_get = manual_gate.get
        old_post = manual_gate.post
        top_statuses = [status(), status(), status()]
        run_statuses = {
            "A": [
                {"id": "A", "state": "holding"},
            ],
            "C": [
                {"id": "C", "state": "failed", "error": "load failed"},
            ],
        }
        execute_count = 0
        standby_posts = []

        def fake_get(_url, path):
            if path == "/status":
                return top_statuses.pop(0) if top_statuses else status()
            if path.startswith("/status?id="):
                run_id = path.rsplit("=", 1)[1]
                return run_statuses[run_id][0]
            self.fail(f"unexpected GET path {path}")

        def fake_post(_url, path, body=None, expected=200):
            nonlocal execute_count
            self.assertEqual(expected, 200)
            if path == "/execute":
                execute_count += 1
                return {"ok": True, "id": "A" if execute_count == 1 else "C"}
            if path == "/standby":
                standby_posts.append(path)
                return {"ok": True}
            self.fail(f"unexpected POST path {path}")

        try:
            manual_gate.get = fake_get
            manual_gate.post = fake_post
            fixtures = {
                key: Path(f"/motions/manual_gate_e2e_safe_{key}.trk")
                for key in manual_gate.FIXTURE_KEYS
            }

            with self.assertRaises(manual_gate.GateError) as raised:
                manual_gate.held_interrupt_handoff_check(
                    SimpleNamespace(passive_password="test", standby_timeout=1.0),
                    "http://tracker",
                    fixtures,
                )
        finally:
            manual_gate.get = old_get
            manual_gate.post = old_post

        self.assertIn("C should not fail", str(raised.exception))
        self.assertEqual(standby_posts, [])

    def test_queue_interrupt_contract_holds_active_safe_fixtures(self):
        old_execute = manual_gate.execute
        old_run_status = manual_gate.run_status
        old_post = manual_gate.post
        old_wait_standby = manual_gate.wait_standby
        old_checkpoint = manual_gate.check_status_checkpoint
        execute_calls = []
        posts = []
        runs = {
            "A": [
                {"id": "A", "state": "running"},
                {"id": "A", "state": "stopped", "stop_reason": "interrupt"},
            ],
            "B": [
                {"id": "B", "state": "queued", "queue_pos": 1},
                {"id": "B", "state": "canceled", "stop_reason": "interrupt"},
            ],
            "C": [
                {"id": "C", "state": "holding"},
            ],
        }

        def fake_execute(_url, path, mode="queue", hold=False):
            run_id = ("A", "B", "C")[len(execute_calls)]
            execute_calls.append({
                "id": run_id,
                "path": path,
                "mode": mode,
                "hold": hold,
            })
            return run_id

        def fake_run_status(_url, run_id):
            states = runs[run_id]
            return states.pop(0) if len(states) > 1 else states[0]

        try:
            manual_gate.execute = fake_execute
            manual_gate.run_status = fake_run_status
            manual_gate.post = lambda _url, path, body=None: posts.append(path) or {"ok": True}
            manual_gate.wait_standby = lambda _url: status(ctrl="standby")
            manual_gate.check_status_checkpoint = (
                lambda *_args, **_kwargs: {"result": "PASS"}
            )
            fixtures = {
                key: Path(f"/motions/manual_gate_e2e_safe_{key}.trk")
                for key in manual_gate.FIXTURE_KEYS
            }

            result = manual_gate.queue_interrupt_status_contract_check(
                SimpleNamespace(), "http://tracker", fixtures
            )
        finally:
            manual_gate.execute = old_execute
            manual_gate.run_status = old_run_status
            manual_gate.post = old_post
            manual_gate.wait_standby = old_wait_standby
            manual_gate.check_status_checkpoint = old_checkpoint

        self.assertEqual(result["result"], "PASS")
        self.assertEqual(posts, ["/standby"])
        self.assertEqual(
            [(call["id"], call["mode"], call["hold"]) for call in execute_calls],
            [("A", "queue", True), ("B", "queue", False), ("C", "interrupt", True)],
        )
        self.assertEqual(execute_calls[0]["path"], fixtures["long_a"])
        self.assertEqual(execute_calls[2]["path"], fixtures["long_c"])


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
            key: manual_gate.write_trk(
                root / f"manual_gate_e2e_safe_{key}.trk",
                manual_gate.E2E_SAFE_FIXTURE_FRAMES[key],
                0.1,
            )
            for key in manual_gate.FIXTURE_KEYS
        }

    def test_e2e_safe_frame_extension_holds_last_reference_frame(self):
        data = b"aabbcc"

        self.assertEqual(manual_gate.repeat_frame_data(data, 3, 5), b"aabbcccccc")
        self.assertEqual(manual_gate.repeat_frame_data(data, 3, 2), b"aabb")

    def test_auto_overwrites_existing_named_fixtures_from_reference(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            stale = {
                key: manual_gate.write_trk(
                    root / f"manual_gate_e2e_safe_{key}.trk",
                    3,
                    9.0,
                )
                for key in manual_gate.FIXTURE_KEYS
            }
            manual_gate.write_trk(root / "manual_gate_long_c.trk", 220, 0.6)
            manual_gate.write_trk(
                root / "et1-generated-turn_around_180_degrees-newer.trk", 20, 0.2
            )

            fixtures, report = manual_gate.resolve_fixtures(self.args("auto"), root)

            self.assertEqual(report["source"], "e2e_safe")
            self.assertGreaterEqual(report["candidate_count"], len(stale))
            self.assertEqual(set(fixtures.keys()), set(manual_gate.FIXTURE_KEYS))
            for key, path in fixtures.items():
                self.assertEqual(path.resolve(), stale[key].resolve())
                self.assertEqual(
                    manual_gate.trk_summary(path).frames,
                    manual_gate.E2E_SAFE_FIXTURE_FRAMES[key],
                )
                self.assertEqual(report["selected"][key]["frames"],
                                 manual_gate.E2E_SAFE_FIXTURE_FRAMES[key])
                self.assertEqual(report["selected"][key]["provenance"],
                                 "app_standby_ref_derived")

    def test_auto_generates_reference_derived_e2e_safe_fixtures_not_recent_generated(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            legacy_long = manual_gate.write_trk(root / "manual_gate_long_c.trk", 220, 0.6)
            generated = manual_gate.write_trk(
                root / "et1-generated-turn_around_180_degrees-newer.trk", 260, 0.2
            )

            fixtures, report = manual_gate.resolve_fixtures(self.args("auto"), root)

            self.assertEqual(report["source"], "e2e_safe")
            self.assertEqual(set(fixtures.keys()), set(manual_gate.FIXTURE_KEYS))
            self.assertNotIn(legacy_long.resolve(), set(fixtures.values()))
            self.assertNotIn(generated.resolve(), set(fixtures.values()))
            for key, path in fixtures.items():
                self.assertEqual(path.name, f"manual_gate_e2e_safe_{key}.trk")
                self.assertLessEqual(
                    manual_gate.trk_summary(path).frames,
                    manual_gate.E2E_SAFE_FIXTURE_MAX_FRAMES,
                )

    def test_obvious_tmp_and_synthetic_files_remain_filtered(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            stable = manual_gate.write_trk(root / "manual_gate_e2e_safe_short.trk", 35, 0.3)
            real = manual_gate.write_trk(root / "et1-generated-real.trk", 20, 0.2)
            tmp_dir = root / "tmp"
            tmp_dir.mkdir()
            manual_gate.write_trk(tmp_dir / "manual_gate_long_a.trk", 260, 0.4)
            manual_gate.write_trk(root / "synthetic_generated.trk", 20, 0.5)
            manual_gate.write_trk(root / "fixture_generated.trk", 20, 0.6)

            candidates = manual_gate.find_existing_trk_candidates(root)

        paths = {candidate.path for candidate in candidates}
        self.assertEqual(paths, {stable.resolve(), real.resolve()})

    def test_existing_source_requires_e2e_safe_named_fixtures(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manual_gate.write_trk(root / "manual_gate_long_c.trk", 220, 0.6)
            manual_gate.write_trk(root / "et1-generated-real.trk", 20, 0.2)

            with self.assertRaises(manual_gate.GateError) as raised:
                manual_gate.resolve_fixtures(self.args("existing"), root)

        self.assertIn("no complete e2e-safe fixture set", str(raised.exception))
        self.assertIn("manual_gate_e2e_safe_", str(raised.exception))

    def test_existing_source_accepts_expected_named_frames_and_reports_provenance(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            named = self.write_named_fixtures(root)

            fixtures, report = manual_gate.resolve_fixtures(self.args("existing"), root)

        self.assertEqual(report["source"], "existing")
        self.assertEqual(fixtures, {key: path.resolve() for key, path in named.items()})
        for key in manual_gate.FIXTURE_KEYS:
            selected = report["selected"][key]
            self.assertEqual(selected["frames"], manual_gate.E2E_SAFE_FIXTURE_FRAMES[key])
            self.assertEqual(selected["provenance"], "existing_named_e2e_safe")
            self.assertRegex(selected["sha256"], r"^[0-9a-f]{64}$")

    def test_existing_source_rejects_named_fixtures_with_wrong_frames(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write_named_fixtures(root)
            manual_gate.write_trk(root / "manual_gate_e2e_safe_short.trk", 99, 0.3)

            with self.assertRaises(manual_gate.GateError) as raised:
                manual_gate.resolve_fixtures(self.args("existing"), root)

        self.assertIn("expected e2e-safe fixture frames", str(raised.exception))
        self.assertIn("short", str(raised.exception))
        self.assertIn("99", str(raised.exception))

    def test_existing_source_fails_when_only_synthetic_files_exist(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manual_gate.write_trk(root / "manual_gate_short.trk", 8, 0.1)

            with self.assertRaises(manual_gate.GateError) as raised:
                manual_gate.resolve_fixtures(self.args("existing"), root)

        self.assertIn("found no usable generated TRK", str(raised.exception))

    def test_auto_uses_e2e_safe_fixtures_when_sim_control_is_available(self):
        old_sim_control_status = manual_gate.sim_control_status
        try:
            manual_gate.sim_control_status = (
                lambda _port, _timeout_ms: {"available": True, "ok": True, "root_z": 0.74}
            )
            with tempfile.TemporaryDirectory() as tmp:
                fixtures, report = manual_gate.resolve_fixtures(self.args("auto"), Path(tmp))
        finally:
            manual_gate.sim_control_status = old_sim_control_status

        self.assertEqual(report["source"], "e2e_safe")
        self.assertEqual(set(fixtures.keys()), set(manual_gate.FIXTURE_KEYS))

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

    def test_manual_soak_and_landing_options_default_to_off(self):
        args = self.parse("visual")

        self.assertFalse(args.mujoco_land_settle)
        self.assertEqual(args.standby_soak_s, 0.0)

    def test_manual_soak_and_landing_options_are_opt_in(self):
        args = self.parse(
            "all",
            "--mujoco-land-settle",
            "--no-mujoco-land-hold",
            "--mujoco-land-contact-samples",
            "3",
            "--standby-soak-s",
            "30",
        )

        self.assertTrue(args.mujoco_land_settle)
        self.assertFalse(args.mujoco_land_hold)
        self.assertEqual(args.mujoco_land_contact_samples, 3)
        self.assertEqual(args.standby_soak_s, 30.0)


if __name__ == "__main__":
    unittest.main()
