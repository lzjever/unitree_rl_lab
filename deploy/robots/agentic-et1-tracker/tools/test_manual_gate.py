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

    def test_final_settle_accepts_stable_standby(self):
        reason = manual_gate.final_settle_reason(
            status(),
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

    def test_parse_sim_control_status(self):
        parsed = manual_gate.parse_sim_control_status(
            "ok band=0 length=1.100 left_contact=1 right_contact=0 both=0 root_z=-0.709"
        )

        self.assertTrue(parsed["available"])
        self.assertTrue(parsed["ok"])
        self.assertEqual(parsed["band"], 0.0)
        self.assertTrue(parsed["left_contact"])
        self.assertFalse(parsed["right_contact"])
        self.assertFalse(parsed["both"])
        self.assertEqual(parsed["root_z"], -0.709)

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


class ManualGateFixtureSelectionTests(unittest.TestCase):
    def args(self, source="auto", gate="e2e"):
        return SimpleNamespace(
            fixture_source=source,
            gate=gate,
            visual_run_action=True,
            sim_control_port=8090,
            sim_control_timeout_ms=200,
        )

    def test_auto_prefers_existing_generated_trk_and_ignores_manual_gate(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            manual_gate.write_trk(root / "manual_gate_short.trk", 8, 0.1)
            existing = manual_gate.write_trk(root / "et1-generated-real.trk", 20, 0.2)

            fixtures, report = manual_gate.resolve_fixtures(self.args("auto"), root)

        self.assertEqual(report["source"], "existing")
        self.assertEqual(report["candidate_count"], 1)
        self.assertEqual(set(fixtures.values()), {existing.resolve()})
        self.assertNotIn("manual_gate", report["selected"]["short"]["path"])

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


if __name__ == "__main__":
    unittest.main()
