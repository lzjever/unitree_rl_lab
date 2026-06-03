#!/usr/bin/env python3

import hashlib
import json
import math
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parent
TRACKER_ROOT = TOOLS_DIR.parent
TOOL = TOOLS_DIR / "derive_standby_ref_candidate.py"
CONFIG_DIR = TRACKER_ROOT / "config"
RELEASE_STANDBY_DIR = TRACKER_ROOT / "config" / "reference" / "standby" / "v0"
RELEASE_STANDBY_REF = RELEASE_STANDBY_DIR / "standby_ref.trk"
CANDIDATE_FILENAME = "standby_ref.candidate.trk"
MANIFEST_FILENAME = "CANDIDATE_MANIFEST.json"

MAGIC = b"ET1TRK1\0"
VERSION = 1
FLOAT32 = 1
INT64 = 5
JOINT_DIM = 26
BODY_COUNT = 27

REQUIRED_ARRAYS = [
    ("joint_pos", FLOAT32, [JOINT_DIM]),
    ("joint_vel", FLOAT32, [JOINT_DIM]),
    ("body_pos_w", FLOAT32, [BODY_COUNT, 3]),
    ("body_quat_w", FLOAT32, [BODY_COUNT, 4]),
    ("body_lin_vel_w", FLOAT32, [BODY_COUNT, 3]),
    ("body_ang_vel_w", FLOAT32, [BODY_COUNT, 3]),
    ("left_foot_contact_state", INT64, []),
    ("right_foot_contact_state", INT64, []),
    ("ref_com_rel_navi", FLOAT32, [3]),
    ("ref_com_vel_navi", FLOAT32, [3]),
]


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def product(values):
    out = 1
    for value in values:
        out *= value
    return out


def float_values_for(name, frames):
    if name == "joint_pos":
        return [float(frame * 10 + joint) for frame in range(frames) for joint in range(JOINT_DIM)]
    if name == "joint_vel":
        return [float(frame) * 0.01 for frame in range(frames) for _ in range(JOINT_DIM)]
    if name == "body_pos_w":
        values = []
        for frame in range(frames):
            for body in range(BODY_COUNT):
                values.extend([frame * 0.01 + body, body * 0.1, 0.7 + body * 0.001])
        return values
    if name == "body_quat_w":
        return [
            component
            for _frame in range(frames)
            for _body in range(BODY_COUNT)
            for component in (1.0, 0.0, 0.0, 0.0)
        ]
    if name == "body_lin_vel_w":
        values = []
        for frame in range(frames):
            for body in range(BODY_COUNT):
                values.extend([frame * 0.001 if body == 0 else 0.0, 0.0, 0.0])
        return values
    if name == "body_ang_vel_w":
        return [0.0 for _ in range(frames * BODY_COUNT * 3)]
    if name == "ref_com_rel_navi":
        return [float(frame) * 0.02 + axis for frame in range(frames) for axis in range(3)]
    if name == "ref_com_vel_navi":
        return [0.0 for _ in range(frames * 3)]
    raise AssertionError(f"unexpected float array {name}")


def contact_values_for(frames, overrides=None):
    values = [1 for _ in range(frames)]
    for index, value in (overrides or {}).items():
        values[index] = value
    return values


def write_fixture_trk(
    path,
    frames=6,
    omit_arrays=None,
    shape_overrides=None,
    float_overrides=None,
    contact_overrides=None,
):
    omit_arrays = set(omit_arrays or [])
    shape_overrides = shape_overrides or {}
    float_overrides = float_overrides or {}
    contact_overrides = contact_overrides or {}
    arrays = [array for array in REQUIRED_ARRAYS if array[0] not in omit_arrays]
    with path.open("wb") as out:
        out.write(MAGIC)
        out.write(struct.pack("<I", VERSION))
        out.write(struct.pack("<I", len(arrays)))
        for name, dtype, trailing_shape in arrays:
            shape = [frames] + shape_overrides.get(name, trailing_shape)
            out.write(struct.pack("<I", len(name)))
            out.write(name.encode("ascii"))
            out.write(struct.pack("<I", dtype))
            out.write(struct.pack("<I", len(shape)))
            for dim in shape:
                out.write(struct.pack("<I", dim))

            if dtype == FLOAT32:
                if name in shape_overrides:
                    values = [0.0 for _ in range(product(shape))]
                else:
                    values = float_values_for(name, frames)
                for index, value in float_overrides.get(name, {}).items():
                    values[index] = value
                payload = struct.pack("<" + "f" * len(values), *values)
            elif dtype == INT64:
                values = contact_values_for(frames, contact_overrides.get(name))
                payload = struct.pack("<" + "q" * len(values), *values)
            else:
                raise AssertionError(f"unsupported fixture dtype {dtype}")
            self_check_elements = product(shape)
            expected_size = self_check_elements * (4 if dtype == FLOAT32 else 8)
            assert len(payload) == expected_size
            out.write(struct.pack("<Q", len(payload)))
            out.write(payload)


def read_trk_arrays(path):
    arrays = {}
    data = path.read_bytes()
    offset = 0

    if data[: len(MAGIC)] != MAGIC:
        raise AssertionError("bad magic")
    offset += len(MAGIC)
    version, array_count = struct.unpack_from("<II", data, offset)
    offset += 8
    if version != VERSION:
        raise AssertionError(f"unexpected version {version}")

    for _ in range(array_count):
        name_len = struct.unpack_from("<I", data, offset)[0]
        offset += 4
        name = data[offset : offset + name_len].decode("ascii")
        offset += name_len
        dtype, ndim = struct.unpack_from("<II", data, offset)
        offset += 8
        shape = list(struct.unpack_from("<" + "I" * ndim, data, offset))
        offset += 4 * ndim
        byte_count = struct.unpack_from("<Q", data, offset)[0]
        offset += 8
        payload = data[offset : offset + byte_count]
        offset += byte_count
        elements = product(shape)
        if dtype == FLOAT32:
            values = list(struct.unpack("<" + "f" * elements, payload))
        elif dtype == INT64:
            values = list(struct.unpack("<" + "q" * elements, payload))
        else:
            raise AssertionError(f"unsupported test dtype {dtype}")
        arrays[name] = {"dtype": dtype, "shape": shape, "values": values}

    if offset != len(data):
        raise AssertionError("trailing bytes")
    return arrays


class DeriveStandbyRefCandidateTest(unittest.TestCase):
    def run_tool(self, *args):
        return subprocess.run(
            [sys.executable, str(TOOL), *args],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

    def test_tail_slice_candidate_manifest(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            source = tmp_path / "source.trk"
            out_dir = tmp_path / "candidate"
            write_fixture_trk(source, frames=6)

            result = self.run_tool(
                "--source",
                str(source),
                "--out-dir",
                str(out_dir),
                "--tail-frames",
                "3",
                "--fps",
                "20",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            candidate = out_dir / "standby_ref.candidate.trk"
            manifest_path = out_dir / "CANDIDATE_MANIFEST.json"
            self.assertTrue(candidate.exists())
            self.assertTrue(manifest_path.exists())
            self.assertFalse((out_dir / "standby_ref.trk").exists())

            arrays = read_trk_arrays(candidate)
            self.assertEqual(arrays["joint_pos"]["shape"], [3, JOINT_DIM])
            self.assertEqual(arrays["joint_pos"]["values"][0], 30.0)
            self.assertEqual(arrays["joint_pos"]["values"][-1], 75.0)
            self.assertEqual(arrays["left_foot_contact_state"]["values"], [1, 1, 1])

            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            self.assertEqual(manifest["status"], "candidate_pending_mujoco_acceptance")
            self.assertEqual(manifest["source"]["path"], str(source.resolve()))
            self.assertEqual(manifest["source"]["sha256"], sha256_file(source))
            self.assertEqual(manifest["candidate"]["path"], str(candidate.resolve()))
            self.assertEqual(manifest["candidate"]["sha256"], sha256_file(candidate))
            self.assertEqual(manifest["source_window"]["frame_start"], 3)
            self.assertEqual(manifest["source_window"]["frame_end"], 5)
            self.assertEqual(manifest["candidate"]["frames"], 3)
            self.assertEqual(manifest["candidate"]["fps"], 20.0)
            self.assertTrue(math.isclose(manifest["candidate"]["duration_s"], 0.1))
            self.assertEqual(manifest["static_metrics"]["contact"]["left_values"], [1])
            self.assertTrue(manifest["static_metrics"]["contact"]["left_constant"])

    def assert_invalid_source(self, source, expected_error):
        out_dir = source.parent / "candidate"
        result = self.run_tool(
            "--source",
            str(source),
            "--out-dir",
            str(out_dir),
            "--tail-frames",
            "2",
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(expected_error, result.stderr)
        self.assertFalse((out_dir / CANDIDATE_FILENAME).exists())
        self.assertFalse((out_dir / MANIFEST_FILENAME).exists())

    def test_invalid_tail_window_is_nonzero(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            source = tmp_path / "source.trk"
            write_fixture_trk(source, frames=2)

            for tail_frames in ("0", "3"):
                with self.subTest(tail_frames=tail_frames):
                    out_dir = tmp_path / f"candidate_{tail_frames}"
                    result = self.run_tool(
                        "--source",
                        str(source),
                        "--out-dir",
                        str(out_dir),
                        "--tail-frames",
                        tail_frames,
                    )
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn("tail-frames", result.stderr)
                    self.assertFalse((out_dir / CANDIDATE_FILENAME).exists())

    def test_config_tree_out_dir_is_rejected_without_writes(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            source = tmp_path / "source.trk"
            write_fixture_trk(source, frames=4)
            out_dir = CONFIG_DIR / f"candidate_reject_{tmp_path.name}"

            result = self.run_tool(
                "--source",
                str(source),
                "--out-dir",
                str(out_dir),
                "--tail-frames",
                "2",
            )
            wrote_candidate = (out_dir / CANDIDATE_FILENAME).exists()
            wrote_manifest = (out_dir / MANIFEST_FILENAME).exists()
            shutil.rmtree(out_dir, ignore_errors=True)

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("config tree", result.stderr)
            self.assertFalse(wrote_candidate)
            self.assertFalse(wrote_manifest)

    def test_release_standby_ref_path_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / "source.trk"
            write_fixture_trk(source, frames=4)
            release_ref_existed = RELEASE_STANDBY_REF.exists()
            release_candidate = RELEASE_STANDBY_DIR / "standby_ref.candidate.trk"
            release_candidate_existed = release_candidate.exists()

            result = self.run_tool(
                "--source",
                str(source),
                "--out-dir",
                str(RELEASE_STANDBY_DIR),
                "--tail-frames",
                "2",
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("config tree", result.stderr)
            self.assertEqual(RELEASE_STANDBY_REF.exists(), release_ref_existed)
            self.assertEqual(release_candidate.exists(), release_candidate_existed)

    def test_allow_contact_changes_option_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            source = tmp_path / "source.trk"
            out_dir = tmp_path / "candidate"
            write_fixture_trk(
                source,
                frames=4,
                contact_overrides={"left_foot_contact_state": {3: 0}},
            )

            result = self.run_tool(
                "--source",
                str(source),
                "--out-dir",
                str(out_dir),
                "--tail-frames",
                "4",
                "--allow-contact-changes",
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("allow-contact-changes", result.stderr)
            self.assertFalse((out_dir / CANDIDATE_FILENAME).exists())
            self.assertFalse((out_dir / MANIFEST_FILENAME).exists())

    def test_bad_magic_is_nonzero(self):
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / "bad_magic.trk"
            source.write_bytes(b"not a trk")

            self.assert_invalid_source(source, "bad TRK magic")

    def test_missing_required_array_is_nonzero(self):
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / "missing_required.trk"
            write_fixture_trk(source, frames=4, omit_arrays={"joint_vel"})

            self.assert_invalid_source(source, "missing required array")

    def test_shape_mismatch_is_nonzero(self):
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / "shape_mismatch.trk"
            write_fixture_trk(
                source,
                frames=4,
                shape_overrides={"body_pos_w": [BODY_COUNT, 2]},
            )

            self.assert_invalid_source(source, "shape mismatch for body_pos_w")

    def test_nan_and_inf_float_values_are_nonzero(self):
        for label, value in (("nan", float("nan")), ("inf", float("inf"))):
            with self.subTest(label=label), tempfile.TemporaryDirectory() as tmp:
                source = Path(tmp) / f"{label}.trk"
                write_fixture_trk(
                    source,
                    frames=4,
                    float_overrides={"joint_pos": {2 * JOINT_DIM: value}},
                )

                self.assert_invalid_source(source, "non-finite value")

    def test_invalid_contact_value_is_nonzero(self):
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / "bad_contact.trk"
            write_fixture_trk(
                source,
                frames=4,
                contact_overrides={"left_foot_contact_state": {2: 7}},
            )

            self.assert_invalid_source(source, "invalid values")

    def test_nonconstant_contact_is_nonzero(self):
        with tempfile.TemporaryDirectory() as tmp:
            source = Path(tmp) / "nonconstant_contact.trk"
            write_fixture_trk(
                source,
                frames=4,
                contact_overrides={"right_foot_contact_state": {3: 0}},
            )

            self.assert_invalid_source(source, "right contact is not constant")


if __name__ == "__main__":
    unittest.main()
