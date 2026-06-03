#!/usr/bin/env python3

import argparse
import hashlib
import json
import math
import struct
import sys
from dataclasses import dataclass
from pathlib import Path


TOOL_NAME = "derive_standby_ref_candidate"
TOOL_VERSION = "0.1.0"
CANDIDATE_FILENAME = "standby_ref.candidate.trk"
MANIFEST_FILENAME = "CANDIDATE_MANIFEST.json"
STATUS = "candidate_pending_mujoco_acceptance"

MAGIC = b"ET1TRK1\0"
VERSION = 1
JOINT_DIM = 26
BODY_COUNT = 27

FLOAT32 = 1
FLOAT64 = 2
BOOL = 3
INT32 = 4
INT64 = 5
UINT8 = 6
INT8 = 7

DTYPE_SIZE = {
    FLOAT32: 4,
    FLOAT64: 8,
    BOOL: 1,
    INT32: 4,
    INT64: 8,
    UINT8: 1,
    INT8: 1,
}

FLOAT_DTYPES = {FLOAT32, FLOAT64}
CONTACT_DTYPES = {INT32, INT64, UINT8, INT8}

REQUIRED_ARRAYS = {
    "joint_pos": ("float", [JOINT_DIM]),
    "joint_vel": ("float", [JOINT_DIM]),
    "body_pos_w": ("float", [BODY_COUNT, 3]),
    "body_quat_w": ("float", [BODY_COUNT, 4]),
    "body_lin_vel_w": ("float", [BODY_COUNT, 3]),
    "body_ang_vel_w": ("float", [BODY_COUNT, 3]),
    "left_foot_contact_state": ("contact", []),
    "right_foot_contact_state": ("contact", []),
    "ref_com_rel_navi": ("float", [3]),
    "ref_com_vel_navi": ("float", [3]),
}


class ToolError(Exception):
    pass


@dataclass(frozen=True)
class ArrayDesc:
    name: str
    dtype: int
    shape: list
    byte_count: int
    payload_offset: int

    @property
    def element_count(self):
        return product(self.shape)

    @property
    def frame_payload_bytes(self):
        if not self.shape or self.shape[0] <= 0:
            raise ToolError(f"array {self.name} is not frame-indexed")
        return self.byte_count // self.shape[0]


@dataclass(frozen=True)
class Track:
    path: Path
    data: bytes
    arrays: list
    frames: int


@dataclass(frozen=True)
class SlicedArray:
    name: str
    dtype: int
    shape: list
    payload: bytes


def product(values):
    out = 1
    for value in values:
        out *= value
    return out


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def unpack_from(fmt, data, offset, label):
    size = struct.calcsize(fmt)
    if offset + size > len(data):
        raise ToolError(f"failed to read {label}")
    values = struct.unpack_from(fmt, data, offset)
    if len(values) == 1:
        return values[0], offset + size
    return values, offset + size


def parse_trk(path):
    if "://" in str(path):
        raise ToolError("source must be a local .trk path")
    if path.suffix != ".trk":
        raise ToolError("source path must end with .trk")
    try:
        resolved = path.expanduser().resolve(strict=True)
    except FileNotFoundError as exc:
        raise ToolError(f"source does not exist: {path}") from exc
    if not resolved.is_file():
        raise ToolError(f"source is not a file: {resolved}")

    data = resolved.read_bytes()
    offset = 0
    if len(data) < len(MAGIC):
        raise ToolError("source is too small")
    if data[: len(MAGIC)] != MAGIC:
        raise ToolError("bad TRK magic")
    offset += len(MAGIC)

    version, offset = unpack_from("<I", data, offset, "version")
    if version != VERSION:
        raise ToolError(f"unsupported TRK version: {version}")
    array_count, offset = unpack_from("<I", data, offset, "array count")
    if array_count <= 0:
        raise ToolError("array count must be positive")

    arrays = []
    seen_required = set()
    for _ in range(array_count):
        name_len, offset = unpack_from("<I", data, offset, "array name length")
        if name_len <= 0:
            raise ToolError("array name is empty")
        if offset + name_len > len(data):
            raise ToolError("failed to read array name")
        try:
            name = data[offset : offset + name_len].decode("ascii")
        except UnicodeDecodeError as exc:
            raise ToolError("array name is not ASCII") from exc
        offset += name_len
        if "\0" in name:
            raise ToolError("array name contains null")

        dtype, offset = unpack_from("<I", data, offset, f"{name} dtype")
        if dtype not in DTYPE_SIZE:
            raise ToolError(f"array {name} has unknown dtype {dtype}")
        ndim, offset = unpack_from("<I", data, offset, f"{name} ndim")
        if ndim <= 0 or ndim > 4:
            raise ToolError(f"array {name} ndim must be in [1, 4]")
        shape = []
        for _dim_index in range(ndim):
            dim, offset = unpack_from("<I", data, offset, f"{name} shape")
            if dim <= 0:
                raise ToolError(f"array {name} shape contains zero")
            shape.append(dim)
        byte_count, offset = unpack_from("<Q", data, offset, f"{name} byte count")
        expected_bytes = product(shape) * DTYPE_SIZE[dtype]
        if byte_count != expected_bytes:
            raise ToolError(f"array {name} byte count does not match shape")
        if offset + byte_count > len(data):
            raise ToolError(f"array {name} payload exceeds file size")
        payload_offset = offset
        offset += byte_count

        if name in REQUIRED_ARRAYS:
            if name in seen_required:
                raise ToolError(f"duplicate required array: {name}")
            seen_required.add(name)
        arrays.append(ArrayDesc(name, dtype, shape, byte_count, payload_offset))

    if offset != len(data):
        raise ToolError("source contains trailing bytes")

    by_name = {array.name: array for array in arrays}
    missing = [name for name in REQUIRED_ARRAYS if name not in by_name]
    if missing:
        raise ToolError("missing required array: " + ", ".join(missing))

    joint_pos = by_name["joint_pos"]
    if joint_pos.shape != [joint_pos.shape[0], JOINT_DIM]:
        raise ToolError("joint_pos shape mismatch")
    frames = joint_pos.shape[0]
    if frames <= 0:
        raise ToolError("source frame count is zero")

    for name, (family, trailing_shape) in REQUIRED_ARRAYS.items():
        array = by_name[name]
        if family == "float" and array.dtype not in FLOAT_DTYPES:
            raise ToolError(f"bad dtype for {name}")
        if family == "contact" and array.dtype not in CONTACT_DTYPES:
            raise ToolError(f"bad dtype for {name}")
        if array.shape != [frames] + trailing_shape:
            raise ToolError(f"shape mismatch for {name}")

    for array in arrays:
        if not array.shape or array.shape[0] != frames:
            raise ToolError(f"array {array.name} first dimension must match joint_pos frames")

    return Track(resolved, data, arrays, frames)


def slice_track(track, start_frame, frames):
    end_frame = start_frame + frames
    out = []
    for array in track.arrays:
        frame_bytes = array.frame_payload_bytes
        begin = array.payload_offset + start_frame * frame_bytes
        end = array.payload_offset + end_frame * frame_bytes
        payload = track.data[begin:end]
        expected = frame_bytes * frames
        if len(payload) != expected:
            raise ToolError(f"failed to slice {array.name}")
        out.append(SlicedArray(array.name, array.dtype, [frames] + array.shape[1:], payload))
    return out


def build_trk_bytes(arrays):
    out = bytearray()
    out.extend(MAGIC)
    out.extend(struct.pack("<I", VERSION))
    out.extend(struct.pack("<I", len(arrays)))
    for array in arrays:
        encoded_name = array.name.encode("ascii")
        out.extend(struct.pack("<I", len(encoded_name)))
        out.extend(encoded_name)
        out.extend(struct.pack("<I", array.dtype))
        out.extend(struct.pack("<I", len(array.shape)))
        for dim in array.shape:
            out.extend(struct.pack("<I", dim))
        out.extend(struct.pack("<Q", len(array.payload)))
        out.extend(array.payload)
    return bytes(out)


def array_by_name(arrays):
    return {array.name: array for array in arrays}


def decode_values(array):
    if len(array.payload) % DTYPE_SIZE[array.dtype] != 0:
        raise ToolError(f"array {array.name} payload size is not aligned")
    fmt_by_dtype = {
        FLOAT32: "<f",
        FLOAT64: "<d",
        INT32: "<i",
        INT64: "<q",
        UINT8: "<B",
        INT8: "<b",
        BOOL: "<?",
    }
    fmt = fmt_by_dtype[array.dtype]
    return [item[0] for item in struct.iter_unpack(fmt, array.payload)]


def require_finite_float_arrays(arrays):
    errors = []
    for array in arrays:
        if array.dtype not in FLOAT_DTYPES:
            continue
        values = decode_values(array)
        for index, value in enumerate(values):
            if not math.isfinite(value):
                errors.append(f"array {array.name} has non-finite value at flat index {index}")
                break
    return errors


def frame_size(array):
    return product(array.shape[1:])


def max_joint_drift(joint_pos):
    values = decode_values(joint_pos)
    frames = joint_pos.shape[0]
    per_frame = frame_size(joint_pos)
    first = values[:per_frame]
    max_drift = 0.0
    for frame in range(frames):
        base = frame * per_frame
        for joint in range(per_frame):
            max_drift = max(max_drift, abs(values[base + joint] - first[joint]))
    return max_drift


def max_root_xyz_drift(body_pos_w):
    values = decode_values(body_pos_w)
    frames = body_pos_w.shape[0]
    per_frame = frame_size(body_pos_w)
    first = values[:3]
    max_drift = 0.0
    for frame in range(frames):
        base = frame * per_frame
        dx = values[base] - first[0]
        dy = values[base + 1] - first[1]
        dz = values[base + 2] - first[2]
        max_drift = max(max_drift, math.sqrt(dx * dx + dy * dy + dz * dz))
    return max_drift


def quat_tilt_deg(w, x, y, z):
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 1e-8:
        raise ToolError("root quaternion norm is zero")
    x /= norm
    y /= norm
    z_axis_z = 1.0 - 2.0 * (x * x + y * y)
    z_axis_z = max(-1.0, min(1.0, z_axis_z))
    return math.degrees(math.acos(z_axis_z))


def max_root_tilt_deg(body_quat_w):
    values = decode_values(body_quat_w)
    frames = body_quat_w.shape[0]
    per_frame = frame_size(body_quat_w)
    max_tilt = 0.0
    for frame in range(frames):
        base = frame * per_frame
        max_tilt = max(max_tilt, quat_tilt_deg(values[base], values[base + 1],
                                               values[base + 2], values[base + 3]))
    return max_tilt


def max_root_lin_vel(body_lin_vel_w):
    values = decode_values(body_lin_vel_w)
    frames = body_lin_vel_w.shape[0]
    per_frame = frame_size(body_lin_vel_w)
    max_vel = 0.0
    for frame in range(frames):
        base = frame * per_frame
        vx = values[base]
        vy = values[base + 1]
        vz = values[base + 2]
        max_vel = max(max_vel, math.sqrt(vx * vx + vy * vy + vz * vz))
    return max_vel


def contact_metrics(left_contact, right_contact):
    errors = []
    left = [int(value) for value in decode_values(left_contact)]
    right = [int(value) for value in decode_values(right_contact)]
    for side, values in (("left", left), ("right", right)):
        invalid = sorted({value for value in values if value not in (0, 1, 2)})
        if invalid:
            errors.append(f"{side} contact has invalid values: {invalid}")
    left_values = sorted(set(left))
    right_values = sorted(set(right))
    left_constant = len(left_values) == 1
    right_constant = len(right_values) == 1
    if not left_constant:
        errors.append("left contact is not constant over tail window")
    if not right_constant:
        errors.append("right contact is not constant over tail window")
    metrics = {
        "left_constant": left_constant,
        "right_constant": right_constant,
        "left_values": left_values,
        "right_values": right_values,
    }
    return metrics, errors


def build_static_metrics(arrays):
    by_name = array_by_name(arrays)
    errors = require_finite_float_arrays(arrays)
    contact, contact_errors = contact_metrics(
        by_name["left_foot_contact_state"],
        by_name["right_foot_contact_state"],
    )
    errors.extend(contact_errors)
    metrics = {
        "joint_max_drift": max_joint_drift(by_name["joint_pos"]),
        "root_xyz_drift": max_root_xyz_drift(by_name["body_pos_w"]),
        "root_tilt_max_deg": max_root_tilt_deg(by_name["body_quat_w"]),
        "root_lin_vel_max": max_root_lin_vel(by_name["body_lin_vel_w"]),
        "contact": contact,
    }
    return metrics, errors


def positive_int(value):
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return parsed


def positive_float(value):
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be positive")
    return parsed


def is_relative_to(path, base):
    try:
        path.relative_to(base)
        return True
    except ValueError:
        return False


def reject_config_out_dir(out_dir):
    tracker_root = Path(__file__).resolve().parents[1]
    config_dir = (tracker_root / "config").resolve(strict=False)
    resolved_out = out_dir.expanduser().resolve(strict=False)
    if resolved_out == config_dir or is_relative_to(resolved_out, config_dir):
        raise ToolError("refusing to write into agentic-et1-tracker config tree")


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Derive an offline standby_ref TRK candidate from an exact tail slice."
    )
    parser.add_argument("--source", required=True, type=Path, help="source .trk path")
    parser.add_argument("--out-dir", required=True, type=Path, help="candidate output directory")
    parser.add_argument("--tail-frames", default=25, type=positive_int,
                        help="number of source tail frames to slice")
    parser.add_argument("--fps", default=50.0, type=positive_float,
                        help="fps to record in the candidate manifest")
    return parser.parse_args(argv)


def build_manifest(track, candidate_path, candidate_bytes, args, start_frame, metrics):
    frames = args.tail_frames
    return {
        "status": STATUS,
        "tool": {
            "name": TOOL_NAME,
            "version": TOOL_VERSION,
        },
        "source": {
            "path": str(track.path),
            "sha256": sha256_file(track.path),
            "size_bytes": len(track.data),
            "frames": track.frames,
        },
        "source_window": {
            "frame_start": start_frame,
            "frame_end": start_frame + frames - 1,
            "frames": frames,
            "mode": "tail_exact_slice",
        },
        "candidate": {
            "path": str(candidate_path.resolve(strict=False)),
            "filename": CANDIDATE_FILENAME,
            "sha256": sha256_bytes(candidate_bytes),
            "size_bytes": len(candidate_bytes),
            "frames": frames,
            "fps": args.fps,
            "duration_s": (frames - 1) / args.fps,
        },
        "static_metrics": metrics,
    }


def write_outputs(out_dir, candidate_bytes, manifest):
    out_dir.mkdir(parents=True, exist_ok=True)
    candidate_path = out_dir / CANDIDATE_FILENAME
    manifest_path = out_dir / MANIFEST_FILENAME
    candidate_path.write_bytes(candidate_bytes)
    with manifest_path.open("w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2, sort_keys=True)
        f.write("\n")
    return candidate_path, manifest_path


def run(args):
    reject_config_out_dir(args.out_dir)
    track = parse_trk(args.source)
    if args.tail_frames > track.frames:
        raise ToolError(
            f"tail-frames must be <= source frames ({args.tail_frames} > {track.frames})"
        )

    start_frame = track.frames - args.tail_frames
    sliced_arrays = slice_track(track, start_frame, args.tail_frames)
    metrics, metric_errors = build_static_metrics(sliced_arrays)
    if metric_errors:
        raise ToolError("static validation failed: " + "; ".join(metric_errors))

    candidate_bytes = build_trk_bytes(sliced_arrays)
    out_dir = args.out_dir.expanduser().resolve(strict=False)
    candidate_path = out_dir / CANDIDATE_FILENAME
    manifest = build_manifest(track, candidate_path, candidate_bytes, args, start_frame, metrics)
    written_candidate, manifest_path = write_outputs(out_dir, candidate_bytes, manifest)

    print(f"candidate={written_candidate}")
    print(f"manifest={manifest_path}")
    print(
        "metrics="
        f"joint_max_drift={metrics['joint_max_drift']:.9g} "
        f"root_xyz_drift={metrics['root_xyz_drift']:.9g} "
        f"root_tilt_max_deg={metrics['root_tilt_max_deg']:.9g} "
        f"root_lin_vel_max={metrics['root_lin_vel_max']:.9g} "
        f"left_contact={metrics['contact']['left_values']} "
        f"right_contact={metrics['contact']['right_values']}"
    )


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        run(args)
    except ToolError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
