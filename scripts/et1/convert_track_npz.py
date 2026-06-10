#!/usr/bin/env python3
"""Convert an ET1 tracking NPZ file into a raw binary cache for deployment.

The cache stores selected arrays from the source NPZ without preprocessing.
The default footstate profile requires and stores foot-support labels used by
newer GeneralTracker policies. Reference-COM arrays are stored when present.
"""

from __future__ import annotations

import argparse
import struct
from pathlib import Path

import numpy as np


MAGIC = b"ET1TRK1\x00"
CACHE_VERSION = 1

DTYPE_CODES = {
    np.dtype(np.float32): 1,
    np.dtype(np.float64): 2,
    np.dtype(np.bool_): 3,
    np.dtype(np.int32): 4,
    np.dtype(np.int64): 5,
    np.dtype(np.uint8): 6,
    np.dtype(np.int8): 7,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert ET1 track npz to runtime cache.")
    parser.add_argument("--input", required=True, help="Path to the source npz file.")
    parser.add_argument("--output", required=True, help="Path to the output .et1trk file.")
    parser.add_argument(
        "--profile",
        choices=("footstate", "auto", "latest_general_tracker", "legacy"),
        default="footstate",
        help=(
            "footstate requires left/right foot-support arrays and keeps optional ref-COM arrays when present. "
            "auto requires base motion arrays and keeps optional foot/ref-COM arrays when present. "
            "latest_general_tracker requires foot-support and ref-COM arrays. "
            "legacy only requires the original motion arrays."
        ),
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    src = Path(args.input)
    dst = Path(args.output)

    required_base = {
        "joint_pos",
        "joint_vel",
        "body_pos_w",
        "body_quat_w",
        "body_lin_vel_w",
        "body_ang_vel_w",
    }
    required_footstate = {
        "left_foot_contact_state",
        "right_foot_contact_state",
    }
    required_com = {
        "ref_com_rel_navi",
        "ref_com_vel_navi",
    }
    required = set(required_base)
    if args.profile in ("footstate", "latest_general_tracker"):
        required.update(required_footstate)
    if args.profile == "latest_general_tracker":
        required.update(required_com)

    data = np.load(src, allow_pickle=False)
    missing = required.difference(data.files)
    if missing:
        raise ValueError(
            f"Expected ET1 motion arrays missing from npz for profile "
            f"{args.profile!r}: {sorted(missing)}"
        )
    arrays = {key: np.ascontiguousarray(np.asarray(data[key])) for key in required}
    for key in (
        "left_foot_contact_state",
        "right_foot_contact_state",
        "ref_com_rel_navi",
        "ref_com_vel_navi",
    ):
        if key in data.files and key not in arrays:
            arrays[key] = np.ascontiguousarray(np.asarray(data[key]))

    frame_count = validate_arrays(arrays, args.profile)

    dst.parent.mkdir(parents=True, exist_ok=True)
    with dst.open("wb") as f:
        f.write(struct.pack("<8sII", MAGIC, CACHE_VERSION, len(arrays)))
        for name, arr in arrays.items():
            dtype = arr.dtype
            if dtype not in DTYPE_CODES:
                raise ValueError(f"Unsupported dtype for {name}: {dtype}")
            name_bytes = name.encode("utf-8")
            shape = arr.shape
            f.write(struct.pack("<I", len(name_bytes)))
            f.write(name_bytes)
            f.write(struct.pack("<II", DTYPE_CODES[dtype], len(shape)))
            if shape:
                f.write(struct.pack("<" + "I" * len(shape), *shape))
            f.write(struct.pack("<Q", arr.nbytes))
            f.write(arr.tobytes(order="C"))
    print(
        f"Wrote {dst} with {frame_count} frames and {len(arrays)} arrays "
        f"for profile {args.profile}."
    )


def validate_arrays(arrays: dict[str, np.ndarray], profile: str) -> int:
    joint_pos = arrays["joint_pos"]
    frame_count = joint_pos.shape[0]
    expected_shapes = {
        "joint_pos": (frame_count, 26),
        "joint_vel": (frame_count, 26),
        "body_pos_w": (frame_count, 27, 3),
        "body_quat_w": (frame_count, 27, 4),
        "body_lin_vel_w": (frame_count, 27, 3),
        "body_ang_vel_w": (frame_count, 27, 3),
    }
    latest_shapes = {
        "left_foot_contact_state": (frame_count,),
        "right_foot_contact_state": (frame_count,),
        "ref_com_rel_navi": (frame_count, 3),
        "ref_com_vel_navi": (frame_count, 3),
    }
    if profile == "latest_general_tracker":
        expected_shapes.update(latest_shapes)
    else:
        expected_shapes.update({key: shape for key, shape in latest_shapes.items() if key in arrays})

    for key, shape in expected_shapes.items():
        if arrays[key].shape != shape:
            raise ValueError(f"{key} shape {arrays[key].shape} does not match {shape}.")

    for key, arr in arrays.items():
        if arr.dtype not in DTYPE_CODES:
            raise ValueError(f"Unsupported dtype for {key}: {arr.dtype}")
        if arr.dtype.kind == "f" and not np.isfinite(arr).all():
            raise ValueError(f"{key} contains NaN or Inf values.")

    for key in ("left_foot_contact_state", "right_foot_contact_state"):
        if key not in arrays:
            continue
        invalid = np.setdiff1d(arrays[key], np.array([0, 1, 2], dtype=arrays[key].dtype))
        if invalid.size:
            raise ValueError(f"{key} contains invalid support states: {invalid.tolist()}")

    root_quat_norm = np.linalg.norm(arrays["body_quat_w"][:, 0], axis=1)
    if np.max(np.abs(root_quat_norm - 1.0)) > 1e-3:
        raise ValueError(
            "body_quat_w root quaternion is not normalized; "
            f"norm range [{root_quat_norm.min()}, {root_quat_norm.max()}]."
        )
    return frame_count


if __name__ == "__main__":
    main()
