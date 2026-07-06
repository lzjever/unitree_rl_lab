#!/usr/bin/env python3
"""Convert an ET1 style-conditioned tracking NPZ into a deployment cache.

This converter is for GeneralTrackerCJM policies trained with:

  actor + actor_z_style + actor_style_phase + actor_history

It requires the arrays needed by the style-conditioned observation path instead
of treating them as optional side data.
"""

from __future__ import annotations

import argparse
import struct
from pathlib import Path

import numpy as np

from convert_track_npz import CACHE_VERSION, DTYPE_CODES, MAGIC


REQUIRED_BASE = (
    "joint_pos",
    "joint_vel",
    "body_pos_w",
    "body_quat_w",
    "body_lin_vel_w",
    "body_ang_vel_w",
)
REQUIRED_GENERAL_TRACKER = (
    "left_foot_contact_state",
    "right_foot_contact_state",
    "ref_com_rel_navi",
    "ref_com_vel_navi",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert an ET1 style-conditioned track NPZ to .et1trk."
    )
    parser.add_argument("--input", required=True, help="Path to the source npz file.")
    parser.add_argument("--output", required=True, help="Path to the output .et1trk file.")
    parser.add_argument(
        "--style-key",
        default="z_style_50",
        help="Style latent array in the source NPZ. Default: z_style_50.",
    )
    parser.add_argument(
        "--output-style-key",
        default=None,
        choices=("z_style_50", "z_style"),
        help="Style latent array name written to .et1trk. Default: same as --style-key.",
    )
    parser.add_argument(
        "--style-dim",
        type=int,
        default=16,
        help="Expected style latent width. Default: 16.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    src = Path(args.input)
    dst = Path(args.output)
    output_style_key = args.output_style_key or args.style_key

    data = np.load(src, allow_pickle=False)
    required = [*REQUIRED_BASE, *REQUIRED_GENERAL_TRACKER, args.style_key]
    missing = [key for key in required if key not in data.files]
    if missing:
        raise ValueError(
            "Expected ET1 style motion arrays missing: "
            f"{missing}. Available arrays: {sorted(data.files)}"
        )
    if output_style_key not in ("z_style_50", "z_style"):
        raise ValueError(
            f"Output style key must be 'z_style_50' or 'z_style', got {output_style_key!r}."
        )

    arrays: dict[str, np.ndarray] = {}
    for key in [*REQUIRED_BASE, *REQUIRED_GENERAL_TRACKER]:
        arrays[key] = np.ascontiguousarray(np.asarray(data[key]))
    arrays[output_style_key] = np.ascontiguousarray(np.asarray(data[args.style_key]))

    frame_count = validate_arrays(arrays, output_style_key, args.style_dim)
    write_cache(dst, arrays)
    print(
        f"Wrote {dst} with {frame_count} frames, {len(arrays)} arrays, "
        f"style key {output_style_key!r}."
    )


def validate_arrays(
    arrays: dict[str, np.ndarray],
    style_key: str,
    style_dim: int,
) -> int:
    joint_pos = arrays["joint_pos"]
    if joint_pos.ndim != 2:
        raise ValueError(f"joint_pos must be 2D, got {joint_pos.shape}.")
    frame_count = joint_pos.shape[0]
    expected_shapes = {
        "joint_pos": (frame_count, 26),
        "joint_vel": (frame_count, 26),
        "body_pos_w": (frame_count, 27, 3),
        "body_quat_w": (frame_count, 27, 4),
        "body_lin_vel_w": (frame_count, 27, 3),
        "body_ang_vel_w": (frame_count, 27, 3),
        "left_foot_contact_state": (frame_count,),
        "right_foot_contact_state": (frame_count,),
        "ref_com_rel_navi": (frame_count, 3),
        "ref_com_vel_navi": (frame_count, 3),
        style_key: (frame_count, style_dim),
    }

    for key, shape in expected_shapes.items():
        if arrays[key].shape != shape:
            raise ValueError(f"{key} shape {arrays[key].shape} does not match {shape}.")

    for key, arr in arrays.items():
        if arr.dtype not in DTYPE_CODES:
            raise ValueError(f"Unsupported dtype for {key}: {arr.dtype}")
        if arr.dtype.kind == "f" and not np.isfinite(arr).all():
            raise ValueError(f"{key} contains NaN or Inf values.")

    for key in ("left_foot_contact_state", "right_foot_contact_state"):
        invalid = np.setdiff1d(arrays[key], np.array([0, 1, 2], dtype=arrays[key].dtype))
        if invalid.size:
            raise ValueError(f"{key} contains invalid support states: {invalid.tolist()}")

    style = arrays[style_key].astype(np.float32, copy=False)
    style_norm = np.linalg.norm(style, axis=1)
    zero_frames = np.flatnonzero(style_norm <= 1e-6)
    if zero_frames.size:
        preview = zero_frames[:10].tolist()
        raise ValueError(
            f"{style_key} contains zero style latent frames; first bad indices: {preview}. "
            "Style-conditioned GeneralTrackerCJM requires real z_style data."
        )

    root_quat_norm = np.linalg.norm(arrays["body_quat_w"][:, 0], axis=1)
    if np.max(np.abs(root_quat_norm - 1.0)) > 1e-3:
        raise ValueError(
            "body_quat_w root quaternion is not normalized; "
            f"norm range [{root_quat_norm.min()}, {root_quat_norm.max()}]."
        )
    return frame_count


def write_cache(dst: Path, arrays: dict[str, np.ndarray]) -> None:
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


if __name__ == "__main__":
    main()
