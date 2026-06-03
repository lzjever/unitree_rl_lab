#!/usr/bin/env python3
"""Minimal ET1 live track ZMQ publisher example."""

from __future__ import annotations

import argparse
import math
import struct
import time

import numpy as np
import zmq


MAGIC = b"ET1LIVE1"
VERSION = 1
FLAG_RESET = 1 << 0
JOINT_DIM = 26


def build_payload(sequence: int, t: float, reset: bool) -> bytes:
    joint_pos = np.zeros(JOINT_DIM, dtype=np.float32)
    joint_vel = np.zeros(JOINT_DIM, dtype=np.float32)

    # Small visible arm/head motion for smoke testing. Replace this with the
    # upstream planner/model output in production.
    joint_pos[15] = 0.1745 + 0.2 * math.sin(2.0 * math.pi * 0.5 * t)
    joint_vel[15] = 0.2 * 2.0 * math.pi * 0.5 * math.cos(2.0 * math.pi * 0.5 * t)
    joint_pos[24] = 0.2 * math.sin(2.0 * math.pi * 0.25 * t)

    root_quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
    root_lin_vel_w = np.zeros(3, dtype=np.float32)
    root_ang_vel_w = np.zeros(3, dtype=np.float32)
    foot_contact = np.array([1.0, 1.0], dtype=np.float32)
    ref_com_rel_navi = np.zeros(3, dtype=np.float32)
    ref_com_vel_navi = np.zeros(3, dtype=np.float32)
    values = np.concatenate(
        [
            joint_pos,
            joint_vel,
            root_quat_wxyz,
            root_lin_vel_w,
            root_ang_vel_w,
            foot_contact,
            ref_com_rel_navi,
            ref_com_vel_navi,
        ]
    ).astype(np.float32, copy=False)

    flags = FLAG_RESET if reset else 0
    header = struct.pack(
        "<8sIIQQII",
        MAGIC,
        VERSION,
        flags,
        sequence,
        time.time_ns(),
        int(values.size),
        0,
    )
    return header + values.tobytes()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind", default="tcp://*:5557")
    parser.add_argument("--topic", default="et1_track")
    parser.add_argument("--hz", type=float, default=50.0)
    args = parser.parse_args()

    context = zmq.Context.instance()
    socket = context.socket(zmq.PUB)
    socket.sndhwm = 1000
    socket.bind(args.bind)

    dt = 1.0 / args.hz
    sequence = 1
    start = time.monotonic()
    next_tick = start
    while True:
        now = time.monotonic()
        if now < next_tick:
            time.sleep(next_tick - now)
        t = time.monotonic() - start
        socket.send_multipart(
            [args.topic.encode("utf-8"), build_payload(sequence, t, reset=(sequence == 1))]
        )
        sequence += 1
        next_tick += dt


if __name__ == "__main__":
    main()
