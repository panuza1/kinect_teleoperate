#!/usr/bin/env python3
"""Loopback SONIC v1 smoke source: neutral → left arm → neutral → right arm."""

import json
import math
import struct
import sys
import time

import zmq


PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 5556
PHASE_SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
RATE_HZ = 50
HEADER_SIZE = 1280
NEUTRAL = [
    -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
    -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
    0.0, 0.0, 0.0,
    0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0,
    0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0,
]


def packet(topic, fields, payload):
    header = json.dumps({"v": 1, "endian": "le", "count": 1, "fields": fields},
                        separators=(",", ":")).encode()
    if len(header) > HEADER_SIZE:
        raise ValueError("SONIC v1 header exceeds 1280 bytes")
    return topic + header + bytes(HEADER_SIZE - len(header)) + payload


COMMAND_FIELDS = [
    {"name": "start", "dtype": "u8", "shape": [1]},
    {"name": "stop", "dtype": "u8", "shape": [1]},
    {"name": "planner", "dtype": "u8", "shape": [1]},
]
POSE_FIELDS = [
    {"name": "joint_pos", "dtype": "f32", "shape": [1, 29]},
    {"name": "joint_vel", "dtype": "f32", "shape": [1, 29]},
    {"name": "body_quat_w", "dtype": "f32", "shape": [1, 4]},
    {"name": "frame_index", "dtype": "i64", "shape": [1]},
    {"name": "catch_up", "dtype": "u8", "shape": [1]},
]


def main():
    if PHASE_SECONDS <= 0:
        raise ValueError("phase duration must be positive")
    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    publisher.setsockopt(zmq.LINGER, 0)
    publisher.bind(f"tcp://127.0.0.1:{PORT}")
    time.sleep(0.5)
    period = 1.0 / RATE_HZ
    total_frames = round(PHASE_SECONDS * RATE_HZ) * 5
    started = time.monotonic()
    next_tick = started + 0.5
    sent = 0
    try:
        for frame in range(total_frames):
            if frame == 0 or frame % RATE_HZ == 0:
                command = packet(b"command", COMMAND_FIELDS,
                                 struct.pack("BBB", int(frame == 0), 0, 0))
                publisher.send(command)
            phase, phase_frame = divmod(frame, round(PHASE_SECONDS * RATE_HZ))
            amount = 0.5 - 0.5 * math.cos(math.pi * phase_frame / (round(PHASE_SECONDS * RATE_HZ) - 1))
            pose = NEUTRAL.copy()
            if phase == 1:
                pose[15] += 0.45 * amount
            elif phase == 3:
                pose[22] += 0.45 * amount
            payload = (struct.pack("<29f", *pose) + struct.pack("<29f", *([0.0] * 29))
                       + struct.pack("<4f", 1.0, 0.0, 0.0, 0.0)
                       + struct.pack("<qB", frame, 0))
            publisher.send(packet(b"pose", POSE_FIELDS, payload))
            sent += 1
            if frame % RATE_HZ == 0:
                print(f"synthetic_pose_tx_fps={sent / max(time.monotonic() - started, 1e-6):.1f} frame={frame}", flush=True)
            next_tick += period
            time.sleep(max(0.0, next_tick - time.monotonic()))
    finally:
        publisher.close()
        context.term()


if __name__ == "__main__":
    main()
