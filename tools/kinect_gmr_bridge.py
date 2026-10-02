#!/usr/bin/env python3
"""Loopback bridge from packed K4ABT frames to official GMR G1 references."""

from __future__ import annotations

import argparse
import contextlib
import io
import pathlib
import struct
import sys
import time
from dataclasses import dataclass

import numpy as np
import zmq
from scipy.spatial.transform import Rotation


JOINT_NAMES = (
    "Pelvis", "SpineNavel", "SpineChest", "Neck", "Head",
    "LeftShoulder", "LeftElbow", "LeftWrist",
    "RightShoulder", "RightElbow", "RightWrist",
    "LeftHip", "LeftKnee", "LeftAnkle", "LeftFoot",
    "RightHip", "RightKnee", "RightAnkle", "RightFoot",
)

GMR_TARGETS = {
    "Pelvis": "Pelvis",
    "Chest": "SpineChest",
    "Left_UpperLeg": "LeftHip",
    "Left_LowerLeg": "LeftKnee",
    "Left_Foot": "LeftFoot",
    "Right_UpperLeg": "RightHip",
    "Right_LowerLeg": "RightKnee",
    "Right_Foot": "RightFoot",
    "Left_UpperArm": "LeftShoulder",
    "Left_Forearm": "LeftElbow",
    "Left_Hand": "LeftWrist",
    "Right_UpperArm": "RightShoulder",
    "Right_Forearm": "RightElbow",
    "Right_Hand": "RightWrist",
}

G1_JOINT_NAMES = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)

SONIC_NEUTRAL = np.array((
    -0.312, 0, 0, 0.669, -0.363, 0, -0.312, 0, 0, 0.669, -0.363, 0,
    0, 0, 0, 0.2, 0.2, 0, 0.6, 0, 0, 0, 0.2, -0.2, 0, 0.6, 0, 0, 0,
))

# K4ABT: +X camera-right, +Y down, +Z away from camera. The intermediate
# right-handed MuJoCo frame is +X away, +Y camera-left, +Z up. Neutral
# calibration below then aligns the person's anatomical forward/left/up axes.
K4_TO_MUJOCO = np.array(((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0)))

REQUEST_HEADER = struct.Struct("<4sQI")
REQUEST_JOINT = struct.Struct("<7dB")
RESPONSE_HEADER = struct.Struct("<4sHBBQHHHd")
RESPONSE_VALUES = struct.Struct("<58d")


@dataclass
class Joint:
    position_mm: np.ndarray
    orientation_wxyz: np.ndarray
    confidence: int


@dataclass
class AdapterResult:
    human_frame: dict[str, tuple[np.ndarray, np.ndarray]] | None
    valid: int
    held: int
    stale: int
    calibrated: bool


class KinectToGMRAdapter:
    """Convert K4ABT joints to GMR's global metre + wxyz representation."""

    def __init__(self, calibration_frames: int = 15, hold_seconds: float = 0.15):
        self.calibration_frames = max(1, calibration_frames)
        self.hold_us = int(hold_seconds * 1_000_000)
        self.cache: dict[str, tuple[Joint, int]] = {}
        self.calibration_samples: list[dict[str, Joint]] = []
        self.neutral: dict[str, Joint] = {}
        self.world_basis: np.ndarray | None = None
        self.origin: np.ndarray | None = None
        self.actual_human_height: float | None = None

    @staticmethod
    def _finite(joint: Joint) -> bool:
        norm = float(np.dot(joint.orientation_wxyz, joint.orientation_wxyz))
        return (np.isfinite(joint.position_mm).all() and
                np.isfinite(joint.orientation_wxyz).all() and 0.25 < norm < 4.0)

    @staticmethod
    def _world_joint(joint: Joint) -> Joint:
        position = K4_TO_MUJOCO @ (joint.position_mm * 0.001)
        rotation = Rotation.from_quat(joint.orientation_wxyz, scalar_first=True).as_matrix()
        rotation = K4_TO_MUJOCO @ rotation @ K4_TO_MUJOCO.T
        quaternion = Rotation.from_matrix(rotation).as_quat(scalar_first=True)
        return Joint(position, quaternion, joint.confidence)

    def _finish_calibration(self) -> None:
        for name in JOINT_NAMES:
            positions = np.stack([frame[name].position_mm for frame in self.calibration_samples])
            rotations = Rotation.from_quat(
                np.stack([frame[name].orientation_wxyz for frame in self.calibration_samples]),
                scalar_first=True,
            )
            self.neutral[name] = Joint(positions.mean(axis=0), rotations.mean().as_quat(scalar_first=True), 3)

        pelvis = self.neutral["Pelvis"].position_mm
        left = self.neutral["LeftShoulder"].position_mm
        right = self.neutral["RightShoulder"].position_mm
        head = self.neutral["Head"].position_mm
        lateral = left - right
        lateral /= np.linalg.norm(lateral)
        up = head - pelvis
        up -= lateral * np.dot(up, lateral)
        up /= np.linalg.norm(up)
        forward = np.cross(lateral, up)
        forward /= np.linalg.norm(forward)
        up = np.cross(forward, lateral)
        self.world_basis = np.column_stack((forward, lateral, up))

        floor = min(
            self.neutral["LeftFoot"].position_mm[2], self.neutral["RightFoot"].position_mm[2],
            self.neutral["LeftAnkle"].position_mm[2], self.neutral["RightAnkle"].position_mm[2],
        )
        self.origin = np.array((pelvis[0], pelvis[1], floor))
        measured_height = float((self.world_basis.T @ (head - np.array((pelvis[0], pelvis[1], floor))))[2] + 0.18)
        self.actual_human_height = float(np.clip(measured_height, 1.3, 2.2))

    def update(self, joints: dict[str, Joint], timestamp_us: int) -> AdapterResult:
        selected: dict[str, Joint] = {}
        states: dict[str, str] = {}
        for name in JOINT_NAMES:
            incoming = joints[name]
            if incoming.confidence >= 1 and self._finite(incoming):
                world = self._world_joint(incoming)
                self.cache[name] = (world, timestamp_us)
                selected[name], states[name] = world, "fresh"
            elif name in self.cache and timestamp_us - self.cache[name][1] <= self.hold_us:
                selected[name], states[name] = self.cache[name][0], "held"
            elif name in self.neutral:
                selected[name], states[name] = self.neutral[name], "stale"
            else:
                states[name] = "invalid"

        if self.world_basis is None:
            if all(states[name] == "fresh" for name in JOINT_NAMES):
                self.calibration_samples.append({name: selected[name] for name in JOINT_NAMES})
                if len(self.calibration_samples) >= self.calibration_frames:
                    self._finish_calibration()
            return AdapterResult(None, 0, 0, len(GMR_TARGETS), self.world_basis is not None)

        human_frame: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        valid = held = stale = 0
        basis = self.world_basis
        for target, source in GMR_TARGETS.items():
            state = states[source]
            if state == "fresh": valid += 1
            elif state == "held": held += 1
            else: stale += 1
            joint = selected.get(source, self.neutral[source])
            pos = basis.T @ (joint.position_mm - self.origin)
            current = Rotation.from_quat(joint.orientation_wxyz, scalar_first=True).as_matrix()
            neutral = Rotation.from_quat(self.neutral[source].orientation_wxyz, scalar_first=True).as_matrix()
            delta = basis.T @ current @ neutral.T @ basis
            quat = Rotation.from_matrix(delta).as_quat(scalar_first=True)
            human_frame[target] = (pos, quat)
        return AdapterResult(human_frame, valid, held, stale, True)


class GMRPipeline:
    def __init__(self, gmr_root: pathlib.Path, calibration_frames: int = 15, smoothing: float = 0.45):
        sys.path.insert(0, str(gmr_root))
        import mujoco
        from general_motion_retargeting import GeneralMotionRetargeting

        self.mujoco = mujoco
        self.GMR = GeneralMotionRetargeting
        self.adapter = KinectToGMRAdapter(calibration_frames=calibration_frames)
        self.retargeter = None
        self.smoothing = float(np.clip(smoothing, 0.0, 1.0))
        self.previous_qpos: np.ndarray | None = None
        self.previous_timestamp = 0
        self.qpos_addresses: list[int] = []
        self.lower = self.upper = None
        self.reference_quats: dict[str, np.ndarray] = {}

    def _start_gmr(self) -> None:
        with contextlib.redirect_stdout(io.StringIO()):
            self.retargeter = self.GMR(
                src_human="xsens_mvn", tgt_robot="unitree_g1",
                actual_human_height=self.adapter.actual_human_height,
                solver="daqp", damping=1.0, verbose=False, use_velocity_limit=True,
            )
        model = self.retargeter.model
        names = []
        self.qpos_addresses = []
        limits = []
        for expected in G1_JOINT_NAMES:
            joint_id = self.mujoco.mj_name2id(model, self.mujoco.mjtObj.mjOBJ_JOINT, expected)
            if joint_id < 0:
                raise RuntimeError(f"GMR model is missing {expected}")
            names.append(self.mujoco.mj_id2name(model, self.mujoco.mjtObj.mjOBJ_JOINT, joint_id))
            self.qpos_addresses.append(int(model.jnt_qposadr[joint_id]))
            limits.append(model.jnt_range[joint_id].copy())
        if tuple(names) != G1_JOINT_NAMES:
            raise RuntimeError("GMR G1 order does not match SONIC's 29-DoF order")
        limits = np.asarray(limits)
        self.lower, self.upper = limits[:, 0], limits[:, 1]

        qpos = model.qpos0.copy()
        qpos[:7] = (0.0, 0.0, 0.793, 1.0, 0.0, 0.0, 0.0)
        for index, (address, value) in enumerate(zip(self.qpos_addresses, SONIC_NEUTRAL)):
            qpos[address] = np.clip(value, self.lower[index], self.upper[index])
        self.retargeter.configuration.update(qpos)

        data = self.retargeter.configuration.data
        for robot_body, entry in self.retargeter.ik_match_table1.items():
            human_body = entry[0]
            body_id = self.mujoco.mj_name2id(model, self.mujoco.mjtObj.mjOBJ_BODY, robot_body)
            robot_rotation = Rotation.from_quat(data.xquat[body_id], scalar_first=True)
            self.reference_quats[human_body] = (
                robot_rotation * self.retargeter.rot_offsets1[human_body].inv()
            ).as_quat(scalar_first=True)

    def update(self, joints: dict[str, Joint], timestamp_us: int):
        adapted = self.adapter.update(joints, timestamp_us)
        if adapted.human_frame is None:
            return None, adapted, 0.0
        if self.retargeter is None:
            self._start_gmr()

        for body_name, (position, delta_quat) in adapted.human_frame.items():
            orientation = Rotation.from_quat(delta_quat, scalar_first=True)
            reference = Rotation.from_quat(self.reference_quats[body_name], scalar_first=True)
            adapted.human_frame[body_name] = (position, (orientation * reference).as_quat(scalar_first=True))

        started = time.perf_counter()
        solved = self.retargeter.retarget(adapted.human_frame, offset_to_ground=False)
        solve_ms = (time.perf_counter() - started) * 1000.0
        raw = np.asarray([solved[address] for address in self.qpos_addresses])
        if not np.isfinite(raw).all():
            return None, adapted, solve_ms
        raw = np.clip(raw, self.lower, self.upper)

        if self.previous_qpos is None:
            filtered = raw
            velocity = np.zeros(29)
        else:
            delta = np.arctan2(np.sin(raw - self.previous_qpos), np.cos(raw - self.previous_qpos))
            delta[np.abs(delta) > 1.2] = 0.0
            filtered = np.clip(self.previous_qpos + self.smoothing * delta, self.lower, self.upper)
            dt = (timestamp_us - self.previous_timestamp) * 1e-6
            velocity = np.clip((filtered - self.previous_qpos) / dt, -12.0, 12.0) if 0.001 <= dt <= 0.25 else np.zeros(29)

        self.previous_qpos = filtered
        self.previous_timestamp = timestamp_us
        return (filtered, velocity), adapted, solve_ms


def decode_request(message: bytes):
    expected = REQUEST_HEADER.size + len(JOINT_NAMES) * REQUEST_JOINT.size
    if len(message) != expected:
        raise ValueError(f"bad request size {len(message)}, expected {expected}")
    magic, timestamp_us, body_id = REQUEST_HEADER.unpack_from(message)
    if magic != b"KGM1":
        raise ValueError("bad request magic")
    joints = {}
    offset = REQUEST_HEADER.size
    for name in JOINT_NAMES:
        values = REQUEST_JOINT.unpack_from(message, offset)
        joints[name] = Joint(np.asarray(values[:3]), np.asarray(values[3:7]), values[7])
        offset += REQUEST_JOINT.size
    return timestamp_us, body_id, joints


def response(status: int, timestamp_us: int, adapted: AdapterResult | None = None,
             solve_ms: float = 0.0, values=None) -> bytes:
    valid = adapted.valid if adapted else 0
    held = adapted.held if adapted else 0
    stale = adapted.stale if adapted else len(GMR_TARGETS)
    header = RESPONSE_HEADER.pack(b"GMR1", 1, status, 0, timestamp_us, valid, held, stale, solve_ms)
    if values is None:
        values = (np.zeros(29), np.zeros(29))
    return header + RESPONSE_VALUES.pack(*values[0], *values[1])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=5558)
    parser.add_argument("--gmr-root", type=pathlib.Path,
                        default=pathlib.Path(__file__).resolve().parents[2] / "GMR")
    parser.add_argument("--calibration-frames", type=int, default=15)
    parser.add_argument("--smoothing", type=float, default=0.45)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be 1..65535")
    if not (args.gmr_root / "general_motion_retargeting").is_dir():
        parser.error(f"GMR checkout not found: {args.gmr_root}")

    pipeline = GMRPipeline(args.gmr_root, args.calibration_frames, args.smoothing)
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.setsockopt(zmq.LINGER, 0)
    socket.bind(f"tcp://127.0.0.1:{args.port}")
    print(f"gmr_bridge=ready source={args.gmr_root} commit=bb1bbe40774794fceb2a7c579a3464a28e68c844 port={args.port}", flush=True)
    last_verbose = 0.0
    try:
        while True:
            message = socket.recv()
            try:
                timestamp_us, _, joints = decode_request(message)
                values, adapted, solve_ms = pipeline.update(joints, timestamp_us)
                socket.send(response(2 if values is not None else 1, timestamp_us, adapted, solve_ms, values))
                if args.verbose and values is not None and time.monotonic() - last_verbose >= 1.0:
                    for body_name, (human_pos, human_quat) in adapted.human_frame.items():
                        gmr_pos, gmr_quat = pipeline.retargeter.scaled_human_data[body_name]
                        print(f"target={body_name} human_pos={human_pos.tolist()} human_wxyz={human_quat.tolist()} "
                              f"gmr_pos={gmr_pos.tolist()} gmr_wxyz={gmr_quat.tolist()}")
                    last_verbose = time.monotonic()
            except Exception as error:
                print(f"gmr_error={error}", file=sys.stderr, flush=True)
                socket.send(response(0, 0))
    except KeyboardInterrupt:
        return 0
    finally:
        socket.close()
        context.term()


if __name__ == "__main__":
    raise SystemExit(main())
