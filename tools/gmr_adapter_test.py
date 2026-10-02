#!/usr/bin/env python3
"""Hardware-free semantic checks for KinectToGMRAdapter and official GMR."""

import pathlib
import sys

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from kinect_gmr_bridge import (  # noqa: E402
    G1_JOINT_NAMES, JOINT_NAMES, K4_TO_MUJOCO, GMRPipeline, Joint, SONIC_NEUTRAL,
)


POSE = {
    "Pelvis": (0, 0, 1), "SpineNavel": (0, 0, 1.15), "SpineChest": (0, 0, 1.35),
    "Neck": (0, 0, 1.5), "Head": (0, 0, 1.65),
    "LeftShoulder": (0, .25, 1.35), "LeftElbow": (0, .25, .95), "LeftWrist": (0, .25, .55),
    "RightShoulder": (0, -.25, 1.35), "RightElbow": (0, -.25, .95), "RightWrist": (0, -.25, .55),
    "LeftHip": (0, .12, 1), "LeftKnee": (0, .12, .55), "LeftAnkle": (0, .12, .1), "LeftFoot": (.2, .12, .05),
    "RightHip": (0, -.12, 1), "RightKnee": (0, -.12, .55), "RightAnkle": (0, -.12, .1), "RightFoot": (.2, -.12, .05),
}
INDEX = {name: index for index, name in enumerate(G1_JOINT_NAMES)}


def camera_quaternion(axis: int, angle: float):
    desired = Rotation.from_rotvec(np.eye(3)[axis] * angle).as_matrix()
    return Rotation.from_matrix(K4_TO_MUJOCO.T @ desired @ K4_TO_MUJOCO).as_quat(scalar_first=True)


def frame(rotations=None, positions=None, confidence=None):
    rotations, positions, confidence = rotations or {}, positions or {}, confidence or {}
    return {
        name: Joint(
            K4_TO_MUJOCO.T @ np.asarray(positions.get(name, POSE[name])) * 1000.0,
            np.asarray(rotations.get(name, (1.0, 0.0, 0.0, 0.0))),
            confidence.get(name, 2),
        ) for name in JOINT_NAMES
    }


class Scenario:
    def __init__(self, gmr_root):
        self.pipeline = GMRPipeline(gmr_root, calibration_frames=1, smoothing=0.45)
        self.timestamp = 1_000_000
        output, adapted, _ = self.pipeline.update(frame(), self.timestamp)
        assert output is None and adapted.calibrated
        self.neutral = self.run(frame(), 16)[0]

    def run(self, sample, count=12):
        result = None
        for _ in range(count):
            self.timestamp += 33_333
            result, adapted, solve_ms = self.pipeline.update(sample, self.timestamp)
            assert result is not None and solve_ms > 0
            qpos, qvel = result
            assert qpos.shape == (29,) and qvel.shape == (29,)
            assert np.isfinite(qpos).all() and np.isfinite(qvel).all()
            assert (qpos >= self.pipeline.lower - 1e-9).all()
            assert (qpos <= self.pipeline.upper + 1e-9).all()
            assert np.max(np.abs(qvel)) <= 12.0 + 1e-9
        return result

    def delta(self, rotations, positions=None):
        qpos, qvel = self.run(frame(rotations, positions))
        delta = qpos - self.neutral
        self.run(frame(), 14)
        return delta, qvel


def main():
    gmr_root = pathlib.Path(__file__).resolve().parents[2] / "GMR"
    scenario = Scenario(gmr_root)
    assert np.max(np.abs(scenario.neutral - SONIC_NEUTRAL)) < 0.35
    assert np.count_nonzero(np.isclose(scenario.neutral, scenario.pipeline.lower, atol=1e-4)) == 0
    assert np.count_nonzero(np.isclose(scenario.neutral, scenario.pipeline.upper, atol=1e-4)) == 0

    qy = camera_quaternion(1, .7)
    delta, _ = scenario.delta({name: qy for name in ("LeftShoulder", "LeftElbow", "LeftWrist")})
    assert delta[INDEX["left_shoulder_pitch_joint"]] > .2
    assert abs(delta[INDEX["left_shoulder_pitch_joint"]]) > abs(delta[INDEX["right_shoulder_pitch_joint"]])

    delta, _ = scenario.delta({name: qy for name in ("RightShoulder", "RightElbow", "RightWrist")})
    assert delta[INDEX["right_shoulder_pitch_joint"]] > .2
    assert abs(delta[INDEX["right_shoulder_pitch_joint"]]) > abs(delta[INDEX["left_shoulder_pitch_joint"]])

    left_side, right_side = camera_quaternion(0, .7), camera_quaternion(0, -.7)
    delta, _ = scenario.delta({name: left_side for name in ("LeftShoulder", "LeftElbow", "LeftWrist")} |
                              {name: right_side for name in ("RightShoulder", "RightElbow", "RightWrist")})
    assert delta[INDEX["left_shoulder_roll_joint"]] > .2
    assert delta[INDEX["right_shoulder_roll_joint"]] < -.2

    delta, _ = scenario.delta({name: qy for name in ("LeftElbow", "LeftWrist")})
    assert delta[INDEX["left_elbow_joint"]] > .25
    delta, _ = scenario.delta({name: qy for name in ("RightElbow", "RightWrist")})
    assert delta[INDEX["right_elbow_joint"]] > .25

    delta, _ = scenario.delta({"SpineChest": camera_quaternion(2, .5)})
    assert delta[INDEX["waist_yaw_joint"]] > .25

    delta, _ = scenario.delta({name: camera_quaternion(1, .6) for name in ("LeftHip", "LeftKnee", "LeftAnkle", "LeftFoot")})
    assert delta[INDEX["left_hip_pitch_joint"]] > .12
    assert delta[INDEX["left_hip_pitch_joint"]] > delta[INDEX["right_hip_pitch_joint"]]
    delta, _ = scenario.delta({name: camera_quaternion(1, .6) for name in ("RightHip", "RightKnee", "RightAnkle", "RightFoot")})
    assert delta[INDEX["right_hip_pitch_joint"]] > .12
    assert delta[INDEX["right_hip_pitch_joint"]] > delta[INDEX["left_hip_pitch_joint"]]

    delta, _ = scenario.delta({name: camera_quaternion(0, .5) for name in ("LeftHip", "LeftKnee", "LeftAnkle", "LeftFoot")})
    assert delta[INDEX["left_hip_roll_joint"]] > .08
    delta, _ = scenario.delta({name: camera_quaternion(1, .7) for name in ("LeftKnee", "LeftAnkle", "LeftFoot")})
    assert delta[INDEX["left_knee_joint"]] > .25

    # A single-leg reference combines thigh flexion with a raised knee/foot.
    raised = dict(POSE)
    raised.update({"LeftKnee": (.25, .12, .75), "LeftAnkle": (.35, .12, .45), "LeftFoot": (.5, .12, .4)})
    delta, qvel = scenario.delta(
        {name: camera_quaternion(1, .6) for name in ("LeftHip", "LeftKnee", "LeftAnkle", "LeftFoot")},
        raised,
    )
    assert delta[INDEX["left_knee_joint"]] > .3 and np.isfinite(qvel).all()

    # LOW-confidence loss holds only the affected target, then falls back to neutral.
    scenario.timestamp += 33_333
    result, adapted, _ = scenario.pipeline.update(frame(confidence={"LeftFoot": 0}), scenario.timestamp)
    assert result is not None and adapted.held == 1 and adapted.stale == 0
    scenario.timestamp += 200_000
    result, adapted, _ = scenario.pipeline.update(frame(confidence={"LeftFoot": 0}), scenario.timestamp)
    assert result is not None and adapted.stale == 1

    print("GMR Kinect adapter semantic tests passed")


if __name__ == "__main__":
    main()
