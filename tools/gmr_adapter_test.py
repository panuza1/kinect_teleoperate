#!/usr/bin/env python3
"""Hardware-free semantic checks for KinectToGMRAdapter and official GMR."""

import pathlib
import sys

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from kinect_gmr_bridge import (  # noqa: E402
    G1_JOINT_NAMES, JOINT_NAMES, K4_TO_MUJOCO, GMRPipeline, Joint,
    KinectToGMRAdapter, SONIC_NEUTRAL,
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
    def __init__(self, gmr_root, smoothing=0.45):
        self.pipeline = GMRPipeline(gmr_root, calibration_frames=1, smoothing=smoothing)
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


def lower_body_abduction_diagnostic(gmr_root):
    """Position-only leg abduction isolates GMR IK coupling from Kinect transforms."""
    import contextlib
    import io
    import mujoco
    from scipy.spatial.transform import Rotation

    pipeline = GMRPipeline(gmr_root, calibration_frames=1, smoothing=0.20, profile="kinect_g1")
    timestamp = 1_000_000
    with contextlib.redirect_stdout(io.StringIO()):
        pipeline.update(frame(), timestamp)  # neutral calibration

    model = mujoco.MjModel.from_xml_path(
        str(pathlib.Path(__file__).resolve().parents[1] /
            "../GR00T-WholeBodyControl/gear_sonic_deploy/g1/g1_29dof.xml")
    )
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    base_qpos = data.qpos[:7].copy()
    addresses = [model.jnt_qposadr[mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, name)] for name in G1_JOINT_NAMES]
    left_roll = INDEX["left_hip_roll_joint"]
    right_roll = INDEX["right_hip_roll_joint"]

    def sample(side=None):
        positions = dict(POSE)
        if side:
            sign = 1 if side == "Left" else -1
            hip = np.asarray(POSE[f"{side}Hip"], dtype=float)
            abduct = Rotation.from_rotvec([np.deg2rad(25) * sign, 0, 0])
            for joint in ("Knee", "Ankle", "Foot"):
                name = f"{side}{joint}"
                positions[name] = tuple(hip + abduct.apply(np.asarray(POSE[name]) - hip))
        return frame(positions=positions), positions

    def run_phase(label, side, count):
        nonlocal timestamp
        result = None
        for frame_number in range(count):
            source, positions = sample(side)
            timestamp += 50_000
            with contextlib.redirect_stdout(io.StringIO()):
                result, _, _ = pipeline.update(source, timestamp)
            assert result is not None
            qpos = result[0]
            data.qpos[:7] = base_qpos  # fixed-base MuJoCo application path
            for index, address in enumerate(addresses):
                data.qpos[address] = qpos[index]
            mujoco.mj_forward(model, data)
            targets = pipeline.retargeter.scaled_human_data
            print(
                f"phase={label} frame={frame_number:02d} "
                f"Kinect_LeftHip={source['LeftHip'].position_mm.tolist()} "
                f"Kinect_RightHip={source['RightHip'].position_mm.tolist()} "
                f"Kinect_LeftKnee={source['LeftKnee'].position_mm.tolist()} "
                f"Kinect_RightKnee={source['RightKnee'].position_mm.tolist()} "
                f"GMR_Left_UpperLeg={targets['Left_UpperLeg'][0].tolist()} "
                f"GMR_Right_UpperLeg={targets['Right_UpperLeg'][0].tolist()} "
                f"left_hip_roll_joint={data.qpos[addresses[left_roll]]:.6f} "
                f"right_hip_roll_joint={data.qpos[addresses[right_roll]]:.6f}",
                flush=True,
            )
        return (np.asarray([data.qpos[address] for address in addresses]),
                {name: value[0].copy() for name, value in pipeline.retargeter.scaled_human_data.items()})

    neutral_before, neutral_targets = run_phase("neutral", None, 40)
    left, left_targets = run_phase("left_leg_abduction", "Left", 30)
    neutral_between, neutral_between_targets = run_phase("neutral", None, 60)
    right, right_targets = run_phase("right_leg_abduction", "Right", 30)
    left_move = abs(left[left_roll] - neutral_before[left_roll])
    left_opposite = abs(left[right_roll] - neutral_before[right_roll])
    right_move = abs(right[right_roll] - neutral_between[right_roll])
    right_opposite = abs(right[left_roll] - neutral_between[left_roll])
    assert np.linalg.norm(left_targets["Left_LowerLeg"] - neutral_targets["Left_LowerLeg"]) > 0.10
    assert np.linalg.norm(left_targets["Right_LowerLeg"] - neutral_targets["Right_LowerLeg"]) < 1e-6
    assert np.linalg.norm(right_targets["Right_LowerLeg"] - neutral_between_targets["Right_LowerLeg"]) > 0.10
    assert np.linalg.norm(right_targets["Left_LowerLeg"] - neutral_between_targets["Left_LowerLeg"]) < 1e-6
    assert left_move > 0.15 and left_opposite <= 0.25 * left_move, (left_move, left_opposite)
    assert right_move > 0.15 and right_opposite <= 0.25 * right_move, (right_move, right_opposite)


def realistic_abduction_regression(gmr_root):
    """Leg rotations present: ensure pelvis weighting preserves ipsilateral response."""
    scenario = Scenario(gmr_root, smoothing=0.20)
    for side, moving, opposite in (
        ("Left", "left_hip_roll_joint", "right_hip_roll_joint"),
        ("Right", "right_hip_roll_joint", "left_hip_roll_joint"),
    ):
        angle = np.deg2rad(25) * (1 if side == "Left" else -1)
        rotations = {f"{side}{joint}": camera_quaternion(0, angle)
                     for joint in ("Hip", "Knee", "Ankle", "Foot")}
        positions = dict(POSE)
        hip = np.asarray(POSE[f"{side}Hip"], dtype=float)
        abduct = Rotation.from_rotvec([angle, 0, 0])
        for joint in ("Knee", "Ankle", "Foot"):
            name = f"{side}{joint}"
            positions[name] = tuple(hip + abduct.apply(np.asarray(POSE[name]) - hip))
        delta, _ = scenario.delta(rotations, positions)
        assert abs(delta[INDEX[moving]]) > 0.15
        assert abs(delta[INDEX[opposite]]) <= 0.25 * abs(delta[INDEX[moving]])


def solver_recovery_regression(gmr_root):
    """A corrupt solve rolls back, yields briefly, then permits stale safety."""
    pipeline = GMRPipeline(gmr_root, calibration_frames=1, smoothing=0.20, profile="kinect_g1")
    timestamp = 1_000_000
    pipeline.update(frame(), timestamp)  # neutral calibration

    def next_frame():
        nonlocal timestamp
        timestamp += 50_000
        return pipeline.update(frame(), timestamp)

    first, _, _ = next_frame()
    second, _, _ = next_frame()
    assert first is not None and second is not None
    last_good_full = pipeline.last_good_full_qpos.copy()
    last_good_filtered = pipeline.last_good_filtered_qpos.copy()
    sequence = 98
    sequence += 2  # valid frames 99 and 100

    original_retarget = pipeline.retargeter.retarget

    def nonfinite_once(*args, **kwargs):
        solved = original_retarget(*args, **kwargs)
        solved[0] = np.nan
        pipeline.retargeter.configuration.data.qpos[0] = np.nan
        return solved

    pipeline.retargeter.retarget = nonfinite_once
    held, _, _ = next_frame()
    sequence += held is not None
    assert sequence == 101 and held is not None
    np.testing.assert_array_equal(held[0], last_good_filtered)
    np.testing.assert_array_equal(held[1], np.zeros(29))
    np.testing.assert_allclose(pipeline.retargeter.configuration.data.qpos, last_good_full)
    assert pipeline.solve_failures == 1 and pipeline.recoveries >= 1

    pipeline.retargeter.retarget = original_retarget
    recovered, _, _ = next_frame()
    assert recovered is not None
    sequence += 1
    assert sequence == 102

    # Keep injecting the same corruption through the one permitted reinit.
    real_start = pipeline._start_gmr
    reinitializations = 0

    def restart_with_failure(initial_qpos=None):
        nonlocal reinitializations
        real_start(initial_qpos)
        reinitializations += 1
        inject_failure()

    def inject_failure():
        current_retarget = pipeline.retargeter.retarget

        def always_nonfinite(*args, **kwargs):
            solved = current_retarget(*args, **kwargs)
            solved[0] = np.nan
            pipeline.retargeter.configuration.data.qpos[0] = np.nan
            return solved

        pipeline.retargeter.retarget = always_nonfinite

    pipeline._start_gmr = restart_with_failure
    inject_failure()
    previous_sequence = sequence
    for _ in range(8):
        held, _, _ = next_frame()
        if held is not None:
            sequence += 1
    assert reinitializations == 1
    assert sequence > previous_sequence
    assert pipeline.consecutive_failures >= 8
    assert pipeline.fallback_frames >= 7
    assert held is None  # no reference beyond the 300 ms hold window


def confidence_hold_regression():
    adapter = KinectToGMRAdapter(calibration_frames=1)
    timestamp = 1_000_000
    assert adapter.update(frame(), timestamp).calibrated
    timestamp += 50_000
    fresh = adapter.update(frame(), timestamp)
    assert fresh.valid == 14 and fresh.held == fresh.stale == 0

    names = ("SpineChest", "Neck", "Head", "LeftShoulder", "LeftElbow", "LeftWrist",
             "RightShoulder", "RightElbow", "RightWrist")
    moved = {name: tuple(np.asarray(POSE[name]) + np.array((0.25, 0.1, -0.15))) for name in names}
    moved["RightHip"] = (0.0, -0.25, 1.0)
    timestamp += 50_000
    low = adapter.update(frame(positions=moved, confidence={name: 1 for name in names}), timestamp)
    assert low.valid == 7 and low.low == 7 and low.held == 7
    for target in ("Chest", "Left_UpperArm", "Left_Forearm", "Left_Hand",
                   "Right_UpperArm", "Right_Forearm", "Right_Hand"):
        np.testing.assert_allclose(low.human_frame[target][0], fresh.human_frame[target][0])
    assert np.linalg.norm(low.human_frame["Right_UpperLeg"][0] - fresh.human_frame["Right_UpperLeg"][0]) > 0.05

    timestamp += 300_001
    stale = adapter.update(frame(positions=moved, confidence={name: 0 for name in names}), timestamp)
    assert stale.stale == 7 and stale.valid == 7
    for target in ("Chest", "Left_UpperArm", "Left_Forearm", "Left_Hand",
                   "Right_UpperArm", "Right_Forearm", "Right_Hand"):
        np.testing.assert_allclose(stale.human_frame[target][0], fresh.human_frame[target][0])


def confidence_ab_regression(gmr_root):
    """Compare LOW-as-fresh against LOW-held during scripted arm motion/occlusion."""
    arm = [INDEX[name] for name in (
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
        "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    )]
    phases = [
        (dict(POSE), {}),
        ({**POSE, "LeftElbow": (.3, .25, 1.35), "LeftWrist": (.6, .25, 1.35)}, {}),
        ({**POSE, "LeftElbow": (0, .6, 1.35), "LeftWrist": (0, .9, 1.35),
          "RightElbow": (0, -.6, 1.35), "RightWrist": (0, -.9, 1.35)}, {}),
        ({**POSE, "LeftElbow": (.25, .25, 1.0), "LeftWrist": (.1, .25, .75)}, {}),
    ]
    occluded = {"LeftShoulder": (.4, .6, 1.0), "LeftElbow": (-.4, .8, .7), "LeftWrist": (.5, -.5, .5)}

    def run(low_is_fresh):
        pipeline = GMRPipeline(gmr_root, calibration_frames=1, smoothing=0.20, profile="kinect_g1")
        timestamp = 1_000_000
        pipeline.update(frame(), timestamp)
        qpos = None
        for positions, _ in phases:
            for _ in range(12):
                timestamp += 50_000
                qpos = pipeline.update(frame(positions=positions), timestamp)[0][0]
        before = qpos.copy()
        deltas, limits = [], 0
        shoulder_jitter = wrist_jitter = 0.0
        previous = qpos.copy()
        confidence = {name: (2 if low_is_fresh else 1) for name in occluded}
        for _ in range(6):
            timestamp += 50_000
            qpos = pipeline.update(frame(positions=occluded, confidence=confidence), timestamp)[0][0]
            deltas.append(float(np.max(np.abs(qpos - previous))))
            shoulder_jitter = max(shoulder_jitter, float(np.max(np.abs(qpos[arm[:3]] - before[arm[:3]]))))
            wrist_jitter = max(wrist_jitter, float(np.max(np.abs(qpos[arm[4:]] - before[arm[4:]]))))
            limits += int(np.count_nonzero((qpos <= pipeline.lower + 1e-4) | (qpos >= pipeline.upper - 1e-4)))
            previous = qpos.copy()
        timestamp += 100_000  # no body for 100 ms; the live path holds its last reference
        timestamp += 50_000   # body reacquired
        recovered = pipeline.update(frame(), timestamp)[0][0]
        recovery_delta = float(np.max(np.abs(recovered - previous)))
        arm_jitter = float(np.max(np.abs(qpos[arm] - before[arm])))
        return shoulder_jitter, wrist_jitter, arm_jitter, max(deltas), limits, recovery_delta

    old = run(low_is_fresh=True)
    held = run(low_is_fresh=False)
    assert old[2] > 0.03, old
    assert all(held[index] < 0.65 * old[index] for index in (0, 1, 2, 3)), (old, held)
    assert held[5] < 0.20, (old, held)
    print(f"confidence_ab old_shoulder_jitter={old[0]:.4f} old_wrist_jitter={old[1]:.4f} "
          f"old_arm_delta={old[2]:.4f} old_max_qpos_frame_delta={old[3]:.4f} "
          f"old_joint_limit_hits={old[4]} old_recovery_delta={old[5]:.4f} "
          f"new_shoulder_jitter={held[0]:.4f} "
          f"new_wrist_jitter={held[1]:.4f} new_arm_delta={held[2]:.4f} "
          f"new_max_qpos_frame_delta={held[3]:.4f} new_joint_limit_hits={held[4]} "
          f"new_recovery_delta={held[5]:.4f} dropout_count=1", flush=True)


def main():
    gmr_root = pathlib.Path(__file__).resolve().parents[2] / "GMR"
    if sys.argv[1:] == ["--lower-body-only"]:
        lower_body_abduction_diagnostic(gmr_root)
        realistic_abduction_regression(gmr_root)
        return
    scenario = Scenario(gmr_root)
    for index, table in enumerate((scenario.pipeline.retargeter.ik_match_table1,
                                   scenario.pipeline.retargeter.ik_match_table2), start=1):
        weights = {entry[0]: entry[1:3] for entry in table.values()}
        assert weights["Left_UpperArm"][1] == weights["Right_UpperArm"][1] == 2
        assert weights["Left_Forearm"][1] == weights["Right_Forearm"][1] == 1
        assert weights["Left_Hand"][1] == weights["Right_Hand"][1] == (0 if index == 1 else 2)
    elbow_indices = [INDEX["left_elbow_joint"], INDEX["right_elbow_joint"]]
    non_elbows = np.ones(29, dtype=bool)
    non_elbows[elbow_indices] = False
    assert np.max(np.abs(scenario.neutral[non_elbows] - SONIC_NEUTRAL[non_elbows])) < 0.35
    assert np.all((scenario.neutral[elbow_indices] > .5) & (scenario.neutral[elbow_indices] < 1.6))
    assert np.count_nonzero(np.isclose(scenario.neutral, scenario.pipeline.lower, atol=1e-4)) == 0
    assert np.count_nonzero(np.isclose(scenario.neutral, scenario.pipeline.upper, atol=1e-4)) == 0

    left_forward = dict(POSE)
    left_forward.update({"LeftElbow": (.3, .25, 1.35), "LeftWrist": (.6, .25, 1.35)})
    delta, _ = scenario.delta({}, left_forward)
    assert abs(delta[INDEX["left_shoulder_pitch_joint"]]) > .2
    assert abs(delta[INDEX["left_shoulder_pitch_joint"]]) > abs(delta[INDEX["right_shoulder_pitch_joint"]])

    right_forward = dict(POSE)
    right_forward.update({"RightElbow": (.3, -.25, 1.35), "RightWrist": (.6, -.25, 1.35)})
    delta, _ = scenario.delta({}, right_forward)
    assert abs(delta[INDEX["right_shoulder_pitch_joint"]]) > .2
    assert abs(delta[INDEX["right_shoulder_pitch_joint"]]) > abs(delta[INDEX["left_shoulder_pitch_joint"]])

    side = dict(POSE)
    side.update({"LeftElbow": (0, .5, 1.35), "LeftWrist": (0, .8, 1.35),
                 "RightElbow": (0, -.5, 1.35), "RightWrist": (0, -.8, 1.35)})
    delta, _ = scenario.delta({}, side)
    assert delta[INDEX["left_shoulder_roll_joint"]] > .2
    assert delta[INDEX["right_shoulder_roll_joint"]] < -.2

    qy = camera_quaternion(1, .7)
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
    scenario.timestamp += 300_000
    result, adapted, _ = scenario.pipeline.update(frame(confidence={"LeftFoot": 0}), scenario.timestamp)
    assert result is not None and adapted.stale == 1

    # Raw wrist quaternion spikes must not move G1 wrists or leak into shoulders.
    baseline = Scenario(gmr_root)
    spiked = Scenario(gmr_root)
    baseline_qpos = baseline.run(frame(), 12)[0]
    wrist_spike = camera_quaternion(0, np.pi)
    spiked_qpos = spiked.run(frame(rotations={
        "LeftWrist": wrist_spike, "RightWrist": wrist_spike,
    }), 12)[0]
    affected = [INDEX[f"{side}_{joint}_{axis}_joint"]
                for side in ("left", "right")
                for joint, axes in (("shoulder", ("pitch", "roll", "yaw")),
                                    ("wrist", ("roll", "pitch", "yaw")))
                for axis in axes]
    assert np.max(np.abs(spiked_qpos[affected] - baseline_qpos[affected])) < 1e-6

    lower_body_abduction_diagnostic(gmr_root)
    realistic_abduction_regression(gmr_root)
    solver_recovery_regression(gmr_root)
    confidence_hold_regression()
    confidence_ab_regression(gmr_root)

    print("GMR Kinect adapter semantic tests passed")


if __name__ == "__main__":
    main()
