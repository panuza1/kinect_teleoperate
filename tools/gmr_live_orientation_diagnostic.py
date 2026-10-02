#!/usr/bin/env python3
"""Replay captured live Kinect targets through GMR A/B/C profiles."""

import ast
import contextlib
import copy
import io
import pathlib
import re

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from kinect_gmr_bridge import G1_JOINT_NAMES, GMRPipeline, SONIC_NEUTRAL


ROOT = pathlib.Path(__file__).resolve().parents[1]
GMR_LOG = ROOT / "artifacts/live_debug/gmr_bridge.log"
KINECT_LOG = ROOT / "artifacts/live_debug/kinect_fixed.log"
GMR_ROOT = ROOT.parent / "GMR"
# ponytail: the fixed prefix keeps this live-log repro deterministic; switch to a
# labelled trace only when more pose classes need to become regression fixtures.
MAX_BLOCKS = 300
LIMBS = ("Left_UpperArm", "Left_Forearm", "Left_Hand",
         "Right_UpperArm", "Right_Forearm", "Right_Hand")
POSES = {
    "neutral": 27,
    "T_pose": 225,
    "left_side": 156,
    "right_side": 179,
    "left_elbow": 125,
    "right_elbow": 291,
    "left_forward": 258,
    "right_forward": 254,
    "return_neutral": 250,
}


def gmr_blocks():
    pattern = re.compile(
        r"^target=(\S+) human_pos=(\[[^]]+\]) human_wxyz=(\[[^]]+\]) "
        r"gmr_pos=(\[[^]]+\]) gmr_wxyz=(\[[^]]+\])"
    )
    blocks, block = [], {}
    for line in GMR_LOG.open(errors="replace"):
        match = pattern.match(line)
        if not match:
            continue
        name = match.group(1)
        if name == "Pelvis" and block:
            if len(block) == 14:
                blocks.append(block)
                if len(blocks) == MAX_BLOCKS:
                    return blocks
            block = {}
        block[name] = tuple(np.asarray(ast.literal_eval(value)) for value in match.groups()[1:])
    if len(block) == 14:
        blocks.append(block)
    return blocks[:MAX_BLOCKS]


def raw_blocks():
    pattern = re.compile(
        r"^joint=(\d+) confidence=(\d+) held=\d+ stale=\d+ "
        r"orientation_wxyz=([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+)"
    )
    blocks, block = [], {}
    for line in KINECT_LOG.open(errors="replace"):
        match = pattern.match(line)
        if not match:
            continue
        index = int(match.group(1))
        if index == 0 and block:
            if len(block) == 19 and min(value[0] for value in block.values()) >= 1:
                blocks.append(block)
                if len(blocks) == MAX_BLOCKS:
                    return blocks
            block = {}
        block[index] = (int(match.group(2)), np.asarray(match.groups()[2:], dtype=float))
    return blocks[:MAX_BLOCKS]


def unit(vector):
    return vector / np.linalg.norm(vector)


def make_pipeline(profile="kinect_g1", position_only=False):
    pipeline = GMRPipeline(GMR_ROOT, calibration_frames=1, smoothing=1.0, profile=profile)
    pipeline.adapter.actual_human_height = 1.7
    with contextlib.redirect_stdout(io.StringIO()):
        pipeline._start_gmr()
    if position_only:
        for table in (1, 2):
            tasks = getattr(pipeline.retargeter, f"human_body_to_task{table}")
            for name in LIMBS:
                tasks[name].cost[3:] = 0.0
    return pipeline


def solve(pipeline, frame, iterations=12):
    frame = copy.deepcopy(frame)
    if pipeline.gmr_source == "kinect":
        for name in ("Left_Hand", "Right_Hand"):
            frame[name] = (frame[name][0], pipeline.reference_quats[name])
    for _ in range(iterations):
        solved = pipeline.retargeter.retarget(copy.deepcopy(frame), offset_to_ground=False)
    qpos = np.asarray([solved[address] for address in pipeline.qpos_addresses])
    errors = []
    for table in (1, 2):
        match_table = getattr(pipeline.retargeter, f"ik_match_table{table}")
        for human_name in LIMBS:
            robot_name = next(name for name, entry in match_table.items() if entry[0] == human_name)
            body_id = mujoco.mj_name2id(
                pipeline.retargeter.model, mujoco.mjtObj.mjOBJ_BODY, robot_name
            )
            target = pipeline.retargeter.scaled_human_data[human_name][0]
            errors.append(np.linalg.norm(pipeline.retargeter.configuration.data.xpos[body_id] - target))
    return qpos, float(np.sqrt(np.mean(np.square(errors))))


def metrics(profile, position_only, frames, iterations, fresh=False):
    pipeline = make_pipeline(profile, position_only)
    joint_index = {name: index for index, name in enumerate(G1_JOINT_NAMES)}
    shoulders = [joint_index[f"{side}_shoulder_{axis}_joint"]
                 for side in ("left", "right") for axis in ("pitch", "roll", "yaw")]
    wrists = [joint_index[f"{side}_wrist_{axis}_joint"]
              for side in ("left", "right") for axis in ("roll", "pitch", "yaw")]
    qposes, errors = [], []
    for frame in frames:
        if fresh:
            pipeline = make_pipeline(profile, position_only)
        qpos, error = solve(pipeline, frame, iterations)
        qposes.append(qpos)
        errors.append(error)
    qposes = np.asarray(qposes)
    deltas = np.arctan2(np.sin(np.diff(qposes, axis=0)), np.cos(np.diff(qposes, axis=0)))
    discontinuity = np.linalg.norm(deltas, axis=1)
    saturated = np.isclose(qposes, pipeline.lower, atol=1e-3) | np.isclose(qposes, pipeline.upper, atol=1e-3)
    saturation_run = 0
    for joint in saturated.T:
        run = 0
        for value in joint:
            run = run + 1 if value else 0
            saturation_run = max(saturation_run, run)
    return {
        "position_rms": float(np.mean(errors)),
        "shoulder_magnitude": float(np.mean(np.linalg.norm(qposes[:, shoulders] - SONIC_NEUTRAL[shoulders], axis=1))),
        "wrist_displacement": float(np.mean(np.linalg.norm(qposes[:, wrists] - SONIC_NEUTRAL[wrists], axis=1))),
        "limit_saturation_pct": float(100 * np.mean(saturated)),
        "limit_saturation_max_run": saturation_run,
        "discontinuity_median": float(np.median(discontinuity)),
        "discontinuity_p90": float(np.percentile(discontinuity, 90)),
        "discontinuity_max": float(np.max(discontinuity)),
    }


def main():
    frames, raw = gmr_blocks(), raw_blocks()
    assert len(frames) == MAX_BLOCKS and len(raw) == MAX_BLOCKS, "need 300 complete live log blocks"

    pipeline = make_pipeline("xsens_mvn")
    viewer = mujoco.MjModel.from_xml_path(str(ROOT.parent / "GR00T-WholeBodyControl/gear_sonic_deploy/g1/g1_29dof.xml"))
    for index, name in enumerate(G1_JOINT_NAMES):
        gmr_id = mujoco.mj_name2id(pipeline.retargeter.model, mujoco.mjtObj.mjOBJ_JOINT, name)
        viewer_id = mujoco.mj_name2id(viewer, mujoco.mjtObj.mjOBJ_JOINT, name)
        assert pipeline.retargeter.model.jnt_qposadr[gmr_id] == viewer.jnt_qposadr[viewer_id] == index + 7
    print("qpos_map=PASS 29/29 named addresses and MuJoCo qpos addresses match")

    pairs = (("Left_UpperArm", "Left_UpperArm", "Left_Forearm"),
             ("Left_Forearm", "Left_Forearm", "Left_Hand"),
             ("Right_UpperArm", "Right_UpperArm", "Right_Forearm"),
             ("Right_Forearm", "Right_Forearm", "Right_Hand"))
    for orientation_name, start, end in pairs:
        initial_vector = unit(frames[0][end][0] - frames[0][start][0])
        initial_rotation = Rotation.from_quat(frames[0][orientation_name][1], scalar_first=True)
        global_errors, reversed_errors = [], []
        for frame in frames:
            vector = unit(frame[end][0] - frame[start][0])
            current = Rotation.from_quat(frame[orientation_name][1], scalar_first=True)
            global_errors.append(np.degrees(np.arccos(np.clip(
                (current * initial_rotation.inv()).apply(initial_vector) @ vector, -1.0, 1.0))))
            reversed_errors.append(np.degrees(np.arccos(np.clip(
                (initial_rotation.inv() * current).apply(initial_vector) @ vector, -1.0, 1.0))))
        assert np.percentile(global_errors, 90) < 1e-3
        assert np.median(reversed_errors) > 5.0
        print(f"rotation_order={orientation_name} current_neutral_inv_p90={np.percentile(global_errors, 90):.4f}deg "
              f"neutral_inv_current_median={np.median(reversed_errors):.1f}deg")

    shoulder_steps, wrist_steps = [], []
    for first, second in zip(raw, raw[1:]):
        for destination, indices in ((shoulder_steps, (5, 8)), (wrist_steps, (7, 10))):
            destination.extend(np.degrees((Rotation.from_quat(second[i][1], scalar_first=True) *
                                           Rotation.from_quat(first[i][1], scalar_first=True).inv()).magnitude())
                               for i in indices)
    assert np.median(wrist_steps) > 2.0 * np.median(shoulder_steps)
    print(f"raw_k4abt_step shoulder_median={np.median(shoulder_steps):.1f}deg "
          f"wrist_median={np.median(wrist_steps):.1f}deg wrist_p90={np.percentile(wrist_steps, 90):.1f}deg")

    variants = {
        "A_full_orientation": ("xsens_mvn", False),
        "B_position_only": ("xsens_mvn", True),
        "C_kinect_g1": ("kinect_g1", False),
    }
    pose_frames = [frames[index] for index in POSES.values()]
    pose_results = {name: metrics(*variant, pose_frames, 12, fresh=True) for name, variant in variants.items()}
    sequence_results = {name: metrics(*variant, frames, 1) for name, variant in variants.items()}
    for name in variants:
        pose = pose_results[name]
        sequence = sequence_results[name]
        print(
            f"abc={name} position_rms={pose['position_rms']:.4f}m "
            f"shoulder_magnitude={pose['shoulder_magnitude']:.3f}rad "
            f"wrist_displacement={pose['wrist_displacement']:.3f}rad "
            f"joint_limit_saturation={sequence['limit_saturation_pct']:.2f}% "
            f"joint_limit_max_run={sequence['limit_saturation_max_run']} "
            f"qpos_discontinuity_median={sequence['discontinuity_median']:.3f}rad "
            f"qpos_discontinuity_p90={sequence['discontinuity_p90']:.3f}rad "
            f"qpos_discontinuity_max={sequence['discontinuity_max']:.3f}rad"
        )
    final = pose_results["C_kinect_g1"]
    full = pose_results["A_full_orientation"]
    final_sequence = sequence_results["C_kinect_g1"]
    full_sequence = sequence_results["A_full_orientation"]
    assert final["position_rms"] < full["position_rms"]
    assert final["wrist_displacement"] < 0.4 * full["wrist_displacement"]
    assert final_sequence["limit_saturation_pct"] < full_sequence["limit_saturation_pct"]
    assert final_sequence["limit_saturation_max_run"] <= 1
    assert final_sequence["discontinuity_p90"] < full_sequence["discontinuity_p90"]
    print("kinect_gmr_abc=PASS")


if __name__ == "__main__":
    main()
