# Kinect full-body to G1 architecture

## Ownership boundary

Debug path:

`Kinect -> full-body retargeter -> G1 qpos[29] -> fixed-base/kinematic MuJoCo`

Final simulation path:

`Kinect -> full-body retargeter -> G1 qpos[29] + qvel[29] -> SONIC Protocol v1 -> released SONIC policy -> floating-base MuJoCo`

Retargeting decides the desired G1 pose. SONIC is the whole-body controller responsible for converting that reference into dynamically stable G1 actions. Retargeting is not the balance controller. Direct mode is only a mapping diagnostic and makes no stability claim.

The retargeter uses parent-child quaternion deltas captured relative to a neutral frame after camera-to-robot basis conversion. Knee flexion also uses hip-knee-ankle geometry. Unobservable DoFs use the released policy's conservative standing default. Every target is clamped to the SONIC G1 range.

## SONIC Protocol v1 contract

- One ZMQ message with topic prefix `pose`.
- Exactly 1280 null-padded bytes of JSON after the topic.
- Header: `v=1`, `endian="le"`, `count=N`.
- Required fields: `joint_pos` f32 `[N,29]`, `joint_vel` f32 `[N,29]`, `body_quat_w` f32 `[N,4]`, `frame_index` i64 `[N]`.
- Live flag: `catch_up` u8 `[1]`, false.
- Payload is contiguous row-major data in header field order.
- Protocol v1 selects SONIC encode mode 0. This path sends no `smpl_joints` or `smpl_pose`.

## Authoritative 29-DoF reference order

The order/defaults come from SONIC's local `policy_parameters.hpp` MuJoCo/reference order. Limits come from `gear_sonic_deploy/g1/g1_29dof.xml`, not the legacy Kinect hand model.

| index | G1 joint name | Kinect source joints | conversion method | neutral/default | joint limit |
|---:|---|---|---|---:|---|
| 0 | left_hip_pitch_joint | pelvis, left hip | neutral-relative parent/child quaternion pitch | -0.312 | [-2.5307, 2.8798] |
| 1 | left_hip_roll_joint | pelvis, left hip | neutral-relative parent/child quaternion roll | 0 | [-0.5236, 2.9671] |
| 2 | left_hip_yaw_joint | pelvis, left hip | neutral-relative parent/child quaternion yaw | 0 | [-2.7576, 2.7576] |
| 3 | left_knee_joint | left hip, knee, ankle | neutral-relative hinge bend | 0.669 | [-0.087267, 2.8798] |
| 4 | left_ankle_pitch_joint | left knee, ankle, foot | neutral-relative ankle quaternion pitch | -0.363 | [-0.87267, 0.5236] |
| 5 | left_ankle_roll_joint | left knee, ankle, foot | neutral-relative ankle quaternion roll | 0 | [-0.2618, 0.2618] |
| 6 | right_hip_pitch_joint | pelvis, right hip | neutral-relative parent/child quaternion pitch | -0.312 | [-2.5307, 2.8798] |
| 7 | right_hip_roll_joint | pelvis, right hip | neutral-relative parent/child quaternion roll | 0 | [-2.9671, 0.5236] |
| 8 | right_hip_yaw_joint | pelvis, right hip | neutral-relative parent/child quaternion yaw | 0 | [-2.7576, 2.7576] |
| 9 | right_knee_joint | right hip, knee, ankle | neutral-relative hinge bend | 0.669 | [-0.087267, 2.8798] |
| 10 | right_ankle_pitch_joint | right knee, ankle, foot | neutral-relative ankle quaternion pitch | -0.363 | [-0.87267, 0.5236] |
| 11 | right_ankle_roll_joint | right knee, ankle, foot | neutral-relative ankle quaternion roll | 0 | [-0.2618, 0.2618] |
| 12 | waist_yaw_joint | pelvis, spine chest | neutral-relative torso quaternion yaw | 0 | [-2.618, 2.618] |
| 13 | waist_roll_joint | pelvis, spine chest | neutral-relative torso quaternion roll | 0 | [-0.52, 0.52] |
| 14 | waist_pitch_joint | pelvis, spine chest | neutral-relative torso quaternion pitch | 0 | [-0.52, 0.52] |
| 15 | left_shoulder_pitch_joint | chest, left shoulder | neutral-relative shoulder quaternion pitch | 0.2 | [-3.0892, 2.6704] |
| 16 | left_shoulder_roll_joint | chest, left shoulder | neutral-relative shoulder quaternion roll | 0.2 | [-1.5882, 2.2515] |
| 17 | left_shoulder_yaw_joint | chest, left shoulder | neutral-relative shoulder quaternion yaw | 0 | [-2.618, 2.618] |
| 18 | left_elbow_joint | left shoulder, elbow, wrist | neutral-relative hinge bend | 0.6 | [-1.0472, 2.0944] |
| 19 | left_wrist_roll_joint | left elbow, wrist | neutral-relative wrist quaternion roll | 0 | [-1.97222, 1.97222] |
| 20 | left_wrist_pitch_joint | left elbow, wrist | neutral-relative wrist quaternion pitch | 0 | [-1.61443, 1.61443] |
| 21 | left_wrist_yaw_joint | left elbow, wrist | neutral-relative wrist quaternion yaw | 0 | [-1.61443, 1.61443] |
| 22 | right_shoulder_pitch_joint | chest, right shoulder | neutral-relative shoulder quaternion pitch | 0.2 | [-3.0892, 2.6704] |
| 23 | right_shoulder_roll_joint | chest, right shoulder | neutral-relative shoulder quaternion roll | -0.2 | [-2.2515, 1.5882] |
| 24 | right_shoulder_yaw_joint | chest, right shoulder | neutral-relative shoulder quaternion yaw | 0 | [-2.618, 2.618] |
| 25 | right_elbow_joint | right shoulder, elbow, wrist | neutral-relative hinge bend | 0.6 | [-1.0472, 2.0944] |
| 26 | right_wrist_roll_joint | right elbow, wrist | neutral-relative wrist quaternion roll | 0 | [-1.97222, 1.97222] |
| 27 | right_wrist_pitch_joint | right elbow, wrist | neutral-relative wrist quaternion pitch | 0 | [-1.61443, 1.61443] |
| 28 | right_wrist_yaw_joint | right elbow, wrist | neutral-relative wrist quaternion yaw | 0 | [-1.61443, 1.61443] |

## Confidence, filtering, and time

LOW-or-better K4ABT samples update a joint. A confidence loss holds only that joint for a short interval; it does not reset the body/session. After expiry, the affected chain returns toward neutral while valid chains continue. Position is exponentially filtered. Velocity is the wrapped filtered-position delta divided by Kinect timestamp `dt`. Non-finite input, backwards/implausible `dt`, and large discontinuities are rejected.

## Simulation safety boundary

Direct debug resolves joints/actuators by name in SONIC's G1 XML, pins the free-base qpos to its initial pose, writes named joint qpos, zeros qvel, and calls `mj_forward`. It never opens DDS. Whole-body balance and contacts are validated only through the existing SONIC floating-base MuJoCo path on DDS domain 42 and loopback.

## Validation evidence (2026-10-02)

- The project builds and all seven `kinect_*` CTest targets pass.
- The protocol test is decoded by SONIC's real `ZMQPackedMessageSubscriber` and checks known 29-joint values and commands.
- A five-second synthetic stream drove the released SONIC controller into `CONTROL` against floating-base MuJoCo on DDS domain 42; it stopped cleanly, published no hand commands, and had no reset while controlled.
- Live sensor validation is blocked because no Femto Bolt/K4A-compatible device is connected. No physical G1 was addressed.
