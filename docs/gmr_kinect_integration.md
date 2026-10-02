# Kinect to official GMR integration

## Dependency and API contract

The dependency is the official `YanjieZe/GMR` repository at sibling path
`../GMR`, commit `bb1bbe40774794fceb2a7c579a3464a28e68c844`
(2026-04-02), package version 0.2.0. It is not copied into this repository.

The inspected upstream contract is:

- `GeneralMotionRetargeting(src_human="kinect", tgt_robot="unitree_g1")`, with
  the project-local Kinect profile registered at runtime;
- one human frame is a dictionary of global body names to `(position,
  orientation)`;
- positions are metres and orientations are normalized scalar-first `wxyz`
  quaternions;
- upstream applies the derived per-body scale table, target offsets, weights,
  joint configuration limits, and Mink optimization;
- `retarget()` returns MuJoCo qpos: free-root translation `[0:3]`, root
  quaternion `[3:7]`, then the 29 G1 joints;
- the GMR model is `assets/unitree_g1/g1_mocap_29dof.xml`; the project-local
  target config is `config/kinect_to_g1.json`, derived from (without modifying)
  upstream `general_motion_retargeting/ik_configs/xsens_mvn_to_g1.json`.

`tools/kinect_gmr_bridge.py` is the Python-first adapter/service. The C++
`KinectToGMRAdapter` sends raw timestamped K4ABT joints to it over a
loopback-only request/reply socket. No GMR solver code is duplicated.

## Coordinate transform and calibration

K4ABT depth coordinates use millimetres with `+x` camera-right, `+y` down,
and `+z` away from the camera. GMR/MuJoCo uses a right-handed, metre,
z-up world. First convert position and global orientation using the proper
rotation

```text
B = [ 0  0  1 ]
    [-1  0  0 ]
    [ 0 -1  0 ]

p_m = B p_k4abt / 1000
R_m = B R_k4abt B^T
```

The first 15 fully fresh frames form one neutral calibration; calibration is
not continuously recomputed. Averaged shoulders define anatomical `+y`
(left), pelvis-to-head defines `+z`, and `+x = +y × +z`. With these unit axes
as columns of `C`, calibrated positions are

```text
p_gmr = C^T (p_m - o)
```

where `o` is the neutral pelvis horizontal origin plus the lowest neutral
ankle/foot height. Human height is estimated from neutral head-to-floor plus
0.18 m, bounded to `[1.3, 2.2]` m, and passed to GMR's official
`actual_human_height` scaling mechanism.

Raw K4ABT segment frames do not share Xsens/GMR neutral local axes. The adapter
therefore never decomposes them into hand-tuned Euler channels. It computes a
neutral-relative rotation

```text
Delta R = C^T R_current R_neutral^T C
```

and left-multiplies the mathematically derived GMR neutral source frame. That
source frame is obtained from the official GMR G1 model at the released SONIC
standing pose and GMR's official orientation offset:

```text
R_source_neutral = R_G1_link_neutral R_GMR_offset^-1
R_source_current = Delta R R_source_neutral
```

This preserves full quaternion coupling and removes K4ABT's joint-frame bias
without unexplained Euler swaps.

For `Left_Hand` and `Right_Hand`, the Kinect profile replaces the raw wrist
delta with the calibrated neutral source orientation before GMR. Their
orientation is held to a low-weight neutral target, so raw K4ABT wrist
quaternion spikes cannot enter the IK objective; wrist positions remain active.

## Kinect to GMR target mapping

All 19 requested K4ABT joints are converted, confidence-checked, cached, and
used by calibration/diagnostics. Only the 14 names consumed by upstream's
official G1 `xsens_mvn` configuration are submitted as optimization targets.

| K4ABT joint | GMR target/use | position | orientation | confidence behavior |
|---|---|---:|---:|---|
| Pelvis | `Pelvis` | yes | yes | fresh, 150 ms hold, then neutral fallback |
| SpineNavel | calibration/diagnostic | yes | yes | fresh, hold, stale |
| SpineChest | `Chest` | yes | yes | fresh, hold, then neutral fallback |
| Neck | calibration/diagnostic | yes | yes | fresh, hold, stale |
| Head | height/calibration/diagnostic | yes | yes | fresh, hold, stale |
| LeftShoulder | `Left_UpperArm` | yes (5) | reduced (2) | fresh, hold, then neutral fallback |
| LeftElbow | `Left_Forearm` | yes (20) | reduced (1) | fresh, hold, then neutral fallback |
| LeftWrist | `Left_Hand` | yes (50) | neutral, reduced (2) | fresh, hold, then neutral fallback |
| RightShoulder | `Right_UpperArm` | yes (5) | reduced (2) | fresh, hold, then neutral fallback |
| RightElbow | `Right_Forearm` | yes (20) | reduced (1) | fresh, hold, then neutral fallback |
| RightWrist | `Right_Hand` | yes (50) | neutral, reduced (2) | fresh, hold, then neutral fallback |
| LeftHip | `Left_UpperLeg` | yes | yes | fresh, hold, then neutral fallback |
| LeftKnee | `Left_LowerLeg` | yes | yes | fresh, hold, then neutral fallback |
| LeftAnkle | floor/calibration/diagnostic | yes | yes | fresh, hold, stale |
| LeftFoot | `Left_Foot` | yes | yes | fresh, hold, then neutral fallback |
| RightHip | `Right_UpperLeg` | yes | yes | fresh, hold, then neutral fallback |
| RightKnee | `Right_LowerLeg` | yes | yes | fresh, hold, then neutral fallback |
| RightAnkle | floor/calibration/diagnostic | yes | yes | fresh, hold, stale |
| RightFoot | `Right_Foot` | yes | yes | fresh, hold, then neutral fallback |

LOW-or-better samples are fresh. A bad sample holds only that body target for
150 ms. After expiry it returns to the captured neutral target; it never
fabricates random motion or resets unrelated limbs.

## G1 output contract

The bridge resolves every qpos address by joint name and asserts that GMR and
SONIC share this order:

```text
left hip pitch, roll, yaw; left knee; left ankle pitch, roll;
right hip pitch, roll, yaw; right knee; right ankle pitch, roll;
waist yaw, roll, pitch;
left shoulder pitch, roll, yaw; left elbow; left wrist roll, pitch, yaw;
right shoulder pitch, roll, yaw; right elbow; right wrist roll, pitch, yaw
```

GMR's model limits are a conservative subset of the SONIC model limits for
this order. Output is clamped to GMR's named limits. A 0.45 exponential filter
is applied, changes above 1.2 rad/frame are rejected, and qvel is the wrapped
filtered delta divided by the K4ABT timestamp interval, bounded to 12 rad/s.
Invalid values and timestamp intervals outside 1–250 ms never produce an
unbounded velocity.

## Modes and evidence boundary

- `--retargeter gmr` is the default and requires the Python bridge.
- The bridge defaults to `--profile kinect_g1` and prints `gmr_source=kinect`,
  `gmr_profile=kinect_g1`, `upper_limb_orientation=position_dominant`, and
  `wrist_orientation=neutral_low_weight` at startup. `--profile xsens_mvn`
  exists only for the A/B diagnostic.
- `--retargeter legacy` preserves the previous custom mapping for comparison.
- `--output mujoco-direct` pins the SONIC G1 free root and applies named qpos;
  it is a kinematic mapping viewer, not a balance controller.
- `--output sonic-v1` reuses the joint Protocol v1 publisher; SONIC remains the
  balance controller and no SMPL fields are emitted. The idempotent start
  command is sent with every active pose because Protocol v1 has no command
  acknowledgement and a subscriber may still be joining when the first pose
  is published.

Synthetic tests cover neutral, T-pose, unilateral arm raises, elbow bend,
torso yaw, hip flexion/abduction, knee bend, single-leg reference, confidence
hold/stale behavior, limits, finite velocity, and a real C++/Python bridge
comparison with the legacy retargeter. This is software evidence only. A
tracked-person Femto Bolt session is still required before claiming that the
new mapping fixes live motion.

The connected Femto Bolt and CUDA body tracker were started with the fixed-base
command and sustained roughly 22-24 capture FPS, but the unattended check had
no tracked person in view: all 19 joints remained at confidence NONE. Therefore
neutral calibration, live GMR solves, limb-direction checks, and visual
coherence are not yet validated.

The synthetic GMR publisher was also exercised through the unmodified released
SONIC encoder/decoder/policy and floating-base simulator on loopback DDS domain
42. SONIC entered streamed G1 mode, consumed 465 GMR-derived Protocol v1
frames, ran 18.7 simulated seconds, exited on the stop command, and reported no
simulator reset or hand command. The policy's observe-only simulator safety
diagnostics did report some generated motor commands outside nominal limits;
this is not a live-hardware safety approval. No policy file or weight was
changed.
