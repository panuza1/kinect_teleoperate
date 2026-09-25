# MuJoCo software validation

The supported software integration uses the sibling SONIC controller's
`SONIC_SIM_ORT` build, DDS domain 42 on loopback, and the floating-base
`scene_43dof.xml` model with the elastic band disabled. Physical interfaces
must not be selected.

The required order is simulator, controller, wait for `Init Done`, then the
bridge replay with `config/sim_e2e.json`, `--publish`, and an explicit `--arm`.
Starting replay before the subscriber is ready can lose the one-shot start
event because ZMQ PUB has no persistence; this is a failed run, not evidence.

Capture controller and simulator stdout separately and validate them with:

```bash
python3 scripts/validate_pipeline.py sim-validate \
  --controller-log artifacts/software/controller.log \
  --sim-log artifacts/software/simulator.log \
  --min-sim-time 1800 \
  --report artifacts/software/sim.json
```

A passing soak requires at least 1,800 simulated seconds, entry into CONTROL,
zero `MotorSafety` rejection, zero simulator reset/fall, no raw target outside
the model range, no elastic support, and clean stop handling.

## Checkpoint compatibility result — 2026-09-24

The action/model contract, not the final guard, is the remaining blocker. The
deployment conversion is the same as training:

`target = default_angle + residual * (0.25 * effort_limit / stiffness)`

For `left_ankle_pitch_joint`, the scale is `0.4385773139`, the default is
`-0.363`, and the legal upper target is `0.5236`. The largest compatible
positive residual is therefore about `2.0215`. The default decoder emitted
about `2.8464`, producing `0.885374` rad. The training wrapper only applies a
global symmetric action clip of `20.0`; it does not impose the asymmetric
per-joint residual envelope implied by the named model limits. Joint ordering,
action scaling, default angles, observation dimensions, and matching
checkpoint/config hashes were verified. No decoder/controller normalization
mismatch was found.

All currently published official variants were then run with their matching
encoder, decoder, and observation configuration through the actual isolated
ZMQ → SONIC ORT → DDS domain 42/`lo` → floating-base MuJoCo path:

| Variant | Matching artifact SHA-256 (decoder / encoder / config) | First final-guard rejection | Report |
|---|---|---|---|
| default | `c7241a12…` / `013ab028…` / `466d0594…` | `left_ankle_pitch_joint=0.885374`, max `0.5236` | `artifacts/software/sim.json` |
| sonic_v1_1 | `34bae857…` / `fb97de22…` / `4a67713b…` | `waist_pitch_joint=0.586278`, max `0.52` | `artifacts/software/v1_1/sim.json` |
| low_latency | `c4ac2e74…` / `60be4315…` / `582b9a27…` | `waist_pitch_joint=0.563316`, max `0.52` | `artifacts/software/low_latency/sim.json` |

Each variant entered or began CONTROL with dimensions validated, and each was
stopped safely before an illegal target reached MuJoCo. This proves that none
of the three released checkpoint/config trios satisfies the project's strict
raw-target contract for this valid complete trace. A different checkpoint
trained or validated with the authoritative per-joint envelope is required.
Post-hoc clamping, limit widening, global rescaling, or relying on MuJoCo
clipping would change the learned controller contract and are not accepted.

The 1,800-second soak was deliberately not run: its prerequisite—zero raw
illegal commands in the production closed loop—fails immediately. After a
compatible checkpoint is supplied, rerun all three steps in order: the Phase 6
safety/decoder tests, the Phase 7 diagnostics tests, and this production E2E;
only a passing E2E may start the soak.

## Bounded residual experiment — 2026-09-24

An explicit per-joint Euclidean projection was evaluated before target
generation. For joint `i`, it derived the asymmetric residual interval
`[(q_min-default)/scale, (q_max-default)/scale]`, preserved every already-legal
residual exactly, projected only values outside that interval, fed the applied
residual back into the policy's last-action observation, and logged every
activation. The authoritative named mapping and final guard remained active.

This transform did not preserve usable closed-loop behavior. In the first
production run it activated repeatedly and the simulator recorded 13 safety
resets in 38.2 simulated seconds. Targets escaped the model range after the
existing acceleration limiter, including the knee, ankle and waist. That
revealed a separate final-boundary defect: a target was checked before
derivative limiting but not after it. `MotorSafety` now computes derivative
limited candidates without mutating history and rejects any candidate that
cannot satisfy both derivative and position constraints. A regression test
covers this case.

With that guard corrected, the same bounded transform failed safely at 0.1
simulated seconds: `right_knee_joint: derivative limits cannot preserve
position limit`. The experimental transform was removed rather than shipped as
an unstable controller behavior; the strengthened final guard and shared
authoritative limit arrays were retained. Evidence is under
`artifacts/software/bounded/`, including `sim.json` (unsafe experimental run)
and `sim-guard.json` (correct fail-safe run).

Conclusion: static per-joint projection is mathematically bounded but is not a
valid deployment fix for this policy. It drives setpoints to hard boundaries
without the dynamically feasible approach behavior learned by the controller.
Training or fine-tuning must incorporate the bounded action parameterization
and derivative-feasible margin so the policy learns to remain controllable
inside it.

## Dynamically feasible training action smoke — 2026-09-24

The minimum training-side action term now uses the same stateful transform as
deployment and asserts the resolved 29-joint order, nominal defaults, scales,
and 20 ms control period. The feasible fine-tune config disables the existing
joint-default randomization so it cannot silently invalidate that contract.
Python/C++ transform parity and strict mismatch tests pass.

The unchanged released checkpoint was then tested with
`--dynamically-feasible-actions --no-hand-publish` on the required isolated
production path. It entered CONTROL and responded to the replay, but was
dynamically unstable: MuJoCo recorded 24 resets, maximum command displacement
`2.5690` rad and maximum simulated joint displacement `2.6787` rad, with only
`0.9` s maximum uninterrupted simulation time. The final guard stopped on a
dynamically infeasible command. No Dex3 command was observed. This is a failed
functional E2E, not soak evidence; see
`artifacts/software/2026-09-24-feasible-policy/smoke-report.json`.

Fine-tuning from `sonic_release/last.pt` is therefore required. The requested
debug run was attempted with `sonic_release_feasible.yaml` and stopped before
environment creation because Isaac Lab/Isaac Sim is not installed. The required
motion datasets are also absent and this host has a 4 GiB RTX 3050 Laptop GPU.
Move fine-tuning to a compatible Isaac Lab 2.3+ training machine, export the
matching encoder/decoder/config, then restart this validation at the short
smoke. Do not start the 1,800-second soak unless the smoke has zero resets and
zero rejections while meeting the functional movement thresholds.

## Stock SONIC baseline and Kinect replay comparison — 2026-09-24

For the current input-baseline task, the preceding fine-tuning recommendation
is superseded: no fine-tuning or experimental action transform was used. Both
runs used the official matched release decoder, encoder and observation config,
stock action scaling, isolated DDS domain 42/`lo`, floating-base MuJoCo, and
`--no-hand-publish`. They did not touch physical Kinect or G1.

- Model contract audit: training MJCF/URDF, stock simulator XML and both
  deployment XMLs resolve the same 29 named body joints and limits. The
  deployment `isaaclab_to_mujoco` permutation exactly maps the IsaacLab order
  to SDK/MuJoCo order. The stock config is
  `gear_sonic/utils/mujoco_sim/wbc_configs/g1_29dof_sonic_model12.yaml`, using
  `gear_sonic/data/robot_model/model_data/g1/scene_43dof.xml` in PR mode. The
  ankle ranges are model data, not locally invented values: left ankle pitch
  `[-0.87267, 0.5236]`, right ankle roll `[-0.2618, 0.2618]`.
- SONIC's own `reference/example/` input entered CONTROL and ran for 35.9
  simulated seconds with zero reset/fall while the controller was active.
  Maximum commanded displacement was `2.1028` rad, maximum measured joint
  displacement `1.4144` rad and Dex3 command count zero.
- The Kinect bridge replayed all 1,801 frames. SONIC entered streamed-motion
  and SMPL mode with matching 994D policy and 1762D encoder observations. At
  replay completion the simulator had run 60.6 simulated seconds with zero
  reset/fall, maximum commanded displacement `2.4360` rad, maximum measured
  joint displacement `1.5564` rad and Dex3 command count zero. A later fall
  occurred only after the controller was manually terminated and is not part
  of the active replay interval.
- Each Kinect packet contains SMPL joints `[1,24,3]`, SMPL pose `[1,21,3]`, the
  canonical 24-joint order, wxyz root orientation, meters in the root-local
  SONIC frame, joint/velocity fields and monotonic frame/epoch/sequence/source
  timing. The 30 Hz one-frame stream matches SONIC's checked-in live-camera
  publisher. `StreamedMotionMerger` buffers those frames and gathers the
  requested future window for the 50 Hz control loop, so no input-side
  resampling was added.

The discrepancy was the locally added strict desired-setpoint contract, not a
stock model, map, policy or Kinect mismatch. Stock SONIC publishes PD setpoints
and the official MuJoCo joints/actuators constrain the physical state; an
out-of-range desired setpoint is not itself an out-of-range simulated joint.
The extra `MotorSafety` contract is now observe-only only in `SONIC_SIM_ORT`,
so it records discrepancies without changing the stock simulation command.
It remains enforcing in non-simulator builds. Simulator raw-target rejection
is likewise an explicit opt-in diagnostic; the default follows stock behavior.

Result: the stock SONIC and Kinect-replay software baselines pass. No policy
modification, resampling, training or long soak was required by this task. No
physical Kinect or G1 validation was performed; the next step is Kinect-only
Phase 9 acquisition and coordinate/calibration validation when hardware is
available.
