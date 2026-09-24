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
