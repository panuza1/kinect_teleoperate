# Kinect → G1 planning and implementation scope

Read `IMPLEMENTATION_PLAN.md` and current gate evidence before implementation. The parent workspace rules still apply, but this Kinect task requires live MuJoCo validation before real G1; the parent Quest/LeRobot plan's optional-simulation rule does not apply.

- Follow the plan's sequential phases and record the phase's exact task classification and validation results.
- Extend the existing `src/kinect_smpl_zmq_bridge` and existing sibling SONIC controller. Preserve tested contracts; do not replace them with the legacy Euler/fixed-base demonstration.
- The user has authorized implementation of `SOFTWARE_ONLY` plan tasks. Do not interpret that authorization as permission to open or control Kinect, DDS, or G1 hardware; mark those steps `HARDWARE_VALIDATION_REQUIRED`.
- Default to observe/no-publish. Automated tests must not initialize physical robot command channels, including through startup, cleanup, or hand-control paths.
- Do not bring real G1 into this workflow before SOFTWARE_READY, KINECT_READY and SIM_LIVE_READY pass. Real application commands also require REAL_G1_PREFLIGHT_READY and explicit supervised session authorization.
- Never claim a hardware gate passed from source inspection, synthetic input, or historical SIM_READY notes. Record missing hardware/evidence as a blocker.
- Preserve unrelated changes. External SONIC changes must follow that checkout's applicable instructions and filesystem permissions; do not create a duplicate controller to bypass them.
- At phase completion, update the plan's evidence/status ledger with exact tests, artifacts, hashes, blockers and next phase. Safety/config/model changes invalidate affected downstream gates.

Current evidence (2026-09-24): Phases 2–7 software work is implemented and its
hardware-free suites pass. Phase 8 is blocked by a real production-decoder
output: the isolated ZMQ→SONIC→DDS→floating-base MuJoCo run entered CONTROL,
then the final guard rejected an out-of-range ankle-pitch target (latest
realistic fixture: left 0.885374 rad versus model maximum 0.5236 rad).
The exact matching official default, `sonic_v1_1`, and `low_latency`
checkpoint/config trios have now all failed the strict raw-target contract;
the latter two were rejected on waist pitch. Scaling, joint order,
normalization, and artifact pairing were verified. The released training
contract clips residual actions globally at 20 rather than to each joint's
asymmetric legal residual envelope. Do not weaken the guard, widen limits,
rescale post hoc, or rely on simulator clipping. Resume only with a compatible
or retrained checkpoint, rerun all invalidated suites/E2E, and then complete
the 1,800-second zero-reset soak. No Kinect or G1 gate has passed.

A 2026-09-24 experiment projected raw residuals into each named joint's exact
asymmetric legal envelope before target generation. It produced 13 simulation
safety resets in 38.2 simulated seconds. After fixing `MotorSafety` to reject
post-derivative-limit position escape, the experiment stopped safely on a
dynamically infeasible right-knee command at 0.1 s. The unstable projection was
removed; the stronger final guard and shared authoritative limit arrays remain.
Do not reintroduce static post-policy projection as a gate workaround.
