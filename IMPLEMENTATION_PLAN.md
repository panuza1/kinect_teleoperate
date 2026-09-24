# Azure Kinect → Unitree G1: implementation and validation plan

Planning baseline: 2026-09-22. This document is the implementation contract for a subsequent GPT-5.6 session. This task changes documentation only. No device session, DDS publisher, controller, or real robot motion was started during this audit.

## 1. Scope, evidence, and decisions

Repository: `/home/panu/Documents/fibo/project_humanoid/g1_inspire_workspace/kinect_teleoperate`.
Audited HEAD: `9ad42a4bfe7abd32c8725ce1f5d532d23f77fe22`; initially clean worktree; 606 tracked files including vendor assets. External controller checkout: `../GR00T-WholeBodyControl`, HEAD `a18d0595df0d0625fcbdd716788370b72382e7f5`. Commit IDs do not capture external uncommitted work: Phase 1 must hash and preserve that checkout's working files too.

The entire repository was inventoried. First-party acquisition, conversion, session, publishing, synthetic and legacy control code, build files, tests, model joint/actuator definitions, documentation, and renderer interfaces were inspected. Bundled GLFW/MuJoCo source, meshes, firmware and Debian packages were inventoried as dependencies/assets, not exhaustively reviewed as first-party implementation. The external SONIC input, policy/output boundaries and simulator safety code were inspected where required by this pipeline; unrelated XR, training and LeRobot systems are not implementation targets.

Evidence vocabulary:

* **Verified now:** inspected code or a command executed in this audit.
* **Historical:** reported by existing documentation; does not count as a current gate pass.
* **Unverified:** requires a new experiment or hardware. Never convert this to PASS because code exists.
* **Planned:** a future interface, file, test, or behavior defined below; not currently available.

The parent `AGENTS.md`, `MASTER_PLAN.md`, `INTEGRATION_SPEC.md`, and `HANDOFF.md` were reviewed. Their main project is Quest/Inspire/LeRobot. Their optional-simulation rule does **not** apply here: this Kinect task explicitly requires live MuJoCo before G1. Parent HANDOFF also contains historical Kinect/SONIC SIM_READY evidence. This plan supersedes those readiness claims for this task without modifying parent files.

### Binding architecture decisions

1. Extend `src/kinect_smpl_zmq_bridge`; retain `BridgeSession`, `SkeletonToSmpl`, `SonicPoseFrame`, and `SonicZmqPublisher` as the main path. Synthetic, replay, and live acquisition enter the same session and safety code.
2. SONIC's existing SMPL encoder/policy is the whole-body retargeter/controller. Do not add a competing IK stack or route the legacy Euler retargeter into production. Validate learned retargeting, reference generation, and final motor commands separately.
3. The controller remains in `../GR00T-WholeBodyControl/gear_sonic_deploy`; DDS remains owned there. A bridge-only safety fix cannot establish real safety because the policy subsequently generates 29 motor targets.
4. Use the existing SONIC floating-base MuJoCo scene. The local fixed-base `src/unitree_g1/g1.xml` is an old visualization model, not a dynamics readiness oracle.
5. Start in observe-only mode. Publishing, arming, and locomotion are separate permissions. Reconnect never restores arming. Real robot commands require all four preceding gates plus a supervised session authorization.
6. Reuse C++17/Eigen, existing SONIC YAML/JSON facilities, Python standard library and installed MuJoCo/NumPy tools. No new middleware, web dashboard, ROS dependency, generic plugin architecture, or controller framework is needed.
7. Every numbered implementation phase has exactly one classification: `SOFTWARE_ONLY`, `KINECT_HARDWARE_REQUIRED`, or `REAL_G1_HARDWARE_REQUIRED`. All tasks within that phase inherit its classification. Hardware testing does not reclassify the software development that prepares it.
8. `REAL_G1_READY` means the exact recorded robot, firmware, model/checkpoints, calibration, configuration, approved motion envelope, and environment passed. It is not permission for unrestricted imitation, stairs, jumping, unseen environments, or arbitrary firmware changes.

## 2. Repository audit and current architecture

### Current data flow

`main.cpp::live → SDK capture RGB/depth → body tracker → selected SkeletonSample → BridgeSession::process → SkeletonToSmpl::calibrate/convert → session fallback/step limiting → publish(command, pose, planner) → SonicZmqPublisher → SONIC ZMQManager/ZMQEndpointInterface → SMPL encode mode 2 → policy/planner → G1Deploy::LowCommandWriter → DDS → SONIC MuJoCo bridge`.

`main.cpp::replay` recomputes recorded session outputs, compares selected fields and publishes unless `--debug-skeleton` is set. `synthetic_sim_sender.cpp` instead bypasses `BridgeSession`, creates its own planner hysteresis and auto-calibrates the converter. The legacy `kinect_teleoperate` executable uses nearest-body selection, Euler mappings and local fixed-base MuJoCo position actuators; real SDK integration there is a placeholder.

### Audit ledger

| Area / exact owner | Current finding | Status and required response |
|---|---|---|
| SDK installation | Local extracted prefix `/home/panu/.local/share/azure-kinect/1.4.1-1.1.2/usr` contains `libk4a.so.1.4.1`, `libk4abt.so.1.1.2`, SDK CMake metadata and both tracking models. Bridge Debug configure/build succeeded. | SDK files verified; successful live initialization, USB access, provider compatibility and camera throughput unverified. Historical HANDOFF's missing-runtime claim is stale. |
| `main.cpp::live`, `Sensor` | Device count/open, 720p MJPG, NFOV_UNBINNED, 30 FPS, synchronized images, selectable CPU/model, bounded SDK calls and RAII device/tracker cleanup exist. | Partial. Device index 0 only; no serial selection/reopen state machine, stream timestamps/intrinsics report, RGB/depth viewer or separate stream failure reason. RGB/depth are released after presence check. |
| Capture scheduling | One pending tracker capture bounds queue growth. Captures arriving while pending are discarded. Capture then enqueue/pop can cumulatively block multiple 100 ms intervals. | Preserve bounded queue idea; measure actual ages and explicit drop reasons, separate capture polling from deadline enforcement. A 500 ms queue-age cutoff is not a proven safe latency budget. |
| Body selection | Remembers body ID while present, else silently selects index 0. | Unsafe identity transfer. Lock selected ID; explicit selection/reselection; ambiguity must inhibit control. |
| `BridgeSession::process` | Two-second neutral collection, ≥8 frames, 80 mm pelvis stability, 11 critical medium-confidence joints, finite checks, timestamp order, pelvis jump check. | Useful but incomplete. Calibration accepts sparse/biased evidence; critical set omits elbows/knees/wrists; only pelvis jumps checked; ID change resets calibration but retains old fallback pose. |
| `SkeletonToSmpl` | Explicit 24-joint mapping/parents, mm→m, basis conversion for positions/orientations, parent-relative orientation, neutral offsets, root-relative positions, quaternion sign handling. | Retain tested mathematics where correct. Full SMPL rest-basis/FK consistency not demonstrated by all-identity synthetic orientations. Neck populates both spine3 and neck; this is an approximation requiring explicit error limits. |
| Coordinate/scale assumptions | Fixed camera basis, 1.70 m head-to-foot scale clamped [0.8,1.2], root-derived orientation and camera-axis pelvis velocity. | No floor/gravity alignment, surveyed heading, mount/extrinsic persistence or configurable anthropometric reference. Head joint is not stature. Facing-camera “forward” may not mean +camera Z. |
| Partial skeleton handling | Converter accepts LOW confidence and holds previous individual joints without an age limit; session demands MEDIUM for only selected joints. | Conflicting confidence rules; limb freshness and parent-chain validity required. |
| Motion preprocessing | Fixed-frame EMA 0.75, per-component SMPL position 1.2 m/s and axis-angle 2.4 rad/s step bounds, root quaternion blend. | Frame-rate dependent smoothing; Euclidean axis-angle blending crosses branch discontinuities; no acceleration bound, bone consistency or joint-specific sudden jump rejection. Pelvis is not included in session position-step limiter. |
| `BridgeSession::fallback/reset_calibration` | Zero velocity and fade toward stored human neutral; one-second loss resets converter/calibration. | Fallback bypasses normal step limits; quaternion blend lacks hemisphere correction; fresh fallback packets can conceal indefinitely stale tracking. Reset leaves `have_neutral_`/old output. Neutral human pose is not automatically a safe robot trajectory. |
| `SonicZmqPublisher` | Loopback default, v3 pose, fixed 1280-byte header, SMPL arrays, root wxyz, sequence, monotonic timestamp, finite guards, zero linger. | Preserve wire contract. `joint_pos/joint_vel` are zero placeholders for SMPL mode, not retargeted commands. No receiver acknowledgment, source-health channel, explicit send outcome/drop counters or coordinated topic epoch. Native byte append advertises little endian without host check. |
| Planner publication | Session enter/exit thresholds .08/.04, publisher moving threshold .05; publisher integrates facing using wall clock. Turning introduces minimum .10 m/s command speed. | Three different decisions; in-place turn can become translation. Move semantic intent into one deterministic session step; encode only in publisher. Joint/body reference safety after policy remains necessary. |
| `bridge_session.cpp` trace functions | Magic `KSMPLR1`, raw struct serialization, truncation/header checks. | ABI/padding/compiler-dependent, no configuration/calibration/model hash/checksum, no images or stage timestamps. Boolean/enum/float/size trust boundaries incomplete. Preserve R1 read compatibility where ABI matches, add explicit portable R2. |
| `main.cpp::replay/same_pose` | Recalculates state/publish/planner and pose arrays with 1e-4 tolerance; replay sleeps and rebases output timestamp. | Does not validate publisher's clock-driven planner packet sequence; no fast/no-publish default, no malformed-input suite, no downstream deterministic validation. |
| Existing C++ tests | `bridge_session_test.cpp` passes in Debug, covers basic calibration, arms, walking/turning intent, loss, repeated timestamp and trace roundtrip. | Incomplete but valuable; retain. Asserts can vanish with NDEBUG. `synthetic_test.cpp` and `synthetic_sim_sender.cpp` are not CMake targets. CTest reports zero tests. |
| Legacy `main.cpp` | Shared unprotected `s_isRunning`, skeleton angle globals, frame handoff and shared `mjData`; busy control loop, no fresh-skeleton watchdog, unchecked actuator lookup results. | Static race/stale-data risks. Quarantine as legacy simulation demonstration; do not spend required phases rebuilding a duplicate control path. |
| Legacy helpers | `MovingAverageFilter::average_` is uninitialized before a first invalid update; duplicate values suppress sampling; gesture detector toggles based on loop count. Euler mapping comments admit coordinate uncertainty. | Known legacy defects; remove from supported production path. Do not claim these functions are correct or reuse them for safety/calibration. |
| Local G1 XML | Free joint is commented out; `torso_joint`, `left_elbow_pitch_joint`, elbow roll and hand joints differ from SONIC 29-body-DOF names/model. | Conflicting model, ranges and actuator semantics. Never copy its joint index/range arrays into SONIC. |
| SONIC external input | Existing v2/v3 → SMPL encode mode 2, stream buffers, safety resets, ~1 s pose/planner timeout. | Preserve tested decoder semantics; tighten/configure freshness. Planner valid branch must check age before consuming delayed data, not only when no valid message remains. |
| SONIC final writer | `G1Deploy::LowCommandWriter` reads buffer data, enables motors, writes targets/gains/torque/CRC, calls Dex3 writer. It does not enforce final per-joint limits/age in this function. | Mandatory final safety boundary for every caller, including INIT/Stop. Dex3 must be disabled for this task unless actual hand variant is separately validated. |
| SONIC simulation | `DefaultEnv` clips q targets, guards torque at bounds, resets bad states; SDK simulation bridge uses policy-start marker. | Defenses can mask policy violations. Log before clipping and count every reset. Sim-only pause/elastic support/reset/CRC bypass is not hardware evidence. |
| Logging/tools | Stdout states/rates, legacy 3D renderer, SDK viewers and SONIC logger exist. | No unified event record, correlated latency percentiles, dropped-frame report, health gate artifact, calibration CLI, SMPL/robot overlay or hardware preflight. Reuse existing viewers/loggers. |
| Build/docs | Root requires SDK before any target, default legacy path pulls visualization dependencies; no root AGENTS or implementation plan initially. | Add test-only build separation, CTest registration, current architecture entry in README and scoped AGENTS. |
| Dead/duplicate scope | H1 models/launch files, dormant Real_Control/hand/torso macros, two synthetic neutral builders, unused trace/test includes and `frame_index_` bookkeeping. | H1/vendor assets are out of this pipeline, not proven globally dead. Synthetic sources are unbuilt, not useless. Consolidate only duplicates involved in production; no broad deletion campaign. |

### Verification executed in this audit

Configure: `cmake -S . -B /tmp/kinect-plan-audit-build -DKINECT_BRIDGE_ONLY=ON -DCMAKE_BUILD_TYPE=Debug -DCMAKE_PREFIX_PATH=/home/panu/.local/share/azure-kinect/1.4.1-1.1.2/usr -DZMQ_INCLUDE_DIR=/home/panu/.local/share/azure-kinect/1.4.1-1.1.2/usr/include -DZMQ_LIBRARY=/usr/lib/x86_64-linux-gnu/libzmq.so.5` — PASS.

Build: `cmake --build /tmp/kinect-plan-audit-build -j2` — PASS. `/tmp/kinect-plan-audit-build/kinect_bridge_session_test` — PASS. `ctest --test-dir /tmp/kinect-plan-audit-build -N` — **0 tests**, discovery gap, not suite success. A 1,801-frame synthetic trace was generated for no-publish replay. Final replay and external test outcomes are recorded in section 15.

Historical `../HANDOFF.md` reports integrated neutral/arms/keyboard-planner simulation, about 0.787 m pelvis height and zero resets. That is useful regression context, not evidence of Kinect hardware, leg raise, real controller parity, or live readiness. No original `left_leg_raise` fixture/log was found in the audited repository or searched workspace text. Its reported joint-limit failure remains an unresolved blocker, not an identified root cause.

### Preserve rather than blindly rewrite

Preserve SDK resource ownership, single-pending-capture backpressure, 24-joint order and parent topology, proper basis-conjugation principle, mm→m conversion, quaternion normalization/hemisphere handling, calibrated parent-relative conversion, loopback default, v3 SMPL mode, explicit operator start separation, valid trace regression fixtures, and existing SONIC observation/action normalization, gains, joint-order definitions and policy pipeline. New golden tests must precede changes to any of these. Never “fix” a motion by swapping axes, changing model limits, disabling collisions, or increasing gains until the recorded discrepancy proves that change is correct.

## 3. Target architecture and contracts

### Ownership and flow

`KinectInput / replay / synthetic → timestamped FrameEnvelope → locked body selection → normalized calibrated skeleton → validated SMPL → bounded motion and intent → loopback ZMQ → SONIC input validation → learned retargeting/body reference → final MotorSafety → selected command sink → MuJoCo OR real G1`.

Telemetry and recording observe each boundary. The controller's writer watchdog runs independently of camera/input and inference. The robot's verified native stop mechanism remains independent of these processes. Viewer failure cannot stall control. No consumer is allowed to bypass the final writer guard.

Proposed types in existing headers:

* `bridge_session.hpp`: extend `SkeletonSample` with session epoch, acquisition/receive/tracker timestamps, stream validity, body count, selected-ID evidence and per-joint ages; add `FrameEnvelope`, `CalibrationState`, `SessionHealth`, `LocomotionIntent`. Use SDK-neutral 32-joint storage in the pure core; translate SDK structs once in acquisition. Keep an adapter overload for existing tests during migration.
* `BridgeResult`: accepted pose, intent, source sequence/time, last valid tracking age, validity masks, rejection reason and safe state. `publish` means eligible for a configured transport, not authorization to arm.
* `SonicPoseFrame`: retain 24×3 metre positions, 21×3 local axis-angle values and wxyz root; append internal provenance and validity outside the legacy payload where necessary. 29 zero wire fields must never be interpreted as a robot neutral.
* External `robot_parameters.hpp::MotorCommand`: attach monotonic generation time, source epoch/sequence and safety result through a timestamped envelope; validate all q/dq/kp/kd/tau and mode fields. Mapping is by joint name with generated checked index tables.

### Frames, skeleton and SMPL

Use notation `T_A_B` to map coordinates in B into A. All transforms are proper rotations (RᵀR≈I, determinant +1) plus metre translations. Store quaternions as wxyz internally and explicitly convert at any xyzw interface.

| Frame | Definition / responsibility |
|---|---|
| D | Kinect depth optical origin, +x right, +y down, +z forward, SDK positions in mm. Raw skeleton belongs here. |
| C | Color optical frame, separate calibrated intrinsics/extrinsics; use SDK D↔C projection for RGB overlay. Do not assume RGB and depth pixels/axes coincide. |
| A | Intermediate axis convention from D using existing R = [[0,0,1],[-1,0,0],[0,-1,0]]. This alone does not level a tilted camera. |
| W | Surveyed local floor frame, +z against gravity, +x chosen operator forward, +y left. Estimate/store `T_W_D` using depth floor and SDK IMU extrinsics when available. |
| H0 | Neutral human reference: pelvis origin and operator heading captured in W. Calibrate heading from anatomical landmarks and a directed forward test, not raw pelvis quaternion alone. |
| Ht | Current human root frame. Root-relative joint positions remove root translation/orientation exactly once. |
| S | SONIC/SMPL convention, including neutral joint bases verified against encoder fixtures/FK. 24 joint positions and 21 non-root local rotations are related but not interchangeable. |
| B/R | Robot base/world frames defined by SONIC model and IMU heading. Robot start heading maps H0 forward into robot forward; never use camera position as robot global position. |

For positions use `p_W = R_W_D * (0.001*p_D) + t_W_D`. A change of coordinate basis for an orientation is conjugation; changing its parent/reference is quaternion composition. Keep those operations distinct. Existing `orientation_sonic` is a basis change; neutral root subtraction alone does not establish floor alignment. Validate all six axis directions, ±90° rotations, yaw branch crossing, camera tilt, anatomical left/right and tilted-camera forward walking.

SDK conventions and independent sensor frames are confirmed by [Microsoft coordinate documentation](https://learn.microsoft.com/en-us/previous-versions/azure/kinect-dk/coordinate-systems). SDK joints use millimetre positions, normalized quaternions and sensor-global orientation per [Body Tracking 1.1.2 joint reference](https://microsoft.github.io/Azure-Kinect-Body-Tracking/release/1.1.x/structk4abt__joint__t.html). Physical sign/heading must still be validated on this installation.

Calibration captures ≥60 accepted frames across ≥2 seconds, using confidence/stationarity thresholds. Fit the floor from a configured depth ROI with deterministic sampling and robust residual rejection; use gravity as a consistency check, not as a substitute for depth-camera extrinsics. Require floor residual ≤15 mm and normal disagreement ≤3° in the initial acceptance profile. Save sensor serial, mount identity, intrinsics/extrinsic hashes, floor plane, H0 heading, neutral offsets, measured segment lengths, scale, confidence statistics and version. Reject degenerate skeletons, collinear heading landmarks, implausible segment lengths or unstable neutral. Persist atomically; validate schema/hash/serial on load. A moved camera, changed mode or different person invalidates relevant calibration.

Normalize measured segment lengths by an explicit reference skeleton; keep pose rotation independent of body size. Begin with a single robust height/leg-length scale and per-segment consistency checks; do not introduce shape optimization unless measured residuals require it. Out-of-range scale is a calibration failure, not silent clamping. Validate SMPL rest bases with non-identity articulated fixtures and FK. Resolve the duplicated-neck approximation by documented interpolation of spine3/neck positions and rotation distribution, preserving the encoder contract; retain current mapping if measured FK/pose error meets the approved fixture limits.

### Timing and preprocessing

Use device time for capture interval, host steady clock for local deadlines and UTC only for human audit. Map device clock to host monotonic with offset/drift estimate and uncertainty; reject backwards timestamps, jumps and excessive uncertainty. Cross-machine monotonic values are not comparable. Initially require bridge and SONIC on one host; a future remote profile must have measured synchronization uncertainty or use receipt-age plus independently bounded transport delay and will require revalidation.

Keep one latest accepted frame per stage, bounded queues, explicit sequence/drop counters. Deadline check is not contingent on a camera call returning. `BridgeSession::process` takes supplied time and has no wall-clock reads. Filters use elapsed time: `alpha = 1-exp(-dt/tau)`. Store orientations as quaternions, smooth with shortest-arc interpolation, then serialize axis-angle continuously; avoid filtering three angle-axis components independently across π. Apply deadzones/hysteresis to intention, not to global validity.

Validate finite values, quaternion norms, segment lengths, joint velocities and per-limb age before updating history. Rejected samples cannot poison prior state. Critical lower-body/root loss inhibits whole-body imitation and walking immediately. An isolated hand/wrist loss may hold its last safe local value for ≤100 ms in an upper-body-approved mode; beyond this enter degraded hold. No indefinite extrapolation. All normal, fallback, startup, recovery and shutdown trajectories pass position/velocity/acceleration checks.

### Retargeting, body references and command safety

Keep v3→SMPL encode mode 2 and existing encoder/decoder. In standing mode SONIC generates a balanced body reference from accepted SMPL; in walking mode SONIC's planner owns feet/base/leg references and the approved upper-body reference is blended using existing merger facilities. Human pelvis movement is intention, not direct base teleportation. Disable locomotion by default; require explicit deadman and mode selection. Compute velocities in calibrated heading frame, enforce vector-norm speed as well as component limits, acceleration/deceleration limits, yaw limits, dwell/hysteresis and workspace boundary. Turning must not implicitly request translation; if policy needs a minimum translational speed, prohibit in-place turn until validated and expose the actual behavior in configuration.

Resolve 29 body joints by names using external `G1JointIndex`, policy permutations and model actuator-to-joint tables. Record three orders separately: policy DOF, SDK motor, MuJoCo qpos/actuator. Hands are excluded from active Kinect commands; the actual robot attachment mass/collision geometry must match simulation. No assumption that Dex3 and Inspire are interchangeable.

After policy action scaling/default offsets, validate q/dq/tau/gains and joint range, then generate a bounded feasible target. Intersect robot-specific verified limits, model limits and commissioned soft envelope; a mismatch blocks startup until reconciled. Never weaken manufacturer/model limits to make a fixture pass. Small numerical overshoot ≤1e-5 rad may be clamped and logged; larger raw violation rejects the command and increments a blocking counter. Track raw and applied targets so simulator clipping cannot hide defects.

Runtime feasibility uses name-mapped FK and a configured conservative set of torso/limb capsules, joint-dependent coupled constraints and contact/support envelope. Validate that conservative checker against MuJoCo collision/contact queries offline. It must evaluate the post-policy target, with maximum age 40 ms, before the writer accepts that target. Check static support margin in standing; walking uses controller balance/contact diagnostics, tilt/rate/slip and bounded workspace, not a static support polygon alone. If no trustworthy balance/contact diagnostic exists for the controller, walking readiness fails. Do not claim kinematic feasibility proves dynamic stability.

### Controller and DDS abstraction

Reuse existing `InputInterface`, `ZMQManager`, `ZMQEndpointInterface`, `MotorCommand` and SDK `ChannelPublisher`. Add a small command sink seam at `G1Deploy::LowCommandWriter` with `observe`, `compute`, `sim`, `real` modes (`--compute-only` on the preflight tool selects `compute`). Observe/compute modes never construct a motor publisher and suppress hand publication and `Stop()`'s final write. Tests use an in-memory sink. This is a seam for a concrete safety need, not a second controller framework.

SIM uses DDS domain 42 and loopback only, explicit simulator model hash, and denies physical NICs. REAL uses explicitly selected NIC/domain/robot identity and rejects the simulation build/marker and `--disable-crc-check`. Validate ownership of low-level control so competing motion services/publishers cannot issue commands. Preserve CRC generation. Robot-side watchdog/native stop behavior is a required hardware fact; never assume silence means a stable stand or damping is harmless.

### Transport, recording, visualization, observability

Keep existing `pose`, `planner`, `command` packets for compatibility; add a versioned `health` topic carrying epoch, sequence, accepted tracking time/age, readiness, confidence and mode. Add matching parser before requiring health for Kinect profiles. Correlate all topics by epoch/sequence using optional schema fields supported by the decoder; require coherent health and pose for arming, reject future/reordered/duplicate/stale sequences. A PUB send success is not connection evidence. Receiver acknowledgments/status use existing SONIC output endpoint where possible. Bound per-topic queues without applying one global conflate that could erase different topics. Flush/invalidate buffers on disconnect and new epoch.

Trace R2 uses explicit little-endian field serialization with lengths, schema versions, checksums and limits; header contains config/calibration/model/build hashes. Store raw skeleton/confidence, capture stream validity/times, selection events, all processing results, planner packet semantics, source ages, robot reference/raw/applied/measured targets, safety events and stage timestamps. Optional synchronized MKV RGB/depth recording uses SDK recording and is linked by device timestamps; skeleton-only traces remain available. A truncated last record is a failed validation with salvage explicitly separate. Writes check disk errors; full-disk recording failure disarms real runs that require evidence.

Reuse SDK `k4aviewer` for initial RGB/depth inspection, `Window3dWrapper` for optional live skeleton/SMPL overlays, SONIC MuJoCo viewer for ghost raw/applied robot targets and contact/limit overlays. Add floor axes, locked ID, rejected joints/age, limb confidence and units. No viewer runs in the control thread. Use bounded snapshots; viewer stalls drop visual frames.

JSONL events have schema/version, UTC+monotonic time, epoch/sequence, component, event/reason, old/new state, body ID, config/calibration/model hashes and applicable values/thresholds. Summaries include capture/tracker/accepted/published/received/control FPS, attempted/accepted/dropped counts per stage, p50/p95/p99/max latency, timestamp uncertainty, command/lowstate age, control deadline misses, CPU process usage/RSS and GPU utilization/memory/provider. OS/GPU sampling is ≤1 Hz outside control loops. SDK queue time and measured end-to-end delay are separately named; the existing +25 ms estimate is never gate evidence.

## 4. Safety architecture and state machines

Safety precedence: physical/native emergency stop or manual override → controller health/state loss → final command guard → source validity/tracking → requested pose/locomotion. An application stop must not depend on the same ZMQ path that failed.

Runtime supervisor states, owned across bridge health and controller authorization:

`BOOT → CONFIG_VALID → INPUT_STARTING → OBSERVE → CALIBRATING → READY_DISARMED → ARMED_STAND → ACTIVE_STAND / ACTIVE_WALK`.

Any fault leads to `DEGRADED_HOLD` (brief fresh-state recoverable interruption), `CONTROLLED_STOP` (bounded controller-owned cessation), or latched `FAULT/ESTOP`. `RECONNECTING` may restore input to `OBSERVE`, never to armed. Keep `BridgeState` source states for compatibility; add separate authorization state so READY is not a motor-enable signal.

Startup validates gate evidence/hashes and mode, creates only allowed transports, validates controller state freshness, warms inference without publishing, confirms calibration/locked ID, then requires operator arm+deadman. A Kinect gesture can request UI selection but cannot independently arm hardware. Arming is a fresh-session action and expires on any critical loss.

Shutdown first revokes motion authorization and zeroes requested locomotion through validated deceleration; a healthy controller maintains balance, then performs the commissioned stop/handover. Bound thread waits and camera cleanup. SIGINT/SIGTERM request this state machine; SIGKILL is covered by external/robot watchdog. Do not home or write a zero-joint vector on exit. Never call an unbounded join before initiating the safe controller response. The existing damping shutdown requires explicit commissioning and can cause collapse if unsupported.

Safe freeze means freeze human intent while a healthy balance controller continues a validated supported reference; it does not mean lock every joint in mid-stride. Safe neutral is a configured feasible standing reference approached only when state/contact feedback is fresh and the trajectory is feasible. If current support is unknown, do not blend toward neutral. On communication loss, local code cannot guarantee a stop command arrived: rely on verified robot-native response and physical support/stop equipment.

### Initial software acceptance budgets

These are explicit engineering acceptance targets to test, not claims of safe physical limits. Phase 12 approves a robot-specific profile; stricter values win. Missing physical values blocks real arming.

| Parameter | Initial value / enforcement |
|---|---|
| Capture | 30 FPS configured; tracker/accepted rate ≥20 Hz, no unhandled >100 ms accepted-input gap in healthy run |
| Tracking gap | Inhibit new imitation/walking when age >100 ms; controlled stop by 200 ms; invalidate calibration/identity after 1 s or immediately on identity/transform change |
| Source-to-controller age | ≤150 ms p95, ≤200 ms p99, absolute reject at 250 ms; same-host measured timestamps; uncertainty ≤5 ms |
| Final motor envelope age / lowstate age | ≤40 ms each; writer checks every 2 ms (500 Hz); policy cadence 20 ms (50 Hz) |
| Safety deadline | Detection within one guard tick after threshold; safe action requested within 20 ms; physical settling time is separately measured/approved |
| Simulation stepping | 5 ms (200 Hz), fixed seed, log wall and sim time; no silent reset/pause counted as successful standing |
| Input filtering | Initial tau ≈0.116 s (equivalent to alpha .25 at 30 Hz); tune to latency budget, never hide excessive delay |
| Intent bounds, simulation | Forward .15 m/s, lateral .10 m/s, vector .15 m/s, yaw .20 rad/s; accel .15 m/s², yaw accel .20 rad/s²; real stages start lower |
| Soft joint margin | At least .05 rad inside reconciled hard bounds unless a documented joint-specific narrower usable range makes this invalid; then explicitly review, never silently remove margin |
| Human pose jumps | Starting pelvis 0.20 m/100 ms and limb 0.15 m/33 ms gates; use dt-aware speed and segment checks for all joints; validate thresholds in recorded data |
| CPU/GPU health | No throttling/OOM; p99 inference <20 ms; no repeated >40 ms policy gaps; memory growth <5% after warmup over a 30 min run |
| Tracking recovery | ≥2 s stable valid samples, same locked identity/calibration, explicit rearm; no automatic motion replay |

Joint velocity/acceleration/effort/gain caps must be per-joint and robot/controller-specific. Do not globally impose an arbitrary low leg velocity on a balance policy: validate full controller feasibility in simulation before commissioning. Input/reference range limits and final emergency bounds are different quantities. The rollout section gives conservative human-command caps, not replacement balance gains.

Distinguish two deadlines in implementation: `tracking_gap` is host-monotonic elapsed time since the last newly accepted tracking result (fallback packets do not reset it); `source_age` is elapsed time since that result's mapped capture timestamp, including tracker/transport delay. Both checks must pass independently. An explicit critical-invalid/body-loss event inhibits immediately; an absent event is caught by the gap deadline. A 200 ms tracking-loss deadline means the controller has entered its validated stop response by then, not that a walking robot physically settles in 200 ms. Physical stopping distance/time is commissioned and used to size the cleared workspace.

### Failure behavior matrix

All rows log the common JSONL envelope plus the listed detail. Recovery never clears an ESTOP or authorizes motion automatically. `HOLD/STOP` below means the validated controller behavior above, not a fixed vector or a blind damping write.

| Failure | Detection | Fallback | Safety action | Recovery | Additional log |
|---|---|---|---|---|---|
| Kinect not detected | SDK count/serial/open failure | OBSERVE unavailable | No publisher/arm | Bounded retry 1/2/4/8 s capped 10 s; explicit device selection | SDK result, USB/serial, retry |
| Kinect disconnects | Failed capture or stream age | RECONNECTING | HOLD/STOP, revoke epoch/arm | Close/reopen same serial, validate mount, recalibrate, rearm | Last good time, capture status |
| RGB unavailable | Missing color image/age/skew | Fault required-RGB profile | Inhibit arming; active HOLD/STOP | Restart/verify synchronized streams | Stream mode, age, skew |
| Depth unavailable | Missing depth/age | No skeleton | Immediate input invalidation and HOLD/STOP | Reopen and full calibration | Depth status, invalid-pixel fraction |
| Body tracker fails | Create/enqueue/pop error, missed deadline | No accepted body | HOLD/STOP; isolate failed provider | Bounded tracker restart, same model/provider validation and rearm | Provider/model/error/queue age |
| No person | Body count zero | NO_BODY | No initial arm; active HOLD/STOP | Stable locked identity or explicit reselection | Count, last-seen ID/time |
| Multiple people | Count/selection ROI ambiguity | Keep locked ID only while unambiguous; otherwise hold | Never choose first/nearest automatically | Explicit ID confirmation when ambiguity clears | Candidate IDs, ROI/distances |
| Wrong body selected | Operator veto, identity/shape discontinuity, lost-ID replacement | Latched selection fault | Revoke arm, STOP | Explicit reselection and neutral calibration | Previous/new ID, veto evidence |
| Person leaves FOV | Critical landmarks missing/edge proximity | NO_BODY/degraded | Zero walking intention then STOP | Return, stable 2 s, recalibrate if lost >1 s, rearm | FOV margin, missing joints |
| Partial body | Per-chain validity/age | Brief approved upper-limb hold only | Disable lower-body/walking; stop if critical chain missing | Valid parent chain and recovery dwell | Mask, age, active restrictions |
| Low confidence | Per-joint threshold | ≤100 ms eligible limb hold | Reject critical skeleton, HOLD/STOP | Confidence dwell and rearm | Confidence histogram, held joints |
| Sudden skeleton jump | dt-aware position/orientation/segment residual | Last accepted intent | Reject before history update; repeated jump latches fault | Stable data/identity check and rearm | Raw/accepted deltas, threshold |
| Timestamp drift | Clock fit residual, backwards/duplicate/gap | Reject sample | HOLD/STOP; invalidate clock epoch | Refit clock with ≤5 ms uncertainty; rearm | Device/host time, drift ppm, offset |
| Invalid transform | Nonfinite, determinant/orthogonality, serial/mount mismatch | No conversion | No publish/arm | Recalibrate, validate axes/floor | Matrix/hash/residual |
| Invalid SMPL | Shape/nonfinite/norm/FK/segment validation | Last safe reference briefly | Reject; stop on deadline | Valid consistent sequence and rearm | Joint/rotation/FK discrepancy |
| Out-of-range joint | Raw and applied q vs named limits | Reject invalid command | Final writer guards all modes; STOP | Root cause fixed, rerun regression, rearm | Name/index/raw/applied/bounds |
| Impossible robot pose | FK/capsule clearance/coupled/contact envelope | Last feasible reference | STOP; prohibit infeasible transition | Feasibility restored, operator reset | Pair/clearance/support/coupled limit |
| Excess velocity | dq and finite-difference q/dt | Feasible bounded trajectory if possible | Reject if no safe bounded solution | Valid low-speed reference, rearm after fault | Joint, dt, raw/applied dq |
| Excess acceleration | (dq-new − dq-old)/dt | Bounded trajectory if feasible | Reject/STOP if incompatible with limits | Reset limiter from fresh measured state and rearm | Joint/ddq/previous accepted state |
| Retargeting failure | Inference error/NaN/deadline/FK mismatch | No new MotorCommand | Writer age watchdog, controller STOP | Reload model only disarmed; parity/regression pass | Model hash, inference error/time |
| ZMQ disconnect | Receiver heartbeat/pose age, socket monitor as diagnostic | Source unavailable | Receiver STOP regardless of publisher send success | New epoch, buffer clear, handshake, manual rearm | Last rx sequence/age/reconnect |
| SONIC/controller disconnect | Controller health/state heartbeat absent | Bridge observe only | Independent controller/native watchdog | Reconnect disarmed, lowstate/model check | Heartbeat age/process/sink mode |
| Stale command | Source age, epoch/seq and motor-generation age | No stale reuse | Reject at receiver and final writer | Fresh epoch/coherent frame, rearm | Both receive/source ages, seq |
| Process crash | Independent heartbeat expiration/process exit | Native commissioned response | Robot-side watchdog/physical stop; no laptop-only guarantee | Supported robot inspection and full startup | Exit signal, last command, watchdog evidence |
| Locomotion instability | Tilt/rate/slip/contact/tracking error/height bounds | Controller deceleration/recovery if validated | STOP/native stop if unstable; latch | Inspect robot/environment, rerun lower stage | IMU/contact/foot slip/target-state error |
| Walking target unsafe | Workspace boundary, FOV, speed/accel, obstacle/operator veto | Zero walking request via bounded decel | Disable walking, STOP if stopping corridor unsafe | Workspace cleared and explicit walk reenable | Boundary distance, stop corridor, veto |

Depth-only operation may exist as an explicit diagnostic profile but cannot satisfy the required RGB/depth KINECT_READY gate. Wrong identity cannot be perfectly detected from ID alone: locked-ID discontinuity checks and an operator deadman are both required. No obstacle-navigation capability is assumed; walking is limited to a cleared bounded test area with spotter.

## 5. Configuration architecture

One reviewed configuration is loaded into typed runtime settings. Proposed `config/teleop.json` is the source for this application and contains references to existing SONIC model/YAML files, not copies of policy constants. `config/real_g1.json` is a deny-by-default overlay with `commissioned:false` and missing physical caps represented as null, never guessed values. Read this JSON-compatible object format directly using the already installed `yaml-cpp` library in C++ (`/usr/include/yaml-cpp/yaml.h` was verified during audit); Python tooling uses standard-library `json`. `bridge_config.hpp/.cpp` handles typed schema validation, not a new text parser. Add the explicit CMake package/link requirement and a parser parity test. A small `scripts/validate_pipeline.py config` command validates/merges the same files and emits canonical effective JSON plus SHA-256. Reject unknown/duplicate keys, NaN/Inf, invalid enums/ranges, ambiguous scalar coercion and incompatible units in both loaders. YAML-specific tags, anchors, merges and multiple documents are outside the supported input contract. The C++ CLI `--config` accepts the documented source JSON directly; there is no generated intermediate configuration language or implicit subprocess launch.

Precedence: schema defaults → one explicit profile → allowed CLI diagnostic overrides. REAL rejects CLI overrides of safety/gate/model/mapping values and hashes the exact effective configuration. CLI may reduce a motion envelope while disarmed; increasing it requires revalidation. Print effective mode/endpoints/model/calibration/hash before starting. No safety hot reload while armed. Paths are explicit or relative to the configuration file; never hardcode `/home/panu` in implementation.

| Namespace | Required keys / validation | Owner |
|---|---|---|
| `device` | serial or explicit index, SDK prefix/version, USB expectation, open/retry/backoff, capture deadline | KinectInput |
| `rgb` | required, format MJPG, 720p, exposure/white balance if supported, FPS; supported SDK combination checks | KinectInput/viewer |
| `depth` | required, NFOV_UNBINNED, FPS, valid depth range/ROI, synchronization/skew budget | KinectInput |
| `tracker` | CPU/CUDA provider, model path/hash, SDK smoothing (initially 0 to avoid hidden double filter), queue bound, deadline | KinectInput |
| `selection` | explicit body ID/selection action, ROI, ambiguity policy, identity continuity thresholds, reselection latch | BridgeSession |
| `confidence` | named critical joints/chains, MEDIUM default, optional-joint hold_ms≤100, valid dwell, maximum missing mask | BridgeSession/converter |
| `frames` | `T_W_D`, anatomical basis version, robot heading registration, transform tolerance | CalibrationState |
| `calibration` | file, sensor/mount/person IDs, schema/version/hash, 60-frame/2 s minimum, stationarity, residual limits, valid age | converter/calibration tool |
| `floor` | ROI, deterministic fit seed, residual .015 m, normal error 3°, IMU agreement, origin | calibration |
| `scale` | reference segment lengths/height definition, allowed scale interval, length residual, no silent clamp | SkeletonToSmpl |
| `smpl` | mapping/rest basis version, expected 24/21 shape, quaternion norm tolerance, FK residual profile | converter/validator |
| `filter` | time constants, jitter deadzones, maximum dt/gap, limb jump speed/rotation and bone residual | converter/session |
| `retarget` | encoder/decoder/planner paths and hashes, mode 2, input scale once, body-reference interpolation/dwell | SONIC |
| `robot` | model variant, serial/firmware expected, 29 named body joints, hand attachment/inertia/collision model, policy/SDK/model permutations | controller/preflight |
| `limits` | named hard and soft q bounds, dq/ddq, tau, kp/kd bounds, coupled limits, numeric clamp epsilon, provenance | MotorSafety |
| `timeouts` | tracking=100/200 ms, calibration invalidation=1 s, input absolute age=250 ms, lowstate/motor age=40 ms, reconnect dwell | both guards |
| `locomotion` | enabled=false, deadman, standing/walking mode, enter .08/exit .04 m/s and rad/s initially, dwell .3 s, speed norm/component/yaw/accel bounds, workspace polygon, stopping margin | session/planner |
| `safety` | stage, deny publish default, native stop profile ID, watchdog limits, feasibility clearance, support margin, tilt/slip/height/tracking error bounds, controller owner | supervisor/controller |
| `zmq` | bind=127.0.0.1, port=5556, topics/versions, bounded queue, status endpoint, health required, session epoch, local clock policy | publisher/receiver |
| `controller` | root path, backend, model hashes, policy period 20 ms, writer period 2 ms, sink mode, warmup count, hand publish=false | G1Deploy |
| `dds` | sim domain=42, interface=lo; real explicit domain/NIC/robot identity; no accidental fallback, CRC=true for real | controller SDK factory |
| `mujoco` | scene path/hash, timestep=.005 s, no elastic support for gates, no reset acceptance, render=false default, seed | existing simulator |
| `real_g1` | commissioned=false default, approved stage, evidence paths/hashes, operator/session token, support rig, firmware/manual revision, native stop verification, battery/temperature thresholds | preflight |
| `record` | R2 path, exclusive creation/overwrite flag, flush cadence, image recording optional, disk minimum, required for real | recorder |
| `logging` | JSONL destination, severity, bounded queue/drop policy, rotation/disk budget, UTC/monotonic, mandatory safety events | existing logger + bridge |
| `telemetry` | sampling=1 Hz, stage histograms, CPU/GPU monitor command/provider, metadata, report path | health tool |

Real per-joint caps, operating temperature/battery limits, collision clearance and tilt/support thresholds must come from the exact robot/controller evidence and be reviewed in Phase 12. Their absence is a deliberate startup failure. Phase 8 requires a fully populated **simulation** profile and software tests for refusal of an incomplete real profile. No numerical placeholder can accidentally authorize hardware.

## 6. Sequential implementation phases

Path shorthand used only to make repeated phase references readable:

* `B/` = `src/kinect_smpl_zmq_bridge/` in this repository.
* `D/` = `../GR00T-WholeBodyControl/gear_sonic_deploy/`.
* `C/` = `D/src/g1/g1_deploy_onnx_ref/`.
* `M/` = `../GR00T-WholeBodyControl/gear_sonic/utils/mujoco_sim/`.
* `T/` = `../GR00T-WholeBodyControl/gear_sonic/tests/`.

All paths below expand to exact files. External modifications are future coordinated implementation work, not authorized edits performed by this planning task. Before editing that checkout, read its applicable instructions and obtain the normal filesystem permission if needed. Do not duplicate its controller locally to avoid a blocked edit. Each phase lists the full task set for that phase; testing and documentation inherit its single classification. New functions/types are proposed explicitly; existing symbols are named as such.

### Phase 1 — Reproducible baseline and supported entry points

**Classification:** SOFTWARE_ONLY. **Gate:** SOFTWARE_READY.
**Objective/current problem:** establish executable software evidence and prevent accidental use of legacy or hardware output; current CTest discovers nothing.
**Dependencies:** audited baseline above; no hardware.
**Files affected / modify:** root `CMakeLists.txt`, `src/CMakeLists.txt`, `B/CMakeLists.txt`, `B/README.md`, `README.md`, `AGENTS.md`, `IMPLEMENTATION_PLAN.md`. **New:** `scripts/validate_pipeline.py`, `docs/hardware_setup.md`, `docs/troubleshooting.md`.
**Classes/functions:** add `validate_pipeline.py::main`, `audit_environment`, `write_report`; no production algorithm changes yet. Register existing `bridge_session_test` and `synthetic_test` as test targets; build existing `synthetic_sim_sender` explicitly as a simulator-only tool.
**Configuration:** add explicit SDK/pure-core/legacy-visualization build options; default supported workflow is bridge observe-only, diagnostic tools never invoke real SDK channels.
**Safety:** legacy remains simulation-only; do not enable Real_Control. A test must fail if assertions are disabled; use explicit failure checks or compile test targets with NDEBUG undefined.
**Exact tasks:** (1) record commits, dirty diffs/hashes, compiler/library/model versions; (2) wire CTest, avoid silent ZMQ target omission in required builds; (3) preserve existing green fixtures; (4) inventory policy/model/backend assets and environment; (5) label historical evidence and unsupported legacy path; (6) make test scripts verify nonzero test count and no hardware publishers.
**Tests add:** build/discovery and configuration refusal cases under existing C++ tests and Python script self-check. **Tests run:** Debug and Release CMake build, `ctest --output-on-failure`, existing synthetic/session checks and external hardware-free input/sim tests.
**Expected output:** baseline report with versions, command outcomes and explicit test counts; supported build instructions.
**Failure conditions:** missing required assets/dependencies, disabled assertions, test count zero, legacy/hardware sink selected silently. **Completion:** baseline failures fixed or explicitly environmental-blocked (blocked means gate FAIL, not pass); no regressions and no hardware access.

### Phase 2 — Typed configuration and unified input contracts

**Classification:** SOFTWARE_ONLY. **Gate:** SOFTWARE_READY.
**Objective/current problem:** replace scattered constants and three divergent input paths with a deterministic source-neutral session contract.
**Dependencies:** Phase 1.
**Modify:** `B/main.cpp`, `B/bridge_session.hpp`, `B/bridge_session.cpp`, `B/skeleton_to_smpl.hpp`, `B/skeleton_to_smpl.cpp`, `B/synthetic_sim_sender.cpp`, `B/bridge_session_test.cpp`, `B/CMakeLists.txt`, root `CMakeLists.txt`, `scripts/validate_pipeline.py`.
**New:** `B/bridge_config.hpp`, `B/bridge_config.cpp`, `B/kinect_input.hpp`, `B/kinect_input.cpp`, `config/teleop.json`, `config/real_g1.json`.
**Classes/functions add:** `BridgeConfig::load/validate`, SDK-neutral joint/sample fields, `KinectInput::open/poll/close`, acquisition event enum; `select_body` in `BridgeSession`. **Modify:** `Options/parse/live/replay`, `Sensor` ownership moved into KinectInput, `BridgeSession::process`, synthetic sender `main` to call BridgeSession.
**Configuration:** implement section 5 schema/parser parity, environment profiles, explicit `--mode observe|compute|sim|real`, `--config`, `--no-publish`; omitted mode means observe. Real profile uncommissioned. Reuse installed yaml-cpp for C++ loading and Python json for tooling, with canonical effective JSON shared by controller/bridge.
**Safety:** locked selection; multiple candidates at startup require explicit selection; missing selected ID does not switch. Pure tests/replay need no physical SDK runtime initialization.
**Exact tasks:** extract acquisition only, translate SDK once, carry validity/times/IDs, inject time, consolidate synthetic fixtures and planner decision owner, reject incompatible flags before resource creation, isolate pure core target from sensor libraries, add serial-based selection and bounded queue/drop counters.
**Tests add:** input event injection (no-device, missing streams, multi-person, selected-ID disappearance), serial/config errors, same synthetic data through replay/live adapter, constructor/cleanup no-publish spy. **Run:** all Phase 1 tests plus abstraction/config fixtures in `bridge_session_test`.
**Expected output:** one session path and effective configuration/hash report. **Failure:** any source bypasses safety or initializes hardware in tests; unknown keys accepted; identity switches. **Completion:** source parity and parser/input tests pass in no-device environment.

### Phase 3 — Coordinate, floor, neutral and scale calibration

**Classification:** SOFTWARE_ONLY. **Gate:** SOFTWARE_READY; prepares KINECT_READY.
**Objective/current problem:** remove unverified camera-level/heading/stature assumptions and persist calibration without altering known-correct coordinate order.
**Dependencies:** Phase 2.
**Modify:** `B/skeleton_to_smpl.hpp`, `B/skeleton_to_smpl.cpp`, `B/bridge_session.hpp`, `B/bridge_session.cpp`, `B/main.cpp`, `B/kinect_input.hpp`, `B/kinect_input.cpp`, `B/synthetic_test.cpp`, `B/bridge_session_test.cpp`, `config/teleop.json`, `scripts/validate_pipeline.py`.
**New:** `docs/calibration.md`; calibration files are generated run artifacts, not committed personal measurements.
**Add:** `CalibrationState::validate/load/save`, `fit_floor`, `validate_transform`, `calibrate_heading`, converter calibration access/import/export, `validate_smpl`. **Modify:** `SkeletonToSmpl::calibrate/convert`, `position_m`, `orientation_sonic`, `neutral_pose`, `reset_calibration`.
**Configuration:** frame transforms, floor ROI/residual, anthropometric reference/allowed scale, serial/mount ID and per-joint confidence; refuse stale/incompatible files.
**Safety:** calibration does not publish motor commands; rotations and positions consistent, invalid scaling/fit stops progression; failed/new calibration invalidates old fallback history and authorization.
**Exact tasks:** implement section 3 frame chain; SDK D↔C projection and IMU extrinsic access; synthetic tilted-floor plane fitting; 60-frame stationarity/quality collection; segment scale estimate; validate anatomical facing sign; resolve SMPL rest basis/neck approximation against FK; atomic persisted artifact and checksum; expose diagnostic calibration/coordinate/floor commands.
**Tests add:** all six axes, proper rotation checks, tilted camera, heading reversed, mm↔m, left/right, non-identity chain FK, wrong serial/hash, degenerate/moving neutral, height outlier, reload equivalence. **Run:** synthetic/session/coordinate/calibration/SMPL matrix rows.
**Expected output:** valid synthetic calibration artifact and numerical residual report. **Failure:** ambiguity, FK tolerance exceeded, degenerate plane or identity rest fixtures falsely treated as sufficient. **Completion:** deterministic calibration and transform tests pass; physical values still unverified until Phase 10.

### Phase 4 — Motion preprocessing and fault/recovery lifecycle

**Classification:** SOFTWARE_ONLY. **Gate:** SOFTWARE_READY.
**Objective/current problem:** bounded per-joint freshness and all-path motion handling instead of indefinite holds, frame-dependent smoothing and unsafe neutral fade.
**Dependencies:** Phase 3.
**Modify:** `B/bridge_session.hpp`, `B/bridge_session.cpp`, `B/skeleton_to_smpl.hpp`, `B/skeleton_to_smpl.cpp`, `B/kinect_input.hpp`, `B/kinect_input.cpp`, `B/main.cpp`, `B/bridge_session_test.cpp`, `B/synthetic_test.cpp`, `config/teleop.json`.
**New files:** none.
**Add:** `SessionHealth`, `LocomotionIntent`, session `update_health`, `request_arm`, `request_stop`, `reset_epoch`, per-chain validity/age bookkeeping; `KinectInput::reconnect`. **Modify:** `process/timeout/fallback/reset_calibration`, `limit_pose_step`, converter filtering and `stop` signal handling.
**Configuration:** dt-based tau/deadzones, jumps, timeout hierarchy, recovery dwell, reconnect backoff, maximum bounded hold; default walking off.
**Safety:** uniform reject-before-history-update, bounded fallback/neutral, explicit arming state distinct from BridgeState; independent deadline tick despite blocked capture; no automatic reconnect rearm.
**Exact tasks:** quaternion smoothing/continuous serialization; finite/segment/kinematic checks; fresh-parent-chain handling; dt-aware velocity/acceleration for input references; zero intent and controller stop request on critical loss; recovery blending from last safe/measured reference; capture/tracker restart and epoch invalidation; bounded shutdown/resource teardown. Do not claim input limits constrain final policy motors.
**Tests add:** stationary noise, irregular FPS, π crossing, single-limb dropout, full-body loss, every reset path, NaN, sustained low confidence, disconnect/reconnect, long blocked SDK fake call, fallback derivatives and repeated SIGTERM request. **Run:** all core tests and sanitizer-enabled pure-core tests.
**Expected output:** deterministic fault/state-transition traces. **Failure:** stale joint accepted, out-of-bound fallback, event deadlock, automatic rearm. **Completion:** every source failure matrix row exercised with a fake clock and source.

### Phase 5 — Transport freshness, portable record/replay and telemetry

**Classification:** SOFTWARE_ONLY. **Gate:** SOFTWARE_READY.
**Objective/current problem:** PUB send is not delivery, native structs are not portable recordings, and existing replay does not cover planner wire determinism.
**Dependencies:** Phase 4.
**Modify:** `B/sonic_zmq_publisher.hpp`, `B/sonic_zmq_publisher.cpp`, `B/bridge_session.hpp`, `B/bridge_session.cpp`, `B/main.cpp`, `B/bridge_session_test.cpp`, `B/CMakeLists.txt`, `C/include/input_interface/zmq_endpoint_interface.hpp`, `C/include/input_interface/zmq_manager.hpp`, `C/include/input_interface/zmq_packed_message_subscriber.hpp`, `C/include/output_interface/zmq_output_handler.hpp`, `C/tests/zmq_pose_subscriber_test.cpp`, `C/tests/test_zmq_manager.py`, `config/teleop.json`, `scripts/validate_pipeline.py`.
**New:** `B/transport_test.cpp`.
**Add:** publisher `publish_health`/send result, portable R2 encoder/decoder within existing trace module, `validate_trace`, `compare_replay`, `summarize_metrics` in validator script. **Modify:** `publish_planner` to accept deterministic intent/time; `publish_command` explicit stop/epoch; `same_pose/replay` and receiver health/age checks.
**Configuration:** health required, bounds/schema/endian, topic queue depth, status endpoint, recording metadata, disk budget and fast/no-publish replay.
**Safety:** apply freshness independently at receiver; do not bless republished fallback as live human data; no global topic conflate; recordings never arm real by default; malformed/unbounded lengths rejected before allocation.
**Exact tasks:** coherent epoch/seq across messages; same-host time provenance; receiver status; stale/out-of-order rejection; socket reconnect and buffer flush; R1 compatible read/R2 write; images optional; output hashes and stage timestamps; JSONL loss/failure events and percentiles; `--replay --no-publish --fast` as validator default. Keep network latency separate from capture queue and control time.
**Tests add:** byte-level C++ publisher→actual C++ decoder, slow subscriber/drop/restart, every malformed field/size/endian, source stale but receive fresh, duplicate/reset seq, disk full/truncation, fast vs timed replay and planner equality. **Run:** core, transport test, external ZMQ tests in loopback isolated sandbox, no DDS.
**Expected output:** portable trace, transport conformance report and repeatable output hashes. **Failure:** parser crash, false-connected report, stale acceptance, replay divergence, unlogged recording loss. **Completion:** tests pass and every processing timestamp has explicit clock domain.

### Phase 6 — Retargeting contract, motor mapping, final safety and left_leg_raise

**Classification:** SOFTWARE_ONLY. **Gate:** SOFTWARE_READY.
**Objective/current problem:** policy-generated targets currently escape bridge guards; simulator clipping may hide invalid actions; original leg-raise evidence is absent.
**Dependencies:** Phase 5, external model/observation assets available.
**Modify:** `C/src/g1_deploy_onnx_ref.cpp`, `C/include/robot_parameters.hpp`, `C/include/policy_parameters.hpp`, `C/include/error_monitor.hpp`, `C/include/state_logger.hpp`, `C/include/fk.hpp` only for necessary diagnostics, `C/CMakeLists.txt`, `M/base_sim.py`, `M/unitree_sdk2py_bridge.py`, `M/configs.py`, `T/test_mujoco_sim_safety.py`, `scripts/validate_pipeline.py`, `config/teleop.json`, `config/real_g1.json`.
**New:** `C/include/motor_safety.hpp`, `C/unit_tests/test_motor_safety.cpp`, `tests/fixtures/left_leg_raise.json` (synthetic until original recovered), `tests/test_pipeline_validation.py`.
**Add:** `MotorSafety::validate/step/reset`, named mapping/limits loader using controller's existing parser, timestamped `MotorCommand` envelope, `G1Deploy::ValidateAndDispatchCommand`, `validate_joint_map`, `validate_targets`, `check_feasibility`, `validate_retargeting` in validator script. **Modify:** `G1Deploy::Control`, `LowCommandWriter`, `LowStateHandler`, `Stop`, `CreateDampingCommand` integration (preserve controller behavior until safe replacement verified), simulator raw-target telemetry.
**Configuration:** exact joint order/permutations, model/SDK limit provenance and margins, per-joint final velocity/acceleration/effort/gain limits, age/deadline checks, native stop profile, deny hand publication, no-publish sink. Existing policy weights/normalization unchanged.
**Safety:** one final guard before every DDS write; fresh measured state, valid command time and arm token required; policy/control stall cannot be masked by writer repetition. Gate both main and shutdown/INIT commands and hand writers. No arbitrary low balance-joint caps without controller validation.
**Exact tasks:** (1) name-map policy→SDK→MuJoCo and compare all limits; (2) instrument pre/post policy scaling and final writer; (3) implement raw rejection plus feasible all-path limiting; (4) add capsule/coupled/support checks with online ≤40 ms validity; (5) add observe/compute/sim/real sink seam and tests; (6) reproduce leg raise following section 7; (7) compare ONNX simulation vs actual production backend using identical observation tensors; (8) commission independent software watchdog behavior with fake sink and process-failure tests.
**Tests add:** every one-hot motor map, asymmetrical sides, nonfinite q/dq/tau/gains, range boundaries, velocity/acceleration conflicts, stale source/state/command, policy crash while writer survives, startup and shutdown no-publish, collision/coupled poses, leg raise and mirror. **Run:** existing FK/unit tests, new motor-safety executable, pipeline validator tests and hardware-free MuJoCo test. Require fixtures instead of accepting an optional FK skip for this gate.
**Expected output:** mapping/limit report, post-policy safety traces, leg-raise diagnosis/regression and production-backend parity report. **Failure:** unprotected writer, unresolved order/limit conflict, clipping hides violation, parity outside agreed tolerance or leg-raise unexplained. **Completion:** all final safety tests pass and known failure has a documented fix or explicit safely rejected envelope; requested whole-body envelope must eventually pass Phase 14.

### Phase 7 — Locomotion/body reference control and diagnostic tools

**Classification:** SOFTWARE_ONLY. **Gate:** SOFTWARE_READY.
**Objective/current problem:** divergent hysteresis and minimum turning speed, missing visual explanations and health tooling.
**Dependencies:** Phase 6.
**Modify:** `B/bridge_session.hpp`, `B/bridge_session.cpp`, `B/sonic_zmq_publisher.cpp`, `B/main.cpp`, `B/CMakeLists.txt`, `B/bridge_session_test.cpp`, `C/include/input_interface/zmq_manager.hpp`, `C/include/input_interface/streamed_motion_merger.hpp`, `C/include/localmotion_kplanner.hpp`, `C/src/g1_deploy_onnx_ref.cpp`, `M/base_sim.py`, `scripts/validate_pipeline.py`, `config/teleop.json`.
**New:** `B/bridge_viewer.hpp`, `B/bridge_viewer.cpp`, `docs/mujoco_validation.md`.
**Add:** session `update_locomotion_intent`; viewer `BridgeViewer::update/render` wrapping existing `Window3dWrapper`; validation script `monitor`, `health`, `sim_validate`. **Modify:** planner intent handling and reference merger only where tests show missing source/guard control; reuse existing policy/planner inference classes.
**Configuration:** explicit stand/walk/deadman, workspace and stopping margin, hysteresis+dwell, pure-turn behavior, optional viewer, ghost targets and metrics.
**Safety:** walking disabled by default; lower-body planner owns support; a stale SMPL or controller health event cancels walking. UI/debug tool has no independent publish capability.
**Exact tasks:** heading-aligned intent; vector speed/accel bound; stand→walk→stop reference continuity; prohibit unintended translation on turn; retain approved upper-body contribution in planner mode; add live skeleton/SMPL/floor axes and model target overlays; CPU/GPU/latency/FPS/drop monitoring; all required tools as commands listed below.
**Tests add:** lateral/forward signs at multiple headings, in-place turn, deadzone chatter, mode transitions during loss, workspace stop distance, walking unsafe, viewer snapshot stall, metrics accounting. **Run:** core/planner/transport and headless sim diagnostic tests.
**Expected output:** unified intent/report/viewer suite and documented limits. **Failure:** unintended walking, inconsistent mode, UI stalls watchdog, reference jump or invisible clipping. **Completion:** every diagnostic command has positive and negative fixtures and locomotion transitions pass software checks.

### Phase 8 — Complete hardware-free integration and SOFTWARE_READY

**Classification:** SOFTWARE_ONLY. **Gate:** SOFTWARE_READY.
**Objective/current problem:** establish reproducible full-path evidence beyond historical SIM_READY and a small unit suite.
**Dependencies:** Phases 1–7 all complete.
**Modify:** `scripts/validate_pipeline.py`, `tests/test_pipeline_validation.py`, `B/bridge_session_test.cpp`, `B/synthetic_test.cpp`, `B/transport_test.cpp`, `T/test_mujoco_sim_safety.py`, `docs/mujoco_validation.md`, `docs/troubleshooting.md`, `IMPLEMENTATION_PLAN.md` evidence ledger.
**New:** `tests/fixtures/pipeline_cases.json`, `docs/real_g1_validation.md` (procedure only).
**Classes/functions:** extend `sim_validate`, `health`, fixture generator and gate report emission; no new architecture.
**Configuration:** frozen simulation profile/checkpoints, floating-base model, deterministic seeds, domain 42/lo, elastic band disabled, resets forbidden.
**Safety:** no physical NIC/real sink; explicitly test that accidental real profile/interface is rejected; no arbitrary replay capture auto-start.
**Exact tasks:** full synthetic/replay→actual ZMQ→SONIC→DDS→MuJoCo loop; neutral/arms/crouch/safe legs/left_leg_raise/standing/walking/turn/stop/recovery; inject all relevant software failure rows at each boundary; perform 30 min soak and exact deterministic pure-core plus tolerance-bounded dynamic replays; production-backend golden observations/parity/deadline checks; emit gate manifest with all test names/results/artifact hashes and no required skips.
**Tests add/run:** all hardware-free rows in section 9, full CTest and external required test suite, integration assertions on resets/collisions/limits/FPS/latency/drop/memory. **Expected output:** SOFTWARE_READY PASS or FAIL with explicit blockers.
**Failure:** missing backend assets/dependency, required skip, untested fault, unsupported dynamic result, any reset/raw illegal target in valid suite. **Completion:** strict SOFTWARE_READY criteria below; otherwise stop advancement and fix software before Kinect validation.

### Phase 9 — Kinect discovery, RGB/depth, tracking and reconnect validation

**Classification:** KINECT_HARDWARE_REQUIRED. **Gate:** KINECT_READY.
**Objective/current problem:** SDK presence does not establish actual capture/tracking behavior.
**Dependencies:** SOFTWARE_READY, connected Azure Kinect, power/USB 3, safe observation area. No G1.
**Modify:** `docs/hardware_setup.md`, `docs/kinect_setup.md` (new), `docs/troubleshooting.md`, `IMPLEMENTATION_PLAN.md`; approved device profile entries in `config/teleop.json`. **New code/classes/functions:** none; if validation exposes a software defect, return to owning SOFTWARE_ONLY phase and rerun its tests.
**Configuration:** bind actual serial, stream modes/provider/model, firmware and USB topology, exposure settings if needed; keep observe mode.
**Safety:** no SONIC motor publisher or real DDS; successful capture never arms anything.
**Exact tasks:** run device preflight; inspect synchronized RGB/depth with SDK viewer; test CPU then intended provider; enumerate one/multiple bodies; confirm locked ID and intentional reselection; measure all stream timestamps/drops; unplug/replug 5 cycles; fail tracker/startup cases without claiming any absent hardware check passed; verify repeated start/stop cleanup.
**Tests add:** record physical observations/short raw synchronized clips and negative cases as evidence fixtures. **Run:** Kinect input/live/reconnect rows for 10 min steady tracking plus 5 restart and 5 disconnect cycles.
**Expected output:** serial/firmware/provider report, RGB/depth evidence and input failure traces. **Failure:** unsynchronized/missing required stream, ID takeover, indefinite stall, provider crash, latency outside profile. **Completion:** acquisition subcriteria pass; KINECT_READY still pending calibration Phase 10.

### Phase 10 — Physical calibration, confidence and KINECT_READY

**Classification:** KINECT_HARDWARE_REQUIRED. **Gate:** KINECT_READY.
**Objective/current problem:** synthetic geometry cannot validate anatomical orientation, floor fit or actual joint noise.
**Dependencies:** Phase 9, SOFTWARE_READY remains valid.
**Modify:** `docs/calibration.md`, `docs/kinect_setup.md`, `IMPLEMENTATION_PLAN.md`, commissioned Kinect-only tuning in `config/teleop.json`. **New:** measured calibration/recordings under chosen run directory, no new implementation files or symbols.
**Configuration:** real floor transform, heading, scale, neutral statistics, confidence and filter tuning under existing acceptance bounds.
**Safety:** observe/compute only; no robot; camera mount locked and clear full-body FOV.
**Exact tasks:** floor/neutral/heading calibration and reload; known measured distances and ±axis movement; turn-in-place without false translation; left/right arm and leg motion; deliberate occlusion/FOV exit/person crossing and identity veto; moving-camera invalidation; three calibration repeats; record min 15 min per intended mounting/provider profile. If one operator only is available, readiness is explicitly scoped to that operator; multi-operator claims require separate profiles/tests.
**Tests add/run:** calibration, coordinate, confidence/lost-joint and live Kinect test rows with physical rulers/floor reference; offline deterministic replay of all recorded traces.
**Expected output:** approved Kinect calibration/profile and KINECT_READY report. **Failure:** axis/heading error, >3° floor error, >15 mm residual, unbounded held joints, false walk intent at rest, budget violation. **Completion:** full KINECT_READY criteria pass, no required live test skipped; fix software and rerun earlier gates if code changed.

### Phase 11 — Live Kinect → SONIC → MuJoCo validation

**Classification:** KINECT_HARDWARE_REQUIRED. **Gate:** SIM_LIVE_READY.
**Objective/current problem:** live sensor variability has not been tested through dynamics/controller.
**Dependencies:** SOFTWARE_READY and KINECT_READY PASS for current hashes.
**Modify:** `docs/mujoco_validation.md`, `docs/troubleshooting.md`, `IMPLEMENTATION_PLAN.md`; simulation tuning only in `config/teleop.json`. **New code/classes/functions:** none; extend artifacts using implemented tools.
**Configuration:** mode sim, loopback/domain42, current Kinect calibration, actual floating-base SONIC model, no elastic support, production-equivalent safety envelopes.
**Safety:** robot physically/network excluded. First observe/compute, then deliberate sim arm. Loss/fault recovery must require rearm.
**Exact tasks:** neutral 5 min; each arm and symmetric arms; safe leg/crouch range including left_leg_raise; stand/walk/turn/stop at approved limits; abrupt FOV exit, occlusion, second-person crossing, USB and ZMQ/controller restarts; repeat 3 runs of ≥30 min including ≥10 min live operation each and bounded failure exercises. Compare raw/applied/measured joint histories and record end-to-end latency.
**Tests add/run:** live Kinect→MuJoCo, full dynamic safety and replay equivalence rows; rerun full software gate if tuning changes core safety behavior.
**Expected output:** SIM_LIVE_READY evidence with zero falls/resets/unapproved contact/raw-range violation in valid operation and correct injected-fault response.
**Failure:** any hidden clipping, safety violation, nuisance identity transfer, unstable recovery or budget overrun. **Completion:** gate PASS; only now may physical G1 enter an observe-only preflight loop.

### Phase 12 — Real G1 preflight and no-publish dry run

**Classification:** REAL_G1_HARDWARE_REQUIRED. **Gate:** REAL_G1_PREFLIGHT_READY.
**Objective/current problem:** robot variant/firmware, controller authority, native stop behavior and physical communication are not evidenced.
**Dependencies:** SOFTWARE_READY, KINECT_READY and SIM_LIVE_READY PASS; trained operator, spotter, approved support rig/clear area and session authorization. No application motion commands yet.
**Modify:** `config/real_g1.json`, `docs/hardware_setup.md`, `docs/real_g1_validation.md`, `docs/troubleshooting.md`, `IMPLEMENTATION_PLAN.md`. **New implementation files/classes/functions:** none; use preflight tool and command-sink spy/capture from Phase 6.
**Configuration:** record robot serial/29-DOF variant, attachments/inertia, firmware/manual revision, actual NIC/domain, CRC on, per-joint caps/provenance, native stop path, temperature/battery ranges and support/balance thresholds. Do not commission from local old XML or SDK maximum velocities alone.
**Safety:** observe-only/compute-only sink must create no motor or hand publisher, including destructors/Stop/INIT; verify via SDK spy plus network capture. Robot mechanically supported in a non-actuating state for preflight. Establish native stop/watchdog response using vendor-supported diagnostics in that state. Any test requiring actuated behavior is deferred to Stage 4 after this gate; if a necessary non-motion diagnostic is unavailable, gate remains blocked rather than bypassed.
**Exact tasks:** inspect hardware/cabling/support/firmware; read-only lowstate/IMU/health; compare measured positions with model names/units; confirm independent remote/native stop per exact firmware (do not assume a button combination); detect competing command owners; 10 min observe and 10 min compute targets with zero publishes; restart/kill process/network during no-publish dry run; validate final targets offline and run production backend golden parity/performance on deployment computer; ensure support/capture can safely contain a native stop/collapse.
**Tests add/run:** real preflight, dry-run zero-write, controller reconnect, manual veto, stale-lowstate diagnostics, signed commissioning profile validation. **Expected output:** REAL_G1_PREFLIGHT_READY PASS evidence and session procedure, still no teleoperation motion.
**Failure:** unknown stop behavior/variant/gains, incomplete caps, active competing controller, any unintended write, lowstate age violation or mismatch. **Completion:** all preflight criteria pass; first application motion only in Phase 13 with an explicit supervised arm.

### Phase 13 — Supported standing and limited upper-body validation

**Classification:** REAL_G1_HARDWARE_REQUIRED. **Gate:** REAL_G1_READY (partial evidence).
**Objective/current problem:** software/dry-run cannot prove closed-loop physical standing or stopping.
**Dependencies:** all four preceding gates PASS, fresh preflight, operator/deadman/spotter and support rig. Corresponds to rollout Stages 4 and 5.
**Modify:** `config/real_g1.json`, `docs/real_g1_validation.md`, `IMPLEMENTATION_PLAN.md`; run artifacts only otherwise. **New implementation symbols:** none.
**Configuration:** approved standing policy and stop response, walking=false, small upper-body envelope initially ±.05 rad command deviation, human-driven rate ≤.10 rad/s; balance controller maintains separately validated leg actuation bounds. Expand only within approved profile after evidence.
**Safety:** emergency stop demonstrated supported, never assume damping preserves stance; record exact physical stopping/settling time; return to no-publish after each trial.
**Exact tasks:** one 5–10 s supported standing trial; review telemetry; progress 30 s, 2 min, 5 min and three repeats; test soft stop/manual override/source loss; then single arm, other arm and symmetric ≤.10 rad deviations with conservative rate. Native E-stop tests under support per vendor procedure. Do not physically inject dangerous instability or unsupported power loss.
**Tests add/run:** standing/minimal-motion and upper-body real validation rows, compare achieved/target error, state freshness, temperatures, support/contact and actual watchdog outcome. **Expected output:** repeatable stable stance/upper-body logs and commissioned stop limits.
**Failure:** unexpected motion, contact, target error beyond profile, native stop mismatch, hard limit/gain/temperature breach, balance oscillation. **Completion:** both stages pass all three repeats; rollback to Phase 12 on any safety failure.

### Phase 14 — Lower-body, locomotion and whole-body commissioning

**Classification:** REAL_G1_HARDWARE_REQUIRED. **Gate:** REAL_G1_READY.
**Objective/current problem:** moving support and whole-body coupling remain unvalidated.
**Dependencies:** Phase 13, all gates current, explicit lower-body/whole-body session authorization. Stages 6 and 7.
**Modify:** `config/real_g1.json`, `docs/calibration.md`, `docs/real_g1_validation.md`, `docs/troubleshooting.md`, `README.md`, `IMPLEMENTATION_PLAN.md`. **New code/classes/functions:** none unless defects send work back to owning software phase.
**Configuration:** begin walking forward ≤.05 m/s, lateral=0, yaw ≤.05 rad/s, command accel ≤.05 m/s², walk deadman and clear workspace. Increment one dimension at a time up to commissioned limits no greater than tested sim envelope; no automatic expansion.
**Safety:** trainer/operator determines support/tether compatible with gait; overhead fall arrest must not supply balancing force during evidence runs. Preserve an independent stop path; reject unsafe walking target before losing stopping margin/FOV.
**Exact tasks:** supported weight shifts and small lower-body reference; validate left_leg_raise safely within approved range after its software/sim fix; slow step/stop, turn/stop, lateral only after forward stable; controlled tracking-loss stop without intentional robot destabilization; whole-body arms+standing then arms+walking; final calibration/tuning and three ≥15 min whole-body trials plus ≥30 min total active walking across successful sessions. Record all command boundaries and physical response.
**Tests add/run:** lower-body/locomotion/whole-body real test rows, stop outcomes, final complete gate report and deterministic replay of evidence. **Expected output:** REAL_G1_READY for exact supported envelope, firmware/hardware/calibration/hash bundle.
**Failure:** any fall, harness load indicating a saved fall, unapproved collision, unresolved leg case, raw invalid motor command, missed watchdog or unexplained drift. **Completion:** all stage criteria and final gate PASS, operator review recorded; publish final operating/troubleshooting procedures and restrictions.

## 7. left_leg_raise investigation and fix contract

The case is a reported failure with no recovered original fixture in this audit. Do not diagnose it from its name. Phase 6 must create a reproducible staged experiment and retain provenance (original vs reconstruction).

1. Search prior operator artifacts/log directories explicitly identified by the user and current simulation outputs. If recovered, preserve raw skeleton/SMPL, checkpoints/config, target and measured q, joint names/order, timestamps, XML and clipping/reset events. If unrecoverable, record that fact and create `tests/fixtures/left_leg_raise.json` with a neutral→slow hip/knee/ankle lift→hold→return sequence and mirrored right-side control. Add anatomically consistent non-identity rotations and bone positions, not just move the ankle point.
2. Replay at boundaries: raw Kinect D → W/H → normalized skeleton → SMPL/FK → v3 decoded motion → encoder/policy scaled action → raw MotorCommand → guarded target → MuJoCo state. Save the **first** violating joint/frame and error against both model and approved bounds. A post-clamp plot is insufficient.
3. Compare fixed-base legacy vs actual floating-base model only to expose a wrong-model mistake; the readiness run always uses SONIC scene. Check anatomical side, hip rotation signs, knee flexion convention, parent/local rotation, π discontinuity, duplicated neck effects, normalization once, policy/SDK permutation and qpos indexing with interleaved hands.
4. Distinguish mapping/calibration error; unrealistic input; learned policy out of distribution; genuine target limit; torque/PD overshoot of a legal target; contact/support instability. A legal target with physical overshoot requires dynamics/controller analysis, not wider q bounds. A raw illegal target requires pre-write rejection even if sim would clip it.
5. Make the smallest root-cause change in the module where the first divergence appears. If source pose is infeasible, explicitly reject/reduce its approved envelope and require operator feedback; if the intended realistic leg raise remains rejected, whole-body readiness stays blocked. Do not silently call blanket clamping a fix.
6. Regression requires both sides, slow/fast trajectories, near-limit targets, low-confidence knee/ankle, tilted camera and return-to-neutral. Run fixed-seed headless dynamics with support force disabled; require zero reset/fall, no raw/applied hard-limit violation, no unapproved self-contact and observed bounds within numerical tolerance. Archive before/after plots, first-divergence report and fixture/checkpoint hashes.

## 8. File-by-file implementation plan

This is the planned touch set, not a request to edit every file immediately. Shorthand expands exactly as defined in section 6. Test IDs refer to section 9. New-file reasons are explicit. Functions are assigned in the owning phase above. Tests and configuration are implementation tasks of that phase and carry its classification. Documentation is updated alongside the phase that changes its contract. No vendor tree or firmware/package asset is rewritten.

### Existing local files

| Exact file | Current role / issue | Required changes | Preserve | Tests |
|---|---|---|---|---|
| `CMakeLists.txt` | Global SDK requirement; no tests registered | Pure-core/test-only and optional sensor build, enable CTest, explicit supported options | C++17, imported SDK linkage for live target | U01/U02 |
| `src/CMakeLists.txt` | Builds legacy, bridge and renderers together | Explicit optional legacy/viewer routing; no hidden target omission | Existing vendor/helper subdirectories | U01 |
| `B/CMakeLists.txt` | Main/session test only | Register existing synthetic test/sender, core library and new test/tool files; test assertions remain active | Eigen and SDK target usage | U01/U02/U21 |
| `B/main.cpp` | Acquisition, replay, CLI, print metrics | Strict config/modes, delegate acquisition, same session for all inputs, no-publish default, calibration/tool flags, lifecycle and JSONL summaries | No auto-start, SDK error reporting, controlled resource ownership | U02/U16–U22/H01–H03 |
| `B/bridge_session.hpp` | Session/sample/result/trace contracts | SDK-neutral frame envelope, health/intent/epoch, calibration data and validity ages, authorization separate from source states | Existing session responsibility and recognizable API | U02/U04/U13/U14/U18 |
| `B/bridge_session.cpp` | Confidence/calibration/timeout/planner/trace | One identity policy, dt-aware preprocessing, calibrated frame validation, all-path bounds, explicit fault/recovery, R2 serialization and metrics | Deterministic injected time, meaningful current regression behavior | U03–U07/U12–U18 |
| `B/skeleton_to_smpl.hpp` | 24/21 SMPL type and converter | Validity/provenance/calibration access and pure-core skeleton; noncopyable or value-safe ownership for Impl | SMPL dimensions/order and wxyz root | U03/U04/U05/U06 |
| `B/skeleton_to_smpl.cpp` | Conversion/filter/scale/pose offsets | Floor/heading pipeline, scale validation, parent-chain ages, quaternion smoothing/FK-consistency validation | Tested basis transform, units, topology and local calibration principle | U03–U07 |
| `B/sonic_zmq_publisher.hpp` | Publisher interface | Deterministic intent input, explicit health/send status, bounded socket configuration | Noncopyable ownership and separate message types | U20/U21 |
| `B/sonic_zmq_publisher.cpp` | Packet encoding and planner semantics | Move intent semantics to session; coherent metadata, explicit endian, finite/bounds validation, status/drop counters | 1280-byte header and existing v3 field dimensions; loopback default | U12/U18/U20/U21 |
| `B/bridge_session_test.cpp` | Core scenario/trace assertions | Expand fault/calibration/selection/time/recovery/replay tests; explicit test failures in Release | Existing neutral/arm/loss/intent regressions | U02–U07/U12–U19 |
| `B/synthetic_test.cpp` | Unbuilt converter regression | Register; realistic articulated orientation/FK/tilt cases and malformed input | Existing handedness/unit/rotation cases | U03/U04/U06/U19 |
| `B/synthetic_sim_sender.cpp` | Standalone auto-calibrating sender | Route through BridgeSession/config, no-publish default and explicit sim guard; share existing fixture generation with tests only as needed | Useful neutral/arm/turn scenarios and loopback target | U12/U19/U23 |
| `B/README.md` | Existing live/replay/sim commands | New safe CLI, mode/gate prerequisites, truthful latency, calibrated tool and R1/R2 instructions | SDK prefix guidance and legacy command examples clearly marked historical | U22/H01–H03 |
| `README.md` | Primarily legacy fixed-base tutorial | Supported architecture/gates pointer; later verified build/setup and rollout summary | Attribution/license/original demo labeled legacy | U01/docs check |
| `AGENTS.md` | Absent at audit start; scoped instructions created by planning task | Require this plan, no hardware shortcuts, preserve tested paths and evidence | Parent git/workspace rules | Documentation review |
| `IMPLEMENTATION_PLAN.md` | This contract/evidence ledger | Update phase completion, exact results and changed hashes as implementation progresses | Gate ordering, fail-closed acceptance and historical distinction | Gate-report check |

### New local files (minimum separate responsibilities)

| Exact new file | Why it should exist / required contents | What not to put here | Tests / phase |
|---|---|---|---|
| `B/bridge_config.hpp` | Typed `BridgeConfig`, key schema and validation interface shared by bridge components | Controller policy constants or hardcoded personal calibration | U01; P2 |
| `B/bridge_config.cpp` | Strict typed configuration loading/range checks around installed yaml-cpp; same schema as Python json tool | Custom text parser, permissive unknown-key fallback, safety hot reload | U01; P2 |
| `B/kinect_input.hpp` | Concrete acquisition event/API seam to inject failure cases without hardware | New generic sensor framework | U02; P2 |
| `B/kinect_input.cpp` | Existing Sensor/live acquisition moved here; serial, image timestamps, tracker lifecycle/reconnect/SDK conversion | Retargeting, locomotion or publish decisions | U02/U15/H01; P2/P4 |
| `B/transport_test.cpp` | Runnable conformance/malformed/stale/drop regression using real encoder and receiver fixtures | Duplicate production decoder implementation | U20/U21; P5 |
| `B/bridge_viewer.hpp` | Optional viewer snapshot API separate from control | Hardware command permissions | U24; P7 |
| `B/bridge_viewer.cpp` | Reuse Window3dWrapper for skeleton/SMPL/floor/ID/age overlays | A second control/smoothing loop | U24; P7 |
| `config/teleop.json` | Source of validated device/calibration/sim/bridge settings and existing SONIC file references | Guessed real hardware limits | U01; P2 |
| `config/real_g1.json` | Explicit commissioned overlay and evidence/robot identity/caps; disabled until physical approval | Default physical NIC/arm enable or blank values interpreted as unlimited | U01/H04; P2/P12 |
| `scripts/validate_pipeline.py` | One stdlib CLI for config/report/trace/health/preflight/validation orchestration; optional installed MuJoCo/NumPy for relevant commands | New controller implementation, automatic package install, automatic hardware arm | U01/U09–U11/U17/U18/U22/U23; P1–8 |
| `tests/test_pipeline_validation.py` | Python unittest checks for tool results, gate manifests, name maps, corrupt trace/config and numerical checker | Mock-only claim of physical correctness | U01/U09–U11/U17/U18/U22; P6 |
| `tests/fixtures/left_leg_raise.json` | Reproducible, provenance-labelled input/time/expected-output case plus mirror parameters | Unlabelled invented “original” evidence | U08–U11/U23; P6 |
| `tests/fixtures/pipeline_cases.json` | Versioned deterministic case definitions for stand/walk/loss/recovery, physical-consistent skeletons | Large generated traces or private images | U19/U23; P8 |
| `docs/hardware_setup.md` | SDK/runtime/power/USB/GPU and deployment computer/robot prerequisites | An unverified claim that installation means hardware-ready | H01/H04; P1/P9/P12 |
| `docs/kinect_setup.md` | Device serial/provider/stream preflight and RGB/depth/body troubleshooting | Hardcoded personal serial in universal examples | H01/H02; P9 |
| `docs/calibration.md` | Frame equations, floor/heading/neutral procedure, persistence and invalidation | Unlabelled manual axis flips | U04/U05/H02; P3/P10 |
| `docs/mujoco_validation.md` | Floating-base sim, exact models, isolation, valid/fault tests and evidence commands | Legacy fixed-base evidence promoted to balance proof | U23/H03; P7/P8/P11 |
| `docs/real_g1_validation.md` | Firmware-specific preflight/stop procedure, stage envelopes, operator/spotter checklists/results | Universal stop button mapping or auto-home cleanup instructions | H04–H07; P8/P12–14 |
| `docs/troubleshooting.md` | Reason-code→diagnostic/recovery map; stop/rearm policy and environment blockers | Advice to disable safety to pass a gate | All negative rows; P1 onward |

Generated files go under `artifacts/<run-id>/` or an explicit external run directory: effective config, calibration, manifest, logs, traces, reports, images and target plots. They are evidence, not new source modules. Artifact manifests must not include secret credentials or unlimited private RGB retention; store only what the validation needs and document deletion/retention separately from command safety.

### Existing external files

Inspect the target checkout's local rules and preserve pre-existing modifications. These paths are actual files; they are included because the requested safety cannot be implemented entirely inside this repository.

| Exact file | Current role / issue | Required changes | Preserve | Tests |
|---|---|---|---|---|
| `C/src/g1_deploy_onnx_ref.cpp` | G1Deploy lifecycle, inference, LowStateHandler, 500 Hz writer; writer copies buffer without final guard | Timestamp/health/arm checks, sink seam, MotorSafety dispatch, bounded lifecycle and no-publish Stop, disable unmatched hands | Policy observation/action assembly and verified CRC/motor conventions | U08–U15/U21/H04–H07 |
| `C/include/robot_parameters.hpp` | 29 motor definitions/index enum/command | Typed timestamped envelope and checked names/limits provenance | G1JointIndex and existing hardware ordering | U09/U10/U21 |
| `C/include/policy_parameters.hpp` | Policy ordering/scaling/defaults | Expose read-only mapping/provenance for conformance; change numerical values only if proven incorrect | Trained policy scaling/defaults/gains | U08/U09/U23 |
| `C/include/error_monitor.hpp` | Existing controller error monitoring | Integrate source/command/state age, feasibility and latched fault reasons | Existing device error checks | U11/U14/U21 |
| `C/include/state_logger.hpp` | Controller state records | Correlated seq/time, pre/post safety target, command age and guard result | Existing measured-state fields | U22/U23 |
| `C/include/fk.hpp` | Existing FK | Expose needed named transforms/checks and finite validation; avoid algorithm rewrite | Kinematics for selected official model | U06/U08/U11 |
| `C/include/input_interface/zmq_endpoint_interface.hpp` | Stream decode/version/SMPL mode/timeout | Health/source freshness/epoch schema, local-clock comparison, invalid packet rejection and reset | v3 encode mode 2, existing buffering semantics where tested | U18/U20/U21 |
| `C/include/input_interface/zmq_manager.hpp` | Mode/planner commands | Unified coherent seq/age checks before consuming valid data, explicit deadman/stage, configured timeouts | InputInterface ownership, existing planner integration | U12/U14/U20/U21 |
| `C/include/input_interface/zmq_packed_message_subscriber.hpp` | Packed message receiver | Size/type/version/endian bounds and per-topic queue/receive metrics where absent | Existing packet parser, avoid clone | U20/U21 |
| `C/include/input_interface/streamed_motion_merger.hpp` | Existing motion merge | Enforce continuity/validity through mode transitions and invalidate prior epoch | Existing interpolation if tests pass | U07/U12/U23 |
| `C/include/localmotion_kplanner.hpp` | Existing planner interface/reference generation | Propagate guarded intent/health and expose useful balance/reference diagnostics | Planner model semantics; no replacement gait generator | U12/U21/U23 |
| `C/include/output_interface/zmq_output_handler.hpp` | Existing output channel | Extend controller status/ack/health correlated metadata | Existing supported telemetry protocol fields | U20/U22 |
| `C/tests/zmq_pose_subscriber_test.cpp` | Decoder test program | Coherent health/invalid/stale/R2 reference fixtures | Real decoder invocation | U20/U21 |
| `C/tests/test_zmq_manager.py` | Existing manager test harness | Mode/timeout/reconnect/isolation assertions | Existing known-good command format | U12/U14/U15/U20 |
| `C/CMakeLists.txt` | Deploy/test build | Add safety test and link existing parsing/kinematic utilities; backend conformance target | Real backend separated from SONIC_SIM_ORT | U01/U08/U21 |
| `M/base_sim.py` | DefaultEnv, stepping, clipping/reset/contact check | Raw/applied/measured metrics, fault-aware no-reset gate, target overlay/feasibility oracle, stale command simulation | Existing finite/reset safeguards as last-resort diagnostics, not pass conditions | U10/U11/U23/H03 |
| `M/unitree_sdk2py_bridge.py` | DDS LowCmd/state simulation | Timestamp/epoch/command-age counters and stale policy test support | Official motor/CRC/state encoding | U14/U15/U21/U23 |
| `M/configs.py` | Simulator runtime configuration | Explicit validation profile, support/reset policy and telemetry flags | Existing simulator options and non-Kinect consumers | U01/U23 |
| `T/test_mujoco_sim_safety.py` | Current safety/order/neutral test | Add raw target violations, named indices, collision/limits/loss/leg regression and dynamics assertions | Three-test baseline remains green | U09–U11/U23 |

New external files are `C/include/motor_safety.hpp` (a shared final-command guard with no middleware ownership) and `C/unit_tests/test_motor_safety.cpp` (table-driven limit/age/derivative/sink tests). Both belong to Phase 6, SOFTWARE_ONLY. Keep them small and integrate with existing controller/build types rather than creating a safety service hierarchy.

### Read-only dependencies and deliberate non-changes

`src/kinect_teleoperate_robot/main.cpp`, its `include/jointRetargeting.hpp`, `include/math_tool.hpp`, `include/StartEndPoseDetector.hpp`, and local `src/unitree_g1/{g1.xml,scene.xml}` are not the supported implementation target. Quarantine via build/docs; retain legacy behavior as historical example with known defects, not as production fallback. If later asked to support the legacy demo, treat its races/filter initialization/actuator lookup as a separate bug task.

Reuse `src/sample_helper_libs/window_controller_3d/Window3dWrapper.{h,cpp}`, `SkeletonRenderer.{h,cpp}`, `CoordinateAxes.{h,cpp}` through existing APIs; avoid editing renderer internals unless a concrete viewer requirement cannot be achieved by the new wrapper. `src/sample_helper_includes/BodyTrackingHelpers.h` supplies joint/bone references. H1, `src/extern`, `src/mujoco-3.1.5`, `lib`, `licenses`, meshes and images remain untouched.

External `C/include/control_policy.hpp::PolicyEngine`, `encoder.hpp`, `ort_sim_engine.hpp`, `localmotion_kplanner_onnx.hpp`, `localmotion_kplanner_tensorrt.hpp`, `observation_config.hpp`, and `D/policy/release/observation_config.yaml` remain model/backend sources; validate parity rather than rewrite. `M/wbc_configs/g1_29dof_sonic_model12.yaml`, `../GR00T-WholeBodyControl/gear_sonic/data/robot_model/model_data/g1/scene_43dof.xml` and its included model assets remain authoritative simulation inputs; any mismatch discovered in Phase 6 requires a documented targeted correction and gate rerun. Do not change model ranges to conceal failures. `../GR00T-WholeBodyControl/gear_sonic/scripts/run_sim_loop.py`, existing build tooling and external `unitree_sdk2` are reused as launch/runtime dependencies.

## 9. Complete test matrix

For every row, PASS means the expected result **and** all listed criteria hold. FAIL includes any contrary result, required skip, missing artifact, nondeterministic unexplained mismatch or a crash. Hardware-free tests must assert no physical DDS/channel creation; loops that need DDS use domain42/lo only. Numerical comparisons report maximum error and first mismatch, never only screenshots.

Fixed mathematical fixtures use float tolerance 1e-5 for transforms/rotations where conditioned, 1e-4 for replay SMPL values; quaternion distance is sign-invariant angular distance. Rotation orthogonality/determinant tolerance 1e-5 and normalization error ≤1e-4. Derived physical tolerances below are engineering acceptance values requiring profile review; changes must be explicit and rerun gates.

| ID / category / purpose | Input | Expected result | Pass criteria | Failure criteria | Hardware |
|---|---|---|---|---|---|
| U01 Unit/build/config | Debug/Release, malformed/valid configs, missing assets | Correct build, strict validated typed config, real denied by default | CTest count>0; all registered tests execute, invalid config nonzero exit; release checks active | Silent omitted target, zero tests, NDEBUG erases checks, permissive invalid config | None; SOFTWARE_ONLY |
| U02 Kinect input abstraction | Fake capture/tracker events, body lists, serial/start failures | SDK boundary emits deterministic valid/lost/error samples | Same events → same results; no device open in core tests; exactly one locked ID | Accidental SDK init, body takeover, unbounded queue/call | None; SOFTWARE_ONLY |
| U03 Skeleton conversion | Consistent 32-joint articulated fixtures, NaN/missing joints | Correct 24 map, parent relations, units and rejection | 100% named-map assertions, finite accepted values, invalid critical input rejected | Side/index mismatch, invented valid joint, NaN passes | None; SOFTWARE_ONLY |
| U04 Coordinate frames | ±axis translations, ±90° rotations, tilted camera, opposite heading | Explicit D→W→H→S transformations | Error≤1e-5 for exact fixtures, proper rotations, forward/left/up signs correct | Reflection, double root rotation, mm/m mismatch, camera-forward confused with body-forward | None; SOFTWARE_ONLY |
| U05 Calibration | Known floor + outliers, stable/moving neutral, serial/mount changes | Accurate robust fit and validated reload | Synthetic floor≤1 mm/0.1°, exact reload equivalence, invalid artifact refused | Moving/degenerate calibration accepted, old identity retained | None; SOFTWARE_ONLY |
| U06 SMPL | 24 positions/21 articulated rotations, root quaternions and FK fixture | Consistent encoder-ready pose | Exact fixture FK≤1e-4 m, finite/shape checks, hand/spine approximations documented and bounded | Identity-only tests, large FK residual, wrong quaternion order | None; SOFTWARE_ONLY |
| U07 Filtering/smoothing/jitter | Stationary noise, ramps, irregular dt, π wrap, fallback/recovery | Bounded smooth continuous result | Limits met every step; stationary output jitter RMS<50% of injected input; imposed .116 s filter response agrees with analytic within 5% | Phase wrap jump, dt-dependent gain mismatch, excessive delay or derivative violation | None; SOFTWARE_ONLY |
| U08 Retargeting | Golden SMPL/observations, leg/arm trajectories, model backend pair | Expected policy mode/order and feasible finite targets | Encode mode2, golden action max abs diff≤1e-3 between backends unless tighter model spec; inference p99<20 ms | Wrong mode, OOD silently accepted, backend divergence, timeout | None physical; SOFTWARE_ONLY; deployment GPU may be required |
| U09 Joint map | All 29 named one-hot motions, model qpos/actuator and SDK indices | Bijective correct mapping, hand exclusion | 29/29 names unique and mapped; physical sign expectations captured; interleaved hands handled | Missing/duplicate names, raw array-order assumption, wrong sign | None; SOFTWARE_ONLY |
| U10 Joint limits | Per-joint boundary±epsilon, invalid dq/tau/gain, derivative conflicts | Guard rejects or boundedly applies only legal commands | Raw invalid >1e-5 rejected/logged; applied within soft envelope; finite-difference dq/ddq within profile | Sim clipping hides raw violation, final writer bypass, bounds widened | None; SOFTWARE_ONLY |
| U11 Pose feasibility | Colliding/coupled/extreme/support-loss targets and safe poses | Conservative online checker vs MuJoCo oracle | No false-safe result in curated hazard suite; post-policy check fresh≤40 ms; safe fixtures accepted | Self-contact missed, static support used as walking proof, stale result | None; SOFTWARE_ONLY |
| U12 Locomotion | Heading-relative forward/lateral/turn, chatter, mode switch, boundary | Controlled standing/walking/reference transition | Bounds/dwell/deadman correct; no unintended translation on turn; stopping distance within cleared margin | Wrong sign, spontaneous mode, continued walk on unsafe target | None; SOFTWARE_ONLY |
| U13 Lost tracking/partial skeleton | Per-joint age/confidence masks, full body loss, ID switch | Brief allowed hold then disarm/stop | Critical loss inhibits within100 ms; no indefinite hold; rearm required | Stale fallback presented as fresh, automatically follows stranger | None; SOFTWARE_ONLY |
| U14 Timeout/watchdog | Fake clock around every deadline; stall input/policy/state/writer process | Correct independent deadline action | Writer guard every2 ms; motor/state age≤40 ms; stop request≤20 ms after fault threshold | Wall-clock jump changes behavior, repeated motor buffer stays enabled, deadline waits on SDK | None; SOFTWARE_ONLY |
| U15 Reconnect | Restart device stub/ZMQ/controller, seq resets and old queued packets | New epoch, flush, disarmed recovery | 10 cycles without leaks/stale command; explicit stable2 s/rearm | Automatic resume, old calibration/packet accepted, unbounded backoff | None; SOFTWARE_ONLY |
| U16 Record | Valid/invalid raw/output/image metadata, disk full and interrupted write | Versioned complete trace or explicit failure | All fields/times/hashes present, write errors visible, no silent data loss | ABI dependency in R2, missing metadata, armed real continues mandatory-log loss | None; SOFTWARE_ONLY |
| U17 Replay | R1 compatible file, R2, corrupt header/length/checksum/truncation | Correct no-publish playback, deterministic errors | Exact record count; parser rejects invalid file before control; no DDS | Crash/unbounded allocation, automatic publish, silent truncation | None; SOFTWARE_ONLY |
| U18 Deterministic replay | Same trace/config twice, fast and timed, golden controller observations | Same decisions/intent and bounded numerical outputs | Pure-core state/events/seq exactly match, poses≤1e-4; planner semantic packets identical after rebasing transport time; controller action≤1e-3 | Publisher wall time changes intent, config mismatch ignored, only one joint compared | None; SOFTWARE_ONLY |
| U19 Synthetic input | Neutral, arms, crouch, legs, gait intent, all faults | Same production session path as replay/live adapter | Covers all scenarios with realistic bone/orientation consistency; seeds repeat | Auto-calibrate bypass, synthetic-only safety logic | None; SOFTWARE_ONLY |
| U20 ZMQ | Actual C++ encode/decode, malformed field/endian, reordered/stale/drop, subscriber restart | Bounded coherent accepted stream | Topic schema/seq/epoch/age correct; no stale accepted; PUB send not reported as delivery | Slow-joiner falsely ready, old queued packet arms, cross-topic conflation | None; SOFTWARE_ONLY; loopback |
| U21 SONIC/controller interface | Mock command sink, real decoder, INIT/Stop, fresh/stale LowState, wrong profile | Correct mode2/29-body contract and no-publish protection | Observe/compute creates zero motor/hand publishers; final guard every dispatch; real denies sim CRC bypass | Any write on import/cleanup, wrong variant/hand activation or age bypass | None; SOFTWARE_ONLY |
| U22 Logging/performance/health | Known counters/time distributions, stall/OOM/error injection | Accurate report/reasons | Counts reconcile, p95/p99 match known distributions, bounded logger, CPU/GPU report provider | Estimated latency passes as measured, missing drops, logging stalls control | None; SOFTWARE_ONLY |
| U23 MuJoCo + hardware-free E2E | Complete synthetic/replay→ZMQ→SONIC→DDS→floating-base simulation, valid/fault suite | Stable accepted trajectories and fail-safe fault response | 30 min soak; no valid-run fall/reset/unapproved contact/raw illegal target; measured joint overshoot≤.01 rad and under commissioned tolerance; timing budgets met | Hidden reset/support force, input moves wrong body, physics starts only when convenient without reporting, fault continues motion | None physical; SOFTWARE_ONLY |
| U24 Visualization | Snapshot skeleton/SMPL/raw/applied/measured robot, viewer stalls | Correct axes/labels/IDs and independent control | Sample selected joint coordinates match logs; floor and units labelled; 10 s stalled viewer doesn't delay guard | Misleading side/root display or control thread blocked | None for replay; SOFTWARE_ONLY |
| H01 Live Kinect/RGB/depth | Physical camera, CPU/intended provider, occlusion, 5 reconnects | Synchronized measured streams and explicit failures | 10 min tracking plus cycles; required streams valid≥99% healthy frames, accepted≥20 Hz, no unintended ID transfer, bounded reopen | Missing required stream, excessive skew>one frame, provider failure or unbounded reconnect | Kinect; KINECT_HARDWARE_REQUIRED |
| H02 Physical coordinate/calibration/confidence | Measured floor/heading/body movement, repeated neutral, multiple people/FOV exit | Correct anatomical/metric mapping and loss protection | Floor residual≤15 mm/normal≤3°; known distance scale error≤5%; stationary wrist RMS≤20 mm and pelvis≤15 mm; correct signs; 3 repeated calibrations scale within3% and heading within3° | Wrong side/heading, bad scale, invalid confidence recovery, false walking at rest | Kinect; KINECT_HARDWARE_REQUIRED |
| H03 Live Kinect→MuJoCo | Live full-body/locomotion plus deliberate sensor/network loss | Stable real-time simulation and safe stop/rearm | 3×30 min, section4 budgets, no valid-operation fall/reset/unapproved contact/raw limit violation; all injected faults correctly handled | Any gate violation, missing correlated timing, simulator rescues bad policy | Kinect; KINECT_HARDWARE_REQUIRED |
| H04 Real G1 preflight/dry-run | Supported robot read-only state, production backend, observer+compute-only, native non-actuating diagnostics | Exact variant/mapping/stop/profile evidence, no commands | 10 min observe+10 min compute, zero outgoing motor/hand writes including startup/exit; fresh lowstate≤40 ms; all profile provenance and stop capability established | Unknown native safety response, incomplete profile, competing writer, any unintended publish | G1; REAL_G1_HARDWARE_REQUIRED; no application motion |
| H05 Real minimal standing | Supported commissioned standing/stop, 5 s→5 min trials | Stable controlled stance and verified independent stop | 3 repeats, no unsafe excursion, measured stop time within commissioned bound; target tracking RMS≤.10 rad and max≤.20 rad or tighter per-joint approved bounds | Collapse caught by rig, unexpected step, missing stop acknowledgment or excess error | G1 + Kinect; REAL_G1_HARDWARE_REQUIRED |
| H06 Real upper-body | Small one-arm/both-arm trajectories with standing balance | Smooth bounded movement, no support loss | 3×5 min, ≤approved range/rate, H05 tracking bounds, no prohibited contact/temperature/effort excursion | Range violation, balance oscillation, unsupported error recovery | G1 + Kinect; REAL_G1_HARDWARE_REQUIRED |
| H07 Real lower-body/whole-body | Gradually expanded weight shift/leg raise/walk/turn/stop/full body | Safe commissioned teleoperation envelope | 3×15 min whole-body +≥30 min cumulative walking, no falls/harness catches/hard-bound violations; all stop/deadman/loss trials meet physical profile | Any unsafe target accepted, unexplained drift, false body takeover, unresolved requested movement | G1 + Kinect; REAL_G1_HARDWARE_REQUIRED |

Unit tests are focused deterministic numerical/state cases in existing test executables, not one mock suite per new function. Python tooling uses `unittest` unless extending existing external pytest files. Use actual parser/policy/model components in boundary/E2E tests so mocks cannot prove false compatibility. ASan/UBSan cover pure core/parsers; a targeted thread-sanitizer check covers new shared safety/snapshot state when the platform supports it. A missing required sanitizer/runtime check must be recorded as a software gate blocker or replaced by an explicitly documented equivalent, not silently skipped.

Integration fixture cases include neutral, left/right/both arms, crouch, spine rotation, left/right leg raise, yaw crossing ±π, forward/lateral intention, start/stop/deadman, occluded wrist/knee, all low confidence, person switch, device-time reset, NaN/Inf, malformed packet, full disk, delayed source, policy stall and shutdown. Functional passes alone do not substitute for continuous timing evidence.

### Commands and test execution contract

Current verified baseline commands are in sections 2 and 15. The following are **planned** commands to implement in Phases 1–8; do not imply they work now:

```bash
cmake -S . -B build -DKINECT_BRIDGE_ONLY=ON -DCMAKE_BUILD_TYPE=Debug -DCMAKE_PREFIX_PATH="$KINECT_SDK_PREFIX"
cmake --build build -j2
ctest --test-dir build --output-on-failure
python3 -m unittest discover -s tests -p 'test_*.py'
python3 scripts/validate_pipeline.py health --config config/teleop.json --gate SOFTWARE_READY --report artifacts/software/report.json
```

Phase 1 must retain required ZMQ include/library arguments where the environment lacks development symlinks, and expose pure-core build mode for no-SDK CI. `health` aggregates results but never grants a gate from source presence or merely invokes `--help`. External tests run from `../GR00T-WholeBodyControl` with its supported environment and `PYTHONPATH=.:external_dependencies/unitree_sdk2_python`; avoid installing/changing global packages as part of gate evaluation.

### Determinism levels

1. Raw trace→session/calibration/SMPL/intent: exact event/state equality and specified float tolerance; fake clock; no hidden wall time.
2. Wire semantics: deterministic payload values/metadata modulo explicitly rebased delivery timestamps; same planner decision and source freshness.
3. Fixed observations→ONNX/TensorRT actions: pinned assets/backend versions; within golden tolerance, not promised bit identity across GPUs.
4. Closed-loop MuJoCo: fixed timestep/model/seed/start state and recorded command schedule; identical backend requires repeatable outcome and state error≤1e-4 for offline deterministic command replay. Live asynchronous full-loop comparison uses event outcomes and documented trajectory tolerance (initially joint RMS≤.02 rad, base-position RMS≤.02 m across matched simulated time); divergence that changes safety decisions fails. Real hardware replay checks constraints/outcomes, never expects bitwise identical motion.

## 10. Strict readiness gates

Every gate produces `artifacts/<run-id>/gate.json`: gate name/status, UTC time, operator where applicable, repo HEAD+dirty-file hashes, binary/compiler/backend/model/SDK versions, config/calibration/robot identifiers, command/test IDs and counts, raw artifact paths/checksums, numerical metrics, failures/skips, approved envelope and prerequisite evidence hashes. Gate evaluation fails if any required item is absent. Evidence is reviewed, not just a writable `ready=true` switch. Real arming requires an explicit session arm action in addition to matching reviewed evidence; copying a gate file is not authorization.

Changes to transforms/mapping/model/filter/limits/transport/controller invalidate SOFTWARE_READY and downstream gates. Changing device mount/serial/provider invalidates KINECT_READY onward; firmware/robot/attachment/controller backend changes invalidate the relevant software parity/sim and physical gates. Documentation-only typo changes do not invalidate numerical results. Critical safety faults revoke the active session and require regression plus stage rollback.

### SOFTWARE_READY

**Prerequisites:** Phases 1–8 complete; current full software and policy assets available; build/run profile isolated from physical networks. Everything feasible without physical Kinect/G1 is implemented, including real-output guards tested with fake sink, actual production backend parity on available compute, calibration/input stubs and hardware procedure tooling.

**Required tests:** U01–U24, all existing relevant baseline tests, CTest nonzero count, actual ZMQ decoder, guarded controller writer, floating-base full E2E and all software fault rows.

**Exact pass criteria:** 100% required tests pass, zero required skips, no failing sanitizer/validation check; valid 30 min sim soak has zero resets/falls/unapproved self-contact/raw illegal commands; source/command/state ages and latency/FPS budgets meet section 4; deterministic outputs meet section 9; 29/29 joint names/orders/bounds verified; left_leg_raise diagnosed and safely reproduced; unknown/invalid real profile rejected; observe/compute generate no motor/hand commands. All software diagnostics and documentation exist with positive/negative checks.

**Fail criteria:** any missing executable/asset/backend test, unexplained policy limit error, mock-only boundary coverage, optional fixture skip used as FK evidence, unmet throughput or gate artifact unavailable.

**Blockers:** missing runtime/SDK development files (for live target compilation), model checkpoints or production backend compute, inability to modify external writer/decoder, unresolved leg case/model mismatch. These are honest blockers, not reasons to label software ready. No physical Kinect/G1 is required for this gate.

### KINECT_READY

**Prerequisites:** SOFTWARE_READY PASS, Phases 9–10 complete, actual serial/provider/mount profile.

**Required tests:** H01/H02, live device discovery and synchronized RGB/depth/body, explicit body selection/multi-person/occlusion, timestamp/clock fit, reconnect, calibration persist/reload, latency/performance and replay of physical traces.

**Exact pass criteria:** 10 min acquisition and ≥15 min calibration/confidence observation; all 5 unplug/replug and 5 startup cycles recover boundedly and disarmed; required streams ≥99% in healthy intervals, accepted skeleton ≥20 Hz; live physical coordinate/scale/floor/repeatability tolerances in H02; stationary motion does not trigger walking; no wrong body accepted; all section4 timing limits satisfied for hardware/provider profile; trace replay passes; real G1 excluded.

**Fail criteria:** wrong axes/scale, silent source switch, required RGB/depth unavailable, stale-joint control, uncertainty>5 ms, timeout/reconnect behavior fails, calibration does not match mount.

**Blockers:** absent Kinect/power/USB3/permissions, unworkable SDK provider, insufficient full-body view or measured compute latency. Simulated camera results do not clear these blockers.

### SIM_LIVE_READY

**Prerequisites:** SOFTWARE_READY + KINECT_READY PASS, Phase 11 complete, exact official SONIC model and isolation profile.

**Required tests:** H03 and its negative cases; neutral/arms/legs/left_leg_raise/standing/walking/turn/stop; comparison of raw/applied/measured robot targets; real-time record/replay and health reports.

**Exact pass criteria:** three ≥30 min runs including ≥10 min live operation each, one current calibration/profile; zero valid-operation falls/resets/unapproved contact/raw limit violations, no elastic support, tested limits/velocity/acceleration and feedback envelope respected; all healthy timing/FPS budgets met, fault stop and rearm behavior pass every time; no stale backlog replay, unintended translation/identity swap, final targets protected before simulation clipping.

**Fail criteria:** any safety failure, excessive tracking error, missed watchdog, measured latency replaced by estimate, evidence from fixed-base legacy model or supported-by-band simulation only.

**Blockers:** live instability, unresolved policy/frame issues, model/real backend parity gap, performance deficit. Real G1 remains excluded until this gate passes.

### REAL_G1_PREFLIGHT_READY

**Prerequisites:** all three prior gates valid; Phase 12 complete; actual supported robot and exact variant/attachments/firmware; trained operator/spotter/support rig and explicit preflight authorization.

**Required tests:** H04; real read-only communication/state age, commissioned config completeness, independent stop/manual override/native watchdog verified via vendor-supported non-actuating diagnostics, output-sink/no-publish lifecycle checks, production backend parity/performance, competing controller detection and network/profile isolation.

**Exact pass criteria:** ≥10 min observe +≥10 min target computation with **zero outgoing motor/hand commands** from this application, including INIT/Stop/crash recovery; every lowstate/motor identity mapping correct and ≤40 ms lowstate age; caps/model limits reconciled; valid CRC required; native stop/watchdog procedure and support consequence documented for exact firmware; physical operating area and stopping margin checked; gate hashes match deployment binaries and calibrated camera; manual arm remains off. Tests requiring actuation are not silently included as preflight passes.

**Fail criteria:** any unintended command, incomplete stop guarantee/procedure, stale data, wrong SDK/model/variant/hand configuration, missing real caps or independent safety mechanism, command ownership unresolved.

**Blockers:** no robot/state connection, unsupported variant, manufacturer safety/limits information unavailable, non-actuating diagnostic capability insufficient to establish preflight safety, unqualified/unavailable operator or support equipment. Stop here; do not perform a “tiny motion” to bypass the blocker.

### REAL_G1_READY

**Prerequisites:** four prior gates valid; Phases 13–14 and rollout Stages 4–7 complete; current session physical inspection.

**Required tests:** H05–H07, commissioned stop/deadman/manual override/tracking-loss recovery, lower-body leg regression and whole-body real trials, post-run record/replay/telemetry review, final calibration/tuning/repeatability.

**Exact pass criteria:** three successful standing/upper-body repeats, three ≥15 min whole-body trials, ≥30 min cumulative walking at commissioned envelope; zero falls/harness catches/unapproved collisions/hard-range excursions/unsafe accepted targets; tracking errors and effort/temperature/tilt/slip/stop times within exact approved profile; all watchdog/override/loss trials respond as specified; correct recovery requires deliberate rearm; complete evidence and operator review identify supported limitations.

**Fail criteria:** any unresolved safety incident, original intended motion still unavailable without explicit scope restriction, latency/performance drift, inconsistent stopping, omitted failure trial or incomplete evidence.

**Blockers:** physical instability, safety path not reliable, robot fault/environment changes, inability to complete controlled repetitions. Label current achieved gate, never infer REAL_G1_READY from code completion.

## 11. Required diagnostic tools and CLI contract

Reuse existing binaries/modules. The commands below are **future planned interfaces** unless explicitly marked existing. They must be implemented in the phases that own them and documented in `--help`; no separate script per diagnostic. Let `BRIDGE=build/kinect_smpl_zmq_bridge`; all examples are run from this repository. `--report PATH` produces machine-readable JSON and a concise result. Common exit codes: 0 PASS, 1 validation FAIL, 2 configuration/usage error, 3 BLOCKED/missing resource. BLOCKED never counts as PASS. View-only tools report their observation session status; a window opening is not a readiness pass.

| Tool / phase and classification | Purpose and inputs | Outputs | Planned CLI | Pass / fail |
|---|---|---|---|---|
| Kinect preflight / P2 software implementation, P9 KINECT_HARDWARE_REQUIRED execution | SDK prefix, config, serial, USB and model/provider | Device/SDK/stream/provider report | Offline/blocking check: `python3 scripts/validate_pipeline.py kinect-preflight --config config/teleop.json --report artifacts/kinect.json`; only after SOFTWARE_READY, physical execution adds `--bridge /tmp/kinect-p2-hw/kinect_smpl_zmq_bridge --duration 600 --trace artifacts/kinect/phase9.trace --execute` | PASS only real open/stream checks satisfy H01; without `--execute` or without a device BLOCKED; missing/incompatible component FAIL |
| RGB/depth viewer / P9 KINECT_HARDWARE_REQUIRED | Physical camera; existing SDK `k4aviewer` and configured stream settings | Synchronized image display + linked capture metadata from preflight | Existing `$KINECT_SDK_PREFIX/bin/k4aviewer`; then `$BRIDGE --config config/teleop.json --mode observe --view rgb-depth --record artifacts/streams.trace` | H01 stream timestamps/drops pass; missing/skewed image FAIL; viewer alone not PASS |
| Live skeleton viewer / P7 SOFTWARE_ONLY tool, P9 live execution | FrameEnvelope/confidence/ID/calibration | 3D labeled raw/normalized bones, selected ID and ages | `$BRIDGE --config config/teleop.json --mode observe --view skeleton` | Same selected coordinates as logs, no publisher; missing axes/selection overlay or blocking UI FAIL |
| Calibration / P3 SOFTWARE_ONLY, P10 measurement | Depth ROI, serial, neutral skeleton, known scale/heading | Versioned artifact, residual report | `$BRIDGE --config config/teleop.json --mode observe --calibrate all --calibration-out artifacts/calibration.json` | U05/H02 tolerances and all metadata; invalid fit/serial FAIL |
| Neutral pose / P3/P10 | ≥60 accepted stable frames over2 s | Neutral offsets/segment lengths/stats | `$BRIDGE --config config/teleop.json --mode observe --calibrate neutral --calibration-out artifacts/calibration.json` | Valid quality/repeatability; moving/partial collection FAIL |
| Coordinate-frame validation / P3 SOFTWARE_ONLY | Synthetic or recorded raw→normalized data and calibration | Named axes/sign/unit residual report | `python3 scripts/validate_pipeline.py coordinates --trace artifacts/input.trace --calibration artifacts/calibration.json --report artifacts/frames.json` | U04 numerical and H02 physical signs; wrong hand/scale/basis FAIL |
| Floor alignment / P3/P10 | Depth/IMU extrinsics/ROI, stationary floor | Plane/normal/residual visualization and artifact | `$BRIDGE --config config/teleop.json --mode observe --calibrate floor --calibration-out artifacts/calibration.json` | H02 fit/gravity agreement; degenerate/nonstatic/missing depth FAIL |
| Latency monitor / P5 SOFTWARE_ONLY | Correlated bridge/controller/sim JSONL | Per-stage p50/p95/p99/max/uncertainty | `python3 scripts/validate_pipeline.py monitor --metrics latency --logs artifacts/run --report artifacts/latency.json` | Section4 measured budgets; uncorrelated clock/estimate only FAIL |
| FPS/drop/performance monitor / P7 SOFTWARE_ONLY | Counters, monotonic times, CPU/GPU samples | Rates, drop causes, resource/deadline report | `python3 scripts/validate_pipeline.py monitor --metrics fps,drops,cpu,gpu --logs artifacts/run --report artifacts/perf.json` | Counts reconcile and section4 budgets; missing interval/counter FAIL |
| Joint-limit validator / P6 SOFTWARE_ONLY | Named raw/applied/measured targets, robot profile/model | First violation and full per-joint ranges/derivatives | `python3 scripts/validate_pipeline.py joint-limits --targets artifacts/targets.jsonl --config config/teleop.json --report artifacts/limits.json` | U10 zero unexpected violations; unnamed order/invalid profile FAIL |
| Pose feasibility checker / P6 SOFTWARE_ONLY | Target sequence + same hashed robot geometry/contact profile | FK/collision/coupled/support report and pairs | `python3 scripts/validate_pipeline.py feasibility --targets artifacts/targets.jsonl --config config/teleop.json --report artifacts/feasibility.json` | U11 agreement/clearance; missed hazard/unknown geometry FAIL |
| SMPL validator / P3 SOFTWARE_ONLY | Trace/SMPL arrays and reference skeleton/rest bases | Dimensional/norm/segment/FK residual report | `python3 scripts/validate_pipeline.py smpl --trace artifacts/input.trace --config config/teleop.json --report artifacts/smpl.json` | U06 residuals valid; malformed/inconsistent arrays FAIL |
| SMPL viewer / P7 SOFTWARE_ONLY | Same trace/calibration | Root/local rotations/bones overlaid with raw input | `$BRIDGE --config config/teleop.json --replay artifacts/input.trace --no-publish --view smpl` | Display coordinates equal validated trace, no control writes; missing joints/axes FAIL |
| Robot target viewer / P7 SOFTWARE_ONLY | Raw/applied/measured target log and official model | Ghost overlays, red limit/contact markers | `python3 scripts/validate_pipeline.py view-targets --targets artifacts/targets.jsonl --config config/teleop.json` | Name-mapped model/targets match; no hardware sink; mismatch FAIL |
| Retargeting validator / P6 SOFTWARE_ONLY | SMPL fixture + exact encoder/decoder/backend/model | Policy observations/actions, mapped q, feasibility report | `python3 scripts/validate_pipeline.py retarget --case tests/fixtures/left_leg_raise.json --config config/teleop.json --report artifacts/retarget.json` | U08–U11 pass; raw violation/unknown model FAIL |
| Replay validator / P5 SOFTWARE_ONLY | Trace and pinned config | Record counts/recomputed output comparison | `$BRIDGE --config config/teleop.json --replay artifacts/input.trace --no-publish --fast --report artifacts/replay.json` | U17/U18; corrupt/truncated/divergent data FAIL |
| Deterministic replay checker / P5 SOFTWARE_ONLY | Same trace and two run manifests | Canonical output/decision diff and hashes | `python3 scripts/validate_pipeline.py deterministic-replay --trace artifacts/input.trace --config config/teleop.json --repeat 2 --report artifacts/determinism.json` | Section9 determinism levels satisfied; discrepancy FAIL |
| End-to-end health / P8 SOFTWARE_ONLY | Effective config, prerequisites, executable test outputs | Aggregate gate status with linked evidence | `python3 scripts/validate_pipeline.py health --config config/teleop.json --gate SOFTWARE_READY --report artifacts/software/gate.json` | All required tests non-skipped and evidence current; missing test/blocker FAIL/BLOCKED |
| MuJoCo live validation / P8 tool SOFTWARE_ONLY, P11 KINECT_HARDWARE_REQUIRED execution | Live Kinect, exact sim model/controller profile | Correlated trace, targets, dynamics/timing gate report | `python3 scripts/validate_pipeline.py sim-live --config config/teleop.json --duration 1800 --report artifacts/sim-live/gate.json` | H03 pass with loopback isolation; any reset/limit/timing failure FAIL |
| Real G1 preflight / P8 tool SOFTWARE_ONLY, P12 REAL_G1_HARDWARE_REQUIRED execution | Four-mode sink code, first3 gate artifacts, supported robot/read-only state, native stop evidence | Zero-publish capture, profile completeness and preflight gate | `python3 scripts/validate_pipeline.py real-preflight --config config/real_g1.json --observe-only --duration 600 --report artifacts/real-preflight/gate.json` followed by `--compute-only --duration 600` | H04 + signed operator review; any outgoing motor/hand write FAIL; missing stop/limits BLOCKED |

For combined hardware tools, implementation is SOFTWARE_ONLY in its owning software phase; physical execution is a separate task in the listed hardware phase. This is not dual classification of one task. The tool must print which phase/classification and whether it will open a device/network before running. `real-preflight` cannot offer an auto-arm option. Real rollout commands are deliberately session-gated through the commissioned controller CLI; documenting a universal ready-to-run motor command before exact robot/profile validation would bypass this plan.

RGB/depth image recording/view decoding belongs to the acquisition/viewer path using SDK facilities; it is not sent through SONIC pose packets. If MJPG display needs a decoder, use the already available SDK viewer or existing installed image tooling rather than adding an image library to the safety core.

## 12. Safe real-hardware rollout stages

Stages describe operational capabilities, not an alternative phase order. Before SIM_LIVE_READY, “observe” means input/simulation telemetry only; physical G1 telemetry first appears in Phase 12. No robot actuation occurs at Stages 0–3. Stage 4 is the first application motor command and requires SOFTWARE_READY, KINECT_READY, SIM_LIVE_READY and REAL_G1_PREFLIGHT_READY all PASS. Stage progression requires a reviewed report, not elapsed time alone.

Common data recorded at every stage: mode/stage, run ID, exact source/controller/model/config/calibration/robot versions, operator decisions, source/receiver/controller ages, frame/body IDs/confidence, all state transitions/faults, target/raw/applied/measured values where present, timing/rates/drops/resources and gate links. Stage 0 records software-only fields; missing physical fields are marked not applicable, not synthesized. Emergency abort at any stage revokes authorization, applies the commissioned response, preserves logs, and returns to observe/disarmed after inspection; no automatic repeat.

### Stage 0 — No robot commands

**Classification/task owner:** SOFTWARE_ONLY, Phases 1–8. **Prerequisites:** none beyond baseline/build prerequisites. **Enabled:** pure core/unit/config/calibration fixtures, synthetic/replay validation and offline viewers. **Disabled:** real DDS/robot connection, all motor/hand writes and live control; network decoder tests loopback only.
**Safety/command limits:** command count exactly zero; output sink is in-memory; constructor/destructor/exception paths included. **Abort:** any network hardware initialization, missing safety config, invalid fixture accepted. **Log:** common data plus test results/seeds/negative cases. **Duration:** per-case bounded tests, repeat deterministic suite twice, parser fuzz corpus bounded to a CI time budget. **Pass:** all applicable software rows green and no hardware access. **Rollback:** restore no-publish test profile, fix owning phase and rerun; preserve user work.

### Stage 1 — Observe-only telemetry

**Classification/task owner:** input observation in Phases 9–10 is KINECT_HARDWARE_REQUIRED; the distinct robot observation task in Phase 12 is REAL_G1_HARDWARE_REQUIRED. **Prerequisites:** SOFTWARE_READY for Kinect; all first3 gates for real G1. **Enabled:** RGB/depth/body/calibration viewers/recording; robot lowstate read-only only during preflight. **Disabled:** publishing pose to an armed controller, motor/hand publisher construction, walking/arming.
**Safety/command limits:** zero motor/hand commands; observation startup/shutdown cannot alter robot mode. **Abort:** identity ambiguity, missing stream, clock drift, stale physical state, unexpected outgoing command. **Log:** device and body selection plus read-only physical mode/state and zero-publish evidence if G1 present. **Duration:** ≥10 min live input; preflight ≥10 min physical observe. **Pass:** H01/H02 applicable measurements and H04 observation portion pass. **Rollback:** disconnect observer, return Stage0 diagnostics; hardware stays in approved supported native state.

### Stage 2 — Compute targets, do not publish

**Classification/task owner:** synthetic/offline target computation P6–8 SOFTWARE_ONLY; separate live input P10 KINECT_HARDWARE_REQUIRED; separate real-state dry-run P12 REAL_G1_HARDWARE_REQUIRED. **Prerequisites:** final guard/sink tests pass; physical G1 only after SIM_LIVE_READY. **Enabled:** full conversion, retargeting, planner intent and final safety computations; actual lowstate consumption only in approved preflight. **Disabled:** all motor/hand writes, automatic mode takeover and startup/cleanup motion.
**Safety/command limits:** publish count zero; compute within same named limits as proposed stage; impossible targets are rejected and logged. **Abort:** wrong mapping, nonfinite/unsafe target, unexpected publish or unknown model. **Log:** every pre/post policy/raw/applied target and named rejection, lowstate age, packet capture. **Duration:** ≥10 min physical compute dry-run plus complete fixture corpus. **Pass:** U08–U11/U21 and H04 computation portion; all target safety violations understood and correctly rejected. **Rollback:** Stage1, retain trace for offline diagnosis; do not try a small motion to diagnose.

### Stage 3 — Publish to simulation only

**Classification/task owner:** P8 SOFTWARE_ONLY synthetic/replay; P11 KINECT_HARDWARE_REQUIRED live. **Prerequisites:** SOFTWARE_READY before live run, KINECT_READY for live input; simulation fixtures used to build SOFTWARE_READY remain isolated. **Enabled:** bridge ZMQ, SONIC policy/planner, domain42/lo SDK, floating-base MuJoCo and safety telemetry. **Disabled:** real NIC/domain, physical motor/hand endpoints, elastic support as gate evidence and accepted automatic reset.
**Safety/command limits:** section4 sim caps and full final MotorSafety; hands inactive. **Abort:** fall/reset/collision/raw illegal target, stale acceptance, missing watchdog or real network selection. **Log:** common data + contacts/support/base height/tilt/raw and applied torque and q; model/sim timestamps. **Duration:** 30 min software soak then 3×30 min live campaign. **Pass:** SOFTWARE_READY and then SIM_LIVE_READY as applicable. **Rollback:** Stage2/offline trace replay; fix, rerun invalidated gates.

### Stage 4 — Real G1 standing / minimal motion

**Classification/task owner:** REAL_G1_HARDWARE_REQUIRED, Phase 13. **Prerequisites:** all four gates PASS, commissioned profile/fresh same-day preflight, operator+spotter+support, independent stop verified. **Enabled:** explicit real sink, standing balance controller and tiny approved test reference, logging/deadman/watchdogs. **Disabled:** walking, unconstrained human leg imitation, hand commands and automatic arm/reconnect.
**Safety/command limits:** human reference deviation≤.05 rad and rate≤.10 rad/s at permitted test joints; legs use proven standing controller envelope, not zero/frozen q; all final bounds/effort/gains remain enforced. **Abort:** unintended step, oscillation, stale state, target error>profile, thermal/electrical error, rig carries unexpected load or operator releases deadman. **Log:** H05 full physical measurements, native stop response/settling duration and rig observation. **Duration:** 5–10 s→30 s→2 min→5 min, inspect between; three successful repetitions. **Pass:** H05 and supported independent stop trials within profile, no unexplained physical excursion. **Rollback:** commissioned controlled/native stop with support, Stage1/preflight; incident review before repeat.

### Stage 5 — Upper-body limited range

**Classification/task owner:** REAL_G1_HARDWARE_REQUIRED, Phase 13. **Prerequisites:** Stage4 PASS, valid gates, explicit approved arm envelope. **Enabled:** one arm then other then symmetric arms while standing balance runs. **Disabled:** walk/turn/leg imitation and hands; large waist lean.
**Safety/command limits:** start≤.10 rad deviation per permitted arm joint, rate≤.10 rad/s and acceleration≤.20 rad/s² for human reference; expand to≤.20 rad only after sim equivalence and reviewed trial. Final balance/motor bounds still independently apply. **Abort:** arm-torso approach violates clearance, shoulder/elbow mapping/sign error, balance/thermal/feedback fault or safety veto. **Log:** upper-body command/actual error, clearance/support and fallback trajectory. **Duration:** 10 s single-joint trials, 1 min combined, then 3×5 min. **Pass:** H06, all directions and source-loss/deadman recoveries stable and no illegal target. **Rollback:** Stage4 standing if stable else commissioned stop/preflight; freeze human intent, do not abruptly lock balancing legs.

### Stage 6 — Lower-body / locomotion

**Classification/task owner:** REAL_G1_HARDWARE_REQUIRED, Phase 14. **Prerequisites:** Stage5 PASS, leg-raise diagnosis fixed and live sim valid, gait-compatible support/clear floor and stopping corridor. **Enabled:** approved weight shifts/leg reference, then planner-owned slow walking and turning, separately authorized. **Disabled:** arbitrary jumps/kicks/deep crouches, obstacles/stairs, automatic lateral translation and unvalidated joint envelope.
**Safety/command limits:** forward≤.05 m/s, lateral=0 initially, yaw≤.05 rad/s, intention acceleration≤.05 m/s², yaw acceleration≤.10 rad/s²; later only previously simulated/commissioned caps. Leg imitation range determined from validated feasible reference, no blanket full human range. **Abort:** slip/contact-loss/tilt/height/effort error, workspace stopping margin crossed, FOV loss, unstable transition, rig catching fall. **Log:** support/contact/base trajectory, planner mode/intention, foot tracking/slip, stop response and confidence. **Duration:** 5 s weight shifts/steps, 10–30 s walk/stop, then ≥5 min repeated bouts; accrue≥30 min successful active walking across sessions. **Pass:** all H07 lower-body checks, left/right leg and return-to-stand safe, no prohibited target/contact/fall. **Rollback:** bounded gait stop to validated supported standing if possible; otherwise native stop with fall arrest, return Stage4/Phase12 and review.

### Stage 7 — Whole-body Kinect teleoperation

**Classification/task owner:** REAL_G1_HARDWARE_REQUIRED, Phase 14. **Prerequisites:** Stage6 PASS, exact full-body sim-equivalent envelope and current gates/profile. **Enabled:** approved SMPL upper-body + standing or planner body-reference modes, source/controller guards, manual override and recording. **Disabled:** any motion outside commissioned envelope; hands remain inactive unless separately scoped/validated; unattended operation.
**Safety/command limits:** no greater than commissioned sim profile and final hardware caps; initially section4 maxima .15 m/s norm, .10 m/s lateral and .20 rad/s yaw only if individually validated in Stage6. Whole-body coupled feasibility checks may reduce usable range. **Abort:** any failure matrix critical event, unapproved contact, unexplained clipping/rejection trend, timing degradation or operator veto. **Log:** complete correlated evidence including camera calibration and all safety decisions, resource/temperature drift and run conditions. **Duration:** 3×15 min whole-body trials, later 30 min endurance for tuning; meet cumulative walking minimum. **Pass:** REAL_G1_READY criteria with operator review and defined envelope. **Rollback:** stop/disarm, Stage5 or earlier based on failure; critical incident invalidates affected gates and prevents resume until fixed/revalidated.

## 13. Documentation and handoff requirements

`README.md`: first distinguish supported bridge/SONIC path from legacy fixed-base demo; link this plan and current readiness state; then provide tested build, observe, record, no-publish replay and isolated sim commands. Later add physical procedures only after commissioning. Preserve upstream attribution and installation context but do not label old versions “latest.”

`AGENTS.md`: scoped behavioral guardrails for future GPT-5.6 sessions: read this plan and last evidence first, stay in phase, preserve good existing code, no silent gate skipping, software vs hardware failures explicit, no commands to real G1 before all prerequisites and session authority. Technical details remain in this plan/config/docs rather than duplicated into agent rules.

`IMPLEMENTATION_PLAN.md`: keep architecture and acceptance contract stable; append compact phase status/evidence/change-log entries. A phase completed means its tests/artifacts exist and pass. If actual findings require changing architecture or limits, document evidence and affected gates before implementing the revised contract; do not quietly relax requirements.

`docs/hardware_setup.md` and `docs/kinect_setup.md`: SDK/runtime/driver versions, power/USB3/topology, GPU/CPU provider, model checksums, non-root permissions and serial selection; distinguish installed library from successful real capture. No automatic firmware flash or administrative package upgrade in a validator.

`docs/calibration.md`: named frames, neutral/floor/gravity/heading/scale steps, visibility/confidence criteria, known-axis physical tests, file/version/serial matching, when to recalibrate, common mirrored/tilted/moving-camera failures and expected viewer outputs.

`docs/mujoco_validation.md`: official SONIC scene/model and 29-body-DOF vs hand indexing, build/backend parity, DDS isolation, no support-force/reset gate evidence, deterministic replay, `left_leg_raise` regression, live failure injection and exact report generation.

`docs/real_g1_validation.md`: robot-specific SDK/controller ownership, rig/spotter/stop procedure verified against current vendor manual, preflight zero-publish check, stages/limits/durations/abort/rollback, startup/shutdown, mode handover and evidence. Link the exact version of vendor documents used when commissioning. The [Unitree developer documentation portal](https://support.unitree.com/home/en/developer/) is a discovery starting point, not proof of an emergency-stop mapping for this robot. Physical stop semantics must be verified in Phase12/13 for the installed firmware.

`docs/troubleshooting.md`: map every failure reason in section4 to detection tools and safe recovery, including SDK provider mismatch, USB bandwidth, missing ZMQ headers/library symlink, image skew, wrong body, clock domains, calibration drift, limits/model mismatch, DDS domain/NIC and stale controller. A workaround cannot disable safety or fake a gate.

At each phase handoff record: phase/classification, exact files/symbols changed, commands and counts, result vs acceptance, linked artifacts/hashes, remaining blockers, current achieved gate and the single next phase. Do not overwrite unrelated parent HANDOFF history or resurrect its older optional-simulation rules.

## 14. Architecture coverage checklist

This traceability table maps the complete requested target architecture to implementation owners; none of these capabilities is satisfied solely by a plan entry.

| Requested capabilities | Owning phases / detailed contract |
|---|---|
| 1–8 device startup, RGB, depth, body stream, selection/multiple people, confidence, timestamps | P2/P4 input contract; P9 live verification; section3 timing/frame envelope |
| 9–14 frames, camera→human, neutral, floor/gravity, skeleton/scale normalization | P3/P10; section3 frames and calibration |
| 15–26 SMPL/validation, filters/smoothing/jitter, velocity/acceleration/jumps, joint/body loss, timeout | P3/P4/P6; section4 matrix and U03–U14 |
| 27 recovery | P4/P5/P11; disarmed reconnect, fresh epoch, stable dwell and manual rearm |
| 28–33 whole-body retargeting, mapping/limits/feasibility/collision, left_leg_raise | P6/P8/P11/P14; section7 diagnosis and post-policy guard |
| 34–36 intention, stand/walk, body references | P7/P11/P14; planner ownership and section12 staged caps |
| 37–41 SONIC/controller/DDS/ZMQ/stale protection | P5/P6; final writer seam, receiver/source age and epoch |
| 42–47 MuJoCo/live/synthetic/replay/record/determinism | P2/P5/P8/P11; U16–U23 and determinism levels |
| 48–51 debug/live skeleton/SMPL/robot target visualization | P7; section11 tools; existing SDK/Window3dWrapper/MuJoCo reuse |
| 52–57 telemetry/logs/latency/FPS/drops/CPU-GPU | P5/P7/P8; U22 and measured stage timestamps |
| 58–65 watchdog/E-stop/startup/shutdown/neutral/freeze/device-ZMQ-controller reconnect | P4/P5/P6/P12/P13; section4 safety/state machines |
| 66–67 config/calibration persistence | P2/P3; section5 and R2 trace metadata |
| 68–70 hardware preflight/Kinect/live MuJoCo | P9/P10/P11; H01–H03 and required gates |
| 71–75 G1 preflight/dry-run/low-risk/whole-body/final tuning | P12/P13/P14; H04–H07, sections10/12 |

## 15. Audit results and implementation status ledger

This section records actual work. It must distinguish implemented software
from the original planning audit and never upgrades a hardware gate.

| Check | Actual result on 2026-09-22 | Scope |
|---|---|---|
| First-party repository / integration boundary audit | Completed, findings in sections2–3 | Static inspection; vendor internals not exhaustively audited |
| CMake Debug bridge configure/build | PASS in `/tmp/kinect-plan-audit-build` | No hardware or network initialization |
| Existing `kinect_bridge_session_test` | PASS, assertions active in Debug | Synthetic core/session/trace checks |
| CTest discovery | 0 tests | Known gap; cannot claim test-suite registration PASS |
| Synthetic 60 s trace generation | PASS, 1,801 records in `/tmp/kinect-plan-audit.trace` | Includes terminal no-body frame |
| Existing replay with `--debug-skeleton` | PASS, `replayed_frames=1801`, exit0 | No publisher created; session output checks, not full downstream determinism |
| External input/MuJoCo safety tests | PASS: 3 tests in1.12 s | `env PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=.:external_dependencies/unitree_sdk2_python /home/panu/miniconda3/envs/gmr/bin/python -m pytest -p no:cacheprovider -q gear_sonic/tests/test_input_readers.py gear_sonic/tests/test_mujoco_sim_safety.py`, from external root; no SDK channels created by these tests |
| Initial external test attempt | Collection blocked by missing `unitree_sdk2py` on PYTHONPATH, resolved with existing external SDK path above | Environment invocation issue, no source fix |
| Full actual SONIC closed-loop run | Not run in this audit | Historical SIM_READY evidence retained but not re-certified |
| Physical Kinect / G1 validation | NOT RUN; no hardware pass claimed | Future Phases9–14 |
| Original left_leg_raise evidence | Not found in inspected repository/searched workspace text | Required reproduction/root-cause task remains |
| Implementation source modifications | None | Planning/documentation task only |

### Software-only execution update — 2026-09-24

| Phase | Status | Evidence / remaining work |
|---|---|---|
| P1 baseline | COMPLETE | CTest now registers/runs `kinect_bridge_session_test` and `kinect_synthetic_test`; Debug and Release clean builds pass. `kinect_synthetic_sim_sender` is a supported simulator-only target. CMake now chooses host shared ZeroMQ instead of the incomplete SDK static archive. |
| P2 configuration/input | COMPLETE (software) | Strict typed profiles, observe/no-publish default, real-profile refusal, SDK-neutral 32-joint `KinectInput`, serial selection, synchronized RGB/depth/body events, locked identity, source-neutral session and hardware-free config/input cases are implemented. Physical open/stream behavior remains Phase9 `HARDWARE_VALIDATION_REQUIRED`. |
| P3 calibration | COMPLETE (software) | `CalibrationState` validates/checksums and atomically persists transform, floor, heading, scale, serial and mount data. Synthetic plane/degenerate/heading/save-load/wrong-serial and SMPL coordinate tests pass. Physical values remain Phase10 `HARDWARE_VALIDATION_REQUIRED`. |
| P4 recovery | COMPLETE | Per-joint bounded hold, critical-chain confidence, reject-before-history, timestamps, pose/velocity/acceleration limits, explicit arm/stop, smooth neutral fallback, timeout, epoch reset and disarmed reconnect behavior are implemented and tested. |
| P5 transport/record/replay | COMPLETE | Coherent epoch/sequence/source/send timestamps, receiver freshness guards, hardened packed decoder, health messages, JSONL telemetry, portable checksummed little-endian R2 with R1 read compatibility, actual publisher→SONIC decoder test and deterministic replay are implemented. |
| P6 final controller safety | IMPLEMENTED; CHECKPOINT COMPATIBILITY FAILED | Sibling SONIC has the named 29-joint map, packed decoder freshness enforcement, `--no-command-publish`, final `MotorSafety`, 100 ms command watchdog, derivative limiting, exact model q rejection, nonfinite/gain checks, and non-fast-math tests. Synthetic reconstructed `left_leg_raise` rejects the reported 0.4 rad ankle-roll violation and accepts corrected/mirrored 0.12 rad cases; the original operator artifact was not found. Root-cause audit verified the deployed formula and training formula are both `target = default + residual * (0.25 * effort / stiffness)`, joint/default ordering is correct, observations use the intended relative-position/velocity/action/gravity contract, and every tested encoder/decoder/config trio passes its declared dimension checks. The default official artifacts exactly match the published hashes (decoder `c7241a123eaa36b5d64bad19540efde93cac1ad443bd4572fd12ca99898118ed`, encoder `013ab0287236aa2721e13f1e936d699db982302d0de0bfcdae76d5c3245362d3`, config `466d05947c78af6c76388adfb86e3a2a77b2a1d921a64883ed3d085ebf58de1b`). For left ankle pitch, scale `0.4385773139`, default `-0.363` and maximum `0.5236` imply maximum residual `2.0215`; the decoder emitted about `2.8464`, producing `0.885374`. Training only clips every residual symmetrically to `[-20,20]`, not to each joint's asymmetric legal residual interval. Matching official `sonic_v1_1` and `low_latency` trios were also tested and failed on waist pitch at `0.586278` and `0.563316` versus `0.52`. A static per-joint projection into the exact asymmetric residual envelopes was evaluated as an explicit new behavior. It preserved legal actions exactly and logged activations, but produced 13 MuJoCo safety resets in 38.2 simulated seconds because boundary targets became dynamically infeasible under the acceleration limiter. This exposed and fixed a final-guard ordering defect: `MotorSafety` now rejects any post-derivative candidate outside the shared authoritative limits without mutating safety history. With that correction the projected controller stopped safely at 0.1 s on `right_knee_joint: derivative limits cannot preserve position limit`. The experimental projection was removed. Thus the root cause remains a released-checkpoint/residual-envelope incompatibility, and a policy trained/fine-tuned with bounded, derivative-feasible actions is required; blanket clamping, global rescaling, limit widening, or MuJoCo clipping remain prohibited. |
| P7 diagnostics/locomotion | IMPLEMENTED; BLOCKED BY P6 CHECKPOINT | Unified session locomotion hysteresis/limits, stale cancellation, raw+SMPL text diagnostics, existing SDK/MuJoCo visualizers, JSONL latency/FPS/drop metrics, monitor, joint-map, retarget, replay, health and `sim-validate` commands exist and have positive/negative checks. The Phase 7 hardware-free unit, sanitizer, external MuJoCo, deterministic replay, map and retarget checks passed again on 2026-09-24. Sequential completion still requires a compatible P6 checkpoint and passing controller transition/E2E. |
| P8 gate | FAILED / NOT READY | Three actual isolated `127.0.0.1` ZMQ→SONIC production ORT decoder→DDS domain42/`lo`→floating-base MuJoCo runs were executed with no physical device, using the matching official default, `sonic_v1_1`, and `low_latency` artifact trios. The final guard rejected a raw target in every run: default left ankle pitch `0.885374 > 0.5236`; v1.1 waist pitch `0.586278 > 0.52`; low-latency waist pitch `0.563316 > 0.52`. The bounded-residual experiment also failed: first with 13 resets/38.2 s, then—after correcting post-derivative validation—with a safe final rejection at 0.1 s. The required 1,800 s zero-fault soak was not run because its zero-illegal-target and zero-reset prerequisites fail. `SOFTWARE_READY` is not established. |

Execution evidence (all software-only; no Kinect or G1 opened; DDS only on isolated domain42/`lo`):

| Check | Result |
|---|---|
| Debug Kinect build + CTest | PASS: 4/4 (`/tmp/kinect-p2-hw`), including actual C++ bridge publisher→SONIC packed decoder/freshness loopback |
| Release Kinect build + CTest | PASS: 4/4 (`/tmp/kinect-software-release`) with assertions forced active in test targets |
| ASan/UBSan bridge CTest | PASS: 4/4, including publisher→decoder transport, with `ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1` (`/tmp/kinect-final-asan`) |
| Offline validator unit suite | PASS: 9 tests (`python3 -m unittest discover -s tests -p 'test_*.py'`), including the no-execution Kinect preflight safety check |
| SONIC C++ unit suite | PASS: 6/6: FK fixture, named map/final guard, post-derivative position rejection, malformed packed input and freshness ordering; no skip |
| SONIC ASan/UBSan unit suite | PASS: 6/6 (`/tmp/sonic-asan-build`), including post-derivative position rejection and nonfinite checks under `-fno-fast-math` |
| External input/MuJoCo pytest | PASS: 3/3 in 0.93 s, including raw invalid target rejection/reset and floating-base/no-elastic checks |
| Deterministic R2 replay | PASS: two 1,801-frame `--fast --no-publish` executions with `config/sim_software.json` produced byte-identical logs (SHA-256 `833519eb3d10f5701b34d54ea3f2bef7da98a01849e2dec1f335e81d2c8bc076`); trace SHA-256 `d02f905071dded351ab64412ef81ad8c97def0800f63210599dc7effbf8ea25f` and payload SHA-256 `7024c1ab633f5cf694e7f8f52bd0d03d5454281a1d83ef3063da4857a87ad402` |
| Joint map / leg regression | PASS: 29/29 unique names match MuJoCo; reconstructed illegal left ankle roll REJECT; corrected and mirrored cases ACCEPT |
| Strict observe/sim config validation | PASS: `config/teleop.json`, `config/sim_software.json`, `config/sim_e2e.json` |
| Real-profile/physical tool behavior | PASS: bridge/offline validator reject real profile; Kinect preflight without explicit `--execute` and `real-preflight` return `BLOCKED` with `HARDWARE_VALIDATION_REQUIRED` and open no hardware |
| Actual production closed loop | **FAIL (safe stop), all published variants:** default entered CONTROL/streamed SMPL and rejected `left_ankle_pitch_joint=0.885374` outside `[-0.87267,0.5236]` (`artifacts/software/sim.json`, SHA-256 `70d4d8d0b9840ef0c1218ee61ec3f554078a8e3a32310f766309fcf704584148`); matching official `sonic_v1_1` rejected `waist_pitch_joint=0.586278` outside `[-0.52,0.52]` (`artifacts/software/v1_1/sim.json`, SHA-256 `5e673aebe8ff043923b932f1f753eac479eaf1bf63ddda67635471f86a713037`); matching official `low_latency` rejected `waist_pitch_joint=0.563316` (`artifacts/software/low_latency/sim.json`, SHA-256 `0650b20939c194e0d28dbe682ddf13e27c06a63f6a2c6580aadaf0807bef6472`). No physical interfaces were used and no illegal command reached MuJoCo. These failures are retained, not bypassed. |
| Explicit bounded-residual experiment | **FAIL:** exact asymmetric residual projection caused 13 simulator resets in 38.2 s (`artifacts/software/bounded/sim.json`, SHA-256 `fc6358fa910ffbed12a9bf1b09e38e1f85bfd1d4b8f504df1079b6641c907f05`). After the final guard was strengthened, it rejected a dynamically infeasible knee boundary command at 0.1 s with zero resets (`artifacts/software/bounded/sim-guard.json`, SHA-256 `f9a0ad2550c199343a6c27b0ef10c5e2244f84176868b98f25d27c88865212c1`). The experimental transform was removed; the guard fix remains. |
| SOFTWARE_READY health manifest | BLOCKED because the official-policy and bounded-transform production reports are invalid gate evidence; `artifacts/software/gate.json` SHA-256 `873c9fd5bb8e73fbfaa38e81c15137ce6f45cd5a9d253f53f2b1a0e658ce8616` |
| 30-minute software soak | NOT RUN after all three released variants and the bounded-transform experiment failed the zero-illegal-target/zero-reset prerequisites; running it cannot convert a prerequisite failure into a pass |

Current strict gates: SOFTWARE_READY **NOT READY** (all three official released policy variants violate an authoritative raw joint limit; P6 checkpoint compatibility, sequential P7 completion and P8 soak remain blocked); KINECT_READY **HARDWARE_VALIDATION_REQUIRED** and cannot start until SOFTWARE_READY; SIM_LIVE_READY **HARDWARE_VALIDATION_REQUIRED** after both earlier gates; REAL_G1_PREFLIGHT_READY **HARDWARE_VALIDATION_REQUIRED**; REAL_G1_READY **HARDWARE_VALIDATION_REQUIRED**. Historical approximate SIM_READY is not one of these strict passes.

Resume at Phase6 by retraining/finetuning SONIC with a per-joint bounded, derivative-feasible action parameterization derived from the authoritative default angles, scales and limits (or supply a checkpoint already trained and proven under that exact contract). Validate that artifact on the retained trace (SHA-256 `d02f905071dded351ab64412ef81ad8c97def0800f63210599dc7effbf8ea25f`) before accepting it. Do not reintroduce static projection, widen model limits, remove the final guard, squash/rescale outputs post hoc, or rely on MuJoCo clipping. After every target is statically and dynamically feasible, rerun P6/P7 suites, the actual closed loop, and only then the required 1,800-second zero-reset soak. Phase9 remains blocked until `SOFTWARE_READY` passes; its first authorized Kinect-only command is documented in `docs/hardware_setup.md`.

## 16. Strict execution order for GPT-5.6

1. Read root/parent `AGENTS.md`, this complete plan, current gate artifacts and repository status. Preserve existing work. Start Phase1 by reproducing the audited C++ tests and external tests; do not start any device or controller to “check” software.
2. Finish Phase1 build/test discovery and baseline evidence. Tests must execute in Debug and Release before new algorithms are changed.
3. Implement Phase2 configuration/input seam and unify synthetic/replay/live processing. Run U01/U02/source-parity tests; no hardware needed.
4. Implement Phase3 coordinate/calibration/SMPL contract with synthetic non-identity fixtures. U03–U06 must pass before retargeting or physical axis interpretation is trusted.
5. Implement Phase4 preprocessing/identity/fault/recovery/lifecycle. Run U07/U13–U15 and all earlier tests. Any stale/fallback derivative violation stops advancement.
6. Implement Phase5 coherent ZMQ, R2 recording, deterministic replay and timestamp telemetry in bridge **and actual receiver**. Run U16–U22 relevant rows and loopback conformance; do not send to physical DDS.
7. Implement Phase6 final controller guard/sink, named motor maps and pose feasibility; reproduce/diagnose left_leg_raise and verify production backend parity. U08–U11/U14/U21 must pass before any controller integration is called safe. If external source edits/compute/assets unavailable, report SOFTWARE_READY blocked rather than implementing a bypass.
8. Implement Phase7 unified intention/body references and all diagnostic tools. U12/U22/U24 and transition tests must pass; no new locomotion engine or legacy Euler fallback.
9. Run Phase8 full hardware-free actual-component E2E, all U01–U24 and 30 min soak. Produce SOFTWARE_READY. If it FAILS, stop progression and fix owning software phase; absence of physical hardware cannot excuse software gaps.
10. Only after SOFTWARE_READY PASS, obtain Kinect for Phase9 acquisition/reconnect and Phase10 physical coordinate/calibration/confidence tests. No G1 connection. Finish H01/H02 and record KINECT_READY. If it FAILS, fix/tune and rerun invalidated software tests; do not move robot.
11. Run Phase11 live Kinect→isolated SONIC/MuJoCo campaign (H03, 3×30 min). Produce SIM_LIVE_READY. No real G1 telemetry/control in this task before it passes. Any fall/reset/unsafe target/timing violation returns to its owning phase.
12. Only after first3 gates PASS, bring the real G1 into Phase12 read-only/compute-only preflight with qualified supervision and support. Complete H04 with zero outgoing motor/hand commands. Validate exact stop/limits/firmware and produce REAL_G1_PREFLIGHT_READY. Missing non-actuating evidence is a blocker; a trial movement cannot substitute.
13. **Only now**, with all four gates still PASS and fresh supervised arm authorization, execute Phase13 Stage4 standing/minimal movement. Inspect each short trial. Do not expand to Stage5 until Stage4 physical stop/balance evidence passes. Finish upper-body H06 with walking disabled.
14. Execute Phase14 Stage6 lower-body/locomotion, incrementing one verified range/speed at a time; leg-raise regression must pass. Then Stage7 full-body trials, final calibration/tuning, cumulative durations and H07. Any failed stage stops motion and rolls back as section12 specifies; software/config changes invalidate dependent gates.
15. Mark REAL_G1_READY only after section10 criteria and reviewed physical evidence are complete. Record exact supported envelope and operating procedure. Until then report the highest actually achieved gate and the concrete blocker, never “almost hardware ready.”

Dependency chain: `P1 → P2 → P3 → P4 → P5 → P6 → P7 → P8 [SOFTWARE_READY] → P9 → P10 [KINECT_READY] → P11 [SIM_LIVE_READY] → P12 [REAL_G1_PREFLIGHT_READY, still zero application robot commands] → P13 [Stages4–5] → P14 [Stages6–7, REAL_G1_READY]`.
