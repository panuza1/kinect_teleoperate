# Kinect to G1 full-body plan

Status values are `DONE`, `BLOCKED`, or `NOT RUN`.

| Phase | Deliverable | Status |
|---|---|---|
| 1 | Verify SONIC v1 framing, 29-joint order, released policy, and simulator; document architecture | DONE |
| 2 | Add isolated full-body retargeter with confidence hold, filtering, limits, timestamps, and velocity | DONE |
| 3 | Feed the existing Kinect pipeline into the retargeter and add diagnostics/CLI modes | DONE |
| 4 | Add fixed-base/kinematic MuJoCo debug output using SONIC's exact G1 model | DONE |
| 5 | Add loopback-only SONIC Protocol v1 publisher | DONE |
| 6 | Add retargeting, real-decoder protocol, and MuJoCo tests | DONE |
| 7 | Build and run hardware-free validation | DONE |
| 8 | Validate with a Femto Bolt, then SONIC and floating-base MuJoCo | BLOCKED: v1 to released SONIC to floating-base MuJoCo passed with synthetic input; no Femto Bolt is connected |

Both modes share one `G1Reference`. Direct mode stops at visualization; SONIC mode owns dynamic balance.
