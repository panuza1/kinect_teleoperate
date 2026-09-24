# Real G1 validation status

Real G1 validation has not been run and is not authorized by software-only
work. `config/real_g1.json` is deliberately incomplete and rejected.

The robot may enter this workflow only after current evidence establishes
`SOFTWARE_READY`, `KINECT_READY`, and `SIM_LIVE_READY`. Phase 12 then performs
read-only telemetry and compute-only target generation with zero motor or hand
publish, verifies the exact robot variant/firmware/joint map/native watchdog,
and records the supervised stop procedure. Only a reviewed
`REAL_G1_PREFLIGHT_READY` gate permits the staged motion trials in
`IMPLEMENTATION_PLAN.md`.

No command in this document arms or moves a robot.
