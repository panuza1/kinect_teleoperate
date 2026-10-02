# Kinect to G1 full-body plan

Status values are `DONE`, `BLOCKED`, or `NOT RUN`.

| Phase | Deliverable | Status |
|---|---|---|
| 1 | Inspect official GMR Unitree G1 model, API, formats, limits, scaling, and live example | DONE |
| 2 | Add calibrated Kinect-to-GMR adapter and loopback C++/Python bridge | DONE |
| 3 | Make GMR primary and keep the custom mapping as `--retargeter legacy` | DONE |
| 4 | Generate filtered, limited G1 qpos/qvel in the shared SONIC order | DONE |
| 5 | Preserve fixed-base MuJoCo and SONIC Protocol v1 output modes | DONE |
| 6 | Add synthetic semantic GMR and legacy-comparison tests | DONE |
| 7 | Build and run hardware-free validation | DONE |
| 8 | Validate GMR visually with a Femto Bolt before rerunning SONIC | BLOCKED: no Femto Bolt is connected |

GMR commit `bb1bbe40774794fceb2a7c579a3464a28e68c844` is installed from the sibling `GMR/` checkout. Both output modes share one `G1Reference`. Direct mode stops at visualization; SONIC owns dynamic balance. The old Euler mapping remains diagnostic only.
