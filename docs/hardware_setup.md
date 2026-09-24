# Hardware setup

Hardware validation is intentionally separate from the software gate. The
installed Azure Kinect packages and a successful CMake build do not demonstrate
that RGB, depth, body tracking, USB bandwidth, GPU provider, or a robot work.

Without `--execute`, `scripts/validate_pipeline.py kinect-preflight` records a
blocked hardware phase and opens nothing. After `SOFTWARE_READY` passes, an
authorized Kinect-only Phase 9 session may add `--execute`; this launches the
bridge in observe/no-publish mode, records a trace, and never opens DDS or G1.
`real-preflight` remains a blocked, no-hardware command in this software work.

Do not run the Phase 9 command while `SOFTWARE_READY` is NOT READY. Once it
passes, use a hardware-capable bridge build and run:

```bash
python3 scripts/validate_pipeline.py kinect-preflight \
  --config config/teleop.json \
  --bridge /tmp/kinect-p2-hw/kinect_smpl_zmq_bridge \
  --duration 600 \
  --trace artifacts/kinect/phase9.trace \
  --report artifacts/kinect/phase9-preflight.json \
  --execute
```

For real validation, record the Kinect serial, USB topology, SDK/model hash,
G1 variant/firmware, controller build hash, network interface, current vendor
manual, commissioned limits, support arrangement, spotter, and independent
stop procedure in the run artifact. Never use this document as an emergency
stop procedure for an unverified robot/firmware combination.
