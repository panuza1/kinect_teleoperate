# Kinect → SONIC → G1 MuJoCo bridge

This bridge accepts Azure Kinect RGB/depth frames, tracks one body, calibrates
from two seconds of neutral standing, and publishes protocol-v3 SMPL poses on
`tcp://127.0.0.1:5556` (`pose`). Planner commands are capped at 0.15 m/s
forward, 0.10 m/s sideways, and 0.20 rad/s turning. If tracking is lost, it
publishes IDLE and fades the pose toward its calibrated neutral. The operator
must press `]` in SONIC after calibration reports `READY`; the bridge never
starts control itself.

The SDK is installed locally at
`/home/panu/.local/share/azure-kinect/1.4.1-1.1.2/usr`, so it does not modify
system packages. It was extracted from Microsoft's Sensor SDK 1.4.1 and Body
Tracking 1.1.2 Debian packages; CMake metadata, headers, model, dynamic
libraries, and their runtime dependencies have been checked locally. These
packages target older Ubuntu releases, so Ubuntu 22.04 live-camera
compatibility remains unverified. `--cpu` avoids CUDA-provider compatibility
problems; use GPU mode only after validating it with a connected camera.

Build (no hardware access):

```bash
cd /home/panu/Documents/fibo/project_humanoid/g1_inspire_workspace/kinect_teleoperate
sdk_root=/home/panu/.local/share/azure-kinect/1.4.1-1.1.2/usr
cmake -S . -B /tmp/kinect-bridge-build -DKINECT_BRIDGE_ONLY=ON \
  -DCMAKE_PREFIX_PATH="$sdk_root" -DZMQ_INCLUDE_DIR="$sdk_root/include" \
  -DZMQ_LIBRARY=/usr/lib/x86_64-linux-gnu/libzmq.so.5
cmake --build /tmp/kinect-bridge-build -j2
/tmp/kinect-bridge-build/kinect_bridge_session_test
```

Live skeleton check, without SONIC publishing:

```bash
sdk_root=/home/panu/.local/share/azure-kinect/1.4.1-1.1.2/usr
LD_LIBRARY_PATH="$sdk_root/lib:$sdk_root/lib/x86_64-linux-gnu" \
  /tmp/kinect-bridge-build/kinect_smpl_zmq_bridge --debug-skeleton --cpu \
  --model "$sdk_root/bin/dnn_model_2_0_lite_op11.onnx"
```

For live simulation, start the known-good G1 MuJoCo simulator in one terminal:

```bash
cd /home/panu/Documents/fibo/project_humanoid/g1_inspire_workspace/GR00T-WholeBodyControl
SONIC_SIM_METRICS=1 PYTHONPATH=.:external_dependencies/unitree_sdk2_python \
  /home/panu/miniconda3/envs/gmr/bin/python gear_sonic/scripts/run_sim_loop.py \
  --interface sim --dds-domain 42 --no-enable-onscreen
```

Start SONIC in a second terminal, using only loopback DDS:

```bash
cd /home/panu/Documents/fibo/project_humanoid/g1_inspire_workspace/GR00T-WholeBodyControl/gear_sonic_deploy
./target/release/g1_deploy_onnx_ref lo policy/release/model_decoder.onnx reference/example/ \
  --obs-config policy/release/observation_config.yaml \
  --encoder-file policy/release/model_encoder.onnx \
  --planner-file planner/target_vel/V2/planner_sonic.onnx \
  --input-type zmq_manager --zmq-host 127.0.0.1 --zmq-port 5556 --disable-crc-check
```

Start the live bridge in a third terminal:

```bash
sdk_root=/home/panu/.local/share/azure-kinect/1.4.1-1.1.2/usr
LD_LIBRARY_PATH="$sdk_root/lib:$sdk_root/lib/x86_64-linux-gnu" \
  /tmp/kinect-bridge-build/kinect_smpl_zmq_bridge --cpu \
  --model "$sdk_root/bin/dnn_model_2_0_lite_op11.onnx" \
  --record /tmp/kinect_live.trace
```

Replay the same trace without Kinect hardware (use this instead of the live
bridge in the third terminal):

```bash
sdk_root=/home/panu/.local/share/azure-kinect/1.4.1-1.1.2/usr
LD_LIBRARY_PATH="$sdk_root/lib:$sdk_root/lib/x86_64-linux-gnu" \
  /tmp/kinect-bridge-build/kinect_smpl_zmq_bridge --replay /tmp/kinect_live.trace
```

The trace stores device and host timestamps, body ID, raw 32-joint skeleton
(positions, orientations, confidence), calibrated SMPL output, planner intent,
and tracking state. Replay recalculates the output and rejects divergence. A
hardware-free 60-second trace can be created with
`/tmp/kinect-bridge-build/kinect_bridge_session_test /tmp/kinect_sim_replay.trace`.

The metrics line reports Kinect capture FPS, Body Tracking FPS, bridge FPS,
ZMQ FPS, capture-to-publish latency, and an estimated capture-to-sim latency
that adds the configured 20 ms SONIC control period plus 5 ms MuJoCo step.
The estimate excludes camera exposure and any unmeasured queueing.

When hardware is connected, verify USB 3.x with `lsusb -t`, use the locally
installed `$sdk_root/bin/k4aviewer` and `$sdk_root/bin/k4abt_simple_3d_viewer`
with the same `LD_LIBRARY_PATH` to check RGB/depth and body tracking, then run
`--debug-skeleton` to check
pelvis, spine, head, both arms and legs. If non-root access fails, install the
official [Kinect udev rule](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/scripts/99-k4a.rules)
with administrator privileges and reconnect the device. No physical G1
interface is part of this workflow.
