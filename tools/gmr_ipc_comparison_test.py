#!/usr/bin/env python3
"""Run the real Python GMR bridge against the C++ adapter/legacy comparison."""

import pathlib
import subprocess
import sys
import time


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: gmr_ipc_comparison_test.py GMR_COMPARISON_BINARY")
    root = pathlib.Path(__file__).resolve().parents[1]
    bridge = root / "tools" / "kinect_gmr_bridge.py"
    port = "5568"
    worker = subprocess.Popen(
        [sys.executable, str(bridge), "--port", port, "--calibration-frames", "1"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            line = worker.stdout.readline()
            if "gmr_bridge=ready" in line:
                break
            if worker.poll() is not None:
                raise RuntimeError(f"GMR bridge exited: {line}")
        else:
            raise RuntimeError("GMR bridge startup timeout")
        subprocess.run([sys.argv[1], port], check=True, timeout=30)
    finally:
        worker.terminate()
        try:
            worker.wait(timeout=5)
        except subprocess.TimeoutExpired:
            worker.kill()


if __name__ == "__main__":
    main()
