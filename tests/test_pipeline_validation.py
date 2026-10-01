import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from scripts import validate_pipeline


class PipelineValidationTest(unittest.TestCase):
    def write_config(self, value):
        path = Path(tempfile.mkdtemp()) / "config.json"
        path.write_text(json.dumps(value), encoding="utf-8")
        return path

    def run_fake_kinect_preflight(self, script, duration=0.1):
        directory = Path(tempfile.mkdtemp())
        config = self.write_config({"schema_version": 1, "mode": "observe", "no_publish": True})
        bridge = directory / "bridge"
        bridge.write_text(f"#!{sys.executable}\nimport sys\n" + script, encoding="utf-8")
        bridge.chmod(0o755)
        args = type("Args", (), {"config": config, "execute": True, "bridge": bridge,
                                  "duration": duration, "trace": directory / "trace",
                                  "report": directory / "report.json"})
        status = validate_pipeline.command_kinect_preflight(args)
        return status, json.loads(args.report.read_text(encoding="utf-8"))

    @staticmethod
    def fake_trace_writer(body=True):
        return f'''import struct, time, signal, sys
trace = sys.argv[sys.argv.index("--record") + 1]
payload = struct.pack("<QBBIQ", 1, {int(body)}, 0, 17, 1) + bytes({validate_pipeline.KINECT_R2_FRAME_SIZE} - 22)
def fnv(data):
    value = 2166136261
    for byte in data:
        value = ((value ^ byte) * 16777619) & 0xffffffff
    return value
open(trace, "wb").write(b"KSMPLR2\\n" + struct.pack("<II", len(payload), fnv(payload)) + payload)
print("bridge_state=OBSERVE", flush=True)
'''

    def test_kinect_preflight_gracefully_terminates_after_duration(self):
        script = self.fake_trace_writer() + '''
import signal
def stop(*_):
    print("graceful shutdown", flush=True)
    sys.exit(0)
signal.signal(signal.SIGTERM, stop)
print("state=OBSERVE body=17 device_us=1", flush=True)
while True:
    time.sleep(0.01)
'''
        status, report = self.run_fake_kinect_preflight(script)
        self.assertEqual(status, 0)
        self.assertEqual(report["status"], "PASS")
        self.assertEqual(report["hardware_commands"], 0)
        self.assertTrue(report["duration_reached"])
        self.assertFalse(report["forced_kill"])
        self.assertEqual(report["trace_frames"], 1)
        self.assertEqual(report["trace_body_frames"], 1)
        self.assertIn("graceful shutdown", report["output_tail"])

    def test_kinect_preflight_rejects_early_bridge_error(self):
        status, report = self.run_fake_kinect_preflight(
            'print("Kinect bridge: camera could not start", flush=True)\nsys.exit(1)\n', duration=2
        )
        self.assertEqual(status, 1)
        self.assertFalse(report["duration_reached"])
        self.assertTrue(any("exited before" in item for item in report["failures"]))
        self.assertTrue(any("reported an error" in item for item in report["failures"]))
        self.assertEqual(report["hardware_commands"], 0)

    def test_kinect_preflight_rejects_sdk_error_output(self):
        script = self.fake_trace_writer() + '''
import signal
signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
print("[K4ABT] [error] Cannot find DNN model", flush=True)
print("state=OBSERVE body=17 device_us=1", flush=True)
while True:
    time.sleep(0.01)
'''
        status, report = self.run_fake_kinect_preflight(script)
        self.assertEqual(status, 1)
        self.assertTrue(any("reported an error" in item for item in report["failures"]))

    def test_kinect_preflight_rejects_trace_without_body(self):
        script = self.fake_trace_writer(body=False) + '''
import signal
signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
while True:
    time.sleep(0.01)
'''
        status, report = self.run_fake_kinect_preflight(script)
        self.assertEqual(status, 1)
        self.assertTrue(any("no valid body data" in item for item in report["failures"]))

    def test_kinect_preflight_kills_process_after_term_timeout(self):
        script = self.fake_trace_writer() + '''
import signal
signal.signal(signal.SIGTERM, signal.SIG_IGN)
print("state=OBSERVE body=17 device_us=1", flush=True)
while True:
    time.sleep(0.01)
'''
        with patch.object(validate_pipeline, "KINECT_SHUTDOWN_TIMEOUT", 0.1):
            status, report = self.run_fake_kinect_preflight(script)
        self.assertEqual(status, 1)
        self.assertTrue(report["forced_kill"])
        self.assertTrue(any("SIGKILL" in item for item in report["failures"]))

    def test_observe_profile_is_accepted(self):
        config = self.write_config({"schema_version": 1, "mode": "observe", "no_publish": True})
        self.assertEqual(validate_pipeline.load_config(config)["mode"], "observe")

    def test_real_profile_is_rejected_offline(self):
        config = self.write_config({"schema_version": 1, "mode": "real", "no_publish": True})
        with self.assertRaisesRegex(ValueError, "real mode"):
            validate_pipeline.load_config(config)

    def test_unknown_key_is_rejected(self):
        config = self.write_config({"schema_version": 1, "mode": "observe", "unexpected": 1})
        with self.assertRaisesRegex(ValueError, "unknown config"):
            validate_pipeline.load_config(config)

    def test_health_requires_evidence(self):
        config = self.write_config({"schema_version": 1, "mode": "observe", "no_publish": True})
        args = type("Args", (), {"config": config, "gate": "SOFTWARE_READY", "evidence": [], "report": None})
        self.assertEqual(validate_pipeline.command_health(args), 3)

    def test_joint_map_and_left_leg_raise_contract(self):
        names, limits = validate_pipeline.joint_contract(validate_pipeline.SONIC_ROOT)
        self.assertEqual(len(names), 29)
        self.assertEqual(limits["left_ankle_roll_joint"], (-0.2618, 0.2618))
        result = validate_pipeline.validate_motion_case(
            Path(__file__).parent / "fixtures/left_leg_raise.json", validate_pipeline.SONIC_ROOT)
        outcomes = {case["name"]: case["observed"] for case in result["cases"]}
        self.assertEqual(outcomes["reported_failure_reconstruction"], "REJECT")
        self.assertEqual(outcomes["left_leg_raise_corrected"], "ACCEPT")

    def test_portable_trace_validator_rejects_corruption(self):
        directory = Path(tempfile.mkdtemp())
        trace = directory / "trace.bin"
        payload = b"frame"
        trace.write_bytes(b"KSMPLR2\n" + struct.pack("<II", len(payload), validate_pipeline.fnv32(payload)) + payload)
        args = type("Args", (), {"trace": trace, "report": None})
        self.assertEqual(validate_pipeline.command_replay(args), 0)
        trace.write_bytes(trace.read_bytes()[:-1] + b"x")
        with self.assertRaisesRegex(ValueError, "checksum"):
            validate_pipeline.command_replay(args)

    def test_joint_hold_cannot_exceed_stop_timeout(self):
        config = self.write_config({"schema_version": 1, "mode": "observe", "no_publish": True,
                                    "timeouts": {"tracking_inhibit_ms": 100,
                                                 "tracking_stop_ms": 200, "joint_hold_ms": 201}})
        with self.assertRaisesRegex(ValueError, "tracking timeouts"):
            validate_pipeline.load_config(config)

    def test_sim_validator_accepts_clean_run_and_rejects_safety_fault(self):
        directory = Path(tempfile.mkdtemp())
        controller = directory / "controller.log"
        simulator = directory / "sim.log"
        controller.write_text("transitioning to CONTROL state\n", encoding="utf-8")
        simulator.write_text(
            "[SONIC_SIM_METRICS] sim_time=1800.0s resets=0 "
            "max_cmd_delta=0.2000rad max_joint_delta=0.1500rad hand_cmds=0\n",
            encoding="utf-8",
        )
        args = type("Args", (), {"controller_log": controller, "sim_log": simulator,
                                  "min_sim_time": 1800.0, "min_command_delta": 0.05,
                                  "min_joint_delta": 0.05, "report": None})
        self.assertEqual(validate_pipeline.command_sim_validate(args), 0)
        controller.write_text("transitioning to CONTROL state\n[MotorSafety] Rejected\n", encoding="utf-8")
        self.assertEqual(validate_pipeline.command_sim_validate(args), 1)

    def test_sim_validator_rejects_no_motion_or_hand_publication(self):
        directory = Path(tempfile.mkdtemp())
        controller = directory / "controller.log"
        simulator = directory / "sim.log"
        controller.write_text("transitioning to CONTROL state\n", encoding="utf-8")
        simulator.write_text(
            "[SONIC_SIM_METRICS] sim_time=60.0s resets=0 "
            "max_cmd_delta=0.2000rad max_joint_delta=0.0000rad hand_cmds=1\n",
            encoding="utf-8",
        )
        args = type("Args", (), {"controller_log": controller, "sim_log": simulator,
                                  "min_sim_time": 30.0, "min_command_delta": 0.05,
                                  "min_joint_delta": 0.05, "report": None})
        self.assertEqual(validate_pipeline.command_sim_validate(args), 1)

    def test_kinect_preflight_defaults_to_blocked_without_execution(self):
        config = self.write_config({"schema_version": 1, "mode": "observe", "no_publish": True})
        args = type("Args", (), {"command": "kinect-preflight", "config": config,
                                  "execute": False, "report": None})
        self.assertEqual(validate_pipeline.command_kinect_preflight(args), 3)


if __name__ == "__main__":
    unittest.main()
