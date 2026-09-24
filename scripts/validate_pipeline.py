#!/usr/bin/env python3
"""Offline validation helpers for the Kinect → SONIC bridge.

This program intentionally has no arm/publish option.  Hardware checks are
reported as BLOCKED until a separately commissioned validation phase runs.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import platform
import re
import signal
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


ROOT_KEYS = {
    "schema_version", "mode", "no_publish", "debug_skeleton", "cpu", "port",
    "bind_host", "model_path", "telemetry_path", "selection", "timeouts", "locomotion",
    "calibration_path", "device_serial", "mount_id", "calibration_required",
}
SELECTION_KEYS = {"body_id"}
TIMEOUT_KEYS = {"tracking_inhibit_ms", "tracking_stop_ms", "joint_hold_ms"}
LOCOMOTION_KEYS = {"enabled", "max_forward_mps", "max_lateral_mps", "max_yaw_rps",
                   "max_linear_accel_mps2", "max_yaw_accel_rps2"}
ROOT = Path(__file__).resolve().parents[1]
SONIC_ROOT = ROOT.parent / "GR00T-WholeBodyControl"


def fail(message: str) -> ValueError:
    return ValueError(message)


def load_config(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise fail(f"config not found: {path}") from error
    except json.JSONDecodeError as error:
        raise fail(f"invalid JSON config: {error}") from error
    if not isinstance(value, dict):
        raise fail("config root must be an object")
    mode = value.get("mode", "observe")
    if mode not in {"observe", "compute", "sim", "real"}:
        raise fail("mode must be observe, compute, sim, or real")
    if mode == "real":
        raise fail("real mode is intentionally not accepted by offline tooling")
    unknown = set(value) - ROOT_KEYS
    if unknown:
        raise fail(f"unknown config keys: {', '.join(sorted(unknown))}")
    if value.get("schema_version") != 1:
        raise fail("schema_version must be 1")
    port = value.get("port", 5556)
    if not isinstance(port, int) or not 1 <= port <= 65535:
        raise fail("port must be an integer in [1, 65535]")
    if not isinstance(value.get("no_publish", True), bool):
        raise fail("no_publish must be boolean")
    if not isinstance(value.get("telemetry_path", ""), str):
        raise fail("telemetry_path must be a string")
    if not isinstance(value.get("calibration_required", False), bool):
        raise fail("calibration_required must be boolean")
    if value.get("calibration_required") and not value.get("calibration_path"):
        raise fail("calibration_required needs calibration_path")
    for section, allowed in (("selection", SELECTION_KEYS), ("timeouts", TIMEOUT_KEYS),
                             ("locomotion", LOCOMOTION_KEYS)):
        child = value.get(section, {})
        if not isinstance(child, dict):
            raise fail(f"{section} must be an object")
        unknown = set(child) - allowed
        if unknown:
            raise fail(f"unknown {section} keys: {', '.join(sorted(unknown))}")
    timeouts = value.get("timeouts", {})
    inhibit = timeouts.get("tracking_inhibit_ms", 100)
    stop = timeouts.get("tracking_stop_ms", 200)
    hold = timeouts.get("joint_hold_ms", 100)
    if any(not isinstance(item, int) for item in (inhibit, stop, hold)) or not 0 < inhibit <= stop or not 0 < hold <= stop:
        raise fail("tracking timeouts must be positive integers with inhibit/hold <= stop")
    locomotion = value.get("locomotion", {})
    if "enabled" in locomotion and not isinstance(locomotion["enabled"], bool):
        raise fail("locomotion.enabled must be boolean")
    for key in (LOCOMOTION_KEYS - {"enabled"}) & set(locomotion):
        value_at_key = locomotion[key]
        if not isinstance(value_at_key, (int, float)) or value_at_key < 0:
            raise fail(f"{key} must be non-negative")
    return value


def canonical_json(value: dict[str, Any]) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_report(path: Path | None, result: dict[str, Any]) -> None:
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if path:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(rendered, encoding="utf-8")
    print(rendered, end="")


def base_report(status: str, config: Path | None = None) -> dict[str, Any]:
    result: dict[str, Any] = {
        "schema_version": 1,
        "status": status,
        "utc": datetime.now(timezone.utc).isoformat(),
        "python": platform.python_version(),
        "platform": platform.platform(),
        "hardware_commands": 0,
    }
    if config:
        result["config"] = str(config)
        result["config_sha256"] = sha256_file(config)
    return result


def command_config(args: argparse.Namespace) -> int:
    config = load_config(args.config)
    result = base_report("PASS", args.config)
    result["effective_config"] = json.loads(canonical_json(config))
    result["effective_config_sha256"] = hashlib.sha256(canonical_json(config).encode()).hexdigest()
    write_report(args.report, result)
    return 0


def command_monitor(args: argparse.Namespace) -> int:
    samples: list[dict[str, Any]] = []
    for path in args.logs:
        with Path(path).open(encoding="utf-8") as file:
            for line_number, line in enumerate(file, 1):
                if line.strip():
                    try:
                        sample = json.loads(line)
                    except json.JSONDecodeError as error:
                        raise fail(f"{path}:{line_number}: invalid JSONL: {error}") from error
                    if not isinstance(sample, dict):
                        raise fail(f"{path}:{line_number}: JSONL value must be an object")
                    samples.append(sample)
    if not samples:
        raise fail("telemetry contains no samples")
    values: dict[str, list[float]] = {}
    for sample in samples:
        for key, value in sample.items():
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                values.setdefault(key, []).append(float(value))
    summary: dict[str, Any] = {}
    for key, data in values.items():
        ordered = sorted(data)
        summary[key] = {"count": len(data), "min": ordered[0], "max": ordered[-1],
                        "p50": ordered[(len(data) - 1) // 2],
                        "p95": ordered[round((len(data) - 1) * .95)],
                        "p99": ordered[round((len(data) - 1) * .99)]}
    result = base_report("PASS")
    result.update({"samples": len(samples), "metrics": summary})
    write_report(args.report, result)
    return 0


def joint_contract(controller_root: Path) -> tuple[list[str], dict[str, tuple[float, float]]]:
    header = controller_root / "gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/robot_parameters.hpp"
    xml_path = controller_root / "gear_sonic/data/robot_model/model_data/g1/g1_29dof_with_hand.xml"
    source = header.read_text(encoding="utf-8")
    match = re.search(r"G1_MOTOR_NAMES\s*=\s*\{(.*?)\};", source, re.S)
    if not match:
        raise fail("G1_MOTOR_NAMES was not found")
    names = re.findall(r'"([^"]+_joint)"', match.group(1))
    if len(names) != 29 or len(set(names)) != 29:
        raise fail("named G1 motor map must contain 29 unique joints")
    root = ET.parse(xml_path).getroot()
    xml_limits = {
        joint.get("name"): tuple(map(float, joint.get("range").split()))
        for joint in root.iter("joint") if joint.get("name") and joint.get("range")
    }
    missing = [name for name in names if name not in xml_limits]
    if missing:
        raise fail(f"motor names missing from MuJoCo model: {missing}")
    return names, {name: xml_limits[name] for name in names}


def command_joint_map(args: argparse.Namespace) -> int:
    names, limits = joint_contract(args.controller_root)
    result = base_report("PASS")
    result.update({"motor_count": len(names), "names": names,
                   "limits": {name: limits[name] for name in names}})
    write_report(args.report, result)
    return 0


def validate_motion_case(path: Path, controller_root: Path) -> dict[str, Any]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("schema_version") != 1 or not str(document.get("provenance", "")).startswith("synthetic_"):
        raise fail("motion case needs schema 1 and explicit synthetic provenance")
    names, limits = joint_contract(controller_root)
    outcomes = []
    for case in document.get("cases", []):
        violations: list[str] = []
        previous = {name: 0.0 for name in names}
        previous_velocity = {name: 0.0 for name in names}
        previous_time: float | None = None
        for frame in case.get("frames", []):
            timestamp = frame.get("t")
            targets = frame.get("targets")
            if not isinstance(timestamp, (int, float)) or not isinstance(targets, dict):
                raise fail(f"{case.get('name')}: invalid frame")
            unknown = set(targets) - set(names)
            if unknown:
                raise fail(f"{case.get('name')}: unknown joints {sorted(unknown)}")
            current = {name: float(targets.get(name, 0.0)) for name in names}
            for name, value in current.items():
                low, high = limits[name]
                if not math.isfinite(value) or value < low or value > high:
                    violations.append(f"{name}: position limit")
            if previous_time is not None:
                dt = float(timestamp) - previous_time
                if not 0 < dt <= 0.200001:
                    violations.append("invalid command interval")
                else:
                    for name in names:
                        velocity = (current[name] - previous[name]) / dt
                        acceleration = (velocity - previous_velocity[name]) / dt
                        if abs(velocity) > 8.0:
                            violations.append(f"{name}: velocity limit")
                        if abs(acceleration) > 80.0:
                            violations.append(f"{name}: acceleration limit")
                        previous_velocity[name] = velocity
            previous, previous_time = current, float(timestamp)
        observed = "REJECT" if violations else "ACCEPT"
        expected = case.get("expected")
        outcomes.append({"name": case.get("name"), "expected": expected,
                         "observed": observed, "violations": sorted(set(violations))})
        if observed != expected:
            raise fail(f"{case.get('name')}: expected {expected}, observed {observed}")
    if not outcomes or not any(item["name"] == "left_leg_raise_corrected" and item["observed"] == "ACCEPT"
                               for item in outcomes):
        raise fail("corrected left_leg_raise acceptance case is missing")
    return {"provenance": document["provenance"], "cases": outcomes}


def command_retarget(args: argparse.Namespace) -> int:
    result = base_report("PASS")
    result.update(validate_motion_case(args.case, args.controller_root))
    write_report(args.report, result)
    return 0


def fnv32(data: bytes) -> int:
    value = 2166136261
    for byte in data:
        value = ((value ^ byte) * 16777619) & 0xFFFFFFFF
    return value


def command_replay(args: argparse.Namespace) -> int:
    frames = 0
    digest = hashlib.sha256()
    with args.trace.open("rb") as file:
        if file.read(8) != b"KSMPLR2\n":
            raise fail("portable KSMPLR2 trace required")
        while prefix := file.read(8):
            if len(prefix) != 8:
                raise fail("truncated trace prefix")
            size, expected = struct.unpack("<II", prefix)
            if not 0 < size <= 1024 * 1024:
                raise fail("invalid trace frame size")
            payload = file.read(size)
            if len(payload) != size or fnv32(payload) != expected:
                raise fail("trace checksum/truncation failure")
            digest.update(prefix)
            digest.update(payload)
            frames += 1
    if frames == 0:
        raise fail("empty trace")
    result = base_report("PASS")
    result.update({"frames": frames, "payload_sha256": digest.hexdigest(), "format": "KSMPLR2"})
    write_report(args.report, result)
    return 0


def command_health(args: argparse.Namespace) -> int:
    config = load_config(args.config)
    required = [Path(path) for path in args.evidence]
    missing = [str(path) for path in required if not path.is_file()]
    invalid = []
    for path in required:
        if path.is_file() and path.suffix == ".json":
            try:
                evidence = json.loads(path.read_text(encoding="utf-8"))
                if evidence.get("status") != "PASS" or evidence.get("hardware_commands", 0) != 0:
                    invalid.append(str(path))
            except (OSError, json.JSONDecodeError):
                invalid.append(str(path))
    if not required:
        missing.append("required test/gate manifest")
    result = base_report("PASS" if not missing and not invalid else "BLOCKED", args.config)
    result["gate"] = args.gate
    result["evidence"] = [str(path) for path in required]
    result["missing_evidence"] = missing
    result["invalid_evidence"] = invalid
    result["note"] = ("This command validates supplied evidence paths only; it never upgrades a gate "
                      "without the complete test manifest.")
    write_report(args.report, result)
    return 0 if not missing and not invalid else 3


def command_sim_validate(args: argparse.Namespace) -> int:
    controller = args.controller_log.read_text(encoding="utf-8")
    simulator = args.sim_log.read_text(encoding="utf-8")
    metrics = [
        (float(sim_time), int(resets))
        for sim_time, resets in re.findall(r"sim_time=([0-9.]+)s.*resets=([0-9]+)", simulator)
    ]
    failures = []
    if "transitioning to CONTROL state" not in controller:
        failures.append("controller never entered CONTROL")
    for marker in ("[MotorSafety] Rejected", "simulation safety reset", "fallen ("):
        if marker in controller or marker in simulator:
            failures.append(marker)
    max_sim_time = max((value[0] for value in metrics), default=0.0)
    max_resets = max((value[1] for value in metrics), default=0)
    if max_sim_time < args.min_sim_time:
        failures.append(f"simulated time {max_sim_time:.3f}s below {args.min_sim_time:.3f}s")
    if max_resets:
        failures.append(f"simulation reset count {max_resets}")
    result = base_report("PASS" if not failures else "FAIL")
    result.update({"controller_log": str(args.controller_log), "sim_log": str(args.sim_log),
                   "simulated_seconds": max_sim_time, "resets": max_resets,
                   "failures": failures})
    write_report(args.report, result)
    return 0 if not failures else 1


def command_hardware_blocked(args: argparse.Namespace) -> int:
    config = load_config(args.config)
    result = base_report("BLOCKED", args.config)
    result["operation"] = args.command
    result["reason"] = "HARDWARE_VALIDATION_REQUIRED: this offline tool does not open Kinect, DDS, or G1 hardware."
    write_report(args.report, result)
    return 3


def command_kinect_preflight(args: argparse.Namespace) -> int:
    config = load_config(args.config)
    if not args.execute:
        return command_hardware_blocked(args)
    if config.get("mode", "observe") != "observe" or not config.get("no_publish", True):
        raise fail("Kinect preflight requires mode=observe and no_publish=true")
    if not args.bridge.is_file():
        raise fail(f"bridge executable not found: {args.bridge}")
    args.trace.parent.mkdir(parents=True, exist_ok=True)
    process = subprocess.Popen(
        [str(args.bridge), "--config", str(args.config), "--no-publish",
         "--record", str(args.trace), "--debug-skeleton"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    try:
        output, _ = process.communicate(timeout=args.duration)
    except subprocess.TimeoutExpired:
        process.send_signal(signal.SIGINT)
        output, _ = process.communicate(timeout=10)
    valid_trace = args.trace.is_file() and args.trace.stat().st_size > 8
    passed = process.returncode == 0 and valid_trace and "bridge_state=" in output
    result = base_report("PASS" if passed else "FAIL", args.config)
    result.update({"operation": "kinect-preflight", "duration_s": args.duration,
                   "trace": str(args.trace), "trace_bytes": args.trace.stat().st_size if valid_trace else 0,
                   "bridge_exit": process.returncode, "output_tail": output.splitlines()[-40:]})
    write_report(args.report, result)
    return 0 if passed else 1


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    commands = result.add_subparsers(dest="command", required=True)
    config = commands.add_parser("config", help="validate a software bridge configuration")
    config.add_argument("--config", required=True, type=Path)
    config.add_argument("--report", type=Path)
    config.set_defaults(handler=command_config)
    monitor = commands.add_parser("monitor", help="summarize JSONL telemetry without opening hardware")
    monitor.add_argument("--logs", required=True, nargs="+")
    monitor.add_argument("--report", type=Path)
    monitor.set_defaults(handler=command_monitor)
    joint_map = commands.add_parser("joint-map", help="verify named SDK/MuJoCo joint order and limits")
    joint_map.add_argument("--controller-root", type=Path, default=SONIC_ROOT)
    joint_map.add_argument("--report", type=Path)
    joint_map.set_defaults(handler=command_joint_map)
    retarget = commands.add_parser("retarget", help="validate retargeted joint targets and feasibility")
    retarget.add_argument("--case", required=True, type=Path)
    retarget.add_argument("--config", type=Path)
    retarget.add_argument("--controller-root", type=Path, default=SONIC_ROOT)
    retarget.add_argument("--report", type=Path)
    retarget.set_defaults(handler=command_retarget)
    replay = commands.add_parser("replay", help="validate a portable deterministic bridge trace")
    replay.add_argument("--trace", required=True, type=Path)
    replay.add_argument("--report", type=Path)
    replay.set_defaults(handler=command_replay)
    health = commands.add_parser("health", help="check supplied evidence paths without granting hardware access")
    health.add_argument("--config", required=True, type=Path)
    health.add_argument("--gate", required=True)
    health.add_argument("--evidence", nargs="*", default=[])
    health.add_argument("--report", type=Path)
    health.set_defaults(handler=command_health)
    sim_validate = commands.add_parser("sim-validate", help="validate isolated SONIC/MuJoCo logs")
    sim_validate.add_argument("--controller-log", required=True, type=Path)
    sim_validate.add_argument("--sim-log", required=True, type=Path)
    sim_validate.add_argument("--min-sim-time", type=float, default=1800.0)
    sim_validate.add_argument("--report", type=Path)
    sim_validate.set_defaults(handler=command_sim_validate)
    kinect = commands.add_parser("kinect-preflight", help="run an explicit observe-only Kinect check")
    kinect.add_argument("--config", required=True, type=Path)
    kinect.add_argument("--bridge", type=Path, default=ROOT / "build/kinect_smpl_zmq_bridge")
    kinect.add_argument("--duration", type=float, default=600.0)
    kinect.add_argument("--trace", type=Path, default=ROOT / "artifacts/kinect/phase9.trace")
    kinect.add_argument("--execute", action="store_true",
                        help="open the Kinect; omit to return HARDWARE_VALIDATION_REQUIRED")
    kinect.add_argument("--report", type=Path)
    kinect.set_defaults(handler=command_kinect_preflight)
    for name in ("sim-live", "real-preflight"):
        blocked = commands.add_parser(name, help="report that the requested hardware phase is blocked")
        blocked.add_argument("--config", required=True, type=Path)
        blocked.add_argument("--report", type=Path)
        blocked.set_defaults(handler=command_hardware_blocked)
    return result


def main() -> int:
    args = parser().parse_args()
    try:
        return args.handler(args)
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        result = base_report("FAIL")
        result["error"] = str(error)
        write_report(getattr(args, "report", None), result)
        return 1


if __name__ == "__main__":
    sys.exit(main())
