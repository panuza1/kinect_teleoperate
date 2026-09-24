#pragma once

#include <cstdint>
#include <optional>
#include <string>

enum class BridgeMode { OBSERVE, COMPUTE, SIM, REAL };

struct BridgeConfig {
    int schema_version = 1;
    BridgeMode mode = BridgeMode::OBSERVE;
    bool no_publish = true;
    bool debug_skeleton = false;
    bool cpu = false;
    int port = 5556;
    std::string bind_host = "127.0.0.1";
    std::string model_path;
    std::string telemetry_path;
    std::string calibration_path;
    std::string device_serial;
    std::string mount_id;
    bool calibration_required = false;
    std::optional<std::uint32_t> selected_body_id;
    std::uint64_t tracking_inhibit_us = 100000;
    std::uint64_t tracking_stop_us = 200000;
    std::uint64_t joint_hold_us = 100000;
    bool locomotion_enabled = false;
    float max_forward_mps = 0.15f;
    float max_lateral_mps = 0.10f;
    float max_yaw_rps = 0.20f;
    float max_linear_accel_mps2 = 0.5f;
    float max_yaw_accel_rps2 = 0.8f;

    static BridgeConfig load(const std::string& path);
    void validate() const;
    bool may_publish() const;
};

BridgeMode parse_bridge_mode(const std::string& value);
const char* bridge_mode_name(BridgeMode mode);
