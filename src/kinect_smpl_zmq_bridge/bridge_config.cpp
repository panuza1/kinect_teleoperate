#include "bridge_config.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>

namespace {
template <typename T>
void read_if(const YAML::Node& node, const char* key, T& value) {
    if (node[key]) value = node[key].as<T>();
}

void reject_unknown(const YAML::Node& node, std::initializer_list<const char*> allowed,
                    const char* section) {
    if (!node || !node.IsMap()) return;
    for (const auto& item : node) {
        const auto key = item.first.as<std::string>();
        bool known = false;
        for (const char* candidate : allowed) known |= key == candidate;
        if (!known) throw std::runtime_error(std::string("unknown config key ") + section + "." + key);
    }
}

bool finite_nonnegative(float value) { return std::isfinite(value) && value >= 0.0f; }
}  // namespace

BridgeMode parse_bridge_mode(const std::string& value) {
    if (value == "observe") return BridgeMode::OBSERVE;
    if (value == "compute") return BridgeMode::COMPUTE;
    if (value == "sim") return BridgeMode::SIM;
    if (value == "real") return BridgeMode::REAL;
    throw std::runtime_error("invalid bridge mode: " + value);
}

const char* bridge_mode_name(BridgeMode mode) {
    switch (mode) {
        case BridgeMode::OBSERVE: return "observe";
        case BridgeMode::COMPUTE: return "compute";
        case BridgeMode::SIM: return "sim";
        case BridgeMode::REAL: return "real";
    }
    return "invalid";
}

BridgeConfig BridgeConfig::load(const std::string& path) {
    YAML::Node root = YAML::LoadFile(path);
    if (root["mode"] && root["mode"].as<std::string>() == "real")
        throw std::runtime_error("real mode is not enabled by this bridge configuration");
    reject_unknown(root, {"schema_version", "mode", "no_publish", "debug_skeleton", "cpu", "port",
                          "bind_host", "model_path", "telemetry_path", "calibration_path", "device_serial", "mount_id", "calibration_required",
                          "selection", "timeouts", "locomotion"}, "root");
    BridgeConfig config;
    read_if(root, "schema_version", config.schema_version);
    if (root["mode"]) config.mode = parse_bridge_mode(root["mode"].as<std::string>());
    read_if(root, "no_publish", config.no_publish);
    read_if(root, "debug_skeleton", config.debug_skeleton);
    read_if(root, "cpu", config.cpu);
    read_if(root, "port", config.port);
    read_if(root, "bind_host", config.bind_host);
    read_if(root, "model_path", config.model_path);
    read_if(root, "telemetry_path", config.telemetry_path);
    read_if(root, "calibration_path", config.calibration_path);
    read_if(root, "device_serial", config.device_serial);
    read_if(root, "mount_id", config.mount_id);
    read_if(root, "calibration_required", config.calibration_required);
    if (const auto selection = root["selection"]) {
        reject_unknown(selection, {"body_id"}, "selection");
        if (selection["body_id"]) config.selected_body_id = selection["body_id"].as<std::uint32_t>();
    }
    if (const auto timeouts = root["timeouts"]) {
        reject_unknown(timeouts, {"tracking_inhibit_ms", "tracking_stop_ms", "joint_hold_ms"}, "timeouts");
        if (timeouts["tracking_inhibit_ms"])
            config.tracking_inhibit_us = timeouts["tracking_inhibit_ms"].as<std::uint64_t>() * 1000;
        if (timeouts["tracking_stop_ms"])
            config.tracking_stop_us = timeouts["tracking_stop_ms"].as<std::uint64_t>() * 1000;
        if (timeouts["joint_hold_ms"])
            config.joint_hold_us = timeouts["joint_hold_ms"].as<std::uint64_t>() * 1000;
    }
    if (const auto locomotion = root["locomotion"]) {
        reject_unknown(locomotion, {"enabled", "max_forward_mps", "max_lateral_mps", "max_yaw_rps",
                                    "max_linear_accel_mps2", "max_yaw_accel_rps2"}, "locomotion");
        read_if(locomotion, "enabled", config.locomotion_enabled);
        read_if(locomotion, "max_forward_mps", config.max_forward_mps);
        read_if(locomotion, "max_lateral_mps", config.max_lateral_mps);
        read_if(locomotion, "max_yaw_rps", config.max_yaw_rps);
        read_if(locomotion, "max_linear_accel_mps2", config.max_linear_accel_mps2);
        read_if(locomotion, "max_yaw_accel_rps2", config.max_yaw_accel_rps2);
    }
    config.validate();
    return config;
}

void BridgeConfig::validate() const {
    if (schema_version != 1) throw std::runtime_error("unsupported bridge config schema");
    if (port < 1 || port > 65535 || bind_host.empty()) throw std::runtime_error("invalid transport config");
    if (!tracking_inhibit_us || tracking_inhibit_us > tracking_stop_us || !joint_hold_us || joint_hold_us > tracking_stop_us)
        throw std::runtime_error("invalid tracking timeout config");
    if (!finite_nonnegative(max_forward_mps) || !finite_nonnegative(max_lateral_mps) ||
        !finite_nonnegative(max_yaw_rps) || !finite_nonnegative(max_linear_accel_mps2) ||
        !finite_nonnegative(max_yaw_accel_rps2)) throw std::runtime_error("invalid locomotion limits");
    if (mode == BridgeMode::REAL) throw std::runtime_error("real mode is not enabled by this bridge configuration");
    if (mode != BridgeMode::SIM && !no_publish) throw std::runtime_error("only isolated sim mode may publish");
    if (calibration_required && calibration_path.empty())
        throw std::runtime_error("calibration_required needs calibration_path");
}

bool BridgeConfig::may_publish() const { return mode == BridgeMode::SIM && !no_publish; }
