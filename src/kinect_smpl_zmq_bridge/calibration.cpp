#include "calibration.hpp"

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace {
template <std::size_t N>
void read_array(const YAML::Node& node, std::array<float, N>& output, const char* name) {
    if (!node || !node.IsSequence() || node.size() != N)
        throw std::runtime_error(std::string("invalid calibration field: ") + name);
    for (std::size_t i = 0; i < N; ++i) output[i] = node[i].as<float>();
}

template <std::size_t N>
YAML::Node array_node(const std::array<float, N>& values) {
    YAML::Node node(YAML::NodeType::Sequence);
    for (const float value : values) node.push_back(value);
    return node;
}

std::string checksum(const CalibrationState& state) {
    std::ostringstream text;
    text << state.schema_version << '|' << state.device_serial << '|' << state.mount_id
         << '|' << std::setprecision(9) << state.body_scale;
    for (float v : state.camera_to_body_rotation) text << '|' << v;
    for (float v : state.camera_to_body_translation_m) text << '|' << v;
    for (float v : state.root_calibration) text << '|' << v;
    for (float v : state.neutral_pelvis_m) text << '|' << v;
    for (float v : state.pose_calibration) text << '|' << v;
    std::uint64_t hash = 1469598103934665603ULL;
    for (const unsigned char c : text.str()) hash = (hash ^ c) * 1099511628211ULL;
    std::ostringstream rendered;
    rendered << std::hex << std::setw(16) << std::setfill('0') << hash;
    return rendered.str();
}
}

void CalibrationState::validate(const std::string& expected_serial,
                                const std::string& expected_mount) const {
    if (schema_version != 1) throw std::runtime_error("unsupported calibration schema");
    if (!expected_serial.empty() && device_serial != expected_serial)
        throw std::runtime_error("calibration device serial mismatch");
    if (!expected_mount.empty() && mount_id != expected_mount)
        throw std::runtime_error("calibration mount mismatch");
    Eigen::Matrix3f rotation;
    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column)
            rotation(row, column) = camera_to_body_rotation[row * 3 + column];
    if (!rotation.allFinite() || (rotation.transpose() * rotation - Eigen::Matrix3f::Identity()).norm() > 1e-3f ||
        std::fabs(rotation.determinant() - 1.0f) > 1e-3f)
        throw std::runtime_error("calibration rotation is not proper orthonormal");
    if (!std::isfinite(body_scale) || body_scale < 0.8f || body_scale > 1.2f)
        throw std::runtime_error("calibration body scale outside [0.8, 1.2]");
    for (float v : camera_to_body_translation_m) if (!std::isfinite(v))
        throw std::runtime_error("nonfinite calibration translation");
    for (float v : root_calibration) if (!std::isfinite(v))
        throw std::runtime_error("nonfinite root calibration");
    float root_norm = 0;
    for (float v : root_calibration) root_norm += v * v;
    if (std::fabs(root_norm - 1.0f) > 1e-3f) throw std::runtime_error("invalid root calibration quaternion");
    for (float v : neutral_pelvis_m) if (!std::isfinite(v))
        throw std::runtime_error("nonfinite neutral pelvis");
    for (float v : pose_calibration) if (!std::isfinite(v))
        throw std::runtime_error("nonfinite pose calibration");
    for (int joint = 0; joint < 24; ++joint) {
        float norm = 0;
        for (int component = 0; component < 4; ++component)
            norm += pose_calibration[joint * 4 + component] * pose_calibration[joint * 4 + component];
        if (std::fabs(norm - 1.0f) > 1e-3f) throw std::runtime_error("invalid pose calibration quaternion");
    }
}

void CalibrationState::save(const std::string& path) const {
    validate();
    YAML::Node root;
    root["schema_version"] = schema_version;
    root["device_serial"] = device_serial;
    root["mount_id"] = mount_id;
    root["camera_to_body_rotation"] = array_node(camera_to_body_rotation);
    root["camera_to_body_translation_m"] = array_node(camera_to_body_translation_m);
    root["body_scale"] = body_scale;
    root["root_calibration"] = array_node(root_calibration);
    root["neutral_pelvis_m"] = array_node(neutral_pelvis_m);
    root["pose_calibration"] = array_node(pose_calibration);
    root["checksum"] = checksum(*this);
    const std::filesystem::path output(path);
    const auto temporary = output.string() + ".tmp";
    std::ofstream file(temporary, std::ios::trunc);
    if (!file || !(file << root << '\n')) throw std::runtime_error("failed to write calibration");
    file.close();
    if (std::rename(temporary.c_str(), output.c_str()) != 0) {
        std::remove(temporary.c_str());
        throw std::runtime_error("failed to atomically install calibration");
    }
}

CalibrationState CalibrationState::load(const std::string& path, const std::string& expected_serial,
                                        const std::string& expected_mount) {
    const YAML::Node root = YAML::LoadFile(path);
    CalibrationState state;
    state.schema_version = root["schema_version"].as<int>();
    state.device_serial = root["device_serial"].as<std::string>();
    state.mount_id = root["mount_id"].as<std::string>();
    read_array(root["camera_to_body_rotation"], state.camera_to_body_rotation, "rotation");
    read_array(root["camera_to_body_translation_m"], state.camera_to_body_translation_m, "translation");
    state.body_scale = root["body_scale"].as<float>();
    read_array(root["root_calibration"], state.root_calibration, "root_calibration");
    read_array(root["neutral_pelvis_m"], state.neutral_pelvis_m, "neutral_pelvis_m");
    read_array(root["pose_calibration"], state.pose_calibration, "pose_calibration");
    if (!root["checksum"] || root["checksum"].as<std::string>() != checksum(state))
        throw std::runtime_error("calibration checksum mismatch");
    state.validate(expected_serial, expected_mount);
    return state;
}

FloorPlane fit_floor(const std::vector<std::array<float, 3>>& points_m) {
    if (points_m.size() < 3) throw std::runtime_error("floor fit requires at least three points");
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (const auto& p : points_m) {
        const Eigen::Vector3f point(p[0], p[1], p[2]);
        if (!point.allFinite()) throw std::runtime_error("nonfinite floor point");
        centroid += point;
    }
    centroid /= static_cast<float>(points_m.size());
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (const auto& p : points_m) {
        const Eigen::Vector3f delta = Eigen::Vector3f(p[0], p[1], p[2]) - centroid;
        covariance += delta * delta.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    if (solver.info() != Eigen::Success || solver.eigenvalues()[1] < 1e-8f)
        throw std::runtime_error("degenerate floor points");
    Eigen::Vector3f normal = solver.eigenvectors().col(0).normalized();
    if (normal.z() < 0) normal = -normal;
    FloorPlane result{{normal.x(), normal.y(), normal.z()}, -normal.dot(centroid), 0};
    float squared = 0;
    for (const auto& p : points_m) {
        const float distance = normal.dot(Eigen::Vector3f(p[0], p[1], p[2])) + result.offset_m;
        squared += distance * distance;
    }
    result.rms_residual_m = std::sqrt(squared / points_m.size());
    return result;
}

std::array<float, 9> heading_alignment(float forward_x, float forward_y) {
    const float length = std::hypot(forward_x, forward_y);
    if (!std::isfinite(length) || length < 1e-4f) throw std::runtime_error("invalid heading vector");
    const float c = forward_x / length;
    const float s = forward_y / length;
    return {c, s, 0, -s, c, 0, 0, 0, 1};
}
