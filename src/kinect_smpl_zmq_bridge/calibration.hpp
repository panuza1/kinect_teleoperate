#pragma once

#include <array>
#include <string>
#include <vector>

struct FloorPlane {
    std::array<float, 3> normal{0, 0, 1};
    float offset_m = 0;
    float rms_residual_m = 0;
};

struct CalibrationState {
    int schema_version = 1;
    std::string device_serial;
    std::string mount_id;
    std::array<float, 9> camera_to_body_rotation{1,0,0, 0,1,0, 0,0,1};
    std::array<float, 3> camera_to_body_translation_m{};
    float body_scale = 1;
    std::array<float, 4> root_calibration{1,0,0,0};
    std::array<float, 3> neutral_pelvis_m{};
    std::array<float, 24 * 4> pose_calibration{};

    void validate(const std::string& expected_serial = {}, const std::string& expected_mount = {}) const;
    void save(const std::string& path) const;
    static CalibrationState load(const std::string& path, const std::string& expected_serial = {},
                                 const std::string& expected_mount = {});
};

FloorPlane fit_floor(const std::vector<std::array<float, 3>>& points_m);
std::array<float, 9> heading_alignment(float forward_x, float forward_y);
