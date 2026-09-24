#include "calibration.hpp"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <unistd.h>

int main() {
    std::vector<std::array<float, 3>> floor;
    for (int x = -3; x <= 3; ++x)
        for (int y = -3; y <= 3; ++y)
            floor.push_back({x * .2f, y * .2f, .1f * x * .2f - .2f * y * .2f + .3f});
    const auto plane = fit_floor(floor);
    assert(plane.rms_residual_m < 1e-5f && plane.normal[2] > .97f);
    assert(std::fabs(plane.normal[0] / plane.normal[2] + .1f) < 1e-4f);
    assert(std::fabs(plane.normal[1] / plane.normal[2] - .2f) < 1e-4f);
    bool rejected = false;
    try { fit_floor({{{0,0,0}}, {{1,0,0}}, {{2,0,0}}}); } catch (const std::runtime_error&) { rejected = true; }
    assert(rejected);

    const auto heading = heading_alignment(0, 1);
    assert(std::fabs(heading[1] - 1) < 1e-6f && std::fabs(heading[3] + 1) < 1e-6f);

    CalibrationState state;
    state.device_serial = "synthetic-device";
    state.mount_id = "synthetic-mount";
    for (int joint = 0; joint < 24; ++joint) state.pose_calibration[joint * 4] = 1;
    const auto path = std::filesystem::temp_directory_path() /
        ("kinect_calibration_" + std::to_string(getpid()) + ".json");
    state.save(path.string());
    const auto loaded = CalibrationState::load(path.string(), state.device_serial, state.mount_id);
    assert(loaded.camera_to_body_rotation == state.camera_to_body_rotation);
    rejected = false;
    try { CalibrationState::load(path.string(), "wrong-device", state.mount_id); }
    catch (const std::runtime_error&) { rejected = true; }
    assert(rejected);
    std::filesystem::remove(path);
}
