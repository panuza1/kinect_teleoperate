#pragma once

#include <k4abt.h>

#include <array>
#include <cstdint>
#include <vector>

struct SonicPoseFrame {
    std::array<float, 24 * 3> smpl_joints{};
    std::array<float, 21 * 3> smpl_pose{};
    std::array<float, 4> body_quat_w{1.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, 3> pelvis_position_m{};
    std::array<float, 3> velocity_mps{};
    float yaw_rate_rps = 0.0f;
    double timestamp_monotonic_s = 0.0;
};

class SkeletonToSmpl {
public:
    explicit SkeletonToSmpl(float smoothing = 0.75f, bool auto_calibrate = true);
    ~SkeletonToSmpl();

    bool calibrate(const std::vector<k4abt_skeleton_t>& neutral_frames);
    bool convert(const k4abt_skeleton_t& skeleton, std::uint64_t frame_index,
                 float dt_s, SonicPoseFrame& output);

private:
    float smoothing_;
    bool auto_calibrate_;
    bool calibrated_ = false;
    bool have_previous_ = false;
    float body_scale_ = 1.0f;
    std::uint64_t frame_index_ = 0;
    std::array<float, 24 * 3> last_joints_{};
    std::array<float, 21 * 3> last_pose_{};
    std::array<float, 4> last_root_{1.0f, 0.0f, 0.0f, 0.0f};
    std::array<float, 3> last_pelvis_{};
    std::array<float, 3> filtered_velocity_{};
    float last_yaw_ = 0.0f;
    float filtered_yaw_rate_ = 0.0f;

    struct Impl;
    Impl* impl_;
};
