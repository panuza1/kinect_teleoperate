#pragma once

#include "bridge_config.hpp"
#include "skeleton_to_smpl.hpp"

#include <cstdint>
#include <fstream>
#include <memory>
#include <optional>
#include <vector>

enum class BridgeState : std::uint8_t {
    NO_BODY, CALIBRATING, READY, TRACKING, LOW_CONFIDENCE, TIMEOUT
};

struct BridgeResult {
    SonicPoseFrame pose{};
    BridgeState state = BridgeState::NO_BODY;
    bool publish = false;
    bool planner = false;
    bool dispatch = false;
    bool stop = false;
    std::uint64_t epoch = 0;
    std::uint64_t sequence = 0;
    std::uint64_t source_timestamp_us = 0;
};

struct SessionHealth {
    bool calibrated = false;
    bool armed = false;
    std::uint64_t epoch = 0;
    std::uint64_t accepted = 0;
    std::uint64_t rejected = 0;
    std::uint64_t last_valid_us = 0;
};

class BridgeSession {
public:
    explicit BridgeSession(const BridgeConfig& config = {});
    BridgeResult process(const std::optional<SkeletonSample>& sample, std::uint64_t now_us);
    BridgeResult process(const std::vector<SkeletonSample>& bodies, std::uint64_t now_us);
    BridgeResult timeout(std::uint64_t now_us);
    bool request_arm();
    void request_stop();
    void reset_epoch();
    SessionHealth health() const;

private:
    BridgeResult fallback(BridgeState state, std::uint64_t now_us);
    void reset_calibration();

    std::unique_ptr<SkeletonToSmpl> converter_ = std::make_unique<SkeletonToSmpl>(0.75f, false);
    std::vector<BodySkeleton> neutral_frames_;
    std::optional<std::uint32_t> body_id_;
    const std::optional<std::uint32_t> selected_body_id_;
    const std::string calibration_path_;
    const std::string device_serial_;
    const std::string mount_id_;
    const bool calibration_required_;
    const std::uint64_t tracking_inhibit_us_;
    const std::uint64_t tracking_stop_us_;
    const std::uint64_t joint_hold_us_;
    const bool locomotion_enabled_;
    const float max_forward_mps_;
    const float max_lateral_mps_;
    const float max_yaw_rps_;
    const float max_linear_accel_mps2_;
    const float max_yaw_accel_rps2_;
    const BridgeMode mode_;
    std::uint64_t calibration_start_us_ = 0;
    std::uint64_t previous_device_us_ = 0;
    std::uint64_t last_valid_us_ = 0;
    std::uint64_t last_output_us_ = 0;
    std::uint64_t frame_index_ = 0;
    bool calibrated_ = false;
    bool have_neutral_ = false;
    bool planner_ = false;
    bool armed_ = false;
    std::uint64_t epoch_ = 1;
    std::uint64_t sequence_ = 0;
    std::uint64_t accepted_ = 0;
    std::uint64_t rejected_ = 0;
    std::array<JointSample, JOINT_COUNT> last_joint_{};
    std::array<std::uint64_t, JOINT_COUNT> last_joint_us_{};
    std::optional<Position3> previous_pelvis_;
    SonicPoseFrame neutral_{};
    SonicPoseFrame last_output_{};
};

// Local replay format: fixed ABI, versioned, with raw skeleton and computed result.
struct BridgeTraceFrame {
    std::uint64_t time_us = 0;
    SkeletonSample sample{};
    BridgeResult result{};
    bool has_body = false;
    bool capture_timeout = false;
};

void write_trace_header(std::ofstream& file);
void write_trace_frame(std::ofstream& file, const BridgeTraceFrame& frame);
bool read_trace_frame(std::ifstream& file, BridgeTraceFrame& frame);
