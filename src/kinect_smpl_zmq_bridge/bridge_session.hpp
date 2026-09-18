#pragma once

#include "skeleton_to_smpl.hpp"

#include <cstdint>
#include <fstream>
#include <memory>
#include <optional>
#include <vector>

enum class BridgeState : std::uint8_t {
    NO_BODY, CALIBRATING, READY, TRACKING, LOW_CONFIDENCE, TIMEOUT
};

struct SkeletonSample {
    k4abt_skeleton_t skeleton{};
    std::uint32_t body_id = 0;
    std::uint64_t device_timestamp_us = 0;
};

struct BridgeResult {
    SonicPoseFrame pose{};
    BridgeState state = BridgeState::NO_BODY;
    bool publish = false;
    bool planner = false;
};

class BridgeSession {
public:
    BridgeResult process(const std::optional<SkeletonSample>& sample, std::uint64_t now_us);
    BridgeResult timeout(std::uint64_t now_us);

private:
    BridgeResult fallback(BridgeState state, std::uint64_t now_us);
    void reset_calibration();

    std::unique_ptr<SkeletonToSmpl> converter_ = std::make_unique<SkeletonToSmpl>(0.75f, false);
    std::vector<k4abt_skeleton_t> neutral_frames_;
    std::optional<std::uint32_t> body_id_;
    std::uint64_t calibration_start_us_ = 0;
    std::uint64_t previous_device_us_ = 0;
    std::uint64_t last_valid_us_ = 0;
    std::uint64_t last_output_us_ = 0;
    std::uint64_t frame_index_ = 0;
    bool calibrated_ = false;
    bool have_neutral_ = false;
    bool planner_ = false;
    std::optional<k4a_float3_t> previous_pelvis_;
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
