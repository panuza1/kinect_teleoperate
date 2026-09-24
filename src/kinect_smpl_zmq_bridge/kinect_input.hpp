#pragma once

#include "bridge_config.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

enum JointId : std::size_t {
    JOINT_PELVIS, JOINT_SPINE_NAVEL, JOINT_SPINE_CHEST, JOINT_NECK,
    JOINT_CLAVICLE_LEFT, JOINT_SHOULDER_LEFT, JOINT_ELBOW_LEFT, JOINT_WRIST_LEFT,
    JOINT_HAND_LEFT, JOINT_HANDTIP_LEFT, JOINT_THUMB_LEFT,
    JOINT_CLAVICLE_RIGHT, JOINT_SHOULDER_RIGHT, JOINT_ELBOW_RIGHT, JOINT_WRIST_RIGHT,
    JOINT_HAND_RIGHT, JOINT_HANDTIP_RIGHT, JOINT_THUMB_RIGHT,
    JOINT_HIP_LEFT, JOINT_KNEE_LEFT, JOINT_ANKLE_LEFT, JOINT_FOOT_LEFT,
    JOINT_HIP_RIGHT, JOINT_KNEE_RIGHT, JOINT_ANKLE_RIGHT, JOINT_FOOT_RIGHT,
    JOINT_HEAD, JOINT_NOSE, JOINT_EYE_LEFT, JOINT_EAR_LEFT, JOINT_EYE_RIGHT,
    JOINT_EAR_RIGHT, JOINT_COUNT
};

enum JointConfidence : std::uint8_t {
    CONFIDENCE_NONE, CONFIDENCE_LOW, CONFIDENCE_MEDIUM, CONFIDENCE_HIGH
};

struct Position3 { struct { float x = 0, y = 0, z = 0; } xyz; };
struct Quaternion4 { struct { float w = 1, x = 0, y = 0, z = 0; } wxyz; };
struct JointSample {
    Position3 position{};
    Quaternion4 orientation{};
    JointConfidence confidence_level = CONFIDENCE_NONE;
};
struct BodySkeleton { std::array<JointSample, JOINT_COUNT> joints{}; };
struct SkeletonSample {
    BodySkeleton skeleton{};
    std::uint32_t body_id = 0;
    std::uint64_t device_timestamp_us = 0;
};

enum class AcquisitionEvent : std::uint8_t {
    FRAME, TIMEOUT, NO_BODY, MULTIPLE_BODIES, MISSING_STREAMS, DISCONNECTED, TRACKER_ERROR
};

struct KinectFrame {
    AcquisitionEvent event = AcquisitionEvent::TIMEOUT;
    std::vector<SkeletonSample> bodies;
    std::uint64_t color_timestamp_us = 0;
    std::uint64_t depth_timestamp_us = 0;
    std::string detail;
};

class KinectInput {
public:
    explicit KinectInput(BridgeConfig config);
    ~KinectInput();
    KinectInput(const KinectInput&) = delete;
    KinectInput& operator=(const KinectInput&) = delete;

    void open();
    KinectFrame poll(int timeout_ms = 100);
    void close();
    void reconnect();
    bool is_open() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
