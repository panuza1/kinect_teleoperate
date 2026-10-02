#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

struct G1Reference {
    std::array<double, 29> joint_pos{};
    std::array<double, 29> joint_vel{};
    std::uint64_t timestamp_us = 0;
};

enum class KinectJoint : std::size_t {
    Pelvis, SpineNavel, SpineChest, Neck, Head,
    LeftShoulder, LeftElbow, LeftWrist,
    RightShoulder, RightElbow, RightWrist,
    LeftHip, LeftKnee, LeftAnkle, LeftFoot,
    RightHip, RightKnee, RightAnkle, RightFoot,
    Count
};

struct KinectJointSample {
    std::array<double, 3> position_mm{};
    std::array<double, 4> orientation_wxyz{1.0, 0.0, 0.0, 0.0};
    std::uint8_t confidence = 0;
};

struct KinectSkeletonSample {
    std::array<KinectJointSample, static_cast<std::size_t>(KinectJoint::Count)> joints{};
    std::uint64_t timestamp_us = 0;
    std::uint32_t body_id = 0;
};

struct RetargetStats {
    std::array<bool, static_cast<std::size_t>(KinectJoint::Count)> held{};
    std::array<bool, static_cast<std::size_t>(KinectJoint::Count)> stale{};
    std::size_t valid_joints = 0;
    std::size_t held_joints = 0;
    std::size_t stale_joints = 0;
    bool calibrated = false;
    bool accepted = false;
};

class KinectToG1Retargeter {
public:
    struct Config {
        std::uint64_t hold_us = 150000;
        double smoothing = 0.35;
        double max_velocity = 12.0;
        double max_step = 2.5;
    };

    KinectToG1Retargeter();
    explicit KinectToG1Retargeter(Config config);

    G1Reference update(const KinectSkeletonSample& skeleton);
    void reset();
    const RetargetStats& stats() const { return stats_; }

    static const std::array<const char*, 29> joint_names;
    static const std::array<const char*, 29> actuator_names;
    static const std::array<double, 29> neutral;
    static const std::array<double, 29> lower_limits;
    static const std::array<double, 29> upper_limits;

private:
    struct Quaternion { double w, x, y, z; };
    struct Vec3 { double x, y, z; };

    Config config_;
    std::array<KinectJointSample, static_cast<std::size_t>(KinectJoint::Count)> cached_{};
    std::array<std::uint64_t, static_cast<std::size_t>(KinectJoint::Count)> cached_at_{};
    std::array<Quaternion, 9> neutral_relative_{};
    std::array<double, 4> neutral_bend_{};
    G1Reference previous_{};
    RetargetStats stats_{};
    bool calibrated_ = false;

    static bool finite(const KinectJointSample& joint);
    static Quaternion quaternion(const KinectJointSample& joint);
    static Quaternion relative(Quaternion parent, Quaternion child);
    static Quaternion camera_to_robot(Quaternion value);
    static std::array<double, 3> euler_xyz(Quaternion value);
    static Vec3 robot_position(const KinectJointSample& joint);
    static double bend(Vec3 parent, Vec3 joint, Vec3 child);
    static double wrapped_delta(double a, double b);
};
