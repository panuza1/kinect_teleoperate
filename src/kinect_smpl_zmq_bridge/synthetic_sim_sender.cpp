#include "skeleton_to_smpl.hpp"
#include "sonic_zmq_publisher.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace {
void set(k4abt_skeleton_t& s, k4abt_joint_id_t id, float x, float y, float z = 1000.0f) {
    auto& j = s.joints[id];
    j.position.xyz = {x, y, z};
    j.orientation.wxyz = {1, 0, 0, 0};
    j.confidence_level = K4ABT_JOINT_CONFIDENCE_HIGH;
}

k4abt_skeleton_t neutral_skeleton() {
    k4abt_skeleton_t s{};
    for (auto& j : s.joints) {
        j.position.xyz = {0, 0, 1000};
        j.orientation.wxyz = {1, 0, 0, 0};
        j.confidence_level = K4ABT_JOINT_CONFIDENCE_HIGH;
    }
    set(s, K4ABT_JOINT_PELVIS, 0, 0);
    set(s, K4ABT_JOINT_SPINE_NAVEL, 0, -160);
    set(s, K4ABT_JOINT_SPINE_CHEST, 0, -350);
    set(s, K4ABT_JOINT_NECK, 0, -540);
    set(s, K4ABT_JOINT_HEAD, 0, -710);
    for (int side : {-1, 1}) {
        const bool left = side < 0;
        set(s, left ? K4ABT_JOINT_HIP_LEFT : K4ABT_JOINT_HIP_RIGHT, 100 * side, 50);
        set(s, left ? K4ABT_JOINT_KNEE_LEFT : K4ABT_JOINT_KNEE_RIGHT, 110 * side, 430);
        set(s, left ? K4ABT_JOINT_ANKLE_LEFT : K4ABT_JOINT_ANKLE_RIGHT, 110 * side, 790);
        set(s, left ? K4ABT_JOINT_FOOT_LEFT : K4ABT_JOINT_FOOT_RIGHT, 110 * side, 900, 1080);
        set(s, left ? K4ABT_JOINT_CLAVICLE_LEFT : K4ABT_JOINT_CLAVICLE_RIGHT, 140 * side, -390);
        set(s, left ? K4ABT_JOINT_SHOULDER_LEFT : K4ABT_JOINT_SHOULDER_RIGHT, 250 * side, -400);
        set(s, left ? K4ABT_JOINT_ELBOW_LEFT : K4ABT_JOINT_ELBOW_RIGHT, 280 * side, -200);
        set(s, left ? K4ABT_JOINT_WRIST_LEFT : K4ABT_JOINT_WRIST_RIGHT, 300 * side, 0);
        set(s, left ? K4ABT_JOINT_HAND_LEFT : K4ABT_JOINT_HAND_RIGHT, 310 * side, 70);
    }
    return s;
}

void raise_arm(k4abt_skeleton_t& s, bool left, float amount) {
    const float side = left ? -1.0f : 1.0f;
    set(s, left ? K4ABT_JOINT_ELBOW_LEFT : K4ABT_JOINT_ELBOW_RIGHT,
        side * 280, -200 - 330 * amount);
    set(s, left ? K4ABT_JOINT_WRIST_LEFT : K4ABT_JOINT_WRIST_RIGHT,
        side * 300, -700 * amount);
    set(s, left ? K4ABT_JOINT_HAND_LEFT : K4ABT_JOINT_HAND_RIGHT,
        side * 310, 70 - 820 * amount);
}

float ramp(float t, float begin, float hold_end) {
    return std::clamp(std::min((t - begin) / 2.0f, (hold_end + 2.0f - t) / 2.0f), 0.0f, 1.0f);
}
}

int main(int argc, char** argv) {
    const float seconds = argc > 1 ? std::atof(argv[1]) : 145.0f;
    if (seconds < 1.0f || seconds > 600.0f) return 2;
    SkeletonToSmpl converter;
    SonicZmqPublisher publisher(5556, "127.0.0.1");
    const auto start = std::chrono::steady_clock::now();
    const auto period = std::chrono::duration<double>(1.0 / 30.0);
    bool planner_mode = false;
    for (std::uint64_t frame_index = 0; frame_index < static_cast<std::uint64_t>(seconds * 30); ++frame_index) {
        const float t = static_cast<float>(frame_index) / 30.0f;
        auto s = neutral_skeleton();
        raise_arm(s, true, ramp(t, 45.0f, 50.0f) + ramp(t, 75.0f, 80.0f));
        raise_arm(s, false, ramp(t, 60.0f, 65.0f) + ramp(t, 75.0f, 80.0f));
        if (t >= 90.0f && t < 97.0f) {
            const float yaw = 0.15f * ramp(t, 90.0f, 95.0f);
            for (auto& j : s.joints) {
                const float x = j.position.xyz.x;
                const float z = j.position.xyz.z - 1000.0f;
                j.position.xyz.x = std::cos(yaw) * x + std::sin(yaw) * z;
                j.position.xyz.z = 1000.0f - std::sin(yaw) * x + std::cos(yaw) * z;
                j.orientation.wxyz = {std::cos(yaw / 2), 0, std::sin(yaw / 2), 0};
            }
        }
        const float forward_mm = 150.0f * std::clamp(t - 115.0f, 0.0f, 5.0f);
        const float turn = 0.15f * std::clamp(t - 130.0f, 0.0f, 5.0f);
        for (auto& j : s.joints) {
            j.position.xyz.z += forward_mm;
            const float x = j.position.xyz.x;
            const float z = j.position.xyz.z - (1000.0f + forward_mm);
            j.position.xyz.x = std::cos(turn) * x + std::sin(turn) * z;
            j.position.xyz.z = 1000.0f + forward_mm - std::sin(turn) * x + std::cos(turn) * z;
            j.orientation.wxyz = {std::cos(turn / 2), 0, std::sin(turn / 2), 0};
        }
        SonicPoseFrame pose;
        if (!converter.convert(s, frame_index, 1.0f / 30.0f, pose)) return 3;
        const float speed = std::hypot(pose.velocity_mps[0], pose.velocity_mps[1]);
        const float yaw_rate = std::fabs(pose.yaw_rate_rps);
        if (speed > 0.08f || yaw_rate > 0.08f) planner_mode = true;
        else if (speed < 0.04f && yaw_rate < 0.04f) planner_mode = false;
        publisher.publish_command(false, planner_mode);
        publisher.publish_pose(pose, frame_index);
        publisher.publish_planner(pose);
        if (frame_index % 30 == 0) std::cout << "synthetic_frame=" << frame_index << " phase=" << t << '\n';
        std::this_thread::sleep_until(start + period * (frame_index + 1));
    }
}
