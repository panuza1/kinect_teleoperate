#include "bridge_session.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <filesystem>
#include <limits>
#include <string>
#include <unistd.h>

namespace {
void set(BodySkeleton& s, JointId id, float x, float y, float z = 1000) {
    auto& j = s.joints[id];
    j.position.xyz = {x, y, z};
    j.orientation.wxyz = {1, 0, 0, 0};
    j.confidence_level = CONFIDENCE_HIGH;
}

BodySkeleton neutral() {
    BodySkeleton s{};
    for (auto& j : s.joints) {
        j.position.xyz = {0, 0, 1000};
        j.orientation.wxyz = {1, 0, 0, 0};
        j.confidence_level = CONFIDENCE_HIGH;
    }
    set(s, JOINT_PELVIS, 0, 0);
    set(s, JOINT_SPINE_NAVEL, 0, -180);
    set(s, JOINT_HEAD, 0, -710);
    set(s, JOINT_SPINE_CHEST, 0, -350);
    set(s, JOINT_NECK, 0, -540);
    set(s, JOINT_CLAVICLE_LEFT, -120, -390);
    set(s, JOINT_SHOULDER_LEFT, -250, -400);
    set(s, JOINT_ELBOW_LEFT, -300, -190);
    set(s, JOINT_SHOULDER_RIGHT, 250, -400);
    set(s, JOINT_CLAVICLE_RIGHT, 120, -390);
    set(s, JOINT_ELBOW_RIGHT, 300, -190);
    set(s, JOINT_WRIST_LEFT, -300, 0);
    set(s, JOINT_WRIST_RIGHT, 300, 0);
    set(s, JOINT_HAND_LEFT, -300, 60);
    set(s, JOINT_HANDTIP_LEFT, -300, 110);
    set(s, JOINT_THUMB_LEFT, -270, 60);
    set(s, JOINT_HAND_RIGHT, 300, 60);
    set(s, JOINT_HANDTIP_RIGHT, 300, 110);
    set(s, JOINT_THUMB_RIGHT, 270, 60);
    set(s, JOINT_HIP_LEFT, -100, 0);
    set(s, JOINT_KNEE_LEFT, -100, 400);
    set(s, JOINT_FOOT_LEFT, -100, 900);
    set(s, JOINT_HIP_RIGHT, 100, 0);
    set(s, JOINT_KNEE_RIGHT, 100, 400);
    set(s, JOINT_FOOT_RIGHT, 100, 900);
    set(s, JOINT_ANKLE_LEFT, -100, 790);
    set(s, JOINT_ANKLE_RIGHT, 100, 790);
    set(s, JOINT_NOSE, 0, -720);
    set(s, JOINT_EYE_LEFT, -30, -725);
    set(s, JOINT_EAR_LEFT, -70, -710);
    set(s, JOINT_EYE_RIGHT, 30, -725);
    set(s, JOINT_EAR_RIGHT, 70, -710);
    return s;
}

float joint_z(const BridgeResult& result, int smpl_joint) {
    return result.pose.smpl_joints[smpl_joint * 3 + 2];
}
}

int main(int argc, char** argv) {
    {
        BridgeSession slow_session;
        SkeletonSample slow_sample;
        slow_sample.skeleton = neutral();
        BridgeResult slow_result;
        for (int i = 0; i < 11; ++i) {
            slow_sample.device_timestamp_us += 200000;
            slow_result = slow_session.process(slow_sample, 1000000 + i * 200000);
        }
        assert(slow_result.state == BridgeState::READY && slow_result.publish);
    }
    BridgeConfig session_config;
    session_config.locomotion_enabled = true;
    session_config.mode = BridgeMode::SIM;
    BridgeSession session(session_config);
    SkeletonSample sample;
    sample.skeleton = neutral();
    sample.body_id = 7;
    std::uint64_t time = 1000000;
    auto step = [&] {
        time += 33334;
        sample.device_timestamp_us += 33334;
        return session.process(sample, time);
    };
    for (int i = 0; i < 59; ++i) {
        const auto result = step();
        assert(result.state == BridgeState::CALIBRATING && !result.publish);
    }
    BridgeResult result;
    for (int i = 0; i < 4; ++i) result = step();
    assert(result.publish && result.state == BridgeState::TRACKING && !result.planner);
    assert(session.request_arm());
    result = step();
    assert(result.dispatch && session.health().armed);
    const float left_neutral = joint_z(result, 20);
    const float right_neutral = joint_z(result, 21);

    sample.skeleton.joints[JOINT_WRIST_LEFT].position.xyz.y -= 250;
    for (int i = 0; i < 20; ++i) result = step();
    assert(joint_z(result, 20) > left_neutral + 0.20f);
    assert(std::fabs(joint_z(result, 21) - right_neutral) < 0.01f);
    sample.skeleton.joints[JOINT_WRIST_LEFT].position.xyz.y += 250;
    for (int i = 0; i < 40; ++i) result = step();
    assert(std::fabs(joint_z(result, 20) - left_neutral) < 0.001f);

    for (int i = 0; i < 30; ++i) {
        for (auto& j : sample.skeleton.joints) j.position.xyz.z += 5;
        result = step();
    }
    assert(result.planner && result.pose.velocity_mps[0] > 0.10f &&
           result.pose.velocity_mps[0] <= 0.15f);
    for (int i = 0; i < 40; ++i) result = step();
    assert(!result.planner);

    for (int i = 0; i < 30; ++i) {
        const float angle = 0.01f * i;
        for (auto& j : sample.skeleton.joints)
            j.orientation.wxyz = {std::cos(angle / 2), 0, std::sin(angle / 2), 0};
        result = step();
    }
    assert(result.planner && std::fabs(result.pose.yaw_rate_rps) > 0.10f &&
           std::fabs(result.pose.yaw_rate_rps) <= 0.20f);
    const float left_turn_rate = result.pose.yaw_rate_rps;
    for (int i = 29; i >= 0; --i) {
        const float angle = 0.01f * i;
        for (auto& j : sample.skeleton.joints)
            j.orientation.wxyz = {std::cos(angle / 2), 0, std::sin(angle / 2), 0};
        result = step();
    }
    for (int i = 0; i < 40; ++i) result = step();
    assert(!result.planner);

    for (int i = 0; i < 30; ++i) {
        const float angle = -0.01f * i;
        for (auto& j : sample.skeleton.joints)
            j.orientation.wxyz = {std::cos(angle / 2), 0, std::sin(angle / 2), 0};
        result = step();
    }
    assert(result.planner && result.pose.yaw_rate_rps * left_turn_rate < -0.01f);
    for (auto& j : sample.skeleton.joints)
        j.orientation.wxyz = {1, 0, 0, 0};
    for (int i = 0; i < 40; ++i) result = step();
    assert(!result.planner);

    for (auto& j : sample.skeleton.joints)
        j.orientation.wxyz = {std::cos(0.1f), std::sin(0.1f), 0, 0};
    for (int i = 0; i < 30; ++i) result = step();
    assert(std::fabs(result.pose.body_quat_w[1]) + std::fabs(result.pose.body_quat_w[2]) > 0.05f);
    for (auto& j : sample.skeleton.joints)
        j.orientation.wxyz = {1, 0, 0, 0};
    for (int i = 0; i < 40; ++i) result = step();
    assert(std::fabs(result.pose.body_quat_w[1]) + std::fabs(result.pose.body_quat_w[2]) < 0.01f);

    result = session.process(sample, time += 33334);
    assert(result.state == BridgeState::TIMEOUT && result.publish && !result.planner);

    sample.skeleton.joints[JOINT_PELVIS].confidence_level = CONFIDENCE_NONE;
    result = step();
    assert(result.state == BridgeState::TRACKING && result.publish);
    for (int i = 0; i < 4; ++i) result = step();
    assert(result.state == BridgeState::LOW_CONFIDENCE && !result.publish && !result.planner && result.stop);
    assert(!session.health().armed);
    sample.skeleton.joints[JOINT_PELVIS].confidence_level = CONFIDENCE_HIGH;
    result = session.process(std::nullopt, time += 33334);
    assert(result.state == BridgeState::NO_BODY && !result.publish && !result.planner);
    result = session.timeout(time += 1200000);
    assert(result.state == BridgeState::TIMEOUT && !result.publish && !result.planner);
    result = step();
    assert(result.state == BridgeState::CALIBRATING && !result.publish && !result.planner);

    SkeletonSample stranger = sample;
    stranger.body_id = 9;
    stranger.device_timestamp_us += 33334;
    result = session.process(stranger, time += 33334);
    assert(result.state == BridgeState::NO_BODY && !result.publish);

    BridgeSession ambiguous(session_config);
    std::vector<SkeletonSample> two{sample, stranger};
    assert(ambiguous.process(two, time).state == BridgeState::NO_BODY);

    const auto path = argc > 1 ? std::filesystem::path(argv[1])
        : std::filesystem::temp_directory_path() /
          ("kinect_bridge_session_" + std::to_string(getpid()) + ".trace");
    {
        BridgeSession recorder(session_config);
        std::ofstream file(path, std::ios::binary);
        write_trace_header(file);
        const int trace_frames = argc > 1 ? 1800 : 80;
        for (int i = 0; i < trace_frames; ++i) {
            time += 33334;
            sample.device_timestamp_us += 33334;
            sample.skeleton = neutral();
            if (argc > 1) {
                const float t = static_cast<float>(i) / 30.0f;
                const auto arm = [t](float begin) {
                    return std::clamp(std::min((t - begin) / 2.0f, (begin + 6.0f - t) / 2.0f), 0.0f, 1.0f);
                };
                sample.skeleton.joints[JOINT_WRIST_LEFT].position.xyz.y -= 300 * (arm(10) + arm(30));
                sample.skeleton.joints[JOINT_WRIST_RIGHT].position.xyz.y -= 300 * (arm(20) + arm(30));
                const float forward_mm = 150.0f * std::clamp(t - 40.0f, 0.0f, 5.0f);
                const float turn = 0.10f * std::clamp(t - 50.0f, 0.0f, 5.0f);
                for (auto& j : sample.skeleton.joints) {
                    j.position.xyz.z += forward_mm;
                    const float x = j.position.xyz.x;
                    const float z = j.position.xyz.z - (1000.0f + forward_mm);
                    j.position.xyz.x = std::cos(turn) * x + std::sin(turn) * z;
                    j.position.xyz.z = 1000.0f + forward_mm - std::sin(turn) * x + std::cos(turn) * z;
                    j.orientation.wxyz = {std::cos(turn / 2), 0, std::sin(turn / 2), 0};
                }
            } else if (i == 70) sample.skeleton.joints[JOINT_WRIST_RIGHT].position.xyz.y -= 200;
            BridgeTraceFrame frame;
            frame.time_us = time;
            frame.sample = sample;
            frame.has_body = true;
            frame.result = recorder.process(sample, time);
            write_trace_frame(file, frame);
        }
        BridgeTraceFrame frame;
        frame.time_us = time + 33334;
        frame.result = recorder.process(std::nullopt, frame.time_us);
        write_trace_frame(file, frame);
    }
    {
        BridgeSession player(session_config);
        std::ifstream file(path, std::ios::binary);
        BridgeTraceFrame frame;
        int count = 0;
        bool saw_forward = false, saw_turn = false, saw_left_arm = false, saw_right_arm = false;
        while (read_trace_frame(file, frame)) {
            const auto sample_or_none = frame.has_body ? std::optional<SkeletonSample>(frame.sample) : std::nullopt;
            const auto played = player.process(sample_or_none, frame.time_us);
            assert(played.state == frame.result.state && played.publish == frame.result.publish);
            if (played.publish)
                assert(std::fabs(joint_z(played, 21) - joint_z(frame.result, 21)) < 1e-5f);
            if (played.publish && argc > 1) {
                saw_forward |= played.pose.velocity_mps[0] > 0.08f && played.planner;
                saw_turn |= std::fabs(played.pose.yaw_rate_rps) > 0.08f && played.planner;
                saw_left_arm |= joint_z(played, 20) > 0.2f;
                saw_right_arm |= joint_z(played, 21) > 0.2f;
            }
            ++count;
        }
        assert(count == (argc > 1 ? 1801 : 81));
        if (argc > 1) assert(saw_forward && saw_turn && saw_left_arm && saw_right_arm);
        assert(!read_trace_frame(file, frame));
    }
    if (argc == 1) std::filesystem::remove(path);
}
