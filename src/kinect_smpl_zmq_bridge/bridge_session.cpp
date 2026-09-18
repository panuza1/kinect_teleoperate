#include "bridge_session.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <type_traits>

namespace {
constexpr char kTraceMagic[8] = {'K', 'S', 'M', 'P', 'L', 'R', '1', '\n'};
constexpr std::array<k4abt_joint_id_t, 11> kCritical = {
    K4ABT_JOINT_PELVIS, K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_HEAD,
    K4ABT_JOINT_SHOULDER_LEFT, K4ABT_JOINT_SHOULDER_RIGHT,
    K4ABT_JOINT_HIP_LEFT, K4ABT_JOINT_HIP_RIGHT,
    K4ABT_JOINT_ANKLE_LEFT, K4ABT_JOINT_ANKLE_RIGHT,
    K4ABT_JOINT_FOOT_LEFT, K4ABT_JOINT_FOOT_RIGHT
};

bool reliable(const k4abt_skeleton_t& skeleton) {
    for (const auto id : kCritical) {
        const auto& j = skeleton.joints[id];
        if (j.confidence_level < K4ABT_JOINT_CONFIDENCE_MEDIUM ||
            !std::isfinite(j.position.xyz.x) || !std::isfinite(j.position.xyz.y) ||
            !std::isfinite(j.position.xyz.z) ||
            !std::isfinite(j.orientation.wxyz.w) || !std::isfinite(j.orientation.wxyz.x) ||
            !std::isfinite(j.orientation.wxyz.y) || !std::isfinite(j.orientation.wxyz.z) ||
            j.orientation.wxyz.w * j.orientation.wxyz.w +
            j.orientation.wxyz.x * j.orientation.wxyz.x +
            j.orientation.wxyz.y * j.orientation.wxyz.y +
            j.orientation.wxyz.z * j.orientation.wxyz.z < 1e-8f) return false;
    }
    return true;
}

bool neutral_pose(const k4abt_skeleton_t& skeleton) {
    const auto& j = skeleton.joints;
    return j[K4ABT_JOINT_WRIST_LEFT].position.xyz.y - j[K4ABT_JOINT_SHOULDER_LEFT].position.xyz.y > 80.0f &&
           j[K4ABT_JOINT_WRIST_RIGHT].position.xyz.y - j[K4ABT_JOINT_SHOULDER_RIGHT].position.xyz.y > 80.0f &&
           j[K4ABT_JOINT_FOOT_LEFT].position.xyz.y - j[K4ABT_JOINT_HEAD].position.xyz.y > 1000.0f;
}

float distance_mm(const k4a_float3_t& a, const k4a_float3_t& b) {
    const float dx = a.xyz.x - b.xyz.x;
    const float dy = a.xyz.y - b.xyz.y;
    const float dz = a.xyz.z - b.xyz.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool finite_pose(const SonicPoseFrame& pose) {
    for (float v : pose.smpl_joints) if (!std::isfinite(v)) return false;
    for (float v : pose.smpl_pose) if (!std::isfinite(v)) return false;
    for (float v : pose.body_quat_w) if (!std::isfinite(v)) return false;
    for (float v : pose.pelvis_position_m) if (!std::isfinite(v)) return false;
    for (float v : pose.velocity_mps) if (!std::isfinite(v)) return false;
    float norm2 = 0;
    for (float v : pose.body_quat_w) norm2 += v * v;
    return std::isfinite(pose.yaw_rate_rps) && norm2 > 0.25f && norm2 < 4.0f;
}

void limit_pose_step(SonicPoseFrame& pose, const SonicPoseFrame& previous, float dt) {
    const float position_step = 1.2f * dt;
    const float angle_step = 2.4f * dt;
    for (std::size_t i = 0; i < pose.smpl_joints.size(); ++i)
        pose.smpl_joints[i] = previous.smpl_joints[i] +
            std::clamp(pose.smpl_joints[i] - previous.smpl_joints[i], -position_step, position_step);
    for (std::size_t i = 0; i < pose.smpl_pose.size(); ++i)
        pose.smpl_pose[i] = previous.smpl_pose[i] +
            std::clamp(pose.smpl_pose[i] - previous.smpl_pose[i], -angle_step, angle_step);
    float dot = 0;
    for (int i = 0; i < 4; ++i) dot += pose.body_quat_w[i] * previous.body_quat_w[i];
    if (dot < 0) {
        for (float& q : pose.body_quat_w) q = -q;
        dot = -dot;
    }
    const float angle = 2.0f * std::acos(std::clamp(dot, 0.0f, 1.0f));
    const float t = angle > angle_step ? angle_step / angle : 1.0f;
    float norm2 = 0;
    for (int i = 0; i < 4; ++i) {
        pose.body_quat_w[i] = previous.body_quat_w[i] +
            t * (pose.body_quat_w[i] - previous.body_quat_w[i]);
        norm2 += pose.body_quat_w[i] * pose.body_quat_w[i];
    }
    if (norm2 > 1e-8f)
        for (float& q : pose.body_quat_w) q /= std::sqrt(norm2);
}

void blend(std::array<float, 24 * 3>& a, const std::array<float, 24 * 3>& b, float t) {
    for (std::size_t i = 0; i < a.size(); ++i) a[i] += t * (b[i] - a[i]);
}
void blend(std::array<float, 21 * 3>& a, const std::array<float, 21 * 3>& b, float t) {
    for (std::size_t i = 0; i < a.size(); ++i) a[i] += t * (b[i] - a[i]);
}
}

void BridgeSession::reset_calibration() {
    converter_ = std::make_unique<SkeletonToSmpl>(0.75f, false);
    neutral_frames_.clear();
    calibration_start_us_ = 0;
    previous_device_us_ = 0;
    last_valid_us_ = 0;
    frame_index_ = 0;
    previous_pelvis_.reset();
    calibrated_ = false;
    planner_ = false;
}

BridgeResult BridgeSession::fallback(BridgeState state, std::uint64_t now_us) {
    BridgeResult out;
    out.state = state;
    if (have_neutral_) {
        const float dt = last_output_us_ && now_us > last_output_us_
            ? std::min(0.2f, static_cast<float>(now_us - last_output_us_) / 1e6f) : 0.033f;
        const float t = std::clamp(dt / 0.35f, 0.0f, 1.0f);
        blend(last_output_.smpl_joints, neutral_.smpl_joints, t);
        blend(last_output_.smpl_pose, neutral_.smpl_pose, t);
        for (int i = 0; i < 3; ++i) last_output_.pelvis_position_m[i] +=
            t * (neutral_.pelvis_position_m[i] - last_output_.pelvis_position_m[i]);
        for (int i = 0; i < 4; ++i) last_output_.body_quat_w[i] +=
            t * (neutral_.body_quat_w[i] - last_output_.body_quat_w[i]);
        float norm = 0;
        for (float q : last_output_.body_quat_w) norm += q * q;
        if (norm > 1e-8f) for (float& q : last_output_.body_quat_w) q /= std::sqrt(norm);
        last_output_.velocity_mps = {0, 0, 0};
        last_output_.yaw_rate_rps = 0;
        last_output_.timestamp_monotonic_s = static_cast<double>(now_us) / 1e6;
        out.pose = last_output_;
        out.publish = true;
        planner_ = false;
    }
    last_output_us_ = now_us;
    return out;
}

BridgeResult BridgeSession::timeout(std::uint64_t now_us) {
    if (last_valid_us_ && now_us > last_valid_us_ + 1000000) reset_calibration();
    return fallback(BridgeState::TIMEOUT, now_us);
}

BridgeResult BridgeSession::process(const std::optional<SkeletonSample>& sample, std::uint64_t now_us) {
    if (!sample) {
        if (last_valid_us_ && now_us > last_valid_us_ + 1000000) reset_calibration();
        return fallback(BridgeState::NO_BODY, now_us);
    }
    if (body_id_ && sample->body_id != *body_id_) reset_calibration();
    body_id_ = sample->body_id;
    if (!reliable(sample->skeleton)) {
        if (last_valid_us_ && now_us > last_valid_us_ + 1000000) reset_calibration();
        return fallback(BridgeState::LOW_CONFIDENCE, now_us);
    }

    if (!calibrated_) {
        if (!neutral_pose(sample->skeleton) ||
            (!neutral_frames_.empty() && distance_mm(
                sample->skeleton.joints[K4ABT_JOINT_PELVIS].position,
                neutral_frames_.front().joints[K4ABT_JOINT_PELVIS].position) > 80.0f)) {
            neutral_frames_.clear();
            calibration_start_us_ = 0;
            return fallback(BridgeState::CALIBRATING, now_us);
        }
        if (neutral_frames_.empty()) calibration_start_us_ = now_us;
        neutral_frames_.push_back(sample->skeleton);
        if (now_us - calibration_start_us_ < 2000000 || neutral_frames_.size() < 8)
            return fallback(BridgeState::CALIBRATING, now_us);
        if (!converter_->calibrate(neutral_frames_)) {
            neutral_frames_.clear();
            calibration_start_us_ = 0;
            return fallback(BridgeState::CALIBRATING, now_us);
        }
        neutral_frames_.clear();
        calibrated_ = true;
    }

    if (previous_device_us_ && sample->device_timestamp_us <= previous_device_us_)
        return fallback(BridgeState::TIMEOUT, now_us);

    const auto pelvis = sample->skeleton.joints[K4ABT_JOINT_PELVIS].position;
    if (previous_pelvis_ && last_valid_us_ && now_us > last_valid_us_ &&
        now_us - last_valid_us_ < 100000 && distance_mm(pelvis, *previous_pelvis_) > 200.0f)
        return fallback(BridgeState::LOW_CONFIDENCE, now_us);
    const float dt = previous_device_us_ && sample->device_timestamp_us > previous_device_us_
        ? static_cast<float>(sample->device_timestamp_us - previous_device_us_) / 1e6f : 1.0f / 30.0f;
    previous_device_us_ = sample->device_timestamp_us;
    SonicPoseFrame pose;
    if (!converter_->convert(sample->skeleton, frame_index_++, std::clamp(dt, 0.01f, 0.2f), pose) ||
        !finite_pose(pose)) return fallback(BridgeState::LOW_CONFIDENCE, now_us);
    pose.velocity_mps[0] = std::clamp(pose.velocity_mps[0], -0.15f, 0.15f);
    pose.velocity_mps[1] = std::clamp(pose.velocity_mps[1], -0.10f, 0.10f);
    pose.yaw_rate_rps = std::clamp(pose.yaw_rate_rps, -0.20f, 0.20f);
    if (have_neutral_ && last_valid_us_) {
        const float pose_dt = last_output_us_ && now_us > last_output_us_
            ? std::clamp(static_cast<float>(now_us - last_output_us_) / 1e6f, 0.01f, 0.1f)
            : 1.0f / 30.0f;
        limit_pose_step(pose, last_output_, pose_dt);
    }
    pose.timestamp_monotonic_s = static_cast<double>(now_us) / 1e6;
    const float speed = std::hypot(pose.velocity_mps[0], pose.velocity_mps[1]);
    const float yaw_rate = std::fabs(pose.yaw_rate_rps);
    if (speed > 0.08f || yaw_rate > 0.08f) planner_ = true;
    else if (speed < 0.04f && yaw_rate < 0.04f) planner_ = false;
    const bool first = !last_valid_us_;
    if (first) {
        neutral_ = pose;
        have_neutral_ = true;
    }
    last_valid_us_ = now_us;
    last_output_us_ = now_us;
    previous_pelvis_ = pelvis;
    last_output_ = pose;
    return {pose, first ? BridgeState::READY : BridgeState::TRACKING, true, planner_};
}

void write_trace_header(std::ofstream& file) {
    file.write(kTraceMagic, sizeof(kTraceMagic));
    if (!file) throw std::runtime_error("failed to write trace header");
}

void write_trace_frame(std::ofstream& file, const BridgeTraceFrame& frame) {
    static_assert(std::is_trivially_copyable_v<BridgeTraceFrame>);
    file.write(reinterpret_cast<const char*>(&frame), sizeof(frame));
    if (!file) throw std::runtime_error("failed to write trace frame");
}

bool read_trace_frame(std::ifstream& file, BridgeTraceFrame& frame) {
    if (file.tellg() == 0) {
        char magic[8];
        file.read(magic, sizeof(magic));
        if (!file || std::memcmp(magic, kTraceMagic, sizeof(magic)))
            throw std::runtime_error("invalid Kinect trace header");
    }
    file.read(reinterpret_cast<char*>(&frame), sizeof(frame));
    if (file.gcount() == 0 && file.eof()) return false;
    if (!file) throw std::runtime_error("truncated Kinect trace frame");
    if (frame.result.state > BridgeState::TIMEOUT) throw std::runtime_error("invalid Kinect trace state");
    return true;
}
