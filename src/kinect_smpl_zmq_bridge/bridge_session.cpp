#include "bridge_session.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <type_traits>

namespace {
constexpr char kTraceMagicR1[8] = {'K', 'S', 'M', 'P', 'L', 'R', '1', '\n'};
constexpr char kTraceMagicR2[8] = {'K', 'S', 'M', 'P', 'L', 'R', '2', '\n'};
constexpr std::array<JointId, 11> kCritical = {
    JOINT_PELVIS, JOINT_SPINE_CHEST, JOINT_HEAD,
    JOINT_SHOULDER_LEFT, JOINT_SHOULDER_RIGHT,
    JOINT_HIP_LEFT, JOINT_HIP_RIGHT,
    JOINT_ANKLE_LEFT, JOINT_ANKLE_RIGHT,
    JOINT_FOOT_LEFT, JOINT_FOOT_RIGHT
};

bool reliable(const BodySkeleton& skeleton) {
    for (const auto id : kCritical) {
        const auto& j = skeleton.joints[id];
        if (j.confidence_level < CONFIDENCE_MEDIUM ||
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

bool fresh_joint(const JointSample& joint) {
    const auto& p = joint.position.xyz;
    const auto& q = joint.orientation.wxyz;
    const float norm2 = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
    return joint.confidence_level >= CONFIDENCE_MEDIUM && std::isfinite(p.x) && std::isfinite(p.y) &&
           std::isfinite(p.z) && std::isfinite(norm2) && norm2 > 1e-8f;
}

bool neutral_pose(const BodySkeleton& skeleton) {
    const auto& j = skeleton.joints;
    return j[JOINT_WRIST_LEFT].position.xyz.y - j[JOINT_SHOULDER_LEFT].position.xyz.y > 80.0f &&
           j[JOINT_WRIST_RIGHT].position.xyz.y - j[JOINT_SHOULDER_RIGHT].position.xyz.y > 80.0f &&
           j[JOINT_FOOT_LEFT].position.xyz.y - j[JOINT_HEAD].position.xyz.y > 1000.0f;
}

float distance_mm(const Position3& a, const Position3& b) {
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

template <typename T>
void put(std::vector<std::uint8_t>& out, T value) {
    static_assert(std::is_arithmetic_v<T>);
    std::array<std::uint8_t, sizeof(T)> bytes{};
    std::memcpy(bytes.data(), &value, sizeof(T));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    std::reverse(bytes.begin(), bytes.end());
#endif
    out.insert(out.end(), bytes.begin(), bytes.end());
}

template <typename T>
T take(const std::vector<std::uint8_t>& in, std::size_t& offset) {
    if (offset > in.size() || sizeof(T) > in.size() - offset)
        throw std::runtime_error("truncated Kinect R2 trace frame");
    std::array<std::uint8_t, sizeof(T)> bytes{};
    std::copy_n(in.data() + offset, sizeof(T), bytes.data());
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    std::reverse(bytes.begin(), bytes.end());
#endif
    T value{};
    std::memcpy(&value, bytes.data(), sizeof(T));
    offset += sizeof(T);
    return value;
}

std::uint32_t checksum(const std::vector<std::uint8_t>& data) {
    std::uint32_t value = 2166136261u;
    for (const auto byte : data) value = (value ^ byte) * 16777619u;
    return value;
}

template <typename T, std::size_t N>
void put_array(std::vector<std::uint8_t>& out, const std::array<T, N>& values) {
    for (const auto value : values) put(out, value);
}

template <typename T, std::size_t N>
void take_array(const std::vector<std::uint8_t>& in, std::size_t& offset, std::array<T, N>& values) {
    for (auto& value : values) value = take<T>(in, offset);
}

std::vector<std::uint8_t> encode_trace_frame(const BridgeTraceFrame& frame) {
    std::vector<std::uint8_t> out;
    out.reserve(1024);
    put(out, frame.time_us);
    put(out, static_cast<std::uint8_t>(frame.has_body));
    put(out, static_cast<std::uint8_t>(frame.capture_timeout));
    put(out, frame.sample.body_id);
    put(out, frame.sample.device_timestamp_us);
    for (const auto& joint : frame.sample.skeleton.joints) {
        put(out, joint.position.xyz.x); put(out, joint.position.xyz.y); put(out, joint.position.xyz.z);
        put(out, joint.orientation.wxyz.w); put(out, joint.orientation.wxyz.x);
        put(out, joint.orientation.wxyz.y); put(out, joint.orientation.wxyz.z);
        put(out, static_cast<std::uint8_t>(joint.confidence_level));
    }
    put(out, static_cast<std::uint8_t>(frame.result.state));
    put(out, static_cast<std::uint8_t>(frame.result.publish));
    put(out, static_cast<std::uint8_t>(frame.result.planner));
    put(out, static_cast<std::uint8_t>(frame.result.dispatch));
    put(out, static_cast<std::uint8_t>(frame.result.stop));
    put(out, frame.result.epoch); put(out, frame.result.sequence); put(out, frame.result.source_timestamp_us);
    put_array(out, frame.result.pose.smpl_joints); put_array(out, frame.result.pose.smpl_pose);
    put_array(out, frame.result.pose.body_quat_w); put_array(out, frame.result.pose.pelvis_position_m);
    put_array(out, frame.result.pose.velocity_mps); put(out, frame.result.pose.yaw_rate_rps);
    put(out, frame.result.pose.timestamp_monotonic_s);
    return out;
}

void decode_trace_frame(const std::vector<std::uint8_t>& in, BridgeTraceFrame& frame) {
    std::size_t offset = 0;
    frame = {};
    frame.time_us = take<std::uint64_t>(in, offset);
    frame.has_body = take<std::uint8_t>(in, offset) != 0;
    frame.capture_timeout = take<std::uint8_t>(in, offset) != 0;
    frame.sample.body_id = take<std::uint32_t>(in, offset);
    frame.sample.device_timestamp_us = take<std::uint64_t>(in, offset);
    for (auto& joint : frame.sample.skeleton.joints) {
        joint.position.xyz.x = take<float>(in, offset); joint.position.xyz.y = take<float>(in, offset);
        joint.position.xyz.z = take<float>(in, offset); joint.orientation.wxyz.w = take<float>(in, offset);
        joint.orientation.wxyz.x = take<float>(in, offset); joint.orientation.wxyz.y = take<float>(in, offset);
        joint.orientation.wxyz.z = take<float>(in, offset);
        joint.confidence_level = static_cast<JointConfidence>(take<std::uint8_t>(in, offset));
        if (joint.confidence_level > CONFIDENCE_HIGH) throw std::runtime_error("invalid trace confidence");
    }
    frame.result.state = static_cast<BridgeState>(take<std::uint8_t>(in, offset));
    frame.result.publish = take<std::uint8_t>(in, offset) != 0;
    frame.result.planner = take<std::uint8_t>(in, offset) != 0;
    frame.result.dispatch = take<std::uint8_t>(in, offset) != 0;
    frame.result.stop = take<std::uint8_t>(in, offset) != 0;
    frame.result.epoch = take<std::uint64_t>(in, offset);
    frame.result.sequence = take<std::uint64_t>(in, offset);
    frame.result.source_timestamp_us = take<std::uint64_t>(in, offset);
    take_array(in, offset, frame.result.pose.smpl_joints); take_array(in, offset, frame.result.pose.smpl_pose);
    take_array(in, offset, frame.result.pose.body_quat_w); take_array(in, offset, frame.result.pose.pelvis_position_m);
    take_array(in, offset, frame.result.pose.velocity_mps);
    frame.result.pose.yaw_rate_rps = take<float>(in, offset);
    frame.result.pose.timestamp_monotonic_s = take<double>(in, offset);
    if (offset != in.size() || frame.result.state > BridgeState::TIMEOUT)
        throw std::runtime_error("invalid Kinect R2 trace frame");
}
}

BridgeSession::BridgeSession(const BridgeConfig& config)
    : converter_(std::make_unique<SkeletonToSmpl>(0.75f, false)),
      selected_body_id_(config.selected_body_id),
      calibration_path_(config.calibration_path),
      device_serial_(config.device_serial),
      mount_id_(config.mount_id),
      calibration_required_(config.calibration_required),
      tracking_inhibit_us_(config.tracking_inhibit_us),
      tracking_stop_us_(config.tracking_stop_us),
      joint_hold_us_(config.joint_hold_us),
      locomotion_enabled_(config.locomotion_enabled),
      max_forward_mps_(config.max_forward_mps),
      max_lateral_mps_(config.max_lateral_mps),
      max_yaw_rps_(config.max_yaw_rps),
      max_linear_accel_mps2_(config.max_linear_accel_mps2),
      max_yaw_accel_rps2_(config.max_yaw_accel_rps2),
      mode_(config.mode) {
    config.validate();
    body_id_ = selected_body_id_;
    if (!calibration_path_.empty() && std::filesystem::is_regular_file(calibration_path_)) {
        converter_->set_calibration(CalibrationState::load(calibration_path_, device_serial_, mount_id_));
        calibrated_ = true;
    } else if (calibration_required_) throw std::runtime_error("required calibration file not found");
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
    last_joint_us_.fill(0);
    reset_epoch();
    if (!calibration_path_.empty() && std::filesystem::is_regular_file(calibration_path_)) {
        converter_->set_calibration(CalibrationState::load(calibration_path_, device_serial_, mount_id_));
        calibrated_ = true;
    } else if (calibration_required_) throw std::runtime_error("required calibration file not found");
}

BridgeResult BridgeSession::fallback(BridgeState state, std::uint64_t now_us) {
    BridgeResult out;
    out.state = state;
    out.epoch = epoch_;
    out.sequence = sequence_++;
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
        const auto age = last_valid_us_ && now_us > last_valid_us_ ? now_us - last_valid_us_ : 0;
        out.publish = last_valid_us_ && age <= tracking_inhibit_us_;
        out.dispatch = out.publish && armed_;
        out.stop = !last_valid_us_ || age > tracking_inhibit_us_;
        if (out.stop) request_stop();
        planner_ = false;
    }
    ++rejected_;
    last_output_us_ = now_us;
    return out;
}

BridgeResult BridgeSession::timeout(std::uint64_t now_us) {
    if (last_valid_us_ && now_us > last_valid_us_ + tracking_stop_us_) reset_calibration();
    return fallback(BridgeState::TIMEOUT, now_us);
}

BridgeResult BridgeSession::process(const std::vector<SkeletonSample>& bodies, std::uint64_t now_us) {
    const auto wanted = selected_body_id_ ? selected_body_id_ : body_id_;
    if (wanted) {
        const auto match = std::find_if(bodies.begin(), bodies.end(),
            [wanted](const SkeletonSample& body) { return body.body_id == *wanted; });
        return match == bodies.end() ? process(std::nullopt, now_us) : process(*match, now_us);
    }
    // Never guess between people: first acquisition requires exactly one body.
    return bodies.size() == 1 ? process(bodies.front(), now_us) : process(std::nullopt, now_us);
}

BridgeResult BridgeSession::process(const std::optional<SkeletonSample>& sample, std::uint64_t now_us) {
    if (!sample) {
        if (last_valid_us_ && now_us > last_valid_us_ + tracking_stop_us_) reset_calibration();
        return fallback(BridgeState::NO_BODY, now_us);
    }
    if (body_id_ && sample->body_id != *body_id_) return fallback(BridgeState::NO_BODY, now_us);
    body_id_ = sample->body_id;
    if (previous_device_us_ && sample->device_timestamp_us <= previous_device_us_)
        return fallback(BridgeState::TIMEOUT, now_us);
    SkeletonSample accepted = *sample;
    auto candidate_joint = last_joint_;
    auto candidate_joint_us = last_joint_us_;
    for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
        const auto& input = sample->skeleton.joints[i];
        if (fresh_joint(input)) {
            candidate_joint[i] = input;
            candidate_joint_us[i] = now_us;
        } else if (candidate_joint_us[i] && now_us >= candidate_joint_us[i] &&
                   now_us - candidate_joint_us[i] <= joint_hold_us_) {
            accepted.skeleton.joints[i] = candidate_joint[i];
        }
    }
    if (!reliable(accepted.skeleton)) {
        if (last_valid_us_ && now_us > last_valid_us_ + tracking_stop_us_) reset_calibration();
        return fallback(BridgeState::LOW_CONFIDENCE, now_us);
    }

    if (!calibrated_) {
        if (!neutral_pose(accepted.skeleton) ||
            (!neutral_frames_.empty() && distance_mm(
                accepted.skeleton.joints[JOINT_PELVIS].position,
                neutral_frames_.front().joints[JOINT_PELVIS].position) > 80.0f)) {
            neutral_frames_.clear();
            calibration_start_us_ = 0;
            return fallback(BridgeState::CALIBRATING, now_us);
        }
        if (neutral_frames_.empty()) calibration_start_us_ = now_us;
        neutral_frames_.push_back(accepted.skeleton);
        if (now_us - calibration_start_us_ < 2000000 || neutral_frames_.size() < 8)
            return fallback(BridgeState::CALIBRATING, now_us);
        if (!converter_->calibrate(neutral_frames_)) {
            neutral_frames_.clear();
            calibration_start_us_ = 0;
            return fallback(BridgeState::CALIBRATING, now_us);
        }
        neutral_frames_.clear();
        calibrated_ = true;
        if (!calibration_path_.empty())
            converter_->calibration_state(device_serial_, mount_id_).save(calibration_path_);
    }

    const auto pelvis = accepted.skeleton.joints[JOINT_PELVIS].position;
    if (previous_pelvis_ && last_valid_us_ && now_us > last_valid_us_ &&
        now_us - last_valid_us_ < 100000 && distance_mm(pelvis, *previous_pelvis_) > 200.0f)
        return fallback(BridgeState::LOW_CONFIDENCE, now_us);
    const float dt = previous_device_us_ && accepted.device_timestamp_us > previous_device_us_
        ? static_cast<float>(accepted.device_timestamp_us - previous_device_us_) / 1e6f : 1.0f / 30.0f;
    previous_device_us_ = accepted.device_timestamp_us;
    SonicPoseFrame pose;
    if (!converter_->convert(accepted.skeleton, frame_index_++, std::clamp(dt, 0.01f, 0.2f), pose) ||
        !finite_pose(pose)) return fallback(BridgeState::LOW_CONFIDENCE, now_us);
    last_joint_ = candidate_joint;
    last_joint_us_ = candidate_joint_us;
    pose.velocity_mps[0] = std::clamp(pose.velocity_mps[0], -max_forward_mps_, max_forward_mps_);
    pose.velocity_mps[1] = std::clamp(pose.velocity_mps[1], -max_lateral_mps_, max_lateral_mps_);
    pose.yaw_rate_rps = std::clamp(pose.yaw_rate_rps, -max_yaw_rps_, max_yaw_rps_);
    if (have_neutral_ && last_valid_us_) {
        const float safe_dt = std::clamp(dt, 0.01f, 0.2f);
        const float linear_step = max_linear_accel_mps2_ * safe_dt;
        for (int axis = 0; axis < 2; ++axis)
            pose.velocity_mps[axis] = last_output_.velocity_mps[axis] +
                std::clamp(pose.velocity_mps[axis] - last_output_.velocity_mps[axis], -linear_step, linear_step);
        const float yaw_step = max_yaw_accel_rps2_ * safe_dt;
        pose.yaw_rate_rps = last_output_.yaw_rate_rps +
            std::clamp(pose.yaw_rate_rps - last_output_.yaw_rate_rps, -yaw_step, yaw_step);
    }
    if (have_neutral_ && last_valid_us_) {
        const float pose_dt = last_output_us_ && now_us > last_output_us_
            ? std::clamp(static_cast<float>(now_us - last_output_us_) / 1e6f, 0.01f, 0.1f)
            : 1.0f / 30.0f;
        limit_pose_step(pose, last_output_, pose_dt);
    }
    pose.timestamp_monotonic_s = static_cast<double>(now_us) / 1e6;
    const float speed = std::hypot(pose.velocity_mps[0], pose.velocity_mps[1]);
    const float yaw_rate = std::fabs(pose.yaw_rate_rps);
    if (!locomotion_enabled_) planner_ = false;
    else if (speed > 0.08f || yaw_rate > 0.08f) planner_ = true;
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
    ++accepted_;
    BridgeResult result{pose, first ? BridgeState::READY : BridgeState::TRACKING, true, planner_};
    result.dispatch = armed_;
    result.epoch = epoch_;
    result.sequence = sequence_++;
    result.source_timestamp_us = accepted.device_timestamp_us;
    return result;
}

bool BridgeSession::request_arm() {
    if (mode_ != BridgeMode::SIM || !calibrated_ || !last_valid_us_) return false;
    armed_ = true;
    return true;
}

void BridgeSession::request_stop() { armed_ = false; planner_ = false; }

void BridgeSession::reset_epoch() {
    ++epoch_;
    sequence_ = 0;
    request_stop();
    previous_device_us_ = 0;
}

SessionHealth BridgeSession::health() const {
    return {calibrated_, armed_, epoch_, accepted_, rejected_, last_valid_us_};
}

void write_trace_header(std::ofstream& file) {
    file.write(kTraceMagicR2, sizeof(kTraceMagicR2));
    if (!file) throw std::runtime_error("failed to write trace header");
}

void write_trace_frame(std::ofstream& file, const BridgeTraceFrame& frame) {
    const auto payload = encode_trace_frame(frame);
    const auto size = static_cast<std::uint32_t>(payload.size());
    const auto hash = checksum(payload);
    std::vector<std::uint8_t> prefix;
    put(prefix, size); put(prefix, hash);
    file.write(reinterpret_cast<const char*>(prefix.data()), prefix.size());
    file.write(reinterpret_cast<const char*>(payload.data()), payload.size());
    if (!file) throw std::runtime_error("failed to write trace frame");
}

bool read_trace_frame(std::ifstream& file, BridgeTraceFrame& frame) {
    static const int version_slot = std::ios_base::xalloc();
    if (file.tellg() == 0) {
        char magic[8];
        file.read(magic, sizeof(magic));
        if (!file) throw std::runtime_error("invalid Kinect trace header");
        if (!std::memcmp(magic, kTraceMagicR1, sizeof(magic))) file.iword(version_slot) = 1;
        else if (!std::memcmp(magic, kTraceMagicR2, sizeof(magic))) file.iword(version_slot) = 2;
        else
            throw std::runtime_error("invalid Kinect trace header");
    }
    if (file.iword(version_slot) == 1) {
        file.read(reinterpret_cast<char*>(&frame), sizeof(frame));
        if (file.gcount() == 0 && file.eof()) return false;
        if (!file) throw std::runtime_error("truncated Kinect R1 trace frame");
        if (frame.result.state > BridgeState::TIMEOUT) throw std::runtime_error("invalid Kinect trace state");
        return true;
    }
    std::array<std::uint8_t, 8> prefix{};
    file.read(reinterpret_cast<char*>(prefix.data()), prefix.size());
    if (file.gcount() == 0 && file.eof()) return false;
    if (!file) throw std::runtime_error("truncated Kinect R2 trace prefix");
    const std::vector<std::uint8_t> prefix_vector(prefix.begin(), prefix.end());
    std::size_t offset = 0;
    const auto size = take<std::uint32_t>(prefix_vector, offset);
    const auto expected_hash = take<std::uint32_t>(prefix_vector, offset);
    if (size == 0 || size > 1024 * 1024) throw std::runtime_error("invalid Kinect R2 frame size");
    std::vector<std::uint8_t> payload(size);
    file.read(reinterpret_cast<char*>(payload.data()), payload.size());
    if (!file) throw std::runtime_error("truncated Kinect R2 trace frame");
    if (checksum(payload) != expected_hash) throw std::runtime_error("Kinect R2 trace checksum mismatch");
    decode_trace_frame(payload, frame);
    return true;
}
