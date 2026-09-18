#include "sonic_zmq_publisher.hpp"

#include <zmq.h>

#include <array>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr std::size_t kHeaderSize = 1280;

std::string header(const std::string& fields, int version) {
    const std::string json = "{\"v\":" + std::to_string(version) + ",\"endian\":\"le\",\"count\":1,\"fields\":[" + fields + "]}";
    if (json.size() > kHeaderSize) throw std::runtime_error("SONIC ZMQ header exceeds 1280 bytes");
    return json + std::string(kHeaderSize - json.size(), '\0');
}

template <typename T>
void append(std::vector<std::uint8_t>& message, const T* data, std::size_t count) {
    const auto* bytes = reinterpret_cast<const std::uint8_t*>(data);
    message.insert(message.end(), bytes, bytes + count * sizeof(T));
}

void send(std::vector<std::uint8_t>& message, _zsock_t* socket) {
    if (zmq_send(socket, message.data(), message.size(), 0) < 0) {
        throw std::runtime_error(zmq_strerror(zmq_errno()));
    }
}

}  // namespace

SonicZmqPublisher::SonicZmqPublisher(int port, const std::string& bind_host) : context_(static_cast<_zctx_t*>(zmq_ctx_new())), socket_(nullptr) {
    if (!context_) throw std::runtime_error("zmq_ctx_new failed");
    socket_ = static_cast<_zsock_t*>(zmq_socket(context_, ZMQ_PUB));
    if (!socket_) {
        zmq_ctx_term(context_);
        throw std::runtime_error("zmq_socket failed");
    }
    const int linger = 0;
    zmq_setsockopt(socket_, ZMQ_LINGER, &linger, sizeof(linger));
    const std::string endpoint = "tcp://" + bind_host + ":" + std::to_string(port);
    if (zmq_bind(socket_, endpoint.c_str()) != 0) {
        const std::string error = zmq_strerror(zmq_errno());
        zmq_close(socket_);
        zmq_ctx_term(context_);
        throw std::runtime_error(error);
    }
}

SonicZmqPublisher::~SonicZmqPublisher() {
    if (socket_) zmq_close(socket_);
    if (context_) zmq_ctx_term(context_);
}

void SonicZmqPublisher::publish_pose(const SonicPoseFrame& frame, std::uint64_t frame_index) {
    for (float value : frame.smpl_joints) if (!std::isfinite(value)) return;
    for (float value : frame.smpl_pose) if (!std::isfinite(value)) return;
    float quat_norm2 = 0.0f;
    for (float value : frame.body_quat_w) {
        if (!std::isfinite(value)) return;
        quat_norm2 += value * value;
    }
    if (quat_norm2 < 0.25f || quat_norm2 > 4.0f) return;
    std::array<float, 29> joint_pos{};
    std::array<float, 29> joint_vel{};
    const std::int64_t index = static_cast<std::int64_t>(frame_index);
    const double timestamp = frame.timestamp_monotonic_s > 0 && std::isfinite(frame.timestamp_monotonic_s)
        ? frame.timestamp_monotonic_s
        : std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    std::vector<std::uint8_t> message{'p','o','s','e'};
    const std::string fields =
        "{\"name\":\"smpl_joints\",\"dtype\":\"f32\",\"shape\":[1,24,3]},"
        "{\"name\":\"smpl_pose\",\"dtype\":\"f32\",\"shape\":[1,21,3]},"
        "{\"name\":\"joint_pos\",\"dtype\":\"f32\",\"shape\":[1,29]},"
        "{\"name\":\"joint_vel\",\"dtype\":\"f32\",\"shape\":[1,29]},"
        "{\"name\":\"body_quat\",\"dtype\":\"f32\",\"shape\":[1,4]},"
        "{\"name\":\"frame_index\",\"dtype\":\"i64\",\"shape\":[1]},"
        "{\"name\":\"timestamp_monotonic\",\"dtype\":\"f64\",\"shape\":[1]}";
    const auto h = header(fields, 3);
    message.insert(message.end(), h.begin(), h.end());
    append(message, frame.smpl_joints.data(), frame.smpl_joints.size());
    append(message, frame.smpl_pose.data(), frame.smpl_pose.size());
    append(message, joint_pos.data(), joint_pos.size());
    append(message, joint_vel.data(), joint_vel.size());
    append(message, frame.body_quat_w.data(), frame.body_quat_w.size());
    append(message, &index, 1);
    append(message, &timestamp, 1);
    send(message, socket_);
}

void SonicZmqPublisher::publish_planner(const SonicPoseFrame& frame) {
    if (!std::isfinite(frame.velocity_mps[0]) || !std::isfinite(frame.velocity_mps[1])) return;
    const float vx = std::clamp(frame.velocity_mps[0], -0.15f, 0.15f);
    const float vy = std::clamp(frame.velocity_mps[1], -0.10f, 0.10f);
    const float speed = std::sqrt(vx * vx + vy * vy);
    if (!std::isfinite(speed) || !std::isfinite(frame.yaw_rate_rps)) return;
    const bool turning = std::fabs(frame.yaw_rate_rps) > 0.05f;
    const bool moving = speed > 0.05f || turning;
    const std::int32_t mode = moving ? 1 : 0;  // SLOW_WALK or IDLE
    std::array<float, 3> movement{vx, vy, 0.0f};
    if (speed > 1e-4f) {
        movement[0] /= speed;
        movement[1] /= speed;
    }
    const float w = frame.body_quat_w[0];
    const float x = frame.body_quat_w[1];
    const float y = frame.body_quat_w[2];
    const float z = frame.body_quat_w[3];
    const float yaw = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
    if (!std::isfinite(yaw)) return;
    const auto now = std::chrono::steady_clock::now();
    if (!have_facing_) {
        facing_yaw_ = yaw;
        have_facing_ = true;
    } else {
        const float dt = std::clamp(std::chrono::duration<float>(now - last_planner_time_).count(), 0.0f, 0.2f);
        const float difference = std::atan2(std::sin(yaw - facing_yaw_), std::cos(yaw - facing_yaw_));
        facing_yaw_ += std::clamp(difference, -0.20f * dt, 0.20f * dt);
    }
    last_planner_time_ = now;
    std::array<float, 3> facing{std::cos(facing_yaw_), std::sin(facing_yaw_), 0.0f};
    const float command_speed = !moving ? 0.0f : turning ? std::max(speed, 0.10f) : speed;
    if (turning && speed <= 1e-4f) movement = facing;
    const float height = -1.0f;
    std::vector<std::uint8_t> message{'p','l','a','n','n','e','r'};
    const std::string fields =
        "{\"name\":\"mode\",\"dtype\":\"i32\",\"shape\":[1]},"
        "{\"name\":\"movement\",\"dtype\":\"f32\",\"shape\":[3]},"
        "{\"name\":\"facing\",\"dtype\":\"f32\",\"shape\":[3]},"
        "{\"name\":\"speed\",\"dtype\":\"f32\",\"shape\":[1]},"
        "{\"name\":\"height\",\"dtype\":\"f32\",\"shape\":[1]}";
    const auto h = header(fields, 1);
    message.insert(message.end(), h.begin(), h.end());
    append(message, &mode, 1);
    append(message, movement.data(), movement.size());
    append(message, facing.data(), facing.size());
    append(message, &command_speed, 1);
    append(message, &height, 1);
    send(message, socket_);
}

void SonicZmqPublisher::publish_command(bool start, bool planner) {
    const std::array<std::uint8_t, 3> values{
        static_cast<std::uint8_t>(start), 0, static_cast<std::uint8_t>(planner)};
    std::vector<std::uint8_t> message{'c','o','m','m','a','n','d'};
    const auto h = header(
        "{\"name\":\"start\",\"dtype\":\"u8\",\"shape\":[1]},"
        "{\"name\":\"stop\",\"dtype\":\"u8\",\"shape\":[1]},"
        "{\"name\":\"planner\",\"dtype\":\"u8\",\"shape\":[1]}", 1);
    message.insert(message.end(), h.begin(), h.end());
    append(message, values.data(), values.size());
    send(message, socket_);
}
