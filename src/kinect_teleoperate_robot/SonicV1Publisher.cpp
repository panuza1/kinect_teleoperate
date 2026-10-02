#include "include/SonicV1Publisher.hpp"

#include <zmq.h>

#include <array>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace {
constexpr std::size_t header_size = 1280;

template <typename T>
void append(std::vector<std::uint8_t>& message, const T* values, std::size_t count) {
    const auto* bytes = reinterpret_cast<const std::uint8_t*>(values);
    message.insert(message.end(), bytes, bytes + sizeof(T)*count);
}
}

SonicV1Publisher::SonicV1Publisher(int port, const std::string& bind_host) {
    if (bind_host != "127.0.0.1" && bind_host != "localhost" && bind_host != "::1")
        throw std::invalid_argument("SONIC simulation publisher is loopback-only");
    const std::uint16_t endian = 1;
    if (*reinterpret_cast<const std::uint8_t*>(&endian) != 1)
        throw std::runtime_error("SONIC Protocol v1 publisher requires little-endian host");
    context_ = static_cast<_zctx_t*>(zmq_ctx_new());
    socket_ = context_ ? static_cast<_zsock_t*>(zmq_socket(context_, ZMQ_PUB)) : nullptr;
    if (!socket_) throw std::runtime_error("failed to create SONIC ZMQ publisher");
    const int linger = 0;
    zmq_setsockopt(socket_, ZMQ_LINGER, &linger, sizeof(linger));
    const std::string endpoint = "tcp://" + bind_host + ':' + std::to_string(port);
    if (zmq_bind(socket_, endpoint.c_str()) != 0) {
        const std::string error = zmq_strerror(zmq_errno());
        zmq_close(socket_); socket_ = nullptr;
        zmq_ctx_term(context_); context_ = nullptr;
        throw std::runtime_error(error);
    }
}

SonicV1Publisher::~SonicV1Publisher() {
    if (socket_) zmq_close(socket_);
    if (context_) zmq_ctx_term(context_);
}

bool SonicV1Publisher::publish(const G1Reference& reference, std::int64_t frame_index) {
    std::array<float,29> pos{}, vel{};
    for (std::size_t i = 0; i < pos.size(); ++i) {
        if (!std::isfinite(reference.joint_pos[i]) || !std::isfinite(reference.joint_vel[i])) return false;
        pos[i] = static_cast<float>(reference.joint_pos[i]);
        vel[i] = static_cast<float>(reference.joint_vel[i]);
    }
    const std::array<float,4> body_quat_w{1,0,0,0};
    const std::uint8_t catch_up = 0;
    const std::string json =
        "{\"v\":1,\"endian\":\"le\",\"count\":1,\"fields\":["
        "{\"name\":\"joint_pos\",\"dtype\":\"f32\",\"shape\":[1,29]},"
        "{\"name\":\"joint_vel\",\"dtype\":\"f32\",\"shape\":[1,29]},"
        "{\"name\":\"body_quat_w\",\"dtype\":\"f32\",\"shape\":[1,4]},"
        "{\"name\":\"frame_index\",\"dtype\":\"i64\",\"shape\":[1]},"
        "{\"name\":\"catch_up\",\"dtype\":\"u8\",\"shape\":[1]}]}";
    if (json.size() > header_size) return false;
    std::vector<std::uint8_t> message{'p','o','s','e'};
    message.insert(message.end(), json.begin(), json.end());
    message.resize(4 + header_size, 0);
    append(message, pos.data(), pos.size());
    append(message, vel.data(), vel.size());
    append(message, body_quat_w.data(), body_quat_w.size());
    append(message, &frame_index, 1);
    append(message, &catch_up, 1);
    return zmq_send(socket_, message.data(), message.size(), 0) == static_cast<int>(message.size());
}

bool SonicV1Publisher::publish_command(bool start, bool stop) {
    const std::string json =
        "{\"v\":1,\"endian\":\"le\",\"count\":1,\"fields\":["
        "{\"name\":\"start\",\"dtype\":\"u8\",\"shape\":[1]},"
        "{\"name\":\"stop\",\"dtype\":\"u8\",\"shape\":[1]},"
        "{\"name\":\"planner\",\"dtype\":\"u8\",\"shape\":[1]}]}";
    std::vector<std::uint8_t> message{'c','o','m','m','a','n','d'};
    message.insert(message.end(), json.begin(), json.end());
    message.resize(7 + header_size, 0);
    const std::array<std::uint8_t,3> values{static_cast<std::uint8_t>(start),static_cast<std::uint8_t>(stop),0};
    append(message,values.data(),values.size());
    return zmq_send(socket_,message.data(),message.size(),0)==static_cast<int>(message.size());
}
