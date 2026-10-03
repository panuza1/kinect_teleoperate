#include "include/KinectToGMRAdapter.hpp"

#include <zmq.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
#pragma pack(push, 1)
struct RequestHeader {
    char magic[4];
    std::uint64_t timestamp_us;
    std::uint32_t body_id;
};
struct RequestJoint {
    double position[3];
    double orientation[4];
    std::uint8_t confidence;
};
struct ResponseHeader {
    char magic[4];
    std::uint16_t version;
    std::uint8_t status;
    std::uint8_t low;
    std::uint64_t timestamp_us;
    std::uint16_t valid;
    std::uint16_t held;
    std::uint16_t stale;
    double solve_ms;
};
struct Response {
    ResponseHeader header;
    double joint_pos[29];
    double joint_vel[29];
};
#pragma pack(pop)

static_assert(sizeof(RequestHeader) == 16);
static_assert(sizeof(RequestJoint) == 57);
static_assert(sizeof(ResponseHeader) == 30);
static_assert(sizeof(Response) == 494);
}

KinectToGMRAdapter::KinectToGMRAdapter(int port, int timeout_ms) {
    if (port < 1 || port > 65535) throw std::invalid_argument("GMR port must be 1..65535");
    const std::uint16_t endian = 1;
    if (*reinterpret_cast<const std::uint8_t*>(&endian) != 1)
        throw std::runtime_error("GMR bridge requires a little-endian host");
    context_ = zmq_ctx_new();
    socket_ = context_ ? zmq_socket(context_, ZMQ_REQ) : nullptr;
    if (!socket_) throw std::runtime_error("failed to create GMR bridge socket");
    const int yes = 1, linger = 0;
    zmq_setsockopt(socket_, ZMQ_LINGER, &linger, sizeof(linger));
    zmq_setsockopt(socket_, ZMQ_IMMEDIATE, &yes, sizeof(yes));
    zmq_setsockopt(socket_, ZMQ_REQ_RELAXED, &yes, sizeof(yes));
    zmq_setsockopt(socket_, ZMQ_REQ_CORRELATE, &yes, sizeof(yes));
    zmq_setsockopt(socket_, ZMQ_SNDTIMEO, &timeout_ms, sizeof(timeout_ms));
    zmq_setsockopt(socket_, ZMQ_RCVTIMEO, &timeout_ms, sizeof(timeout_ms));
    const std::string endpoint = "tcp://127.0.0.1:" + std::to_string(port);
    if (zmq_connect(socket_, endpoint.c_str()) != 0) {
        const std::string error = zmq_strerror(zmq_errno());
        zmq_close(socket_); socket_ = nullptr;
        zmq_ctx_term(context_); context_ = nullptr;
        throw std::runtime_error(error);
    }
}

KinectToGMRAdapter::~KinectToGMRAdapter() {
    if (socket_) zmq_close(socket_);
    if (context_) zmq_ctx_term(context_);
}

std::optional<G1Reference> KinectToGMRAdapter::update(const KinectSkeletonSample& skeleton) {
    stats_ = {};
    std::vector<std::uint8_t> request(sizeof(RequestHeader) + skeleton.joints.size() * sizeof(RequestJoint));
    RequestHeader header{{'K','G','M','1'}, skeleton.timestamp_us, skeleton.body_id};
    std::memcpy(request.data(), &header, sizeof(header));
    std::size_t offset = sizeof(header);
    for (const auto& source : skeleton.joints) {
        RequestJoint joint{};
        std::copy(source.position_mm.begin(), source.position_mm.end(), joint.position);
        std::copy(source.orientation_wxyz.begin(), source.orientation_wxyz.end(), joint.orientation);
        joint.confidence = source.confidence;
        std::memcpy(request.data() + offset, &joint, sizeof(joint));
        offset += sizeof(joint);
    }
    if (zmq_send(socket_, request.data(), request.size(), 0) != static_cast<int>(request.size())) return std::nullopt;
    Response response{};
    if (zmq_recv(socket_, &response, sizeof(response), 0) != static_cast<int>(sizeof(response))) return std::nullopt;
    if (std::memcmp(response.header.magic, "GMR1", 4) || response.header.version != 1) return std::nullopt;
    stats_.connected = true;
    stats_.calibrated = response.header.status >= 1;
    stats_.valid_targets = response.header.valid;
    stats_.low_targets = response.header.low;
    stats_.held_targets = response.header.held;
    stats_.stale_targets = response.header.stale;
    stats_.solve_ms = response.header.solve_ms;
    if (response.header.status != 2 || response.header.timestamp_us != skeleton.timestamp_us) return std::nullopt;
    G1Reference reference;
    reference.timestamp_us = response.header.timestamp_us;
    for (std::size_t i = 0; i < 29; ++i) {
        if (!std::isfinite(response.joint_pos[i]) || !std::isfinite(response.joint_vel[i])) return std::nullopt;
        reference.joint_pos[i] = response.joint_pos[i];
        reference.joint_vel[i] = response.joint_vel[i];
    }
    return reference;
}
