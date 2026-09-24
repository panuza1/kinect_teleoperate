#pragma once

#include "skeleton_to_smpl.hpp"

#include <cstdint>
#include <chrono>
#include <string>

struct _zctx_t;
struct _zsock_t;

class SonicZmqPublisher {
public:
    explicit SonicZmqPublisher(int port = 5556, const std::string& bind_host = "127.0.0.1");
    ~SonicZmqPublisher();

    SonicZmqPublisher(const SonicZmqPublisher&) = delete;
    SonicZmqPublisher& operator=(const SonicZmqPublisher&) = delete;

    void publish_pose(const SonicPoseFrame& frame, std::uint64_t frame_index,
                      std::uint64_t epoch = 0, std::uint64_t sequence = 0,
                      std::uint64_t source_timestamp_us = 0);
    void publish_planner(const SonicPoseFrame& frame, std::uint64_t epoch = 0,
                         std::uint64_t sequence = 0, std::uint64_t source_timestamp_us = 0);
    void publish_command(bool start, bool stop, bool planner, std::uint64_t epoch = 0,
                         std::uint64_t sequence = 0, std::uint64_t source_timestamp_us = 0);
    void publish_health(std::uint64_t epoch, std::uint64_t sequence,
                        std::uint64_t source_timestamp_us, std::uint8_t state);

private:
    _zctx_t* context_;
    _zsock_t* socket_;
    bool have_facing_ = false;
    float facing_yaw_ = 0.0f;
    double last_planner_timestamp_s_ = 0;
};
