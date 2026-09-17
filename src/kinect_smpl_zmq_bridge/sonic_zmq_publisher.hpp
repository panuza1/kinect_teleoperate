#pragma once

#include "skeleton_to_smpl.hpp"

#include <cstdint>
#include <string>

struct _zctx_t;
struct _zsock_t;

class SonicZmqPublisher {
public:
    explicit SonicZmqPublisher(int port = 5556, const std::string& bind_host = "*");
    ~SonicZmqPublisher();

    SonicZmqPublisher(const SonicZmqPublisher&) = delete;
    SonicZmqPublisher& operator=(const SonicZmqPublisher&) = delete;

    void publish_pose(const SonicPoseFrame& frame, std::uint64_t frame_index);
    void publish_planner(const SonicPoseFrame& frame);
    void publish_command(bool start, bool planner);

private:
    _zctx_t* context_;
    _zsock_t* socket_;
};
