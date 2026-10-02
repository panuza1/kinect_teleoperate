#pragma once

#include "KinectToG1Retargeter.hpp"

#include <cstdint>
#include <string>

struct _zctx_t;
struct _zsock_t;

class SonicV1Publisher {
public:
    explicit SonicV1Publisher(int port = 5556, const std::string& bind_host = "127.0.0.1");
    ~SonicV1Publisher();
    SonicV1Publisher(const SonicV1Publisher&) = delete;
    SonicV1Publisher& operator=(const SonicV1Publisher&) = delete;

    bool publish(const G1Reference& reference, std::int64_t frame_index);
    bool publish_command(bool start, bool stop);

private:
    _zctx_t* context_ = nullptr;
    _zsock_t* socket_ = nullptr;
};
