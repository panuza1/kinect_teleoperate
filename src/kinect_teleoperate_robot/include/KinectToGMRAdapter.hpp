#pragma once

#include "KinectToG1Retargeter.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>

struct GMRAdapterStats {
    std::size_t valid_targets = 0;
    std::size_t held_targets = 0;
    std::size_t stale_targets = 0;
    double solve_ms = 0.0;
    bool connected = false;
    bool calibrated = false;
};

class KinectToGMRAdapter {
public:
    explicit KinectToGMRAdapter(int port = 5558, int timeout_ms = 50);
    ~KinectToGMRAdapter();
    KinectToGMRAdapter(const KinectToGMRAdapter&) = delete;
    KinectToGMRAdapter& operator=(const KinectToGMRAdapter&) = delete;

    std::optional<G1Reference> update(const KinectSkeletonSample& skeleton);
    const GMRAdapterStats& stats() const { return stats_; }

private:
    void* context_ = nullptr;
    void* socket_ = nullptr;
    GMRAdapterStats stats_{};
};
