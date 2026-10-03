#pragma once

#include <chrono>
#include <cstdint>

class SonicV1StreamState {
public:
    using Clock = std::chrono::steady_clock;
    static constexpr auto tx_period = std::chrono::milliseconds(20);
    static constexpr auto stale_after = std::chrono::milliseconds(450);
    static constexpr int start_burst_ticks = 10;

    struct Action {
        bool publish_pose = false;
        bool send_start = false;
        bool send_stop = false;
        bool held = false;
        bool stale = false;
        bool stale_event = false;
    };

    Action tick(std::uint64_t sequence, Clock::time_point arrival, Clock::time_point now,
                bool wants_active) {
        Action action;
        if (!sequence) return action;
        action.stale = now - arrival > stale_after;
        action.stale_event = action.stale && !stale_;
        stale_ = action.stale;
        if (!wants_active || action.stale) {
            if (streaming_) action.send_stop = true;
            streaming_ = false;
            start_ticks_ = 0;
            return action;
        }
        if (!streaming_) {
            streaming_ = true;
            start_ticks_ = start_burst_ticks;
        }
        action.send_start = start_ticks_ > 0;
        if (start_ticks_ > 0) --start_ticks_;
        action.publish_pose = true;
        action.held = sequence == last_sequence_;
        last_sequence_ = sequence;
        return action;
    }

    bool streaming() const { return streaming_; }

private:
    bool streaming_ = false;
    bool stale_ = false;
    int start_ticks_ = 0;
    std::uint64_t last_sequence_ = 0;
};
