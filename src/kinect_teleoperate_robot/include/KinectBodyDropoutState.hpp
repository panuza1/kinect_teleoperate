#pragma once

#include <chrono>

class KinectBodyDropoutState {
public:
    using Clock = std::chrono::steady_clock;
    static constexpr auto hold_window = std::chrono::milliseconds(250);

    void body_seen(Clock::time_point now) {
        last_seen_ = now;
        seen_ = true;
    }

    bool should_hold(Clock::time_point now) const {
        return seen_ && now - last_seen_ <= hold_window;
    }

private:
    Clock::time_point last_seen_{};
    bool seen_ = false;
};
