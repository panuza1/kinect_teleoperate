#include "include/SonicV1StreamState.hpp"
#include "include/KinectBodyDropoutState.hpp"

#include <array>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <iostream>

int main() {
    using namespace std::chrono;
    using Clock = SonicV1StreamState::Clock;
    SonicV1StreamState stream;
    const auto origin=Clock::time_point{};
    auto arrival=origin;
    std::uint64_t sequence=1;
    int starts=0, stops=0, stale_ticks=0, stale_events=0, poses=0, held=0;
    auto last_pose=origin;
    bool have_pose=false;
    const std::array<double,2> fresh_velocity{1.0,-2.0};
    for (int tick=0;tick<1500;++tick) { // 30 seconds at 50 Hz
        const auto now=origin+milliseconds(tick*20);
        const auto ms=tick*20;
        const bool pause=(ms>=5000 && ms<5100) || (ms>=10000 && ms<10200) ||
                         (ms>=20000 && ms<20600);
        if (!pause && ms%50==0) { ++sequence; arrival=now; }
        const auto action=stream.tick(sequence,arrival,now,true);
        starts+=action.send_start;
        stops+=action.send_stop;
        stale_ticks+=action.stale;
        stale_events+=action.stale_event;
        if (action.stale) have_pose=false;
        if (action.publish_pose) {
            ++poses;
            if (have_pose) assert(now-last_pose==milliseconds(20));
            have_pose=true;
            last_pose=now;
            auto velocity=fresh_velocity;
            if (action.held) { velocity.fill(0.0); ++held; }
            assert(action.held ? velocity[0]==0.0 && velocity[1]==0.0 : velocity[0]==1.0);
        }
        if (ms>=20460 && ms<20600) assert(!action.publish_pose);
    }
    assert(poses>1400 && held>0);
    assert(stops==1 && stale_ticks>0 && stale_events==1);
    assert(starts==20); // ten bounded startup ticks for each of two sessions

    KinectBodyDropoutState dropout;
    dropout.body_seen(origin);
    assert(dropout.should_hold(origin+milliseconds(100)));
    assert(dropout.should_hold(origin+milliseconds(250)));
    assert(!dropout.should_hold(origin+milliseconds(251)));
    SonicV1StreamState dropout_stream;
    auto dropout_arrival=origin;
    std::uint64_t dropout_sequence=1;
    int dropout_count=0;
    bool tracking=true;
    const std::array<double,2> held_qpos{-0.3,0.0};
    assert(dropout_stream.tick(dropout_sequence,dropout_arrival,origin,true).publish_pose);
    for (int ms=20; ms<=100; ms+=20) {
        const auto now=origin+milliseconds(ms);
        if (tracking) { tracking=false; ++dropout_count; }
        if (dropout.should_hold(now)) { ++dropout_sequence; dropout_arrival=now; }
        const auto action=dropout_stream.tick(dropout_sequence,dropout_arrival,now,true);
        assert(action.publish_pose && !action.held);
        assert(held_qpos[0]==-0.3 && held_qpos[1]==0.0);
    }
    tracking=true;
    dropout.body_seen(origin+milliseconds(120));
    ++dropout_sequence;
    dropout_arrival=origin+milliseconds(120);
    assert(dropout_stream.tick(dropout_sequence,dropout_arrival,dropout_arrival,true).publish_pose);
    auto stopped=dropout_stream.tick(dropout_sequence,dropout_arrival,
                                     dropout_arrival+milliseconds(451),true);
    assert(stopped.send_stop && stopped.stale_event);
    stopped=dropout_stream.tick(dropout_sequence,dropout_arrival,
                                dropout_arrival+milliseconds(471),true);
    assert(!stopped.send_stop && !stopped.stale_event);
    assert(tracking && dropout_count==1 && dropout_sequence==7);
    std::cout << "stream cadence passed: tx=" << poses << " held=" << held
              << " starts=" << starts << " stops=" << stops << '\n';
}
