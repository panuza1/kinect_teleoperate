#pragma once

#include <k4abttypes.h>
#include <chrono>
#include <iostream>

using namespace std::chrono;

// To detect the start and stop actions of the teleoperated robot
class StartEndPoseDetector {
public:
    StartEndPoseDetector() : state(false) {}

    bool isStartEndPose(double ls_r, double ls_p, double ls_y,
                        double rs_r, double rs_p, double rs_y,
                        double le_y, double re_y) {

        const bool in_range = (ls_r > -0.5 && ls_r < 0.5) && (ls_p > -0.5 && ls_p < 0.5) && (ls_y > -0.5 && ls_y < 0.5) &&
                              (rs_r > -0.5 && rs_r < 0.5) && (rs_p > -0.5 && rs_p < 0.5) && (rs_y > -0.5 && rs_y < 0.5) &&
                              (le_y > -0.5 && le_y < 0.5) && (re_y > -0.5 && re_y < 0.5);
        const auto now = steady_clock::now();
        if (!in_range) {
            pose_started = false;
        } else if (!pose_started) {
            pose_start = now;
            pose_started = true;
        } else if (now - pose_start > seconds(3)) {
            flipState();
        }

        return state;
    }

private:
    bool state;
    bool pose_started = false;
    time_point<steady_clock> pose_start;

    void flipState() {
        state = !state;
        pose_started = false;
        std::cout << (state ? " START." : " END.") << std::endl;
    }
};
