#include "skeleton_to_smpl.hpp"
#include "sonic_zmq_publisher.hpp"

#include <k4a/k4a.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <thread>

int main(int argc, char** argv) {
    const int port = argc > 1 ? std::atoi(argv[1]) : 5556;
    k4a_device_t device = nullptr;
    if (k4a_device_open(0, &device) != K4A_RESULT_SUCCEEDED) {
        std::cerr << "Azure Kinect device 0 is not available\n";
        return 2;
    }

    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
    config.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.synchronized_images_only = true;
    if (k4a_device_start_cameras(device, &config) != K4A_RESULT_SUCCEEDED) {
        k4a_device_close(device);
        std::cerr << "Failed to start Azure Kinect cameras\n";
        return 3;
    }

    k4a_calibration_t calibration{};
    if (k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration) != K4A_RESULT_SUCCEEDED) {
        k4a_device_stop_cameras(device);
        k4a_device_close(device);
        std::cerr << "Failed to get Azure Kinect calibration\n";
        return 4;
    }

    k4abt_tracker_t tracker = nullptr;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    if (k4abt_tracker_create(&calibration, tracker_config, &tracker) != K4A_RESULT_SUCCEEDED) {
        k4a_device_stop_cameras(device);
        k4a_device_close(device);
        std::cerr << "Failed to create Azure Kinect Body Tracking tracker\n";
        return 5;
    }

    try {
        SkeletonToSmpl converter;
        SonicZmqPublisher publisher(port);
        std::uint64_t frame_index = 0;
        std::optional<std::uint32_t> tracked_body_id;
        std::uint64_t last_timestamp_us = 0;
        bool planner_mode = false;
        while (true) {
            k4a_capture_t capture = nullptr;
            if (k4a_device_get_capture(device, &capture, 1000) != K4A_WAIT_RESULT_SUCCEEDED) continue;
            const auto enqueue = k4abt_tracker_enqueue_capture(tracker, capture, 1000);
            k4a_capture_release(capture);
            if (enqueue != K4A_WAIT_RESULT_SUCCEEDED) continue;

            k4abt_frame_t body_frame = nullptr;
            if (k4abt_tracker_pop_result(tracker, &body_frame, 1000) != K4A_WAIT_RESULT_SUCCEEDED) continue;
            const uint32_t count = k4abt_frame_get_num_bodies(body_frame);
            if (count > 0) {
                std::optional<uint32_t> body_index;
                for (uint32_t i = 0; i < count; ++i) {
                    if (!tracked_body_id || k4abt_frame_get_body_id(body_frame, i) == *tracked_body_id) {
                        body_index = i;
                        break;
                    }
                }
                k4abt_skeleton_t skeleton{};
                if (body_index && k4abt_frame_get_body_skeleton(body_frame, *body_index, &skeleton) == K4A_RESULT_SUCCEEDED) {
                    tracked_body_id = k4abt_frame_get_body_id(body_frame, *body_index);
                    const auto timestamp_us = k4abt_frame_get_device_timestamp_usec(body_frame);
                    const float dt_s = last_timestamp_us && timestamp_us > last_timestamp_us
                        ? static_cast<float>(timestamp_us - last_timestamp_us) / 1000000.0f
                        : 1.0f / 30.0f;
                    last_timestamp_us = timestamp_us;
                    SonicPoseFrame frame;
                    if (converter.convert(skeleton, frame_index, dt_s, frame)) {
                        const float speed = std::hypot(frame.velocity_mps[0], frame.velocity_mps[1]);
                        const float turn = std::fabs(frame.yaw_rate_rps);
                        if (speed > 0.08f || turn > 0.08f) planner_mode = true;
                        else if (speed < 0.04f && turn < 0.04f) planner_mode = false;
                        // Mode selection is automatic; starting control still
                        // requires an explicit operator command.
                        publisher.publish_command(false, planner_mode);
                        publisher.publish_pose(frame, frame_index);
                        publisher.publish_planner(frame);
                        ++frame_index;
                    }
                }
            }
            k4abt_frame_release(body_frame);
        }
    } catch (const std::exception& error) {
        std::cerr << "Bridge stopped: " << error.what() << "\n";
    }

    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);
    k4a_device_stop_cameras(device);
    k4a_device_close(device);
    return 0;
}
