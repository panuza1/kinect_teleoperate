#include "bridge_session.hpp"
#include "sonic_zmq_publisher.hpp"

#include <k4a/k4a.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
std::atomic<bool> running{true};
void stop(int) { running = false; }

std::uint64_t now_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

const char* state_name(BridgeState state) {
    switch (state) {
        case BridgeState::NO_BODY: return "NO_BODY";
        case BridgeState::CALIBRATING: return "CALIBRATING";
        case BridgeState::READY: return "READY";
        case BridgeState::TRACKING: return "TRACKING";
        case BridgeState::LOW_CONFIDENCE: return "LOW_CONFIDENCE";
        case BridgeState::TIMEOUT: return "TIMEOUT";
    }
    return "INVALID";
}

struct Options {
    int port = 5556;
    bool debug_skeleton = false;
    bool cpu = false;
    std::string model;
    std::string record;
    std::string replay;
};

Options parse(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--port" && i + 1 < argc) options.port = std::stoi(argv[++i]);
        else if (arg == "--record" && i + 1 < argc) options.record = argv[++i];
        else if (arg == "--replay" && i + 1 < argc) options.replay = argv[++i];
        else if (arg == "--model" && i + 1 < argc) options.model = argv[++i];
        else if (arg == "--debug-skeleton") options.debug_skeleton = true;
        else if (arg == "--cpu") options.cpu = true;
        else if (i == 1 && !arg.empty() && arg[0] != '-') options.port = std::stoi(arg);
        else throw std::runtime_error("usage: kinect_smpl_zmq_bridge [--port 5556] [--cpu] [--model onnx-file] [--debug-skeleton] [--record file | --replay file]");
    }
    if (options.port < 1 || options.port > 65535 || (!options.record.empty() && !options.replay.empty()))
        throw std::runtime_error("invalid port or simultaneous --record and --replay");
    return options;
}

struct Sensor {
    k4a_device_t device = nullptr;
    k4abt_tracker_t tracker = nullptr;
    bool cameras_started = false;
    ~Sensor() {
        if (tracker) { k4abt_tracker_shutdown(tracker); k4abt_tracker_destroy(tracker); }
        if (cameras_started) k4a_device_stop_cameras(device);
        if (device) k4a_device_close(device);
    }
};

void print_skeleton(const SkeletonSample& sample, BridgeState state) {
    constexpr k4abt_joint_id_t ids[] = {
        K4ABT_JOINT_PELVIS, K4ABT_JOINT_SPINE_NAVEL, K4ABT_JOINT_SPINE_CHEST,
        K4ABT_JOINT_NECK, K4ABT_JOINT_HEAD,
        K4ABT_JOINT_SHOULDER_LEFT, K4ABT_JOINT_ELBOW_LEFT, K4ABT_JOINT_WRIST_LEFT,
        K4ABT_JOINT_SHOULDER_RIGHT, K4ABT_JOINT_ELBOW_RIGHT, K4ABT_JOINT_WRIST_RIGHT,
        K4ABT_JOINT_HIP_LEFT, K4ABT_JOINT_KNEE_LEFT, K4ABT_JOINT_ANKLE_LEFT,
        K4ABT_JOINT_HIP_RIGHT, K4ABT_JOINT_KNEE_RIGHT, K4ABT_JOINT_ANKLE_RIGHT
    };
    std::cout << "state=" << state_name(state) << " body=" << sample.body_id
              << " device_us=" << sample.device_timestamp_us << '\n';
    for (const auto id : ids) {
        const auto& j = sample.skeleton.joints[id];
        std::cout << static_cast<int>(id) << " mm=" << j.position.xyz.x << ',' << j.position.xyz.y
                  << ',' << j.position.xyz.z << " quat_wxyz=" << j.orientation.wxyz.w << ','
                  << j.orientation.wxyz.x << ',' << j.orientation.wxyz.y << ','
                  << j.orientation.wxyz.z << " confidence=" << j.confidence_level << '\n';
    }
}

void publish(const BridgeResult& result, SonicZmqPublisher* publisher, std::uint64_t index) {
    if (!publisher || !result.publish) return;
    publisher->publish_command(false, result.planner);
    publisher->publish_pose(result.pose, index);
    publisher->publish_planner(result.pose);
}

bool same_pose(const SonicPoseFrame& a, const SonicPoseFrame& b) {
    const auto close = [](float x, float y) { return std::isfinite(x) && std::isfinite(y) && std::fabs(x - y) < 1e-4f; };
    for (std::size_t i = 0; i < a.smpl_joints.size(); ++i)
        if (!close(a.smpl_joints[i], b.smpl_joints[i])) return false;
    for (std::size_t i = 0; i < a.smpl_pose.size(); ++i)
        if (!close(a.smpl_pose[i], b.smpl_pose[i])) return false;
    for (int i = 0; i < 4; ++i)
        if (!close(a.body_quat_w[i], b.body_quat_w[i])) return false;
    for (int i = 0; i < 3; ++i)
        if (!close(a.velocity_mps[i], b.velocity_mps[i]) ||
            !close(a.pelvis_position_m[i], b.pelvis_position_m[i])) return false;
    return close(a.yaw_rate_rps, b.yaw_rate_rps);
}

void replay(const Options& options) {
    std::ifstream file(options.replay, std::ios::binary);
    if (!file) throw std::runtime_error("cannot open Kinect trace: " + options.replay);
    std::unique_ptr<SonicZmqPublisher> publisher;
    if (!options.debug_skeleton) publisher = std::make_unique<SonicZmqPublisher>(options.port, "127.0.0.1");
    BridgeSession session;
    BridgeTraceFrame frame;
    std::uint64_t first_time = 0, previous_time = 0, index = 0;
    const auto start = std::chrono::steady_clock::now();
    while (running && read_trace_frame(file, frame)) {
        if (!first_time) first_time = frame.time_us;
        if (frame.time_us < previous_time || frame.time_us - first_time > 3600000000ULL)
            throw std::runtime_error("invalid Kinect trace timestamps");
        previous_time = frame.time_us;
        std::this_thread::sleep_until(start + std::chrono::microseconds(frame.time_us - first_time));
        const auto sample = frame.has_body ? std::optional<SkeletonSample>(frame.sample) : std::nullopt;
        const BridgeResult result = frame.capture_timeout ? session.timeout(frame.time_us)
                                                          : session.process(sample, frame.time_us);
        if (result.state != frame.result.state || result.publish != frame.result.publish ||
            result.planner != frame.result.planner ||
            (result.publish && !same_pose(result.pose, frame.result.pose)))
            throw std::runtime_error("Kinect trace replay diverged at frame " + std::to_string(index));
        if (options.debug_skeleton && sample && index % 30 == 0) print_skeleton(*sample, result.state);
        BridgeResult current = result;
        current.pose.timestamp_monotonic_s = static_cast<double>(now_us()) / 1e6;
        publish(current, publisher.get(), index++);
    }
    std::cout << "replayed_frames=" << index << '\n';
}

void live(const Options& options) {
    if (!options.model.empty() && !std::filesystem::is_regular_file(options.model))
        throw std::runtime_error("Body Tracking model not found: " + options.model);
    Sensor sensor;
    if (k4a_device_get_installed_count() == 0)
        throw std::runtime_error("Azure Kinect DK not detected; connect it to USB 3.x");
    if (k4a_device_open(0, &sensor.device) != K4A_RESULT_SUCCEEDED)
        throw std::runtime_error("Azure Kinect DK detected but could not be opened; check USB, power, and udev permissions");
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.synchronized_images_only = true;
    if (k4a_device_start_cameras(sensor.device, &config) != K4A_RESULT_SUCCEEDED)
        throw std::runtime_error("failed to start Kinect RGB/depth cameras");
    sensor.cameras_started = true;
    k4a_calibration_t calibration{};
    if (k4a_device_get_calibration(sensor.device, config.depth_mode, config.color_resolution, &calibration) != K4A_RESULT_SUCCEEDED)
        throw std::runtime_error("failed to get Kinect calibration");
    auto tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    if (options.cpu) tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_CPU;
    if (!options.model.empty()) tracker_config.model_path = options.model.c_str();
    if (k4abt_tracker_create(&calibration, tracker_config, &sensor.tracker) != K4A_RESULT_SUCCEEDED)
        throw std::runtime_error("failed to create Kinect Body Tracking tracker; try --cpu if CUDA is incompatible");

    std::ofstream recording;
    if (!options.record.empty()) {
        recording.open(options.record, std::ios::binary | std::ios::trunc);
        if (!recording) throw std::runtime_error("cannot create Kinect trace: " + options.record);
        write_trace_header(recording);
    }
    std::unique_ptr<SonicZmqPublisher> publisher;
    if (!options.debug_skeleton) publisher = std::make_unique<SonicZmqPublisher>(options.port, "127.0.0.1");
    BridgeSession session;
    std::optional<std::uint32_t> tracked_body_id;
    std::uint64_t index = 0, metric_start = now_us();
    std::uint64_t capture_count = 0, tracking_count = 0, body_count = 0,
                  bridge_count = 0, publish_count = 0;
    double latency_ms_sum = 0;
    BridgeState last_state = BridgeState::NO_BODY;
    bool tracking_pending = false;
    std::uint64_t queued_at_us = 0;
    while (running) {
        k4a_capture_t capture = nullptr;
        const auto capture_status = k4a_device_get_capture(sensor.device, &capture, 100);
        if (capture_status == K4A_WAIT_RESULT_FAILED)
            throw std::runtime_error("Kinect RGB/depth capture failed");
        std::optional<SkeletonSample> sample;
        bool capture_timeout = true;
        if (capture_status == K4A_WAIT_RESULT_SUCCEEDED) {
            k4a_image_t color = k4a_capture_get_color_image(capture);
            k4a_image_t depth = k4a_capture_get_depth_image(capture);
            if (color && depth) ++capture_count;
            if (color) k4a_image_release(color);
            if (depth) k4a_image_release(depth);
            if (color && depth && !tracking_pending) {
                const auto status = k4abt_tracker_enqueue_capture(sensor.tracker, capture, 100);
                if (status == K4A_WAIT_RESULT_FAILED) {
                    k4a_capture_release(capture);
                    throw std::runtime_error("Kinect Body Tracking enqueue failed");
                }
                if (status == K4A_WAIT_RESULT_SUCCEEDED) {
                    tracking_pending = true;
                    queued_at_us = now_us();
                }
            }
            k4a_capture_release(capture);
        }
        if (tracking_pending) {
            k4abt_frame_t body_frame = nullptr;
            const auto status = k4abt_tracker_pop_result(sensor.tracker, &body_frame, 100);
            if (status == K4A_WAIT_RESULT_FAILED)
                throw std::runtime_error("Kinect Body Tracking result failed");
            if (status == K4A_WAIT_RESULT_SUCCEEDED) {
                tracking_pending = false;
                capture_timeout = false;
                ++tracking_count;
                const auto count = k4abt_frame_get_num_bodies(body_frame);
                std::uint32_t selected = count;
                for (std::uint32_t i = 0; i < count; ++i)
                    if (tracked_body_id && k4abt_frame_get_body_id(body_frame, i) == *tracked_body_id) selected = i;
                if (selected == count && count) selected = 0;
                if (selected < count) {
                    SkeletonSample body;
                    body.body_id = k4abt_frame_get_body_id(body_frame, selected);
                    body.device_timestamp_us = k4abt_frame_get_device_timestamp_usec(body_frame);
                    if (k4abt_frame_get_body_skeleton(body_frame, selected, &body.skeleton) == K4A_RESULT_SUCCEEDED) {
                        sample = body;
                        tracked_body_id = body.body_id;
                    }
                }
                k4abt_frame_release(body_frame);
            }
        }
        const std::uint64_t time_us = now_us();
        if (sample && time_us - queued_at_us > 500000) {
            sample.reset();
            capture_timeout = true;
        }
        const BridgeResult result = capture_timeout ? session.timeout(time_us) : session.process(sample, time_us);
        if (result.publish) ++bridge_count;
        if (result.state != last_state) {
            std::cout << "bridge_state=" << state_name(result.state) << '\n';
            last_state = result.state;
        }
        if (options.debug_skeleton && sample && index % 30 == 0) print_skeleton(*sample, result.state);
        if (publisher && result.publish) {
            publish(result, publisher.get(), index);
            ++publish_count;
        }
        if (sample) {
            ++body_count;
            latency_ms_sum += static_cast<double>(time_us - queued_at_us) / 1000.0;
        }
        if (recording) {
            BridgeTraceFrame trace;
            trace.time_us = time_us;
            trace.has_body = sample.has_value();
            trace.capture_timeout = capture_timeout;
            if (sample) trace.sample = *sample;
            trace.result = result;
            write_trace_frame(recording, trace);
        }
        ++index;
        if (time_us - metric_start >= 1000000) {
            const double seconds = static_cast<double>(time_us - metric_start) / 1e6;
            std::cout << "kinect_fps=" << capture_count / seconds << " tracking_fps=" << tracking_count / seconds
                      << " bridge_fps=" << bridge_count / seconds << " zmq_fps=" << publish_count / seconds
                      << " capture_to_publish_ms=" << (body_count ? latency_ms_sum / body_count : 0)
                      << " estimated_capture_to_sim_ms=" << (body_count ? latency_ms_sum / body_count + 25.0 : 0)
                      << " state=" << state_name(result.state) << '\n';
            capture_count = tracking_count = body_count = bridge_count = publish_count = 0;
            latency_ms_sum = 0;
            metric_start = time_us;
        }
    }
}
}

int main(int argc, char** argv) {
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    try {
        const Options options = parse(argc, argv);
        if (!options.replay.empty()) replay(options);
        else live(options);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Kinect bridge: " << error.what() << '\n';
        return 1;
    }
}
