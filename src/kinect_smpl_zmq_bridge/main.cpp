#include "bridge_session.hpp"
#include "bridge_config.hpp"
#include "kinect_input.hpp"
#include "sonic_zmq_publisher.hpp"

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
    bool publish = false;
    bool arm = false;
    bool fast = false;
    bool debug_skeleton = false;
    bool cpu = false;
    std::string model;
    std::string record;
    std::string replay;
    std::string config_path;
    std::optional<BridgeMode> mode;
    BridgeConfig bridge_config{};
};

Options parse(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--port" && i + 1 < argc) options.port = std::stoi(argv[++i]);
        else if (arg == "--config" && i + 1 < argc) options.config_path = argv[++i];
        else if (arg == "--mode" && i + 1 < argc) options.mode = parse_bridge_mode(argv[++i]);
        else if (arg == "--publish") options.publish = true;
        else if (arg == "--arm") options.arm = true;
        else if (arg == "--no-publish") options.publish = false;
        else if (arg == "--fast") options.fast = true;
        else if (arg == "--record" && i + 1 < argc) options.record = argv[++i];
        else if (arg == "--replay" && i + 1 < argc) options.replay = argv[++i];
        else if (arg == "--model" && i + 1 < argc) options.model = argv[++i];
        else if (arg == "--debug-skeleton") options.debug_skeleton = true;
        else if (arg == "--cpu") options.cpu = true;
        else if (i == 1 && !arg.empty() && arg[0] != '-') options.port = std::stoi(arg);
        else throw std::runtime_error("usage: kinect_smpl_zmq_bridge [--config file] [--mode observe|compute|sim] [--publish --arm] [--no-publish] [--fast] [--port 5556] [--cpu] [--model onnx-file] [--debug-skeleton] [--record file | --replay file]");
    }
    if (!options.config_path.empty()) options.bridge_config = BridgeConfig::load(options.config_path);
    if (options.mode) options.bridge_config.mode = *options.mode;
    options.bridge_config.validate();
    options.port = options.bridge_config.port;
    options.cpu = options.cpu || options.bridge_config.cpu;
    options.debug_skeleton = options.debug_skeleton || options.bridge_config.debug_skeleton;
    if (options.model.empty()) options.model = options.bridge_config.model_path;
    if (options.publish && !options.bridge_config.may_publish())
        throw std::runtime_error("publishing requires an isolated sim configuration with no_publish=false");
    if (options.arm && !options.publish) throw std::runtime_error("--arm requires --publish");
    if (options.port < 1 || options.port > 65535 || (!options.record.empty() && !options.replay.empty()))
        throw std::runtime_error("invalid port or simultaneous --record and --replay");
    return options;
}

void print_skeleton(const SkeletonSample& sample, const BridgeResult& result) {
    constexpr JointId ids[] = {
        JOINT_PELVIS, JOINT_SPINE_NAVEL, JOINT_SPINE_CHEST,
        JOINT_NECK, JOINT_HEAD,
        JOINT_SHOULDER_LEFT, JOINT_ELBOW_LEFT, JOINT_WRIST_LEFT,
        JOINT_SHOULDER_RIGHT, JOINT_ELBOW_RIGHT, JOINT_WRIST_RIGHT,
        JOINT_HIP_LEFT, JOINT_KNEE_LEFT, JOINT_ANKLE_LEFT,
        JOINT_HIP_RIGHT, JOINT_KNEE_RIGHT, JOINT_ANKLE_RIGHT
    };
    std::cout << "state=" << state_name(result.state) << " body=" << sample.body_id
              << " device_us=" << sample.device_timestamp_us << '\n';
    for (const auto id : ids) {
        const auto& j = sample.skeleton.joints[id];
        std::cout << static_cast<int>(id) << " mm=" << j.position.xyz.x << ',' << j.position.xyz.y
                  << ',' << j.position.xyz.z << " quat_wxyz=" << j.orientation.wxyz.w << ','
                  << j.orientation.wxyz.x << ',' << j.orientation.wxyz.y << ','
                  << j.orientation.wxyz.z << " confidence=" << j.confidence_level << '\n';
    }
    std::cout << "smpl_root_m=" << result.pose.smpl_joints[0] << ','
              << result.pose.smpl_joints[1] << ',' << result.pose.smpl_joints[2]
              << " body_quat_wxyz=" << result.pose.body_quat_w[0] << ','
              << result.pose.body_quat_w[1] << ',' << result.pose.body_quat_w[2] << ','
              << result.pose.body_quat_w[3] << " velocity_mps="
              << result.pose.velocity_mps[0] << ',' << result.pose.velocity_mps[1]
              << ',' << result.pose.velocity_mps[2] << " yaw_rate_rps="
              << result.pose.yaw_rate_rps << '\n';
}

void publish(const BridgeResult& result, SonicZmqPublisher* publisher, std::uint64_t index,
             bool start = false) {
    if (!publisher) return;
    publisher->publish_health(result.epoch, result.sequence, result.source_timestamp_us,
                              static_cast<std::uint8_t>(result.state));
    if (result.stop) {
        publisher->publish_command(false, true, false, result.epoch, result.sequence,
                                   result.source_timestamp_us);
        return;
    }
    if (!result.publish || !result.dispatch) return;
    publisher->publish_command(start, false, result.planner, result.epoch, result.sequence,
                               result.source_timestamp_us);
    publisher->publish_pose(result.pose, index, result.epoch, result.sequence,
                            result.source_timestamp_us);
    publisher->publish_planner(result.pose, result.epoch, result.sequence,
                               result.source_timestamp_us);
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
    if (options.publish) publisher = std::make_unique<SonicZmqPublisher>(options.port, options.bridge_config.bind_host);
    BridgeSession session(options.bridge_config);
    BridgeTraceFrame frame;
    std::uint64_t first_time = 0, previous_time = 0, index = 0;
    const auto start = std::chrono::steady_clock::now();
    bool start_pending = false;
    while (running && read_trace_frame(file, frame)) {
        if (!first_time) first_time = frame.time_us;
        if (frame.time_us < previous_time || frame.time_us - first_time > 3600000000ULL)
            throw std::runtime_error("invalid Kinect trace timestamps");
        previous_time = frame.time_us;
        if (!options.fast)
            std::this_thread::sleep_until(start + std::chrono::microseconds(frame.time_us - first_time));
        const auto sample = frame.has_body ? std::optional<SkeletonSample>(frame.sample) : std::nullopt;
        const BridgeResult result = frame.capture_timeout ? session.timeout(frame.time_us)
                                                          : session.process(sample, frame.time_us);
        if (publisher && options.arm && result.state == BridgeState::READY)
            start_pending = session.request_arm();
        if (result.state != frame.result.state || result.publish != frame.result.publish ||
            result.planner != frame.result.planner ||
            (result.publish && !same_pose(result.pose, frame.result.pose)))
            throw std::runtime_error("Kinect trace replay diverged at frame " + std::to_string(index));
        if (options.debug_skeleton && sample && index % 30 == 0) print_skeleton(*sample, result);
        BridgeResult current = result;
        current.pose.timestamp_monotonic_s = static_cast<double>(now_us()) / 1e6;
        publish(current, publisher.get(), index++, start_pending && current.dispatch);
        if (current.dispatch) start_pending = false;
    }
    std::cout << "replayed_frames=" << index << '\n';
}

void live(const Options& options) {
    if (!options.model.empty() && !std::filesystem::is_regular_file(options.model))
        throw std::runtime_error("Body Tracking model not found: " + options.model);
    KinectInput input(options.bridge_config);
    input.open();

    std::ofstream recording;
    if (!options.record.empty()) {
        recording.open(options.record, std::ios::binary | std::ios::trunc);
        if (!recording) throw std::runtime_error("cannot create Kinect trace: " + options.record);
        write_trace_header(recording);
    }
    std::ofstream telemetry;
    if (!options.bridge_config.telemetry_path.empty()) {
        const std::filesystem::path telemetry_path(options.bridge_config.telemetry_path);
        if (telemetry_path.has_parent_path()) std::filesystem::create_directories(telemetry_path.parent_path());
        telemetry.open(telemetry_path, std::ios::app);
        if (!telemetry) throw std::runtime_error("cannot create telemetry log: " + telemetry_path.string());
    }
    std::unique_ptr<SonicZmqPublisher> publisher;
    if (options.publish) publisher = std::make_unique<SonicZmqPublisher>(options.port, options.bridge_config.bind_host);
    BridgeSession session(options.bridge_config);
    std::uint64_t index = 0, metric_start = now_us();
    std::uint64_t capture_count = 0, tracking_count = 0, body_count = 0,
                  bridge_count = 0, publish_count = 0;
    double latency_ms_sum = 0;
    BridgeState last_state = BridgeState::NO_BODY;
    bool arm_requested = options.arm;
    bool start_pending = false;
    while (running) {
        const auto queued_at_us = now_us();
        const KinectFrame frame = input.poll(100);
        const std::uint64_t time_us = now_us();
        const bool capture_timeout = frame.event == AcquisitionEvent::TIMEOUT;
        if (frame.event == AcquisitionEvent::DISCONNECTED || frame.event == AcquisitionEvent::TRACKER_ERROR) {
            session.reset_epoch();
            arm_requested = false;
            int backoff_ms = 100;
            while (running) {
                try {
                    input.reconnect();
                    std::cerr << "Kinect reconnected; session remains disarmed" << '\n';
                    break;
                } catch (const std::exception& error) {
                    std::cerr << "Kinect reconnect failed: " << error.what() << '\n';
                    std::this_thread::sleep_for(std::chrono::milliseconds(backoff_ms));
                    backoff_ms = std::min(backoff_ms * 2, 2000);
                }
            }
            continue;
        }
        if (frame.color_timestamp_us && frame.depth_timestamp_us) ++capture_count;
        if (!capture_timeout) ++tracking_count;
        const BridgeResult result = capture_timeout ? session.timeout(time_us)
                                                    : session.process(frame.bodies, time_us);
        if (publisher && arm_requested && result.state == BridgeState::READY) {
            start_pending = session.request_arm();
            arm_requested = false;
        }
        if (result.publish) ++bridge_count;
        if (result.state != last_state) {
            std::cout << "bridge_state=" << state_name(result.state) << '\n';
            last_state = result.state;
        }
        if (options.debug_skeleton && frame.bodies.size() == 1 && index % 30 == 0)
            print_skeleton(frame.bodies.front(), result);
        if (publisher && result.publish) {
            publish(result, publisher.get(), index, start_pending && result.dispatch);
            if (result.dispatch) start_pending = false;
            ++publish_count;
        }
        if (frame.bodies.size() == 1) {
            ++body_count;
            latency_ms_sum += static_cast<double>(time_us - queued_at_us) / 1000.0;
        }
        if (recording) {
            BridgeTraceFrame trace;
            trace.time_us = time_us;
            trace.has_body = frame.bodies.size() == 1;
            trace.capture_timeout = capture_timeout;
            if (trace.has_body) trace.sample = frame.bodies.front();
            trace.result = result;
            write_trace_frame(recording, trace);
        }
        ++index;
        if (time_us - metric_start >= 1000000) {
            const double seconds = static_cast<double>(time_us - metric_start) / 1e6;
            const double capture_fps = capture_count / seconds;
            const double tracking_fps = tracking_count / seconds;
            const double bridge_fps = bridge_count / seconds;
            const double zmq_fps = publish_count / seconds;
            const double latency_ms = body_count ? latency_ms_sum / body_count : 0;
            std::cout << "kinect_fps=" << capture_fps << " tracking_fps=" << tracking_fps
                      << " bridge_fps=" << bridge_fps << " zmq_fps=" << zmq_fps
                      << " capture_to_publish_ms=" << latency_ms
                      << " estimated_capture_to_sim_ms=" << latency_ms + 25.0
                      << " state=" << state_name(result.state) << '\n';
            if (telemetry) {
                const auto health = session.health();
                telemetry << "{\"time_us\":" << time_us << ",\"kinect_fps\":" << capture_fps
                          << ",\"tracking_fps\":" << tracking_fps << ",\"bridge_fps\":" << bridge_fps
                          << ",\"zmq_fps\":" << zmq_fps << ",\"dropped_frames\":"
                          << (capture_count > tracking_count ? capture_count - tracking_count : 0)
                          << ",\"capture_to_publish_ms\":" << latency_ms
                          << ",\"accepted\":" << health.accepted << ",\"rejected\":" << health.rejected
                          << ",\"epoch\":" << health.epoch << ",\"armed\":"
                          << (health.armed ? "true" : "false") << "}\n";
                telemetry.flush();
            }
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
