#include "kinect_input.hpp"

#include <k4a/k4a.h>
#include <k4abt.h>

#include <stdexcept>
#include <vector>

struct KinectInput::Impl {
    explicit Impl(BridgeConfig value) : config(std::move(value)) {}
    BridgeConfig config;
    k4a_device_t device = nullptr;
    k4abt_tracker_t tracker = nullptr;
    bool cameras_started = false;
};

KinectInput::KinectInput(BridgeConfig config) : impl_(std::make_unique<Impl>(std::move(config))) {}
KinectInput::~KinectInput() { close(); }

void KinectInput::open() {
    if (is_open()) return;
    const std::uint32_t installed = k4a_device_get_installed_count();
    if (installed == 0)
        throw std::runtime_error("Azure Kinect DK not detected; connect it to USB 3.x");
    for (std::uint32_t index = 0; index < installed && !impl_->device; ++index) {
        k4a_device_t candidate = nullptr;
        if (k4a_device_open(index, &candidate) != K4A_RESULT_SUCCEEDED) continue;
        bool selected = impl_->config.device_serial.empty();
        if (!selected) {
            std::size_t size = 0;
            if (k4a_device_get_serialnum(candidate, nullptr, &size) == K4A_BUFFER_RESULT_TOO_SMALL && size) {
                std::vector<char> serial(size);
                selected = k4a_device_get_serialnum(candidate, serial.data(), &size) == K4A_BUFFER_RESULT_SUCCEEDED &&
                           impl_->config.device_serial == serial.data();
            }
        }
        if (selected) impl_->device = candidate;
        else k4a_device_close(candidate);
    }
    if (!impl_->device)
        throw std::runtime_error(impl_->config.device_serial.empty()
            ? "Azure Kinect DK could not be opened; check USB, power, and permissions"
            : "configured Azure Kinect serial was not found");
    k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    device_config.color_format = K4A_IMAGE_FORMAT_COLOR_MJPG;
    device_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    device_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    device_config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    device_config.synchronized_images_only = true;
    if (k4a_device_start_cameras(impl_->device, &device_config) != K4A_RESULT_SUCCEEDED) {
        close();
        throw std::runtime_error("failed to start synchronized Kinect RGB/depth cameras");
    }
    impl_->cameras_started = true;
    k4a_calibration_t calibration{};
    if (k4a_device_get_calibration(impl_->device, device_config.depth_mode,
                                   device_config.color_resolution, &calibration) != K4A_RESULT_SUCCEEDED) {
        close();
        throw std::runtime_error("failed to get Kinect calibration");
    }
    auto tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    if (impl_->config.cpu) tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_CPU;
    if (!impl_->config.model_path.empty()) tracker_config.model_path = impl_->config.model_path.c_str();
    if (k4abt_tracker_create(&calibration, tracker_config, &impl_->tracker) != K4A_RESULT_SUCCEEDED) {
        close();
        throw std::runtime_error("failed to create Kinect body tracker; try the CPU provider");
    }
}

KinectFrame KinectInput::poll(int timeout_ms) {
    if (!is_open()) throw std::runtime_error("KinectInput::poll called before open");
    KinectFrame result;
    k4a_capture_t capture = nullptr;
    const auto capture_status = k4a_device_get_capture(impl_->device, &capture, timeout_ms);
    if (capture_status == K4A_WAIT_RESULT_TIMEOUT) return result;
    if (capture_status == K4A_WAIT_RESULT_FAILED) {
        result.event = AcquisitionEvent::DISCONNECTED;
        result.detail = "capture failed";
        return result;
    }
    k4a_image_t color = k4a_capture_get_color_image(capture);
    k4a_image_t depth = k4a_capture_get_depth_image(capture);
    if (color) result.color_timestamp_us = k4a_image_get_device_timestamp_usec(color);
    if (depth) result.depth_timestamp_us = k4a_image_get_device_timestamp_usec(depth);
    if (!color || !depth) {
        if (color) k4a_image_release(color);
        if (depth) k4a_image_release(depth);
        k4a_capture_release(capture);
        result.event = AcquisitionEvent::MISSING_STREAMS;
        result.detail = !color ? "RGB unavailable" : "depth unavailable";
        return result;
    }
    k4a_image_release(color);
    k4a_image_release(depth);
    const auto enqueue = k4abt_tracker_enqueue_capture(impl_->tracker, capture, timeout_ms);
    k4a_capture_release(capture);
    if (enqueue == K4A_WAIT_RESULT_TIMEOUT) return result;
    if (enqueue == K4A_WAIT_RESULT_FAILED) {
        result.event = AcquisitionEvent::TRACKER_ERROR;
        result.detail = "body tracker enqueue failed";
        return result;
    }
    k4abt_frame_t body_frame = nullptr;
    const auto tracked = k4abt_tracker_pop_result(impl_->tracker, &body_frame, timeout_ms);
    if (tracked == K4A_WAIT_RESULT_TIMEOUT) return result;
    if (tracked == K4A_WAIT_RESULT_FAILED) {
        result.event = AcquisitionEvent::TRACKER_ERROR;
        result.detail = "body tracker result failed";
        return result;
    }
    const auto count = k4abt_frame_get_num_bodies(body_frame);
    result.bodies.reserve(count);
    for (std::uint32_t index = 0; index < count; ++index) {
        k4abt_skeleton_t sdk{};
        if (k4abt_frame_get_body_skeleton(body_frame, index, &sdk) != K4A_RESULT_SUCCEEDED) continue;
        SkeletonSample body;
        body.body_id = k4abt_frame_get_body_id(body_frame, index);
        body.device_timestamp_us = k4abt_frame_get_device_timestamp_usec(body_frame);
        for (std::size_t joint = 0; joint < JOINT_COUNT; ++joint) {
            const auto& source = sdk.joints[joint];
            auto& target = body.skeleton.joints[joint];
            target.position.xyz = {source.position.xyz.x, source.position.xyz.y, source.position.xyz.z};
            target.orientation.wxyz = {source.orientation.wxyz.w, source.orientation.wxyz.x,
                                       source.orientation.wxyz.y, source.orientation.wxyz.z};
            target.confidence_level = static_cast<JointConfidence>(source.confidence_level);
        }
        result.bodies.push_back(body);
    }
    k4abt_frame_release(body_frame);
    result.event = result.bodies.empty() ? AcquisitionEvent::NO_BODY
                 : result.bodies.size() > 1 ? AcquisitionEvent::MULTIPLE_BODIES
                                            : AcquisitionEvent::FRAME;
    return result;
}

void KinectInput::close() {
    if (impl_->tracker) {
        k4abt_tracker_shutdown(impl_->tracker);
        k4abt_tracker_destroy(impl_->tracker);
        impl_->tracker = nullptr;
    }
    if (impl_->cameras_started && impl_->device) k4a_device_stop_cameras(impl_->device);
    impl_->cameras_started = false;
    if (impl_->device) k4a_device_close(impl_->device);
    impl_->device = nullptr;
}

void KinectInput::reconnect() { close(); open(); }
bool KinectInput::is_open() const { return impl_->device && impl_->tracker && impl_->cameras_started; }
