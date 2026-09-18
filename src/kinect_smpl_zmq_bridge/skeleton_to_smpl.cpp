#include "skeleton_to_smpl.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace {

using Vec3 = Eigen::Vector3f;
using Quat = Eigen::Quaternionf;

constexpr float kPi = 3.14159265358979323846f;
constexpr std::array<k4abt_joint_id_t, 24> kKinectForSmpl = {
    K4ABT_JOINT_PELVIS, K4ABT_JOINT_HIP_LEFT, K4ABT_JOINT_HIP_RIGHT,
    K4ABT_JOINT_SPINE_NAVEL, K4ABT_JOINT_KNEE_LEFT, K4ABT_JOINT_KNEE_RIGHT,
    K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_ANKLE_LEFT, K4ABT_JOINT_ANKLE_RIGHT,
    K4ABT_JOINT_NECK, K4ABT_JOINT_FOOT_LEFT, K4ABT_JOINT_FOOT_RIGHT,
    K4ABT_JOINT_NECK, K4ABT_JOINT_CLAVICLE_LEFT, K4ABT_JOINT_CLAVICLE_RIGHT,
    K4ABT_JOINT_HEAD, K4ABT_JOINT_SHOULDER_LEFT, K4ABT_JOINT_SHOULDER_RIGHT,
    K4ABT_JOINT_ELBOW_LEFT, K4ABT_JOINT_ELBOW_RIGHT, K4ABT_JOINT_WRIST_LEFT,
    K4ABT_JOINT_WRIST_RIGHT, K4ABT_JOINT_HAND_LEFT, K4ABT_JOINT_HAND_RIGHT,
};

constexpr std::array<int, 24> kSmplParents = {
    -1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8,
    9, 9, 9, 12, 13, 14, 16, 17, 18, 19, 20, 21,
};

// Kinect camera: +X right, +Y down, +Z forward. SONIC: +X forward,
// +Y left, +Z up. This matrix also handles rotations, not just positions.
const Eigen::Matrix3f kKinectToSonic = (Eigen::Matrix3f() <<
    0.0f, 0.0f, 1.0f,
   -1.0f, 0.0f, 0.0f,
    0.0f,-1.0f, 0.0f).finished();

Vec3 position_m(const k4abt_joint_t& joint) {
    const auto& p = joint.position.xyz;
    return kKinectToSonic * Vec3(p.x, p.y, p.z) * 0.001f;
}

bool valid_joint(const k4abt_joint_t& joint) {
    const auto& p = joint.position.xyz;
    const auto& q = joint.orientation.wxyz;
    const float norm2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    return joint.confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW &&
           std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
           std::isfinite(norm2) && norm2 > 1e-8f;
}

Quat orientation_sonic(const k4abt_joint_t& joint) {
    const Quat q(joint.orientation.wxyz.w, joint.orientation.wxyz.x,
                 joint.orientation.wxyz.y, joint.orientation.wxyz.z);
    Eigen::Matrix3f r = kKinectToSonic * q.normalized().toRotationMatrix() * kKinectToSonic.transpose();
    return Quat(r).normalized();
}

void store(const Vec3& value, std::array<float, 24 * 3>& out, int joint) {
    out[joint * 3 + 0] = value.x();
    out[joint * 3 + 1] = value.y();
    out[joint * 3 + 2] = value.z();
}

Vec3 load(const std::array<float, 24 * 3>& value, int joint) {
    return Vec3(value[joint * 3 + 0], value[joint * 3 + 1], value[joint * 3 + 2]);
}

void store_pose(const Vec3& value, std::array<float, 21 * 3>& out, int joint) {
    const int offset = (joint - 1) * 3;
    out[offset + 0] = value.x();
    out[offset + 1] = value.y();
    out[offset + 2] = value.z();
}

Vec3 load_pose(const std::array<float, 21 * 3>& value, int joint) {
    const int offset = (joint - 1) * 3;
    return Vec3(value[offset + 0], value[offset + 1], value[offset + 2]);
}

float wrap_angle(float angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

struct SkeletonToSmpl::Impl {
    Quat root_calibration = Quat::Identity();
    Vec3 neutral_pelvis = Vec3::Zero();
    std::array<Quat, 24> pose_calibration{};
    std::array<Quat, 24> last_orientation{};
    std::array<Vec3, 24> last_joint_position{};
    std::array<bool, 24> have_orientation{};
    std::array<bool, 24> have_joint_position{};

    Impl() {
        pose_calibration.fill(Quat::Identity());
        last_orientation.fill(Quat::Identity());
        last_joint_position.fill(Vec3::Zero());
    }
};

SkeletonToSmpl::SkeletonToSmpl(float smoothing, bool auto_calibrate)
    : smoothing_(std::clamp(smoothing, 0.0f, 0.95f)), auto_calibrate_(auto_calibrate), impl_(new Impl()) {}

SkeletonToSmpl::~SkeletonToSmpl() { delete impl_; }

bool SkeletonToSmpl::calibrate(const std::vector<k4abt_skeleton_t>& neutral_frames) {
    if (neutral_frames.empty()) return false;
    std::array<Vec3, 24> average_position{};
    std::array<Eigen::Vector4f, 24> average_quaternion{};
    std::array<int, 24> counts{};
    average_position.fill(Vec3::Zero());
    average_quaternion.fill(Eigen::Vector4f::Zero());
    for (const auto& skeleton : neutral_frames) {
        for (int i = 0; i < 24; ++i) {
            const auto& joint = skeleton.joints[kKinectForSmpl[i]];
            if (!valid_joint(joint)) continue;
            average_position[i] += position_m(joint);
            const Quat q = orientation_sonic(joint);
            Eigen::Vector4f coeffs = q.coeffs();
            if (counts[i] && average_quaternion[i].dot(coeffs) < 0.0f) coeffs *= -1.0f;
            average_quaternion[i] += coeffs;
            ++counts[i];
        }
    }
    std::array<Quat, 24> absolute_orientation{};
    for (int i = 0; i < 24; ++i) {
        if (counts[i] < static_cast<int>(neutral_frames.size() * 3 / 4) ||
            average_quaternion[i].norm() < 1e-6f) return false;
        average_position[i] /= static_cast<float>(counts[i]);
        absolute_orientation[i] = Quat(average_quaternion[i].normalized()).normalized();
    }
    const float feet_z = std::min(average_position[10].z(), average_position[11].z());
    const float height = average_position[15].z() - feet_z;
    if (!std::isfinite(height) || height <= 0.5f) return false;
    body_scale_ = std::clamp(1.70f / height, 0.8f, 1.2f);
    impl_->root_calibration = absolute_orientation[0];
    impl_->neutral_pelvis = average_position[0];
    for (int i = 1; i < 24; ++i) {
        impl_->pose_calibration[i] = absolute_orientation[kSmplParents[i]].conjugate() *
                                      absolute_orientation[i];
    }
    calibrated_ = true;
    return true;
}

bool SkeletonToSmpl::convert(const k4abt_skeleton_t& skeleton, std::uint64_t frame_index,
                             float dt_s, SonicPoseFrame& output) {
    const auto& pelvis = skeleton.joints[K4ABT_JOINT_PELVIS];
    if (!valid_joint(pelvis)) {
        return false;
    }

    std::array<Vec3, 24> joints{};
    std::array<Quat, 24> orientations{};
    std::array<bool, 24> valid{};
    for (int i = 0; i < 24; ++i) {
        const auto source = kKinectForSmpl[i];
        valid[i] = valid_joint(skeleton.joints[source]);
        if (valid[i]) {
            joints[i] = position_m(skeleton.joints[source]);
            impl_->last_joint_position[i] = joints[i];
            impl_->have_joint_position[i] = true;
            orientations[i] = orientation_sonic(skeleton.joints[source]);
            impl_->last_orientation[i] = orientations[i];
            impl_->have_orientation[i] = true;
        } else if (impl_->have_joint_position[i]) {
            joints[i] = impl_->last_joint_position[i];
            orientations[i] = impl_->have_orientation[i] ? impl_->last_orientation[i] : Quat::Identity();
        } else {
            joints[i] = position_m(pelvis);
            orientations[i] = Quat::Identity();
        }
        if (!valid[i] && impl_->have_orientation[i]) {
            orientations[i] = impl_->last_orientation[i];
        }
    }

    const Vec3 root = joints[0];
    const float feet_z = std::min(joints[10].z(), joints[11].z());
    const float height = joints[15].z() - feet_z;
    if (!calibrated_ && auto_calibrate_ && valid[10] && valid[11] && valid[15] && height > 0.5f)
        calibrate({skeleton});
    if (!calibrated_) {
        return false;
    }

    const Quat root_relative = (impl_->root_calibration.conjugate() * orientations[0]).normalized();
    std::array<float, 24 * 3> joints_now{};
    for (int i = 0; i < 24; ++i) {
        const Vec3 root_local = (joints[i] - root) * body_scale_;
        store(orientations[0].conjugate() * root_local, joints_now, i);
        if (!valid[i] && have_previous_) store(load(last_joints_, i), joints_now, i);
    }

    std::array<float, 21 * 3> pose_now{};
    for (int i = 1; i < 22; ++i) {
        const int parent = kSmplParents[i];
        const Quat relative = (orientations[parent].conjugate() * orientations[i]).normalized();
        const Quat calibrated = (impl_->pose_calibration[i].conjugate() * relative).normalized();
        Eigen::AngleAxisf aa(calibrated);
        store_pose(aa.axis() * aa.angle(), pose_now, i);
        if (have_previous_ && (!valid[i] || !valid[parent])) {
            store_pose(load_pose(last_pose_, i), pose_now, i);
        }
    }

    const float alpha = 1.0f - smoothing_;
    if (!have_previous_) {
        last_joints_ = joints_now;
        last_pose_ = pose_now;
        last_root_ = {root_relative.w(), root_relative.x(), root_relative.y(), root_relative.z()};
        last_pelvis_ = {root.x(), root.y(), root.z()};
        last_yaw_ = std::atan2(2.0f * (root_relative.w() * root_relative.z() + root_relative.x() * root_relative.y()),
                               1.0f - 2.0f * (root_relative.y() * root_relative.y() + root_relative.z() * root_relative.z()));
        have_previous_ = true;
    } else {
        for (std::size_t i = 0; i < last_joints_.size(); ++i) last_joints_[i] = smoothing_ * last_joints_[i] + alpha * joints_now[i];
        for (std::size_t i = 0; i < last_pose_.size(); ++i) last_pose_[i] = smoothing_ * last_pose_[i] + alpha * pose_now[i];
        Quat previous(last_root_[0], last_root_[1], last_root_[2], last_root_[3]);
        Quat current = root_relative;
        if (previous.dot(current) < 0.0f) current.coeffs() *= -1.0f;
        current.coeffs() = (smoothing_ * previous.coeffs() + alpha * current.coeffs()).normalized();
        last_root_ = {current.w(), current.x(), current.y(), current.z()};

        const Vec3 current_pelvis = root;
        const float safe_dt = std::isfinite(dt_s) && dt_s > 0.0f ? dt_s : 1.0f / 30.0f;
        Vec3 velocity = (current_pelvis - Vec3(last_pelvis_[0], last_pelvis_[1], last_pelvis_[2])) / safe_dt;
        velocity.z() = 0.0f;
        velocity.x() = std::clamp(velocity.x(), -0.25f, 0.25f);
        velocity.y() = std::clamp(velocity.y(), -0.25f, 0.25f);
        filtered_velocity_[0] = smoothing_ * filtered_velocity_[0] + alpha * velocity.x();
        filtered_velocity_[1] = smoothing_ * filtered_velocity_[1] + alpha * velocity.y();
        filtered_velocity_[2] = 0.0f;
        const float yaw = std::atan2(2.0f * (current.w() * current.z() + current.x() * current.y()),
                                     1.0f - 2.0f * (current.y() * current.y() + current.z() * current.z()));
        float yaw_rate = wrap_angle(yaw - last_yaw_) / safe_dt;
        yaw_rate = std::clamp(yaw_rate, -0.5f, 0.5f);
        filtered_yaw_rate_ = smoothing_ * filtered_yaw_rate_ + alpha * yaw_rate;
        last_yaw_ = yaw;
        last_pelvis_ = {current_pelvis.x(), current_pelvis.y(), current_pelvis.z()};
    }

    output.smpl_joints = last_joints_;
    output.smpl_pose = last_pose_;
    output.body_quat_w = last_root_;
    output.pelvis_position_m = {
        last_pelvis_[0] - impl_->neutral_pelvis.x(),
        last_pelvis_[1] - impl_->neutral_pelvis.y(),
        last_pelvis_[2] - impl_->neutral_pelvis.z()
    };
    output.velocity_mps = filtered_velocity_;
    output.yaw_rate_rps = filtered_yaw_rate_;
    frame_index_ = frame_index;
    return true;
}
