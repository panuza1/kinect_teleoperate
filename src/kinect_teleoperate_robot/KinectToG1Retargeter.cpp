#include "include/KinectToG1Retargeter.hpp"
#include "include/jointRetargeting.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace {
constexpr std::size_t j(KinectJoint value) { return static_cast<std::size_t>(value); }
constexpr double pi = 3.14159265358979323846;
}

const std::array<const char*, 29> KinectToG1Retargeter::joint_names{
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"};

const std::array<const char*, 29> KinectToG1Retargeter::actuator_names{
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"};

const std::array<double, 29> KinectToG1Retargeter::neutral{
    -0.312, 0, 0, 0.669, -0.363, 0, -0.312, 0, 0, 0.669, -0.363, 0,
    0, 0, 0, 0.2, 0.2, 0, 0.6, 0, 0, 0, 0.2, -0.2, 0, 0.6, 0, 0, 0};

const std::array<double, 29> KinectToG1Retargeter::lower_limits{
    -2.5307, -0.5236, -2.7576, -0.087267, -0.87267, -0.2618,
    -2.5307, -2.9671, -2.7576, -0.087267, -0.87267, -0.2618,
    -2.618, -0.52, -0.52, -3.0892, -1.5882, -2.618, -1.0472,
    -1.97222, -1.61443, -1.61443, -3.0892, -2.2515, -2.618, -1.0472,
    -1.97222, -1.61443, -1.61443};

const std::array<double, 29> KinectToG1Retargeter::upper_limits{
    2.8798, 2.9671, 2.7576, 2.8798, 0.5236, 0.2618,
    2.8798, 0.5236, 2.7576, 2.8798, 0.5236, 0.2618,
    2.618, 0.52, 0.52, 2.6704, 2.2515, 2.618, 2.0944,
    1.97222, 1.61443, 1.61443, 2.6704, 1.5882, 2.618, 2.0944,
    1.97222, 1.61443, 1.61443};

KinectToG1Retargeter::KinectToG1Retargeter() : KinectToG1Retargeter(Config{}) {}

KinectToG1Retargeter::KinectToG1Retargeter(Config config) : config_(config) {
    config_.smoothing = std::clamp(config_.smoothing, 0.0, 1.0);
    reset();
}

void KinectToG1Retargeter::reset() {
    cached_at_.fill(0);
    previous_.joint_pos = neutral;
    previous_.joint_vel.fill(0);
    previous_.timestamp_us = 0;
    stats_ = {};
    calibrated_ = false;
}

bool KinectToG1Retargeter::finite(const KinectJointSample& joint) {
    for (double value : joint.position_mm) if (!std::isfinite(value)) return false;
    double norm = 0;
    for (double value : joint.orientation_wxyz) {
        if (!std::isfinite(value)) return false;
        norm += value * value;
    }
    return norm > 0.25 && norm < 4.0;
}

KinectToG1Retargeter::Quaternion KinectToG1Retargeter::quaternion(const KinectJointSample& joint) {
    const auto& q = joint.orientation_wxyz;
    const double norm = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    return {q[0]/norm, q[1]/norm, q[2]/norm, q[3]/norm};
}

KinectToG1Retargeter::Quaternion KinectToG1Retargeter::relative(Quaternion parent, Quaternion child) {
    parent.x = -parent.x; parent.y = -parent.y; parent.z = -parent.z;
    return {
        parent.w*child.w - parent.x*child.x - parent.y*child.y - parent.z*child.z,
        parent.w*child.x + parent.x*child.w + parent.y*child.z - parent.z*child.y,
        parent.w*child.y - parent.x*child.z + parent.y*child.w + parent.z*child.x,
        parent.w*child.z + parent.x*child.y - parent.y*child.x + parent.z*child.w};
}

KinectToG1Retargeter::Quaternion KinectToG1Retargeter::camera_to_robot(Quaternion q) {
    // Proper basis change: robot axes are (camera +z, -x, -y).
    return {q.w, q.z, -q.x, -q.y};
}

std::array<double, 3> KinectToG1Retargeter::euler_xyz(Quaternion q) {
    const double norm = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    q = {q.w/norm, q.x/norm, q.y/norm, q.z/norm};
    const double roll = std::atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y));
    const double sin_pitch = std::clamp(2*(q.w*q.y - q.z*q.x), -1.0, 1.0);
    const double pitch = std::asin(sin_pitch);
    const double yaw = std::atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
    return {roll, pitch, yaw};
}

KinectToG1Retargeter::Vec3 KinectToG1Retargeter::robot_position(const KinectJointSample& joint) {
    return {joint.position_mm[2], -joint.position_mm[0], -joint.position_mm[1]};
}

double KinectToG1Retargeter::bend(Vec3 parent, Vec3 joint, Vec3 child) {
    Vec3 a{parent.x-joint.x, parent.y-joint.y, parent.z-joint.z};
    Vec3 b{child.x-joint.x, child.y-joint.y, child.z-joint.z};
    const double an = std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z);
    const double bn = std::sqrt(b.x*b.x+b.y*b.y+b.z*b.z);
    if (an < 1e-6 || bn < 1e-6) return 0;
    const double angle = std::acos(std::clamp((a.x*b.x+a.y*b.y+a.z*b.z)/(an*bn), -1.0, 1.0));
    return pi - angle;
}

double KinectToG1Retargeter::wrapped_delta(double a, double b) {
    return wrappedAngularDelta(a, b);
}

G1Reference KinectToG1Retargeter::update(const KinectSkeletonSample& skeleton) {
    stats_ = {};
    if (!skeleton.timestamp_us || (previous_.timestamp_us && skeleton.timestamp_us <= previous_.timestamp_us))
        return previous_;

    std::array<KinectJointSample, j(KinectJoint::Count)> joints{};
    for (std::size_t i = 0; i < joints.size(); ++i) {
        const auto& incoming = skeleton.joints[i];
        if (incoming.confidence >= 1 && finite(incoming)) {
            cached_[i] = incoming;
            cached_at_[i] = skeleton.timestamp_us;
            joints[i] = incoming;
            ++stats_.valid_joints;
        } else if (cached_at_[i] && skeleton.timestamp_us - cached_at_[i] <= config_.hold_us) {
            joints[i] = cached_[i];
            stats_.held[i] = true;
            ++stats_.held_joints;
        } else {
            joints[i].confidence = 0;
            stats_.stale[i] = true;
            ++stats_.stale_joints;
        }
    }

    auto available = [&](KinectJoint id) { return joints[j(id)].confidence || stats_.held[j(id)]; };
    auto local = [&](KinectJoint parent, KinectJoint child) {
        return camera_to_robot(relative(quaternion(joints[j(parent)]), quaternion(joints[j(child)])));
    };

    const std::array<std::pair<KinectJoint,KinectJoint>, 9> pairs{{
        {KinectJoint::Pelvis,KinectJoint::LeftHip}, {KinectJoint::Pelvis,KinectJoint::RightHip},
        {KinectJoint::Pelvis,KinectJoint::SpineChest}, {KinectJoint::SpineChest,KinectJoint::LeftShoulder},
        {KinectJoint::SpineChest,KinectJoint::RightShoulder}, {KinectJoint::LeftElbow,KinectJoint::LeftWrist},
        {KinectJoint::RightElbow,KinectJoint::RightWrist},
        {KinectJoint::LeftAnkle,KinectJoint::LeftFoot}, {KinectJoint::RightAnkle,KinectJoint::RightFoot}}};

    if (!calibrated_) {
        for (const auto& pair : pairs) if (!available(pair.first) || !available(pair.second)) return previous_;
        for (std::size_t i = 0; i < pairs.size(); ++i) neutral_relative_[i] = local(pairs[i].first, pairs[i].second);
        const std::array<std::array<KinectJoint,3>,4> chains{{
            {KinectJoint::LeftShoulder,KinectJoint::LeftElbow,KinectJoint::LeftWrist},
            {KinectJoint::RightShoulder,KinectJoint::RightElbow,KinectJoint::RightWrist},
            {KinectJoint::LeftHip,KinectJoint::LeftKnee,KinectJoint::LeftAnkle},
            {KinectJoint::RightHip,KinectJoint::RightKnee,KinectJoint::RightAnkle}}};
        for (std::size_t i = 0; i < chains.size(); ++i) {
            for (auto id : chains[i]) if (!available(id)) return previous_;
            neutral_bend_[i] = bend(robot_position(joints[j(chains[i][0])]), robot_position(joints[j(chains[i][1])]), robot_position(joints[j(chains[i][2])]));
        }
        calibrated_ = true;
        previous_.timestamp_us = skeleton.timestamp_us;
        stats_.calibrated = true;
        stats_.accepted = true;
        return previous_;
    }

    std::array<double,29> target = neutral;
    auto delta_euler = [&](std::size_t index) {
        const auto& pair = pairs[index];
        if (!available(pair.first) || !available(pair.second)) return std::array<double,3>{};
        return euler_xyz(relative(neutral_relative_[index], local(pair.first, pair.second)));
    };
    const auto lh = delta_euler(0), rh = delta_euler(1), torso = delta_euler(2);
    const auto ls = delta_euler(3), rs = delta_euler(4), lw = delta_euler(5), rw = delta_euler(6);
    target[0]+=lh[1]; target[1]+=lh[0]; target[2]+=lh[2];
    target[6]+=rh[1]; target[7]+=rh[0]; target[8]+=rh[2];
    target[12]+=torso[2]; target[13]+=torso[0]; target[14]+=torso[1];
    target[15]+=ls[1]; target[16]+=ls[0]; target[17]+=ls[2];
    target[22]+=rs[1]; target[23]+=rs[0]; target[24]+=rs[2];
    target[19]+=lw[0]; target[20]+=lw[1]; target[21]+=lw[2];
    target[26]+=rw[0]; target[27]+=rw[1]; target[28]+=rw[2];

    auto bend_delta = [&](KinectJoint a, KinectJoint b, KinectJoint c, std::size_t neutral_index) {
        if (!available(a) || !available(b) || !available(c)) return 0.0;
        return bend(robot_position(joints[j(a)]), robot_position(joints[j(b)]), robot_position(joints[j(c)])) - neutral_bend_[neutral_index];
    };
    target[18] += bend_delta(KinectJoint::LeftShoulder,KinectJoint::LeftElbow,KinectJoint::LeftWrist,0);
    target[25] += bend_delta(KinectJoint::RightShoulder,KinectJoint::RightElbow,KinectJoint::RightWrist,1);
    target[3] += bend_delta(KinectJoint::LeftHip,KinectJoint::LeftKnee,KinectJoint::LeftAnkle,2);
    target[9] += bend_delta(KinectJoint::RightHip,KinectJoint::RightKnee,KinectJoint::RightAnkle,3);

    // Foot relative to ankle supplies pitch/roll; yaw is intentionally neutral.
    const auto la = delta_euler(7), ra = delta_euler(8);
    target[4]+=la[1]; target[5]+=la[0];
    target[10]+=ra[1]; target[11]+=ra[0];

    G1Reference output = previous_;
    const double dt = previous_.timestamp_us ? (skeleton.timestamp_us-previous_.timestamp_us)*1e-6 : 0.0;
    for (std::size_t i = 0; i < target.size(); ++i) {
        target[i] = clampG1JointTarget(target[i], lower_limits[i], upper_limits[i]);
        if (!std::isfinite(target[i]) || std::abs(wrapped_delta(target[i], previous_.joint_pos[i])) > config_.max_step)
            target[i] = previous_.joint_pos[i];
        output.joint_pos[i] = clampG1JointTarget(previous_.joint_pos[i] + config_.smoothing*wrapped_delta(target[i], previous_.joint_pos[i]), lower_limits[i], upper_limits[i]);
        output.joint_vel[i] = dt >= 0.001 && dt <= 0.25
            ? std::clamp(wrapped_delta(output.joint_pos[i], previous_.joint_pos[i])/dt, -config_.max_velocity, config_.max_velocity)
            : 0.0;
    }
    output.timestamp_us = skeleton.timestamp_us;
    previous_ = output;
    stats_.calibrated = true;
    stats_.accepted = true;
    return output;
}
