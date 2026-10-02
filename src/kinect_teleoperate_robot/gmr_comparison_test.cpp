#include "include/KinectToG1Retargeter.hpp"
#include "include/KinectToGMRAdapter.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {
constexpr std::size_t id(KinectJoint value) { return static_cast<std::size_t>(value); }

std::array<double,4> camera_quaternion(double roll, double pitch, double yaw) {
    const double cr=std::cos(roll/2), sr=std::sin(roll/2), cp=std::cos(pitch/2), sp=std::sin(pitch/2), cy=std::cos(yaw/2), sy=std::sin(yaw/2);
    const std::array<double,4> robot{cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy};
    return {robot[0], -robot[2], -robot[3], robot[1]};
}

void position(KinectSkeletonSample& sample, KinectJoint joint, double x, double y, double z) {
    sample.joints[id(joint)].position_mm = {-1000*y, -1000*z, 1000*x};
}

KinectSkeletonSample neutral(std::uint64_t timestamp) {
    KinectSkeletonSample sample; sample.timestamp_us=timestamp; sample.body_id=1;
    for (auto& joint:sample.joints) { joint.confidence=2; joint.orientation_wxyz={1,0,0,0}; }
    position(sample,KinectJoint::Pelvis,0,0,1); position(sample,KinectJoint::SpineNavel,0,0,1.15);
    position(sample,KinectJoint::SpineChest,0,0,1.35); position(sample,KinectJoint::Neck,0,0,1.5); position(sample,KinectJoint::Head,0,0,1.65);
    position(sample,KinectJoint::LeftShoulder,0,.25,1.35); position(sample,KinectJoint::LeftElbow,0,.25,.95); position(sample,KinectJoint::LeftWrist,0,.25,.55);
    position(sample,KinectJoint::RightShoulder,0,-.25,1.35); position(sample,KinectJoint::RightElbow,0,-.25,.95); position(sample,KinectJoint::RightWrist,0,-.25,.55);
    position(sample,KinectJoint::LeftHip,0,.12,1); position(sample,KinectJoint::LeftKnee,0,.12,.55); position(sample,KinectJoint::LeftAnkle,0,.12,.1); position(sample,KinectJoint::LeftFoot,.2,.12,.05);
    position(sample,KinectJoint::RightHip,0,-.12,1); position(sample,KinectJoint::RightKnee,0,-.12,.55); position(sample,KinectJoint::RightAnkle,0,-.12,.1); position(sample,KinectJoint::RightFoot,.2,-.12,.05);
    return sample;
}

void validate(const G1Reference& reference) {
    for (std::size_t i=0;i<29;++i) {
        assert(std::isfinite(reference.joint_pos[i]) && std::isfinite(reference.joint_vel[i]));
        assert(reference.joint_pos[i]>=KinectToG1Retargeter::lower_limits[i]-1e-6);
        assert(reference.joint_pos[i]<=KinectToG1Retargeter::upper_limits[i]+1e-6);
    }
}
}

int main(int argc, char** argv) {
    assert(argc==2);
    KinectToGMRAdapter gmr(std::atoi(argv[1]),500);
    KinectToG1Retargeter legacy({150000,1.0,12.0,2.5});
    std::uint64_t timestamp=1000000;
    G1Reference gmr_neutral, legacy_neutral;
    for(int i=0;i<18;++i) {
        auto sample=neutral(timestamp); timestamp+=33333;
        legacy_neutral=legacy.update(sample);
        if(auto output=gmr.update(sample)) gmr_neutral=*output;
    }
    assert(gmr_neutral.timestamp_us && legacy_neutral.timestamp_us);

    auto pose=neutral(timestamp);
    const auto rotation=camera_quaternion(0,.7,0);
    pose.joints[id(KinectJoint::LeftShoulder)].orientation_wxyz=rotation;
    pose.joints[id(KinectJoint::LeftElbow)].orientation_wxyz=rotation;
    pose.joints[id(KinectJoint::LeftWrist)].orientation_wxyz=rotation;
    G1Reference gmr_pose, legacy_pose;
    for(int i=0;i<14;++i) {
        pose.timestamp_us=timestamp; timestamp+=33333;
        legacy_pose=legacy.update(pose);
        const auto output=gmr.update(pose); assert(output); gmr_pose=*output;
    }
    validate(gmr_pose); validate(legacy_pose);
    const double gmr_left=gmr_pose.joint_pos[15]-gmr_neutral.joint_pos[15];
    const double gmr_right=gmr_pose.joint_pos[22]-gmr_neutral.joint_pos[22];
    assert(gmr_left>.15 && std::abs(gmr_left)>std::abs(gmr_right));

    auto metrics=[](const G1Reference& pose,const G1Reference& base) {
        double max_delta=0; int saturated=0;
        for(std::size_t i=0;i<29;++i) {
            max_delta=std::max(max_delta,std::abs(pose.joint_pos[i]-base.joint_pos[i]));
            saturated+=std::abs(pose.joint_pos[i]-KinectToG1Retargeter::lower_limits[i])<1e-3 ||
                       std::abs(pose.joint_pos[i]-KinectToG1Retargeter::upper_limits[i])<1e-3;
        }
        return std::pair<double,int>{max_delta,saturated};
    };
    const auto [legacy_delta,legacy_saturation]=metrics(legacy_pose,legacy_neutral);
    const auto [gmr_delta,gmr_saturation]=metrics(gmr_pose,gmr_neutral);
    assert(gmr_delta<1.2 && gmr_saturation==0);
    std::cout << "legacy_vs_gmr left_raise legacy_max_delta=" << legacy_delta
              << " legacy_saturated=" << legacy_saturation
              << " gmr_max_delta=" << gmr_delta << " gmr_saturated=" << gmr_saturation << '\n';
}
