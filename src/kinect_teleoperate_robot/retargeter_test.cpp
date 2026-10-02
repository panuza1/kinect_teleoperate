#include "include/KinectToG1Retargeter.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

namespace {
constexpr double pi = 3.14159265358979323846;
constexpr std::size_t id(KinectJoint value) { return static_cast<std::size_t>(value); }

std::array<double,4> robot_quaternion(double roll, double pitch, double yaw) {
    const double cr=std::cos(roll/2), sr=std::sin(roll/2), cp=std::cos(pitch/2), sp=std::sin(pitch/2), cy=std::cos(yaw/2), sy=std::sin(yaw/2);
    const std::array<double,4> robot{cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy};
    return {robot[0], -robot[2], -robot[3], robot[1]};
}

void position(KinectSkeletonSample& sample, KinectJoint joint, double x, double y, double z) {
    sample.joints[id(joint)].position_mm = {-1000*y, -1000*z, 1000*x};
}

KinectSkeletonSample neutral(std::uint64_t time=1000000) {
    KinectSkeletonSample sample;
    sample.timestamp_us=time; sample.body_id=7;
    for (auto& joint : sample.joints) { joint.confidence=2; joint.orientation_wxyz={1,0,0,0}; }
    position(sample,KinectJoint::Pelvis,0,0,1.0); position(sample,KinectJoint::SpineNavel,0,0,1.15);
    position(sample,KinectJoint::SpineChest,0,0,1.35); position(sample,KinectJoint::Neck,0,0,1.5); position(sample,KinectJoint::Head,0,0,1.65);
    position(sample,KinectJoint::LeftShoulder,0,0.25,1.35); position(sample,KinectJoint::LeftElbow,0,0.25,0.95); position(sample,KinectJoint::LeftWrist,0,0.25,0.55);
    position(sample,KinectJoint::RightShoulder,0,-0.25,1.35); position(sample,KinectJoint::RightElbow,0,-0.25,0.95); position(sample,KinectJoint::RightWrist,0,-0.25,0.55);
    position(sample,KinectJoint::LeftHip,0,0.12,1.0); position(sample,KinectJoint::LeftKnee,0,0.12,0.55); position(sample,KinectJoint::LeftAnkle,0,0.12,0.1); position(sample,KinectJoint::LeftFoot,0.2,0.12,0.05);
    position(sample,KinectJoint::RightHip,0,-0.12,1.0); position(sample,KinectJoint::RightKnee,0,-0.12,0.55); position(sample,KinectJoint::RightAnkle,0,-0.12,0.1); position(sample,KinectJoint::RightFoot,0.2,-0.12,0.05);
    return sample;
}

G1Reference move(KinectSkeletonSample changed) {
    KinectToG1Retargeter::Config config; config.smoothing=1.0;
    KinectToG1Retargeter retargeter(config);
    retargeter.update(neutral());
    changed.timestamp_us=1033333;
    return retargeter.update(changed);
}

void finite_and_limited(const G1Reference& ref) {
    for (std::size_t i=0;i<29;++i) {
        assert(std::isfinite(ref.joint_pos[i]) && std::isfinite(ref.joint_vel[i]));
        assert(ref.joint_pos[i]>=KinectToG1Retargeter::lower_limits[i]-1e-9);
        assert(ref.joint_pos[i]<=KinectToG1Retargeter::upper_limits[i]+1e-9);
        assert(std::abs(ref.joint_vel[i])<=12.0+1e-9);
    }
}
}

int main() {
    static_assert(std::tuple_size<decltype(G1Reference::joint_pos)>::value==29);
    KinectToG1Retargeter stable({150000,1.0,12.0,2.5});
    auto n=neutral(); auto first=stable.update(n); n.timestamp_us+=33333; auto second=stable.update(n);
    assert(first.joint_pos==KinectToG1Retargeter::neutral && second.joint_pos==first.joint_pos);
    assert(std::all_of(second.joint_vel.begin(),second.joint_vel.end(),[](double v){return v==0;}));

    auto pose=neutral(); pose.joints[id(KinectJoint::LeftShoulder)].orientation_wxyz=robot_quaternion(0,0.7,0);
    auto ref=move(pose); assert(ref.joint_pos[15]>KinectToG1Retargeter::neutral[15] && ref.joint_pos[22]==KinectToG1Retargeter::neutral[22]); finite_and_limited(ref);
    pose=neutral(); pose.joints[id(KinectJoint::RightShoulder)].orientation_wxyz=robot_quaternion(0,0.7,0);
    ref=move(pose); assert(ref.joint_pos[22]>KinectToG1Retargeter::neutral[22] && ref.joint_pos[15]==KinectToG1Retargeter::neutral[15]);

    pose=neutral(); position(pose,KinectJoint::LeftWrist,0.4,0.25,0.95); ref=move(pose);
    assert(ref.joint_pos[18]>KinectToG1Retargeter::neutral[18] && ref.joint_pos[25]==KinectToG1Retargeter::neutral[25]);
    pose=neutral(); position(pose,KinectJoint::RightWrist,0.4,-0.25,0.95); ref=move(pose);
    assert(ref.joint_pos[25]>KinectToG1Retargeter::neutral[25] && ref.joint_pos[18]==KinectToG1Retargeter::neutral[18]);

    pose=neutral(); pose.joints[id(KinectJoint::SpineChest)].orientation_wxyz=robot_quaternion(0,0,0.5); ref=move(pose); assert(ref.joint_pos[12]>0);
    pose=neutral(); pose.joints[id(KinectJoint::LeftHip)].orientation_wxyz=robot_quaternion(0,0.6,0); ref=move(pose); assert(ref.joint_pos[0]>KinectToG1Retargeter::neutral[0]);
    pose=neutral(); pose.joints[id(KinectJoint::RightHip)].orientation_wxyz=robot_quaternion(0,0.6,0); ref=move(pose); assert(ref.joint_pos[6]>KinectToG1Retargeter::neutral[6]);

    pose=neutral(); position(pose,KinectJoint::LeftAnkle,0.45,0.12,0.55); ref=move(pose); assert(ref.joint_pos[3]>KinectToG1Retargeter::neutral[3]);
    pose=neutral(); position(pose,KinectJoint::RightAnkle,0.45,-0.12,0.55); ref=move(pose); assert(ref.joint_pos[9]>KinectToG1Retargeter::neutral[9]);

    pose=neutral(); pose.joints[id(KinectJoint::LeftFoot)].orientation_wxyz=robot_quaternion(0,0.3,0); pose.joints[id(KinectJoint::RightFoot)].orientation_wxyz=robot_quaternion(0,0.3,0); ref=move(pose);
    assert(ref.joint_pos[4]>KinectToG1Retargeter::neutral[4] && ref.joint_pos[10]>KinectToG1Retargeter::neutral[10]);

    pose=neutral(); pose.joints[id(KinectJoint::LeftWrist)].orientation_wxyz=robot_quaternion(0.2,0.1,-0.2); ref=move(pose);
    assert(ref.joint_pos[19]>0 && ref.joint_pos[20]>0 && ref.joint_pos[21]<0);

    pose=neutral(); pose.joints[id(KinectJoint::LeftHip)].orientation_wxyz=robot_quaternion(0.4,0,0); pose.joints[id(KinectJoint::RightHip)].orientation_wxyz=robot_quaternion(-0.4,0,0); ref=move(pose);
    assert(ref.joint_pos[1]>0 && ref.joint_pos[7]<0);

    KinectToG1Retargeter held({150000,1.0,12.0,2.5}); held.update(neutral());
    pose=neutral(1050000); pose.joints[id(KinectJoint::LeftAnkle)].confidence=0; held.update(pose);
    assert(held.stats().held[id(KinectJoint::LeftAnkle)] && held.stats().accepted);
    pose.timestamp_us=1200000; held.update(pose);
    assert(held.stats().stale[id(KinectJoint::LeftAnkle)] && held.stats().accepted);

    pose=neutral(1233333); pose.joints[id(KinectJoint::LeftShoulder)].orientation_wxyz[0]=std::numeric_limits<double>::quiet_NaN();
    const auto safe=held.update(pose); finite_and_limited(safe);
    const auto duplicate=held.update(pose); assert(duplicate.timestamp_us==safe.timestamp_us);
    pose=neutral(1600000); pose.joints[id(KinectJoint::LeftShoulder)].orientation_wxyz=robot_quaternion(0,0.4,0);
    const auto long_gap=held.update(pose);
    assert(std::all_of(long_gap.joint_vel.begin(),long_gap.joint_vel.end(),[](double v){return v==0;}));

    KinectToG1Retargeter discontinuity({150000,1.0,12.0,0.1}); discontinuity.update(neutral());
    pose=neutral(1033333); pose.joints[id(KinectJoint::LeftShoulder)].orientation_wxyz=robot_quaternion(0,0.8,0);
    const auto rejected=discontinuity.update(pose);
    assert(rejected.joint_pos[15]==KinectToG1Retargeter::neutral[15]);

    finite_and_limited(ref);
    std::cout << "retargeter tests passed\n";
}
