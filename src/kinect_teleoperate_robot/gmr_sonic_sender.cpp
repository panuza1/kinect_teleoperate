#include "include/KinectToGMRAdapter.hpp"
#include "include/SonicV1Publisher.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {
constexpr std::size_t id(KinectJoint value) { return static_cast<std::size_t>(value); }
constexpr double pi=3.14159265358979323846;

std::array<double,4> camera_quaternion(double roll,double pitch,double yaw) {
    const double cr=std::cos(roll/2),sr=std::sin(roll/2),cp=std::cos(pitch/2),sp=std::sin(pitch/2),cy=std::cos(yaw/2),sy=std::sin(yaw/2);
    const std::array<double,4> robot{cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy};
    return {robot[0],-robot[2],-robot[3],robot[1]};
}

void position(KinectSkeletonSample& sample,KinectJoint joint,double x,double y,double z) {
    sample.joints[id(joint)].position_mm={-1000*y,-1000*z,1000*x};
}

KinectSkeletonSample sample(std::uint64_t timestamp,double phase) {
    KinectSkeletonSample value; value.timestamp_us=timestamp; value.body_id=1;
    for(auto& joint:value.joints){joint.confidence=2;joint.orientation_wxyz={1,0,0,0};}
    position(value,KinectJoint::Pelvis,0,0,1); position(value,KinectJoint::SpineNavel,0,0,1.15); position(value,KinectJoint::SpineChest,0,0,1.35);
    position(value,KinectJoint::Neck,0,0,1.5); position(value,KinectJoint::Head,0,0,1.65);
    position(value,KinectJoint::LeftShoulder,0,.25,1.35); position(value,KinectJoint::LeftElbow,0,.25,.95); position(value,KinectJoint::LeftWrist,0,.25,.55);
    position(value,KinectJoint::RightShoulder,0,-.25,1.35); position(value,KinectJoint::RightElbow,0,-.25,.95); position(value,KinectJoint::RightWrist,0,-.25,.55);
    position(value,KinectJoint::LeftHip,0,.12,1); position(value,KinectJoint::LeftKnee,0,.12,.55); position(value,KinectJoint::LeftAnkle,0,.12,.1); position(value,KinectJoint::LeftFoot,.2,.12,.05);
    position(value,KinectJoint::RightHip,0,-.12,1); position(value,KinectJoint::RightKnee,0,-.12,.55); position(value,KinectJoint::RightAnkle,0,-.12,.1); position(value,KinectJoint::RightFoot,.2,-.12,.05);
    const double wave=std::sin(phase);
    value.joints[id(KinectJoint::SpineChest)].orientation_wxyz=camera_quaternion(0,0,.08*wave);
    const auto left=camera_quaternion(0,.18*wave,0),right=camera_quaternion(0,-.18*wave,0);
    for(auto joint:{KinectJoint::LeftShoulder,KinectJoint::LeftElbow,KinectJoint::LeftWrist}) value.joints[id(joint)].orientation_wxyz=left;
    for(auto joint:{KinectJoint::RightShoulder,KinectJoint::RightElbow,KinectJoint::RightWrist}) value.joints[id(joint)].orientation_wxyz=right;
    return value;
}
}

int main(int argc,char** argv) {
    const int gmr_port=argc>1?std::atoi(argv[1]):5558;
    const int sonic_port=argc>2?std::atoi(argv[2]):5567;
    const int seconds=argc>3?std::atoi(argv[3]):5;
    KinectToGMRAdapter gmr(gmr_port,500);
    SonicV1Publisher sonic(sonic_port);
    std::uint64_t timestamp=1000000,frame=0,published=0;
    const int total=std::max(1,seconds)*30+15;
    for(int i=0;i<total;++i,timestamp+=33333) {
        const double phase=i<15?0.0:2*pi*(i-15)/90.0;
        if(const auto reference=gmr.update(sample(timestamp,phase))) {
            if(!sonic.publish_command(true,false) ||
               !sonic.publish(*reference,static_cast<std::int64_t>(frame++)))
                throw std::runtime_error("failed to publish SONIC Protocol v1 frame");
            ++published;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
    if(published<static_cast<std::uint64_t>(seconds*25))
        throw std::runtime_error("insufficient GMR frames");
    if(!sonic.publish_command(false,true))
        throw std::runtime_error("failed to publish SONIC stop command");
    std::cout << "gmr_sonic_frames=" << published << " stopped=yes\n";
}
