#include "include/KinectToGMRAdapter.hpp"
#include "include/SonicV1Publisher.hpp"
#include "include/SonicV1StreamState.hpp"

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
    std::uint64_t timestamp=1000000,frame=0,sequence=0,ref_count=0,tx_count=0;
    G1Reference latest{};
    auto arrival=SonicV1StreamState::Clock::time_point{};
    const auto start=SonicV1StreamState::Clock::now();
    auto next_tx=start, next_ref=start;
    SonicV1StreamState stream;
    for(int i=0;i<std::max(1,seconds)*50;++i) {
        const auto now=SonicV1StreamState::Clock::now();
        if(now>=next_ref) {
            const double phase=ref_count<15?0.0:2*pi*(ref_count-15)/60.0;
            if(const auto reference=gmr.update(sample(timestamp,phase))) {
                latest=*reference;
                ++sequence;
                ++ref_count;
                arrival=SonicV1StreamState::Clock::now();
            }
            timestamp+=50000;
            next_ref+=std::chrono::milliseconds(50);
            if(now>next_ref+std::chrono::milliseconds(50)) next_ref=now+std::chrono::milliseconds(50);
        }
        const auto action=stream.tick(sequence,arrival,now,true);
        if(action.send_start && !sonic.publish_command(true,false))
            throw std::runtime_error("failed to publish SONIC start command");
        if(action.publish_pose) {
            if(action.held) latest.joint_vel.fill(0.0);
            if(!sonic.publish(latest,static_cast<std::int64_t>(++frame)))
                throw std::runtime_error("failed to publish SONIC Protocol v1 frame");
            ++tx_count;
        }
        if(action.send_stop && !sonic.publish_command(false,true))
            throw std::runtime_error("failed to publish SONIC stop command");
        next_tx+=SonicV1StreamState::tx_period;
        if(now>next_tx+SonicV1StreamState::tx_period) next_tx=now+SonicV1StreamState::tx_period;
        std::this_thread::sleep_until(next_tx);
    }
    if(stream.streaming() && !sonic.publish_command(false,true))
        throw std::runtime_error("failed to publish SONIC stop command");
    if(ref_count<static_cast<std::uint64_t>(seconds*15) || tx_count<static_cast<std::uint64_t>(seconds*45))
        throw std::runtime_error("insufficient GMR frames");
    std::cout << "gmr_ref_frames=" << ref_count << " pose_tx_frames=" << tx_count
              << " gmr_ref_fps=" << ref_count/static_cast<double>(seconds)
              << " pose_tx_fps=" << tx_count/static_cast<double>(seconds) << " stopped=yes\n";
}
