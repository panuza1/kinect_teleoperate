#include "skeleton_to_smpl.hpp"

#include <cassert>
#include <cmath>
#include <limits>

namespace {
void set_joint(JointSample& joint, float x, float y, float z) {
    joint.position.xyz = {x, y, z};
    joint.orientation.wxyz = {1.0f, 0.0f, 0.0f, 0.0f};
    joint.confidence_level = CONFIDENCE_HIGH;
}

float at(const SonicPoseFrame& frame, int joint, int axis) {
    return frame.smpl_joints[joint * 3 + axis];
}
}

int main() {
    BodySkeleton skeleton{};
    for (auto& joint : skeleton.joints) set_joint(joint, 0, 0, 1000);
    set_joint(skeleton.joints[JOINT_HEAD], 0, -700, 1000);
    set_joint(skeleton.joints[JOINT_FOOT_LEFT], -100, 900, 1000);
    set_joint(skeleton.joints[JOINT_FOOT_RIGHT], 100, 900, 1000);
    set_joint(skeleton.joints[JOINT_SHOULDER_LEFT], -250, -450, 1000);
    set_joint(skeleton.joints[JOINT_SHOULDER_RIGHT], 250, -450, 1000);
    set_joint(skeleton.joints[JOINT_WRIST_LEFT], -300, -300, 1000);

    SkeletonToSmpl converter;
    SonicPoseFrame neutral;
    assert(converter.convert(skeleton, 0, 1.0f / 30.0f, neutral));
    assert(std::fabs(at(neutral, 0, 0)) < 1e-6f);
    assert(at(neutral, 16, 1) > 0.2f && at(neutral, 17, 1) < -0.2f);
    assert(at(neutral, 15, 2) > 0.7f && at(neutral, 10, 2) < -0.9f);
    assert(std::fabs(neutral.body_quat_w[0] - 1.0f) < 1e-6f);

    skeleton.joints[JOINT_WRIST_LEFT].position.xyz.y -= 200;
    skeleton.joints[JOINT_ELBOW_LEFT].orientation.wxyz =
        {std::cos(0.25f), std::sin(0.25f), 0.0f, 0.0f};
    SonicPoseFrame raised;
    assert(converter.convert(skeleton, 1, 1.0f / 30.0f, raised));
    assert(at(raised, 20, 2) > at(neutral, 20, 2) + 0.04f);
    assert(raised.smpl_pose[(18 - 1) * 3 + 1] < -0.1f);

    for (auto& joint : skeleton.joints) joint.position.xyz.z += 100;
    auto& wrist = skeleton.joints[JOINT_WRIST_LEFT];
    wrist.confidence_level = CONFIDENCE_NONE;
    wrist.position.xyz.x = std::numeric_limits<float>::quiet_NaN();
    SonicPoseFrame occluded;
    assert(converter.convert(skeleton, 2, 1.0f / 30.0f, occluded));
    assert(std::fabs(at(occluded, 20, 2) - at(raised, 20, 2)) < 1e-6f);

    wrist.confidence_level = CONFIDENCE_HIGH;
    wrist.position.xyz.x = -300;
    wrist.position.xyz.y += 200;
    skeleton.joints[JOINT_ELBOW_LEFT].orientation.wxyz = {1, 0, 0, 0};
    SonicPoseFrame returned;
    for (int i = 3; i < 43; ++i) {
        assert(converter.convert(skeleton, i, 1.0f / 30.0f, returned));
    }
    assert(std::fabs(at(returned, 20, 2) - at(neutral, 20, 2)) < 1e-3f);
    assert(std::fabs(returned.smpl_pose[(18 - 1) * 3 + 1]) < 1e-3f);

    const float yaw = 0.3f;
    for (auto& joint : skeleton.joints) {
        const float x = joint.position.xyz.x;
        const float z = joint.position.xyz.z - 1100.0f;
        joint.position.xyz.x = std::cos(yaw) * x + std::sin(yaw) * z;
        joint.position.xyz.z = 1100.0f - std::sin(yaw) * x + std::cos(yaw) * z;
        joint.orientation.wxyz = {std::cos(yaw / 2), 0, std::sin(yaw / 2), 0};
    }
    SonicPoseFrame turned;
    for (int i = 43; i < 83; ++i) {
        assert(converter.convert(skeleton, i, 1.0f / 30.0f, turned));
    }
    assert(std::fabs(at(turned, 16, 1) - at(neutral, 16, 1)) < 1e-3f);
    assert(turned.body_quat_w[3] < -0.1f);

    skeleton.joints[JOINT_WRIST_LEFT].position.xyz.y -= 200;
    skeleton.joints[JOINT_WRIST_RIGHT].position.xyz.y -= 200;
    SonicPoseFrame both_raised;
    for (int i = 83; i < 103; ++i) {
        assert(converter.convert(skeleton, i, 1.0f / 30.0f, both_raised));
    }
    assert(at(both_raised, 20, 2) > at(turned, 20, 2) + 0.1f);
    assert(at(both_raised, 21, 2) > at(turned, 21, 2) + 0.1f);
    skeleton.joints[JOINT_WRIST_LEFT].position.xyz.y += 200;
    skeleton.joints[JOINT_WRIST_RIGHT].position.xyz.y += 200;
    SonicPoseFrame both_returned;
    for (int i = 103; i < 143; ++i) {
        assert(converter.convert(skeleton, i, 1.0f / 30.0f, both_returned));
    }
    assert(std::fabs(at(both_returned, 20, 2) - at(turned, 20, 2)) < 1e-3f);
    assert(std::fabs(at(both_returned, 21, 2) - at(turned, 21, 2)) < 1e-3f);

    skeleton.joints[JOINT_PELVIS].position.xyz.x = std::numeric_limits<float>::quiet_NaN();
    assert(!converter.convert(skeleton, 143, 1.0f / 30.0f, both_returned));
}
