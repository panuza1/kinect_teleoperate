#include "include/KinectToG1Retargeter.hpp"

#include <mujoco/mujoco.h>

#include <array>
#include <cassert>
#include <cmath>
#include <iostream>

int main() {
    char error[1024]{};
    mjModel* model=mj_loadXML(KINECT_G1_SCENE_PATH,nullptr,error,sizeof(error));
    assert(model && "SONIC G1 XML must load");
    mjData* data=mj_makeData(model); assert(data);
    mj_resetData(model,data);
    std::array<int,29> qadr{};
    std::array<double,29> neutral{};
    for (std::size_t i=0;i<29;++i) {
        const int joint=mj_name2id(model,mjOBJ_JOINT,KinectToG1Retargeter::joint_names[i]);
        const int actuator=mj_name2id(model,mjOBJ_ACTUATOR,KinectToG1Retargeter::actuator_names[i]);
        assert(joint>=0 && actuator>=0);
        assert(model->jnt_limited[joint]);
        qadr[i]=model->jnt_qposadr[joint];
        data->qpos[qadr[i]]=KinectToG1Retargeter::neutral[i];
        neutral[i]=data->qpos[qadr[i]];
    }
    mj_forward(model,data);
    mj_forward(model,data);
    for (std::size_t i=0;i<29;++i) assert(data->qpos[qadr[i]]==neutral[i]);
    const int left=mj_name2id(model,mjOBJ_JOINT,"left_shoulder_pitch_joint");
    const int left_q=model->jnt_qposadr[left];
    const double before=data->qpos[left_q]; data->qpos[left_q]=before+0.2; mj_forward(model,data);
    assert(std::abs(data->qpos[left_q]-before)>0.1);
    if (model->nq>=7) {
        const double base_z=data->qpos[2]; mj_forward(model,data); assert(data->qpos[2]==base_z);
    }
    mj_deleteData(data); mj_deleteModel(model);
    std::cout << "MuJoCo model test passed\n";
}
