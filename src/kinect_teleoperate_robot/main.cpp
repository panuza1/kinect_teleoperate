// For kinect camera driver
#include <k4arecord/playback.h>
#include <k4a/k4a.h>

// For kinect body tracking and render
#include <k4abt.h>
#include <BodyTrackingHelpers.h>
#include <Utilities.h>
#include <Window3dWrapper.h>

// STL
#include <chrono>
#include <thread>
#include <mutex>
#include <array>
#include <atomic>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>
#include <string>
#include <utility>

// For mujoco render
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>


// For math tool
#include "math_tool.hpp"
// For start or end state detect
#include "StartEndPoseDetector.hpp"
// For retargeting function from the skeleton joint angles to the robot motor joint angles.
#include "jointRetargeting.hpp"
#include "KinectToG1Retargeter.hpp"
#include "KinectToGMRAdapter.hpp"
#include "SonicV1Publisher.hpp"

using namespace std::chrono;

std::atomic_bool s_isRunning{true};
std::atomic_bool s_kinectReady{false};
std::atomic_bool s_kinectRenderReady{false};
bool s_alwaysActive = false;
std::string s_bodyTrackingModelPath;
std::string s_output = "mujoco-direct";
std::string s_retargeterMode = "gmr";
int s_sonicPort = 5556;
int s_gmrPort = 5558;
bool s_debugSkeleton = false;
bool s_verbose = false;
KinectToG1Retargeter s_retargeter;
std::unique_ptr<KinectToGMRAdapter> s_gmrAdapter;
G1Reference s_g1Reference;
RetargetStats s_retargetStats;
GMRAdapterStats s_gmrStats;
KinectSkeletonSample s_lastSkeleton;
std::uint64_t s_referenceSequence = 0;
std::mutex s_referenceMutex;

#define Control_G1 true
#define Control_H1 false
#define Real_Control false    // control real unitree robot in reality
#define Enable_Torso false    // enable torso rotation angle mapping. Test function, open with caution!
#define Enable_Hand  false    // enable hand opening and closing status detection. Test function, open with caution!
#define EchoFrequency false   // Whether to display the running frequency of each thread

#if Real_Control
// SDK include files for unitree robot real control
// Please refer to https://github.com/unitreerobotics/unitree_sdk2
#endif

// kinect body tracking skeleton joint angle
// reference: https://learn.microsoft.com/en-us/azure/kinect-dk/body-joints
// ':=' means that the item on the left hand side is being defined to be what is on the right hand side.
// sc:=spine chest, ls:=left shoulder, le:=left elbow, rs:=right shoulder, re:=right elbow, lh:=left hand, rh:=right hand
// _r:=roll, _p:=pitch, _y:=yaw, _a:=angle
struct KinectJointAngles {
    float sc_r = 0, sc_p = 0, sc_y = 0;
    float ls_r = 0, ls_p = 0, ls_y = 0;
    float le_r = 0, le_p = 0, le_y = 0;
    float rs_r = 0, rs_p = 0, rs_y = 0;
    float re_r = 0, re_p = 0, re_y = 0;
    std::array<int, 5> confidence{};
    uint32_t body_id = 0;
    uint64_t sequence = 0;
};

KinectJointAngles s_jointAngles;
std::mutex s_jointAnglesMutex;

struct hardware_control_signal {
    double left_shoulder_roll = 0.0;
    double left_shoulder_pitch = 0.0;
    double left_shoulder_yaw = 0.0;
    double right_shoulder_roll = 0.0;
    double right_shoulder_pitch = 0.0;
    double right_shoulder_yaw = 0.0;
    double left_elbow_yaw = 0.0;
    double right_elbow_yaw = 0.0;
};

// For control real robot G1
#if Control_G1
hardware_control_signal G1_hardware_signal;
#endif

// For control real robot H1
#if Control_H1
hardware_control_signal H1_hardware_signal;
#endif

/*************************************************Kinect Render, display human skeleton joint tracking*********************************************/

Visualization::Layout3d s_layoutMode = Visualization::Layout3d::OnlyMainView;
bool s_visualizeJointFrame = false;
k4abt_frame_t globalBodyFrameForSkeleton = nullptr;
std::mutex s_bodyFrameMutex;

void PrintUsage()
{
    printf("\n");
    printf(" Basic Navigation:\n\n");
    printf(" Rotate: Rotate the camera by moving the mouse while holding mouse left button\n");
    printf(" Pan: Translate the scene by holding Ctrl key and drag the scene with mouse left button\n");
    printf(" Zoom in/out: Move closer/farther away from the scene center by scrolling the mouse scroll wheel\n");
    printf(" Select Center: Center the scene based on a detected joint by right clicking the joint with mouse\n");
    printf("\n");
    printf(" Key Shortcuts\n\n");
    printf(" ESC: quit\n");
    printf(" h: help\n");
    printf(" b: body visualization mode\n");
    printf(" k: 3d window layout\n");
    printf("\n");
}

int64_t ProcessKey(void* /*context*/, int key)
{
    // https://www.glfw.org/docs/latest/group__keys.html
    switch (key)
    {
        // Quit
    case GLFW_KEY_ESCAPE:
        s_isRunning = false;
        break;
    case GLFW_KEY_K:
        s_layoutMode = (Visualization::Layout3d)(((int)s_layoutMode + 1) % (int)Visualization::Layout3d::Count);
        break;
    case GLFW_KEY_B:
        s_visualizeJointFrame = !s_visualizeJointFrame;
        break;
    case GLFW_KEY_H:
        PrintUsage();
        break;
    }
    return 1;
}

int64_t CloseCallback(void* /*context*/)
{
    s_isRunning = false;
    return 1;
}

void renderSkeletonAndPointCloud(k4abt_frame_t bodyFrame, Window3dWrapper &kinectRenderWindow, int depthWidth, int depthHeight, int step = 2) {
    uint32_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);
    float minDistance = std::numeric_limits<float>::max();
    uint32_t closestBodyIndex = 0;

    if (numBodies == 0) {
        return;
    }
    
    for (uint32_t i = 0; i < numBodies; i++) {
        k4abt_body_t body;
        if (k4abt_frame_get_body_skeleton(bodyFrame, i, &body.skeleton) != K4A_RESULT_SUCCEEDED) {
            std::cerr << "Get skeleton from body frame failed!" << std::endl;
            continue;
        }

        k4abt_joint_t spineChestJoint = body.skeleton.joints[K4ABT_JOINT_SPINE_CHEST];
        float distance = spineChestJoint.position.xyz.z;

        if (distance < minDistance) {
            minDistance = distance;
            closestBodyIndex = i;
        }
    }

    // Obtain original capture that generates the body tracking result
    k4a_capture_t originalCapture = k4abt_frame_get_capture(bodyFrame);
    k4a_image_t depthImage = k4a_capture_get_depth_image(originalCapture);
    std::vector<Color> pointCloudColors(depthWidth * depthHeight, { 1.f, 1.f, 1.f, 1.f });
    // Read body index map and assign colors
    k4a_image_t bodyIndexMap = k4abt_frame_get_body_index_map(bodyFrame);
    const uint8_t* bodyIndexMapBuffer = k4a_image_get_buffer(bodyIndexMap);
    for (int i = 0; i < depthWidth * depthHeight; i = i+step) {
        uint8_t bodyIndex = bodyIndexMapBuffer[i];
        if (bodyIndex == closestBodyIndex) {
            uint32_t bodyId = k4abt_frame_get_body_id(bodyFrame, bodyIndex);
            pointCloudColors[i] = g_bodyColors[bodyId % g_bodyColors.size()];
        }
    }
    k4a_image_release(bodyIndexMap);
    kinectRenderWindow.UpdatePointClouds(depthImage, pointCloudColors);//add parameter 'int step' to accelerate point cloud render
    k4a_capture_release(originalCapture);
    k4a_image_release(depthImage);


    kinectRenderWindow.CleanJointsAndBones();
    k4abt_body_t body;
    VERIFY(k4abt_frame_get_body_skeleton(bodyFrame, closestBodyIndex, &body.skeleton), "Get skeleton from body frame failed!");

    // Assign the correct color based on the body id
    Color color = g_bodyColors[ 1 % g_bodyColors.size()];
    color.a = 0.4f;
    Color lowConfidenceColor = g_bodyColors[ 6 % g_bodyColors.size()];
    lowConfidenceColor.a = 0.3f;

    // Visualize joints
    for (int joint = 0; joint < static_cast<int>(K4ABT_JOINT_COUNT); joint++) {
        if (body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM) {
            const k4a_float3_t& jointPosition = body.skeleton.joints[joint].position;
            const k4a_quaternion_t& jointOrientation = body.skeleton.joints[joint].orientation;

            kinectRenderWindow.AddJoint(
                jointPosition,
                jointOrientation,
                body.skeleton.joints[joint].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM ? color : lowConfidenceColor);
        }
    }

    // Visualize bones
    for (size_t boneIdx = 0; boneIdx < g_boneList.size(); boneIdx++) {
        k4abt_joint_id_t joint1 = g_boneList[boneIdx].first;
        k4abt_joint_id_t joint2 = g_boneList[boneIdx].second;

        if (body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW &&
            body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_LOW) {
            bool confidentBone = body.skeleton.joints[joint1].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM &&
                                 body.skeleton.joints[joint2].confidence_level >= K4ABT_JOINT_CONFIDENCE_MEDIUM;
            const k4a_float3_t& joint1Position = body.skeleton.joints[joint1].position;
            const k4a_float3_t& joint2Position = body.skeleton.joints[joint2].position;

            kinectRenderWindow.AddBone(joint1Position, joint2Position, confidentBone ? color : lowConfidenceColor);
        }
    }
}

void KinectRender_loop(k4a_calibration_t sensorCalibration) {
    std::cout<<"Kinect Render loop start..."<<std::endl;
    std::cout<<"Please use the wake-up action to start or stop the TeleOperation..."<<std::endl;
    Window3dWrapper kinectRenderWindow;
    kinectRenderWindow.Create("Kinect Render", sensorCalibration);
    s_kinectRenderReady = true;
    kinectRenderWindow.SetCloseCallback(CloseCallback);
    kinectRenderWindow.SetKeyCallback(ProcessKey);
    int depthWidth = sensorCalibration.depth_camera_calibration.resolution_width;
    int depthHeight = sensorCalibration.depth_camera_calibration.resolution_height;

    time_point<high_resolution_clock> KinectRender_start;
    while (s_isRunning) {
        k4abt_frame_t bodyFrame = nullptr;
        {
            std::scoped_lock lock(s_bodyFrameMutex);
            std::swap(bodyFrame, globalBodyFrameForSkeleton);
        }
        if (bodyFrame != nullptr) {
            renderSkeletonAndPointCloud(bodyFrame, kinectRenderWindow, depthWidth, depthHeight);
            k4abt_frame_release(bodyFrame);

            kinectRenderWindow.SetLayout3d(s_layoutMode);
            kinectRenderWindow.SetJointFrameVisualization(s_visualizeJointFrame);
            kinectRenderWindow.Render();

            #if EchoFrequency
            time_point<high_resolution_clock> KinectRender_end = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(KinectRender_end - KinectRender_start).count();
            KinectRender_start = KinectRender_end;
            double frequency = 1e6 / duration;
            std::cout << "Kinect Render loop: " << frequency << " Hz" << std::endl;
            #endif
        } 
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    kinectRenderWindow.Delete();
}

/*************************************************Simulation Unitree Robot Control************************************************/

mjModel* m = nullptr;               // MuJoCo model
mjData*  d = nullptr;               // MuJoCo data
std::mutex s_mujocoMutex;

GLFWwindow* mujocoRenderWindow;
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);
  glfwGetCursorPos(window, &lastx, &lasty);
}
// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  int width, height;
  glfwGetWindowSize(window, &width, &height);

  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}
// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

GLFWwindow * InitWindow() {
    if (!glfwInit())
        mju_error("can not initialize GLFW");
    GLFWwindow* window = glfwCreateWindow(1280, 960, "Mujoco Render", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);
    return window;
}
void initCamera(mjvCamera* camera) {
    camera->lookat[0] = 0.0; 
    camera->lookat[1] = 0.0; 
    camera->lookat[2] = 0.5; 

    camera->distance = 3.0;

    camera->azimuth = 0;  
    camera->elevation = -30;
}

void DrawOneFrame(GLFWwindow* window) {
    std::scoped_lock lock(s_mujocoMutex);
    // get framebuffer viewport
    glfwMakeContextCurrent(window);
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    glfwSwapBuffers(window);
    glfwPollEvents();
}

void MujocoRender_loop() {
    std::cout<<"Mujoco Render loop start..."<<std::endl;
    mujocoRenderWindow = InitWindow();
    initCamera(&cam);
    DrawOneFrame(mujocoRenderWindow);

    time_point<high_resolution_clock> mujocoRender_start;
    while (s_isRunning) {
        if (glfwWindowShouldClose(mujocoRenderWindow)) {
            s_isRunning = false;
            break;
        }
        DrawOneFrame(mujocoRenderWindow);

        #if EchoFrequency
        time_point<high_resolution_clock> mujocoRender_end = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(mujocoRender_end - mujocoRender_start).count();
        double frequency = 1e6 / duration;
        mujocoRender_start = mujocoRender_end;
        std::cout << "Mujoco Render loop: " << frequency << " Hz" << std::endl;
        #endif
    }

    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    glfwDestroyWindow(mujocoRenderWindow);
}

/*****************************************************Real Unitree Robot Control*****************************************************************/
#if Real_Control

#if Control_G1
// Replace there with the Unitree SDK control code
// Send your motor joint value by G1_hardware_signal
// Please refer to https://github.com/unitreerobotics/unitree_sdk2
#endif

#if Control_H1
// Replace there with the Unitree SDK control code
// Send your motor joint value by H1_hardware_signal
// Please refer to https://github.com/unitreerobotics/unitree_sdk2
#endif

#endif

/*****************************************************Control Smooth and Transfer*****************************************************************/

void Control_loop() {
    std::cout<<"control loop start..."<<std::endl;
    std::cout << "[direct] arm_order=left_shoulder_pitch,left_shoulder_roll,left_shoulder_yaw,"
                 "left_elbow_pitch,right_shoulder_pitch,right_shoulder_roll,right_shoulder_yaw,"
                 "right_elbow_pitch" << std::endl;
    int left_shoulder_roll_joint_id = mj_name2id(m, mjOBJ_ACTUATOR, "left_shoulder_roll_joint");
    int left_shoulder_pitch_joint_id = mj_name2id(m, mjOBJ_ACTUATOR, "left_shoulder_pitch_joint");
    int left_shoulder_yaw_joint_id = mj_name2id(m, mjOBJ_ACTUATOR, "left_shoulder_yaw_joint");
    int right_shoulder_roll_joint_id = mj_name2id(m, mjOBJ_ACTUATOR, "right_shoulder_roll_joint");
    int right_shoulder_pitch_joint_id = mj_name2id(m, mjOBJ_ACTUATOR, "right_shoulder_pitch_joint");
    int right_shoulder_yaw_joint_id = mj_name2id(m, mjOBJ_ACTUATOR, "right_shoulder_yaw_joint");
    #if Control_H1
    int left_elbow_pitch_joint_id = mj_name2id(m, mjOBJ_ACTUATOR, "left_elbow_joint");
    int right_elbow_pitch_joint_id = mj_name2id(m, mjOBJ_ACTUATOR, "right_elbow_joint");
    #elif Control_G1
    int left_elbow_pitch_joint_id = mj_name2id(m, mjOBJ_ACTUATOR, "left_elbow_pitch_joint");
    int right_elbow_pitch_joint_id = mj_name2id(m, mjOBJ_ACTUATOR, "right_elbow_pitch_joint");
    #endif
    #if Enable_Torso
    int torso_joint_id = mj_name2id(m, mjOBJ_ACTUATOR, "torso_joint");
    #endif

    const std::array<int, 8> actuator_ids{
        left_shoulder_pitch_joint_id, left_shoulder_roll_joint_id, left_shoulder_yaw_joint_id,
        left_elbow_pitch_joint_id, right_shoulder_pitch_joint_id, right_shoulder_roll_joint_id,
        right_shoulder_yaw_joint_id, right_elbow_pitch_joint_id};
    for (int id : actuator_ids) {
        if (id < 0) {
            std::cerr << "Required G1 arm actuator is missing from the MuJoCo model" << std::endl;
            s_isRunning = false;
            return;
        }
    }

    {
        std::scoped_lock lock(s_mujocoMutex);
        mj_step(m, d); // For starting render mujoco
    }

    MovingAverageFilter ls_r_filter,ls_p_filter,ls_y_filter,
                        rs_r_filter,rs_p_filter,rs_y_filter,
                        le_y_filter,re_y_filter,sc_p_filter;

    StartEndPoseDetector pose_detector;

    auto diagnostic_start = steady_clock::now();
    uint64_t previous_sequence = 0;
    while (s_isRunning) {
        KinectJointAngles angles;
        {
            std::scoped_lock lock(s_jointAnglesMutex);
            angles = s_jointAngles;
        }
        if (angles.sequence == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        double left_shoulder_roll = LS_mappingCameraRoll2RobotRadians(angles.ls_r);
        double left_shoulder_pitch = LS_mappingCameraPitch2RobotRadians(angles.ls_p);
        double left_shoulder_yaw = LS_mappingCameraYaw2RobotRadians(angles.ls_y);
        double right_shoulder_roll = RS_mappingCameraRoll2RobotRadians(angles.rs_r);
        double right_shoulder_pitch = RS_mappingCameraPitch2RobotRadians(angles.rs_p);
        double right_shoulder_yaw = RS_mappingCameraYaw2RobotRadians(angles.rs_y);
        double left_elbow_yaw = LE_mappingCameraYaw2RobotRadians(angles.le_y);
        double right_elbow_yaw = RE_mappingCameraYaw2RobotRadians(angles.re_y);

        #if Enable_Torso
        double spine_chest_torso = SC_mappingCameraPitch2RobotTorso(angles.sc_p);
        #endif

        bool start = s_alwaysActive || pose_detector.isStartEndPose(
            left_shoulder_roll, left_shoulder_pitch, left_shoulder_yaw,
            right_shoulder_roll, right_shoulder_pitch, right_shoulder_yaw,
            left_elbow_yaw, right_elbow_yaw);
        if(start)
        {
            // smoothing
            left_shoulder_roll = ls_r_filter.update(left_shoulder_roll);
            left_shoulder_pitch = ls_p_filter.update(left_shoulder_pitch);
            left_shoulder_yaw = ls_y_filter.update(left_shoulder_yaw);
            right_shoulder_roll = rs_r_filter.update(right_shoulder_roll);
            right_shoulder_pitch = rs_p_filter.update(right_shoulder_pitch);
            right_shoulder_yaw = rs_y_filter.update(right_shoulder_yaw);
            left_elbow_yaw = le_y_filter.update(left_elbow_yaw);
            right_elbow_yaw = re_y_filter.update(right_elbow_yaw);
            #if Enable_Torso
            spine_chest_torso = sc_p_filter.update(spine_chest_torso);
            #endif

            std::scoped_lock lock(s_mujocoMutex);
            mj_step1(m, d);
            // The coordinate system of the robot joints is as follows: the x-axis (roll) points forward, the y-axis (pitch) points left, 
            // and the z-axis (yaw) points upward. 
            // Test to get the rotation order, because the coordinate system definition of the Kinect camera is still a mystery
            // Camera:   z(yaw)      x(roll)   y(pitch) 
            //             |           |          |
            //             ∨           ∨          ∨
            // Robot :  y(pitch)     z(yaw)    x(roll)
            d->ctrl[left_shoulder_pitch_joint_id] = left_shoulder_yaw;
            d->ctrl[left_shoulder_roll_joint_id] = left_shoulder_pitch;
            d->ctrl[left_shoulder_yaw_joint_id] = left_shoulder_roll;
            d->ctrl[right_shoulder_pitch_joint_id] = right_shoulder_yaw;
            d->ctrl[right_shoulder_roll_joint_id] = right_shoulder_pitch;
            d->ctrl[right_shoulder_yaw_joint_id] = right_shoulder_roll;
            d->ctrl[left_elbow_pitch_joint_id] = left_elbow_yaw;
            d->ctrl[right_elbow_pitch_joint_id] = right_elbow_yaw;
            #if Enable_Torso
            d->ctrl[torso_joint_id] = spine_chest_torso;
            #endif
            mj_step2(m, d);

            #if Real_Control
            #if Control_H1
            H1_hardware_signal.left_shoulder_pitch = left_shoulder_yaw;
            H1_hardware_signal.left_shoulder_roll = left_shoulder_roll;
            H1_hardware_signal.left_shoulder_yaw = left_shoulder_pitch;
            H1_hardware_signal.right_shoulder_pitch = right_shoulder_yaw;
            H1_hardware_signal.right_shoulder_roll = right_shoulder_pitch;
            H1_hardware_signal.right_shoulder_yaw = right_shoulder_roll;
            H1_hardware_signal.left_elbow_yaw = left_elbow_yaw;
            H1_hardware_signal.right_elbow_yaw = right_elbow_yaw;
            #elif Control_G1
            G1_hardware_signal.left_shoulder_pitch = left_shoulder_yaw;
            G1_hardware_signal.left_shoulder_roll = left_shoulder_roll;
            G1_hardware_signal.left_shoulder_yaw = left_shoulder_pitch;
            G1_hardware_signal.right_shoulder_pitch = right_shoulder_yaw;
            G1_hardware_signal.right_shoulder_roll = right_shoulder_pitch;
            G1_hardware_signal.right_shoulder_yaw = right_shoulder_roll;
            G1_hardware_signal.left_elbow_yaw = left_elbow_yaw;
            G1_hardware_signal.right_elbow_yaw = right_elbow_yaw;
            #endif
            #endif

        }

        const auto now = steady_clock::now();
        if (now - diagnostic_start >= seconds(1)) {
            std::array<double, 8> actuator_targets{};
            {
                std::scoped_lock lock(s_mujocoMutex);
                for (size_t i = 0; i < actuator_ids.size(); ++i)
                    actuator_targets[i] = d->ctrl[actuator_ids[i]];
            }
            std::cout << "[direct] wake=" << (start ? "ACTIVE" : "WAITING")
                      << " raw=" << angles.ls_r << ',' << angles.ls_p << ',' << angles.ls_y << ','
                      << angles.le_y << ',' << angles.rs_r << ',' << angles.rs_p << ','
                      << angles.rs_y << ',' << angles.re_y
                      << " retarget=" << left_shoulder_yaw << ',' << left_shoulder_pitch << ','
                      << left_shoulder_roll << ',' << left_elbow_yaw << ',' << right_shoulder_yaw << ','
                      << right_shoulder_pitch << ',' << right_shoulder_roll << ',' << right_elbow_yaw
                      << " actuator=";
            for (size_t i = 0; i < actuator_targets.size(); ++i)
                std::cout << (i ? "," : "") << actuator_targets[i];
            std::cout << " input_changed=" << (angles.sequence != previous_sequence ? "yes" : "no") << std::endl;
            previous_sequence = angles.sequence;
            diagnostic_start = now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}


void FullBodyControl_loop() {
    std::unique_ptr<SonicV1Publisher> publisher;
    std::array<int,29> qpos_address{};
    std::array<double,7> fixed_base{};
    if (s_output == "sonic-v1") {
        publisher = std::make_unique<SonicV1Publisher>(s_sonicPort);
    } else {
        for (std::size_t i=0;i<29;++i) {
            const int joint=mj_name2id(m,mjOBJ_JOINT,KinectToG1Retargeter::joint_names[i]);
            const int actuator=mj_name2id(m,mjOBJ_ACTUATOR,KinectToG1Retargeter::actuator_names[i]);
            if (joint<0 || actuator<0) {
                std::cerr << "Missing SONIC G1 joint/actuator: " << KinectToG1Retargeter::joint_names[i] << '\n';
                s_isRunning=false;
                return;
            }
            qpos_address[i]=m->jnt_qposadr[joint];
        }
        for (int i=0;i<std::min(7,m->nq);++i) fixed_base[i]=d->qpos[i];
    }

    StartEndPoseDetector pose_detector;
    auto metric_start=steady_clock::now();
    std::uint64_t consumed=0, frames=0, sent=0;
    bool was_active=false;
    while (s_isRunning) {
        G1Reference reference;
        std::uint64_t sequence=0;
        {
            std::scoped_lock lock(s_referenceMutex);
            reference=s_g1Reference;
            sequence=s_referenceSequence;
        }
        if (!sequence || sequence==consumed) {
            std::this_thread::sleep_for(milliseconds(5));
            continue;
        }
        const bool active=s_alwaysActive || pose_detector.isStartEndPose(
            reference.joint_pos[16],reference.joint_pos[15],reference.joint_pos[17],
            reference.joint_pos[23],reference.joint_pos[22],reference.joint_pos[24],
            reference.joint_pos[18],reference.joint_pos[25]);
        if (active) {
            if (publisher) {
                publisher->publish_command(true,false);
                if (publisher->publish(reference,static_cast<std::int64_t>(sequence))) ++sent;
            } else {
                std::scoped_lock lock(s_mujocoMutex);
                for (int i=0;i<std::min(7,m->nq);++i) d->qpos[i]=fixed_base[i];
                for (int i=0;i<m->nv;++i) d->qvel[i]=0;
                for (std::size_t i=0;i<29;++i) d->qpos[qpos_address[i]]=reference.joint_pos[i];
                mj_forward(m,d);
            }
        } else if (publisher && was_active) {
            publisher->publish_command(false,true);
        }
        was_active=active;
        consumed=sequence;
        ++frames;
        const auto now=steady_clock::now();
        const double elapsed=duration<double>(now-metric_start).count();
        if (elapsed>=1.0) {
            const auto limits=std::minmax_element(reference.joint_pos.begin(),reference.joint_pos.end());
            double vmax=0; for(double value:reference.joint_vel) vmax=std::max(vmax,std::abs(value));
            std::cout << (s_retargeterMode=="gmr" ? "gmr_fps=" : "retarget_fps=") << frames/elapsed
                      << " qpos_min=" << *limits.first
                      << " qpos_max=" << *limits.second << " qvel_max=" << vmax
                      << " output=" << s_output << " pose_tx_fps=" << sent/elapsed
                      << " active=" << (active?"yes":"no") << '\n';
            if (s_verbose) {
                for (std::size_t i=0;i<29;++i)
                    std::cout << KinectToG1Retargeter::joint_names[i] << '=' << reference.joint_pos[i]
                              << " vel=" << reference.joint_vel[i] << '\n';
            }
            frames=sent=0;
            metric_start=now;
        }
    }
}

void ProcessFullBodySkeletonData(k4abt_frame_t bodyFrame) {
    const std::uint32_t count=k4abt_frame_get_num_bodies(bodyFrame);
    if (!count) return;
    std::uint32_t closest=0;
    float min_distance=std::numeric_limits<float>::max();
    k4abt_body_t body{};
    for (std::uint32_t i=0;i<count;++i) {
        k4abt_body_t candidate{};
        if (k4abt_frame_get_body_skeleton(bodyFrame,i,&candidate.skeleton)!=K4A_RESULT_SUCCEEDED) continue;
        const float distance=candidate.skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.xyz.z;
        if (distance<min_distance) { min_distance=distance; closest=i; body=candidate; }
    }

    KinectSkeletonSample sample;
    sample.timestamp_us=k4abt_frame_get_device_timestamp_usec(bodyFrame);
    sample.body_id=k4abt_frame_get_body_id(bodyFrame,closest);
    const std::array<std::pair<KinectJoint,k4abt_joint_id_t>,19> mapping{{
        {KinectJoint::Pelvis,K4ABT_JOINT_PELVIS}, {KinectJoint::SpineNavel,K4ABT_JOINT_SPINE_NAVEL},
        {KinectJoint::SpineChest,K4ABT_JOINT_SPINE_CHEST}, {KinectJoint::Neck,K4ABT_JOINT_NECK}, {KinectJoint::Head,K4ABT_JOINT_HEAD},
        {KinectJoint::LeftShoulder,K4ABT_JOINT_SHOULDER_LEFT}, {KinectJoint::LeftElbow,K4ABT_JOINT_ELBOW_LEFT}, {KinectJoint::LeftWrist,K4ABT_JOINT_WRIST_LEFT},
        {KinectJoint::RightShoulder,K4ABT_JOINT_SHOULDER_RIGHT}, {KinectJoint::RightElbow,K4ABT_JOINT_ELBOW_RIGHT}, {KinectJoint::RightWrist,K4ABT_JOINT_WRIST_RIGHT},
        {KinectJoint::LeftHip,K4ABT_JOINT_HIP_LEFT}, {KinectJoint::LeftKnee,K4ABT_JOINT_KNEE_LEFT}, {KinectJoint::LeftAnkle,K4ABT_JOINT_ANKLE_LEFT}, {KinectJoint::LeftFoot,K4ABT_JOINT_FOOT_LEFT},
        {KinectJoint::RightHip,K4ABT_JOINT_HIP_RIGHT}, {KinectJoint::RightKnee,K4ABT_JOINT_KNEE_RIGHT}, {KinectJoint::RightAnkle,K4ABT_JOINT_ANKLE_RIGHT}, {KinectJoint::RightFoot,K4ABT_JOINT_FOOT_RIGHT}}};
    for (const auto& [target,source] : mapping) {
        const auto& input=body.skeleton.joints[source];
        auto& output=sample.joints[static_cast<std::size_t>(target)];
        output.position_mm={input.position.xyz.x,input.position.xyz.y,input.position.xyz.z};
        output.orientation_wxyz={input.orientation.wxyz.w,input.orientation.wxyz.x,input.orientation.wxyz.y,input.orientation.wxyz.z};
        output.confidence=static_cast<std::uint8_t>(input.confidence_level);
    }
    std::optional<G1Reference> reference;
    RetargetStats stats;
    GMRAdapterStats gmr_stats;
    if (s_retargeterMode == "legacy") {
        const auto legacy=s_retargeter.update(sample);
        stats=s_retargeter.stats();
        if (stats.accepted) reference=legacy;
    } else {
        reference=s_gmrAdapter->update(sample);
        gmr_stats=s_gmrAdapter->stats();
    }
    std::scoped_lock lock(s_referenceMutex);
    s_retargetStats=stats;
    s_gmrStats=gmr_stats;
    s_lastSkeleton=sample;
    if (reference) {
        s_g1Reference=*reference;
        ++s_referenceSequence;
    }
}

// Process the newly captured skeleton point data to calculate the joint angles that control the robot
void ProcessNewSkeletonData(k4abt_frame_t bodyFrame) {
    uint32_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);
    float minDistance = std::numeric_limits<float>::max();
    uint32_t closestBodyIndex = 0;

    if (numBodies == 0) {
        return;
    }
    // Take the data of the human body closest to the kinect camera as input
    for (uint32_t i = 0; i < numBodies; i++) {
        k4abt_body_t body;
        if (k4abt_frame_get_body_skeleton(bodyFrame, i, &body.skeleton) != K4A_RESULT_SUCCEEDED) {
            std::cerr << "Get skeleton from body frame failed!" << std::endl;
            continue;
        }

        k4abt_joint_t spineChestJoint = body.skeleton.joints[K4ABT_JOINT_SPINE_CHEST];
        float distance = spineChestJoint.position.xyz.z;

        if (distance < minDistance) {
            minDistance = distance;
            closestBodyIndex = i;
        }
    }

    k4abt_body_t closestBody;
    if (k4abt_frame_get_body_skeleton(bodyFrame, closestBodyIndex, &closestBody.skeleton) != K4A_RESULT_SUCCEEDED) {
        std::cerr << "Get skeleton from body frame failed!" << std::endl;
        return;
    }

    static k4a_quaternion_t spine_chest{}, left_shoulder{}, right_shoulder{};
    const std::array<k4abt_joint_id_t, 5> joints{
        K4ABT_JOINT_SPINE_CHEST, K4ABT_JOINT_SHOULDER_LEFT, K4ABT_JOINT_SHOULDER_RIGHT,
        K4ABT_JOINT_ELBOW_LEFT, K4ABT_JOINT_ELBOW_RIGHT};
    std::scoped_lock lock(s_jointAnglesMutex);
    s_jointAngles.body_id = k4abt_frame_get_body_id(bodyFrame, closestBodyIndex);
    for (size_t i = 0; i < joints.size(); ++i)
        s_jointAngles.confidence[i] = closestBody.skeleton.joints[joints[i]].confidence_level;

    if (s_jointAngles.confidence[0] > K4ABT_JOINT_CONFIDENCE_LOW) {
        spine_chest = closestBody.skeleton.joints[K4ABT_JOINT_SPINE_CHEST].orientation;
        quaternion2Euler(spine_chest, s_jointAngles.sc_r, s_jointAngles.sc_p, s_jointAngles.sc_y);
    }
    if (s_jointAngles.confidence[1] > K4ABT_JOINT_CONFIDENCE_LOW) {
        left_shoulder = closestBody.skeleton.joints[K4ABT_JOINT_SHOULDER_LEFT].orientation;
        const auto relative = calculateRelativeQuaternion(left_shoulder, spine_chest);
        quaternion2Euler(relative, s_jointAngles.ls_r, s_jointAngles.ls_p, s_jointAngles.ls_y);
    }
    if (s_jointAngles.confidence[3] > K4ABT_JOINT_CONFIDENCE_LOW) {
        const auto relative = calculateRelativeQuaternion(
            closestBody.skeleton.joints[K4ABT_JOINT_ELBOW_LEFT].orientation, left_shoulder);
        quaternion2Euler(relative, s_jointAngles.le_r, s_jointAngles.le_p, s_jointAngles.le_y);
    }
    if (s_jointAngles.confidence[2] > K4ABT_JOINT_CONFIDENCE_LOW) {
        right_shoulder = closestBody.skeleton.joints[K4ABT_JOINT_SHOULDER_RIGHT].orientation;
        const auto relative = calculateRelativeQuaternion(right_shoulder, spine_chest);
        quaternion2Euler(relative, s_jointAngles.rs_r, s_jointAngles.rs_p, s_jointAngles.rs_y);
    }
    if (s_jointAngles.confidence[4] > K4ABT_JOINT_CONFIDENCE_LOW) {
        const auto relative = calculateRelativeQuaternion(
            closestBody.skeleton.joints[K4ABT_JOINT_ELBOW_RIGHT].orientation, right_shoulder);
        quaternion2Euler(relative, s_jointAngles.re_r, s_jointAngles.re_p, s_jointAngles.re_y);
    }
    ++s_jointAngles.sequence;
    
    // For hand open and closing status detect
    #if Enable_Hand
    if(closestBody.skeleton.joints[K4ABT_JOINT_HANDTIP_LEFT].confidence_level > K4ABT_JOINT_CONFIDENCE_LOW && 
        closestBody.skeleton.joints[K4ABT_JOINT_WRIST_LEFT].confidence_level > K4ABT_JOINT_CONFIDENCE_LOW){
        k4a_quaternion_t LeftHandTip_orientation = closestBody.skeleton.joints[K4ABT_JOINT_HANDTIP_LEFT].orientation;
        k4a_quaternion_t LeftWrist_orientation = closestBody.skeleton.joints[K4ABT_JOINT_WRIST_LEFT].orientation;
        lh_a = calculateRelativeAngle(LeftHandTip_orientation, LeftWrist_orientation);
        if(lh_a < 30.0)
            std::cout<<"left hand open."<<std::endl;
        else if(lh_a > 50.0)
            std::cout<<"left hand close."<<std::endl;
    }
    else
        std::cout<<"left hand unknow."<<std::endl;

    if(closestBody.skeleton.joints[K4ABT_JOINT_HANDTIP_RIGHT].confidence_level > K4ABT_JOINT_CONFIDENCE_LOW && 
        closestBody.skeleton.joints[K4ABT_JOINT_WRIST_RIGHT].confidence_level > K4ABT_JOINT_CONFIDENCE_LOW){
        k4a_quaternion_t RightHandTip_orientation = closestBody.skeleton.joints[K4ABT_JOINT_HANDTIP_RIGHT].orientation;
        k4a_quaternion_t RightWrist_orientation = closestBody.skeleton.joints[K4ABT_JOINT_WRIST_RIGHT].orientation;
        rh_a = calculateRelativeAngle(RightHandTip_orientation, RightWrist_orientation);
        if(rh_a < 30.0)
            std::cout<<"right hand open."<<std::endl;
        else if(rh_a > 50.0)
            std::cout<<"right hand close."<<std::endl;
    }
    else
        std::cout<<"right hand unknow."<<std::endl;
    #endif
}

void Main_loop(){

    PrintUsage();

    k4a_device_t device = nullptr;
    if (k4a_device_get_installed_count() == 0) {
        std::cerr << "Femto Bolt/K4A-compatible device not detected" << std::endl;
        s_isRunning = false;
        return;
    }
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    // K4A_DEPTH_MODE_NFOV_2X2BINNED, /**< Depth captured at 320x288. Passive IR is also captured at 320x288. */
    // K4A_DEPTH_MODE_NFOV_UNBINNED,  /**< Depth captured at 640x576. Passive IR is also captured at 640x576. */
    // K4A_DEPTH_MODE_WFOV_2X2BINNED, /**< Depth captured at 512x512. Passive IR is also captured at 512x512. */
    deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

    // Get calibration information
    k4a_calibration_t sensorCalibration;
    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration),
        "Get depth camera calibration failed!");

    // Create Body Tracker
    k4abt_tracker_t tracker = nullptr;
    k4abt_tracker_configuration_t trackerConfig = K4ABT_TRACKER_CONFIG_DEFAULT;
    trackerConfig.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU_CUDA;
    trackerConfig.model_path = s_bodyTrackingModelPath.c_str();
    std::cout << "body_tracker=GPU_CUDA model=" << trackerConfig.model_path << std::endl;
    VERIFY(k4abt_tracker_create(&sensorCalibration, trackerConfig, &tracker), "Body tracker initialization failed!");
    // Do not use Kinect's built-in smoothing
    k4abt_tracker_set_temporal_smoothing(tracker, 0.0);
    s_kinectReady = true;

    std::thread KinectRender_thread(KinectRender_loop, sensorCalibration);

    auto metric_start = steady_clock::now();
    uint64_t tracking_frames = 0;
    bool previous_body_detected = false;
    while (s_isRunning)
    {
        k4a_capture_t sensorCapture = nullptr;
        k4a_wait_result_t getCaptureResult = k4a_device_get_capture(device, &sensorCapture, 100);
        if (getCaptureResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            k4a_wait_result_t queueCaptureResult = k4abt_tracker_enqueue_capture(tracker, sensorCapture, 100);
            k4a_capture_release(sensorCapture);
            if (queueCaptureResult == K4A_WAIT_RESULT_FAILED)
            {
                std::cout << "Error! Add capture to tracker process queue failed!" << std::endl;
                break;
            }
        }
        else if (getCaptureResult != K4A_WAIT_RESULT_TIMEOUT)
        {
            std::cout << "Get depth capture returned error: " << getCaptureResult << std::endl;
            break;
        }

        k4abt_frame_t bodyFrame = nullptr;
        k4a_wait_result_t popFrameResult = k4abt_tracker_pop_result(tracker, &bodyFrame, 100);
        if (popFrameResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            {
                std::scoped_lock lock(s_bodyFrameMutex);
                if (globalBodyFrameForSkeleton == nullptr) {
                    globalBodyFrameForSkeleton = bodyFrame;
                    k4abt_frame_reference(globalBodyFrameForSkeleton);
                }
            }
            const bool body_detected = k4abt_frame_get_num_bodies(bodyFrame) > 0;
            ProcessFullBodySkeletonData(bodyFrame);
            k4abt_frame_release(bodyFrame);
            ++tracking_frames;
            if (body_detected != previous_body_detected) {
                std::cout << "[direct] body_detected=" << (body_detected ? "yes" : "no") << std::endl;
                previous_body_detected = body_detected;
            }

            const auto now = steady_clock::now();
            const double elapsed = duration<double>(now - metric_start).count();
            if (elapsed >= 1.0) {
                RetargetStats stats;
                GMRAdapterStats gmr_stats;
                KinectSkeletonSample skeleton;
                {
                    std::scoped_lock lock(s_referenceMutex);
                    stats=s_retargetStats;
                    gmr_stats=s_gmrStats;
                    skeleton=s_lastSkeleton;
                }
                std::cout << "kinect_fps=" << tracking_frames/elapsed << " body_id=" << skeleton.body_id;
                if (s_retargeterMode == "gmr") {
                    std::cout << " valid_targets=" << gmr_stats.valid_targets
                              << " held_targets=" << gmr_stats.held_targets
                              << " stale_targets=" << gmr_stats.stale_targets
                              << " gmr_solve_ms=" << gmr_stats.solve_ms;
                } else {
                    std::cout << " valid_joints=" << stats.valid_joints << " held_joints=" << stats.held_joints
                              << " stale_joints=" << stats.stale_joints;
                }
                std::cout << " retargeter=" << s_retargeterMode << " output=" << s_output << '\n';
                if (s_debugSkeleton || s_verbose) {
                    for (std::size_t i=0;i<skeleton.joints.size();++i) {
                        const auto& joint=skeleton.joints[i];
                        std::cout << "joint=" << i << " confidence=" << static_cast<int>(joint.confidence)
                                  << " held=" << stats.held[i] << " stale=" << stats.stale[i];
                        if (s_verbose) std::cout << " orientation_wxyz=" << joint.orientation_wxyz[0] << ','
                            << joint.orientation_wxyz[1] << ',' << joint.orientation_wxyz[2] << ',' << joint.orientation_wxyz[3];
                        std::cout << '\n';
                    }
                }
                tracking_frames = 0;
                metric_start = now;
            }
        }
    }

    std::cout << "kinect_teleoperate_robot finished!" << std::endl;

    s_isRunning = false;
    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);

    k4a_device_stop_cameras(device);
    k4a_device_close(device);

    KinectRender_thread.join();
    {
        std::scoped_lock lock(s_bodyFrameMutex);
        if (globalBodyFrameForSkeleton) {
            k4abt_frame_release(globalBodyFrameForSkeleton);
            globalBodyFrameForSkeleton = nullptr;
        }
    }
}


struct Options {
    std::string body_model = "/home/panu/.local/share/azure-kinect/1.4.1-1.1.2/usr/bin/dnn_model_2_0_op11.onnx";
    std::string mujoco_scene = KINECT_G1_SCENE_PATH;
    std::string output = "mujoco-direct";
    std::string retargeter = "gmr";
    int port = 5556;
    int gmr_port = 5558;
    bool always_active = false;
    bool fixed_base = false;
    bool debug_skeleton = false;
    bool verbose = false;
    bool help = false;
};

Options ParseOptions(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--model" && i + 1 < argc) options.body_model = argv[++i];
        else if (arg == "--scene" && i + 1 < argc) options.mujoco_scene = argv[++i];
        else if (arg == "--output" && i + 1 < argc) options.output = argv[++i];
        else if (arg == "--retargeter" && i + 1 < argc) options.retargeter = argv[++i];
        else if (arg == "--port" && i + 1 < argc) options.port = std::stoi(argv[++i]);
        else if (arg == "--gmr-port" && i + 1 < argc) options.gmr_port = std::stoi(argv[++i]);
        else if (arg == "--always-active") options.always_active = true;
        else if (arg == "--fixed-base") options.fixed_base = true;
        else if (arg == "--debug-skeleton") options.debug_skeleton = true;
        else if (arg == "--verbose") options.verbose = true;
        else if (arg == "--help") options.help = true;
        else throw std::runtime_error(
            "usage: kinect_teleoperate [--retargeter gmr|legacy] [--output mujoco-direct|sonic-v1] [--fixed-base] "
            "[--always-active] [--debug-skeleton] [--verbose] [--port 5556] [--gmr-port 5558] [--model PATH] [--scene PATH]");
    }
    if (options.output != "mujoco-direct" && options.output != "sonic-v1")
        throw std::runtime_error("--output must be mujoco-direct or sonic-v1");
    if (options.retargeter != "gmr" && options.retargeter != "legacy")
        throw std::runtime_error("--retargeter must be gmr or legacy");
    if (options.port < 1 || options.port > 65535) throw std::runtime_error("--port must be 1..65535");
    if (options.gmr_port < 1 || options.gmr_port > 65535) throw std::runtime_error("--gmr-port must be 1..65535");
    return options;
}

int main(int argc, char** argv)
{
    Options options;
    try {
        options = ParseOptions(argc, argv);
    } catch (const std::exception& error) {
        std::cerr << error.what() << std::endl;
        return 1;
    }
    if (options.help) {
        std::cout << "usage: kinect_teleoperate [--retargeter gmr|legacy] [--output mujoco-direct|sonic-v1] [--fixed-base] "
                     "[--always-active] [--debug-skeleton] [--verbose] [--port 5556] [--gmr-port 5558] [--model PATH] [--scene PATH]\n";
        return 0;
    }
    if (!std::filesystem::is_regular_file(options.body_model)) {
        std::cerr << "Body Tracking model not found: " << options.body_model << std::endl;
        return 1;
    }
    if (options.output == "mujoco-direct" && !std::filesystem::is_regular_file(options.mujoco_scene)) {
        std::cerr << "MuJoCo scene not found: " << options.mujoco_scene << std::endl;
        return 1;
    }
    #if Real_Control
    if (options.always_active) {
        std::cerr << "--always-active is available only when Real_Control=false" << std::endl;
        return 1;
    }
    #endif
    s_bodyTrackingModelPath = std::filesystem::absolute(options.body_model).string();
    s_alwaysActive = options.always_active;
    s_output = options.output;
    s_retargeterMode = options.retargeter;
    s_sonicPort = options.port;
    s_gmrPort = options.gmr_port;
    s_debugSkeleton = options.debug_skeleton;
    s_verbose = options.verbose;
    if (s_retargeterMode == "gmr") {
        s_gmrAdapter=std::make_unique<KinectToGMRAdapter>(s_gmrPort);
        std::cout << "retargeter=gmr bridge=127.0.0.1:" << s_gmrPort << '\n';
    } else {
        std::cout << "retargeter=legacy diagnostic_only=yes\n";
    }

    if (s_output == "mujoco-direct") {
        char error[1000];
        m = mj_loadXML(options.mujoco_scene.c_str(), nullptr, error, 1000);
        if (!m) {
            std::cerr << "Failed to load MuJoCo model '" << options.mujoco_scene << "': " << error << std::endl;
            return 1;
        }
        d = mj_makeData(m);
        if (!d) { mj_deleteModel(m); return 1; }
        mj_resetData(m, d);
        std::cout << "mujoco_scene=" << std::filesystem::absolute(options.mujoco_scene)
                  << " actuators=" << m->nu << " fixed_base=yes Real_Control=false Enable_Torso=false\n";
    } else {
        std::cout << "sonic_protocol=1 encode_mode=0 bind=127.0.0.1:" << s_sonicPort
                  << " Real_Control=false hands=false\n";
    }

    std::thread kinect_thread(Main_loop);
    while (s_isRunning && (!s_kinectReady || !s_kinectRenderReady))
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (!s_isRunning) {
        kinect_thread.join();
        if (d) mj_deleteData(d);
        if (m) mj_deleteModel(m);
        return 1;
    }
    std::thread control_thread(FullBodyControl_loop);
    std::thread mujocoRender_thread;
    if (s_output == "mujoco-direct") mujocoRender_thread=std::thread(MujocoRender_loop);

    kinect_thread.join();
    control_thread.join();
    if (mujocoRender_thread.joinable()) mujocoRender_thread.join();

    if (d) mj_deleteData(d);
    if (m) mj_deleteModel(m);

    return 0;
}
