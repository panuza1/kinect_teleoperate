# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE Release)
# Eigen
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_path(ZMQ_INCLUDE_DIR zmq.h REQUIRED)
find_library(ZMQ_LIBRARY NAMES libzmq.so.5 zmq REQUIRED)

set(MuJoCo_Include_Dir ${CMAKE_CURRENT_SOURCE_DIR}/../mujoco-3.1.5/include)
set(MuJoCo_Library_Dir ${CMAKE_CURRENT_SOURCE_DIR}/../mujoco-3.1.5/lib)
set(MuJoCo_Libraries mujoco)

link_directories(
    ${MuJoCo_Library_Dir}
    ${hx_idl_cpp_Include_Dir}
)

set(SONIC_G1_SCENE "${CMAKE_SOURCE_DIR}/../GR00T-WholeBodyControl/gear_sonic_deploy/g1/g1_29dof.xml")
set(SONIC_INCLUDE "${CMAKE_SOURCE_DIR}/../GR00T-WholeBodyControl/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include")
set(SONIC_JSON_INCLUDE "${CMAKE_SOURCE_DIR}/../GR00T-WholeBodyControl/external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64/include")

add_library(kinect_g1_retargeter KinectToG1Retargeter.cpp)
target_include_directories(kinect_g1_retargeter PUBLIC include/)

add_library(sonic_v1_publisher SonicV1Publisher.cpp)
target_include_directories(sonic_v1_publisher PUBLIC include/ ${ZMQ_INCLUDE_DIR})
target_link_libraries(sonic_v1_publisher PUBLIC kinect_g1_retargeter ${ZMQ_LIBRARY})

add_library(kinect_gmr_adapter KinectToGMRAdapter.cpp)
target_include_directories(kinect_gmr_adapter PUBLIC include/ ${ZMQ_INCLUDE_DIR})
target_link_libraries(kinect_gmr_adapter PUBLIC kinect_g1_retargeter ${ZMQ_LIBRARY})

add_executable(kinect_teleoperate main.cpp)
target_compile_definitions(kinect_teleoperate PRIVATE
    KINECT_G1_SCENE_PATH="${SONIC_G1_SCENE}")

target_include_directories(kinect_teleoperate 
    PRIVATE 
    ../sample_helper_includes 
    ${MuJoCo_Include_Dir}
    include/
    # unitree_sdk2
)

# Dependencies of this library
target_link_libraries(kinect_teleoperate PRIVATE 
    k4a::k4a
    k4abt::k4abt
    ${K4ARECORD_LIBRARY}
    window_controller_3d::window_controller_3d
    glfw::glfw
    ${MuJoCo_Libraries}
    Eigen3::Eigen
    kinect_g1_retargeter
    sonic_v1_publisher
    kinect_gmr_adapter
    # unitree_sdk2
)

add_executable(kinect_gmr_sonic_sender gmr_sonic_sender.cpp)
target_link_libraries(kinect_gmr_sonic_sender PRIVATE kinect_gmr_adapter sonic_v1_publisher)

if (BUILD_TESTING)
    set(GMR_ROOT "${CMAKE_SOURCE_DIR}/../GMR" CACHE PATH "Official GMR checkout")
    set(GMR_PYTHON_EXECUTABLE "/home/panu/miniconda3/envs/gmr/bin/python" CACHE FILEPATH "Python with GMR dependencies")
    if (EXISTS "${GMR_PYTHON_EXECUTABLE}" AND EXISTS "${GMR_ROOT}/general_motion_retargeting")
        add_test(NAME kinect_gmr_adapter_test
            COMMAND "${GMR_PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/tools/gmr_adapter_test.py")
        set_tests_properties(kinect_gmr_adapter_test PROPERTIES
            ENVIRONMENT "PYTHONPATH=${GMR_ROOT}:${CMAKE_SOURCE_DIR}/tools")

        add_executable(kinect_gmr_comparison_test gmr_comparison_test.cpp)
        target_link_libraries(kinect_gmr_comparison_test PRIVATE kinect_gmr_adapter kinect_g1_retargeter)
        target_compile_options(kinect_gmr_comparison_test PRIVATE -UNDEBUG)
        add_test(NAME kinect_gmr_comparison_test
            COMMAND "${GMR_PYTHON_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/tools/gmr_ipc_comparison_test.py"
                    "$<TARGET_FILE:kinect_gmr_comparison_test>")
        set_tests_properties(kinect_gmr_comparison_test PROPERTIES
            ENVIRONMENT "PYTHONPATH=${GMR_ROOT}:${CMAKE_SOURCE_DIR}/tools")
    endif()

    add_executable(kinect_g1_retargeter_test retargeter_test.cpp)
    target_link_libraries(kinect_g1_retargeter_test PRIVATE kinect_g1_retargeter)
    target_compile_options(kinect_g1_retargeter_test PRIVATE -UNDEBUG)
    add_test(NAME kinect_g1_retargeter_test COMMAND kinect_g1_retargeter_test)

    add_executable(kinect_g1_mujoco_test mujoco_model_test.cpp)
    target_compile_definitions(kinect_g1_mujoco_test PRIVATE KINECT_G1_SCENE_PATH="${SONIC_G1_SCENE}")
    target_include_directories(kinect_g1_mujoco_test PRIVATE ${MuJoCo_Include_Dir})
    target_link_libraries(kinect_g1_mujoco_test PRIVATE kinect_g1_retargeter ${MuJoCo_Libraries})
    target_compile_options(kinect_g1_mujoco_test PRIVATE -UNDEBUG)
    add_test(NAME kinect_g1_mujoco_test COMMAND kinect_g1_mujoco_test)

    if (EXISTS "${SONIC_INCLUDE}/input_interface/zmq_packed_message_subscriber.hpp" AND EXISTS "${SONIC_JSON_INCLUDE}/nlohmann/json.hpp")
        add_executable(kinect_sonic_v1_protocol_test sonic_v1_protocol_test.cpp)
        target_include_directories(kinect_sonic_v1_protocol_test PRIVATE ${SONIC_INCLUDE} ${SONIC_JSON_INCLUDE} ${ZMQ_INCLUDE_DIR})
        target_link_libraries(kinect_sonic_v1_protocol_test PRIVATE sonic_v1_publisher ${ZMQ_LIBRARY})
        target_compile_options(kinect_sonic_v1_protocol_test PRIVATE -UNDEBUG)
        add_test(NAME kinect_sonic_v1_protocol_test COMMAND kinect_sonic_v1_protocol_test)
    endif()
endif()

