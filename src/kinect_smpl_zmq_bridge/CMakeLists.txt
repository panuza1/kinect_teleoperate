find_path(ZMQ_INCLUDE_DIR zmq.h)
# The Kinect redistributable may contain a static libzmq.a without its optional
# dependencies.  Prefer the host shared library for the bridge executable.
find_library(ZMQ_LIBRARY NAMES libzmq.so.5 zmq
    HINTS /lib/x86_64-linux-gnu /usr/lib/x86_64-linux-gnu
    NO_CMAKE_PATH NO_CMAKE_ENVIRONMENT_PATH)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

add_library(kinect_bridge_core
    calibration.cpp
    bridge_config.cpp
    bridge_session.cpp
    skeleton_to_smpl.cpp
)
target_link_libraries(kinect_bridge_core PUBLIC Eigen3::Eigen yaml-cpp::yaml-cpp)
target_include_directories(kinect_bridge_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

function(kinect_bridge_runtime target)
    if (KINECT_ENABLE_HARDWARE)
        set_property(TARGET ${target} APPEND PROPERTY BUILD_RPATH
            "$<TARGET_FILE_DIR:k4a::k4a>;$<TARGET_FILE_DIR:k4abt::k4abt>")
    endif()
    if (KINECT_ENABLE_HARDWARE AND CMAKE_SYSTEM_NAME STREQUAL "Linux")
        # k4abt loads libk4a itself.  DT_RPATH is inherited for that indirect
        # dependency whereas DT_RUNPATH is not on glibc.
        target_link_options(${target} PRIVATE -Wl,--disable-new-dtags)
    endif()
endfunction()

if (ZMQ_INCLUDE_DIR AND ZMQ_LIBRARY)
    if (KINECT_ENABLE_HARDWARE)
        add_executable(kinect_smpl_zmq_bridge
            kinect_input.cpp
            main.cpp
            sonic_zmq_publisher.cpp
        )
        target_include_directories(kinect_smpl_zmq_bridge PRIVATE
            ${ZMQ_INCLUDE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}
        )
        target_link_libraries(kinect_smpl_zmq_bridge PRIVATE
            kinect_bridge_core k4abt::k4abt k4a::k4a ${ZMQ_LIBRARY})
        kinect_bridge_runtime(kinect_smpl_zmq_bridge)
    endif()

    add_executable(kinect_synthetic_sim_sender
        synthetic_sim_sender.cpp
        sonic_zmq_publisher.cpp
    )
    target_include_directories(kinect_synthetic_sim_sender PRIVATE ${ZMQ_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(kinect_synthetic_sim_sender PRIVATE kinect_bridge_core ${ZMQ_LIBRARY})

    set(_sonic_include "${CMAKE_SOURCE_DIR}/../GR00T-WholeBodyControl/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include")
    set(_sonic_vendor_include "${CMAKE_SOURCE_DIR}/../GR00T-WholeBodyControl/external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64/include")
    if (EXISTS "${_sonic_include}/input_interface/transport_freshness.hpp" AND
        EXISTS "${_sonic_vendor_include}/nlohmann/json.hpp")
        add_executable(kinect_transport_test transport_test.cpp sonic_zmq_publisher.cpp)
        target_include_directories(kinect_transport_test PRIVATE
            ${ZMQ_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} ${_sonic_include} ${_sonic_vendor_include})
        target_link_libraries(kinect_transport_test PRIVATE kinect_bridge_core ${ZMQ_LIBRARY})
        target_compile_options(kinect_transport_test PRIVATE -UNDEBUG)
    endif()
else()
    if (KINECT_BRIDGE_ONLY AND KINECT_ENABLE_HARDWARE)
        message(FATAL_ERROR "ZMQ development files not found; kinect_smpl_zmq_bridge cannot be built")
    else()
        message(WARNING "ZMQ development files not found; kinect_smpl_zmq_bridge will not be built")
    endif()
endif()

add_executable(kinect_bridge_session_test
    bridge_session_test.cpp
)
target_link_libraries(kinect_bridge_session_test PRIVATE kinect_bridge_core)
target_compile_options(kinect_bridge_session_test PRIVATE -UNDEBUG)

add_executable(kinect_synthetic_test synthetic_test.cpp)
target_link_libraries(kinect_synthetic_test PRIVATE kinect_bridge_core)
target_compile_options(kinect_synthetic_test PRIVATE -UNDEBUG)

add_executable(kinect_calibration_test calibration_test.cpp)
target_link_libraries(kinect_calibration_test PRIVATE kinect_bridge_core)
target_compile_options(kinect_calibration_test PRIVATE -UNDEBUG)

if (BUILD_TESTING)
    add_test(NAME kinect_bridge_session_test COMMAND kinect_bridge_session_test)
    add_test(NAME kinect_synthetic_test COMMAND kinect_synthetic_test)
    add_test(NAME kinect_calibration_test COMMAND kinect_calibration_test)
    if (TARGET kinect_transport_test)
        add_test(NAME kinect_transport_test COMMAND kinect_transport_test)
    endif()
    # The vendor packages install k4a and k4abt in separate library directories.
    # CTest needs both while exercising the body-tracking session without a camera.
    if (KINECT_ENABLE_HARDWARE)
        set(_kinect_test_library_path
            "$<TARGET_FILE_DIR:k4a::k4a>:$<TARGET_FILE_DIR:k4abt::k4abt>:$ENV{LD_LIBRARY_PATH}")
        set_tests_properties(kinect_bridge_session_test kinect_synthetic_test kinect_calibration_test PROPERTIES
            ENVIRONMENT "LD_LIBRARY_PATH=${_kinect_test_library_path}")
    endif()
endif()
