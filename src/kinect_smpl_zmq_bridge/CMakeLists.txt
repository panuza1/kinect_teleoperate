find_path(ZMQ_INCLUDE_DIR zmq.h)
find_library(ZMQ_LIBRARY NAMES zmq)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

if (ZMQ_INCLUDE_DIR AND ZMQ_LIBRARY)
    add_executable(kinect_smpl_zmq_bridge
        main.cpp
        bridge_session.cpp
        skeleton_to_smpl.cpp
        sonic_zmq_publisher.cpp
    )

    target_include_directories(kinect_smpl_zmq_bridge PRIVATE
        ${ZMQ_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    target_link_libraries(kinect_smpl_zmq_bridge PRIVATE
        k4abt::k4abt
        k4a::k4a
        Eigen3::Eigen
        ${ZMQ_LIBRARY}
    )
else()
    if (KINECT_BRIDGE_ONLY)
        message(FATAL_ERROR "ZMQ development files not found; kinect_smpl_zmq_bridge cannot be built")
    else()
        message(WARNING "ZMQ development files not found; kinect_smpl_zmq_bridge will not be built")
    endif()
endif()

add_executable(kinect_bridge_session_test
    bridge_session_test.cpp
    bridge_session.cpp
    skeleton_to_smpl.cpp
)
target_link_libraries(kinect_bridge_session_test PRIVATE Eigen3::Eigen k4abt::k4abt k4a::k4a)
