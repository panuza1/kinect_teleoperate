find_path(ZMQ_INCLUDE_DIR zmq.h)
find_library(ZMQ_LIBRARY NAMES zmq)

if (ZMQ_INCLUDE_DIR AND ZMQ_LIBRARY)
    add_executable(kinect_smpl_zmq_bridge
        main.cpp
        skeleton_to_smpl.cpp
        sonic_zmq_publisher.cpp
    )

    target_include_directories(kinect_smpl_zmq_bridge PRIVATE
        ${ZMQ_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    target_link_libraries(kinect_smpl_zmq_bridge PRIVATE
        k4abt
        k4a
        ${ZMQ_LIBRARY}
    )
else()
    message(WARNING "ZMQ development files not found; kinect_smpl_zmq_bridge will not be built")
endif()
