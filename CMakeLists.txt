#auto-regist no
cmake_minimum_required(VERSION 2.8.3)
project(ros_posenet)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        geometry_msgs
        message_generation
        cv_bridge
        )

add_definitions(-std=c++11)

add_message_files(
        FILES
        Keypoint.msg
        Pose.msg
        Poses.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_lib
        CATKIN_DEPENDS message_runtime geometry_msgs
)

install(PROGRAMS
        scripts/posenet.js
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(FILES
        launch/camera.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(OpenCV_DIR "/usr/local/include/opencv2/")
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "-pthread")

add_executable(camera src/module/json11.cpp src/camera.cpp include/ros_posenet/camera.h)
add_dependencies(camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_executable(kinect src/module/json11.cpp src/kinect.cpp include/ros_posenet/kinect.h)
add_dependencies(kinect ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinect ${OpenCV_LIBS} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(realsense src/module/json11.cpp src/realsense.cpp include/ros_posenet/realsense.h)
add_dependencies(realsense ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(realsense ${OpenCV_LIBS} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
