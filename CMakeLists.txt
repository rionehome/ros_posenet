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

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
#add_executable(viewer src/module/json11.cpp src/viewer.cpp)
#add_dependencies(viewer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(viewer ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_executable(camera src/module/json11.cpp src/camera.cpp include/ros_posenet/camera.h)
add_dependencies(camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(camera ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_executable(kinect src/module/json11.cpp src/kinect.cpp include/ros_posenet/kinect.h)
add_dependencies(kinect ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinect ${OpenCV_LIBS} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#add_executable(injection_pose src/json11.cpp src/injection_pose.cpp)
#add_dependencies(injection_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(injection_pose ${OpenCV_LIBS} ${catkin_LIBRARIES})
#add_executable(pose_centroid src/pose_centroid.cpp)
#add_dependencies(pose_centroid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(pose_centroid ${OpenCV_LIBS} ${catkin_LIBRARIES})
#add_executable(main src/main.cpp)
#target_link_libraries(main  ${catkin_LIBRARIES})
