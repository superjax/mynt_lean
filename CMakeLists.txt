cmake_minimum_required(VERSION 2.8.3)
project(mynt_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
)
add_subdirectory(lib/SDK)
find_package(OpenCV REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp sensor_msgs image_transport
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  lib/SDK/include
)


add_executable(mynt_ros_node
        src/mynt_ros_node.cpp
        src/mynt_ros.cpp
        include/mynt/mynt_ros.h
)
target_link_libraries(mynt_ros_node 
  mynteye 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})
