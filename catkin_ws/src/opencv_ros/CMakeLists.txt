cmake_minimum_required(VERSION 2.8.3)
project(opencv_ros)

set(OpenCV_DIR /opt/ros/kinetic/include/opencv-3.2.0-dev)
add_definitions(-std=gnu++11 -O2 -g -DNDEBUG)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}  ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  sensor_msgs
  visual_servo_msgs
)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Glog  REQUIRED)
add_definitions(${PCL_DEFINITIONS})
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
${GLOG_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES laserscan_to_image
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

add_executable(opencv_ros src/opencv_ros.cpp)
target_link_libraries(opencv_ros ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${GLOG_LIBRARY})

install(TARGETS opencv_ros
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
