cmake_minimum_required(VERSION 2.8)
project(pcl_example)

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
#add_definitions(-std=c++11 -O2 -g -DNDEBUG)
add_definitions(${PCL_DEFINITIONS})

find_package(Boost REQUIRED)
Find_Package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)


include_directories(
  include
${Boost_INCLUDE_DIR}
${EIGEN3_INCLUDE_DIR}
${OpenCV_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(pcl_example src/pcl_example.cpp)
target_link_libraries(pcl_example
${Boost_LIBRARIES}
${OpenCV_LIBS}
${PCL_LIBRARIES}
)

