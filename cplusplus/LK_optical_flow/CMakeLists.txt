cmake_minimum_required(VERSION 2.8)
project(LK_optical_flow)

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
#add_definitions(-std=c++11 -O2 -g -DNDEBUG)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(Boost REQUIRED)
Find_Package(Eigen3 REQUIRED)
find_package( OpenCV REQUIRED )
find_package( Ceres REQUIRED )

include_directories(
  include
${Boost_INCLUDE_DIR}
${EIGEN3_INCLUDE_DIR}
${OpenCV_INCLUDE_DIRS}
${CERES_INCLUDE_DIRS}
)

add_executable(LK_optical_flow src/LK_optical_flow.cpp)
target_link_libraries(LK_optical_flow
${Boost_LIBRARIES}
${OpenCV_LIBS}
${VISP_LIBRARIES}
${CERES_LIBRARIES}
)

