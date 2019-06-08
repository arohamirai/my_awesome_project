cmake_minimum_required(VERSION 2.8)
project(ceres_example)

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

add_executable(ceres_example src/ceres_example.cpp)
target_link_libraries(ceres_example
${Boost_LIBRARIES}
${OpenCV_LIBS}
${VISP_LIBRARIES}
${CERES_LIBRARIES}
)

