cmake_minimum_required(VERSION 2.8)
project(boost_eigen_example)

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
#add_definitions(-std=c++11 -O2 -g -DNDEBUG)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(Boost REQUIRED)
Find_Package(Eigen3 REQUIRED)

include_directories(
  include
${Boost_INCLUDE_DIR}
${EIGEN3_INCLUDE_DIR}
)

add_executable(boost_eigen_example src/boost_eigen_example.cpp)
target_link_libraries(boost_eigen_example
${Boost_LIBRARIES}
)

