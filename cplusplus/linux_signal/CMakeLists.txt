cmake_minimum_required(VERSION 2.8)
project(linux_signal)

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
#add_definitions(-std=c++11 -O2 -g -DNDEBUG)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

include_directories(
  include
)

add_executable(linux_signal src/linux_signal.cpp)
target_link_libraries(linux_signal
)

