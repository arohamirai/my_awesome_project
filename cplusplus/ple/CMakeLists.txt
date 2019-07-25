cmake_minimum_required(VERSION 2.8)
project(ple)

set( CMAKE_BUILD_TYPE Debug )
set( CMAKE_CXX_FLAGS "-std=c++11 -g -O3" )
#add_definitions(-std=c++11 -O2 -g -DNDEBUG)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

link_directories(
/home/lf/Desktop/my_awesome_project/cplusplus/ple/lib
/usr/local/MATLAB/R2018a/runtime/glnxa64
/usr/local/MATLAB/R2018a/bin/glnxa64
)

include_directories(
  include
/usr/local/MATLAB/R2018a/extern/include/
)

add_executable(ple src/ple.cpp src/libmatlabPLE.cpp)
target_link_libraries(ple
matlabPLE
mwmclmcrrt 
MatlabCppSharedLib
)

