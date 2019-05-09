cmake_minimum_required(VERSION 2.8)
project(visp_example)

#set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
add_definitions(-std=c++11 -O2 -g -DNDEBUG)


find_package(Boost REQUIRED)
find_package(VISP REQUIRED)
find_package( OpenCV REQUIRED )

include_directories(
  include
${Boost_INCLUDE_DIR}
"/usr/include/eigen3" 
${OpenCV_INCLUDE_DIRS}
${VISP_INCLUDE_DIRS}
)

add_executable(visp_example src/visp_example.cpp)
target_link_libraries(visp_example
${Boost_LIBRARIES}
${OpenCV_LIBS}
${VISP_LIBRARIES}
)

