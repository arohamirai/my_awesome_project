cmake_minimum_required(VERSION 2.8)
project(g2o_example)

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
#add_definitions(-std=c++11 -O2 -g -DNDEBUG)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(Boost REQUIRED)
Find_Package(Eigen3 REQUIRED)
find_package( OpenCV REQUIRED )
# 官方没有提供.cmake文件，https://github.com/thewtex/jsoncpp-config-cmake 仓库提供了一个.cmake文件
#find_package(jsoncpp REQUIRED)
Find_Package(G2O REQUIRED)
Find_Package(Cholmod REQUIRED)

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear
    g2o_types_icp g2o_types_slam2d g2o_types_sba g2o_types_slam3d g2o_core g2o_interface
    g2o_solver_csparse g2o_solver_structure_only g2o_csparse_extension g2o_opengl_helper g2o_solver_dense
    g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )

include_directories(
  include
${Boost_INCLUDE_DIR}
${EIGEN3_INCLUDE_DIR}
${OpenCV_INCLUDE_DIRS}
${G2O_INCLUDE_DIRS}
)

add_executable(g2o_example src/g2o_example.cpp)
target_link_libraries(g2o_example
${Boost_LIBRARIES}
${OpenCV_LIBS}
${G2O_LIBS}
${CHOLMOD_LIBRARIES}
)

