cmake_minimum_required(VERSION 3.0.2)
project(pluginlib_tutorials_)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pluginlib_tutorials_
#  CATKIN_DEPENDS pluginlib roscpp
#  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
add_library(base src/polygon_base.cpp)
add_library(third_part src/third_part.cpp)
add_library(polygon_plugins_A src/polygon_plugins_A.cpp)
target_link_libraries(polygon_plugins_A third_part base)
add_library(polygon_plugins_B src/polygon_plugins_B.cpp)
target_link_libraries(polygon_plugins_B third_part base)


add_executable(polygon_loader src/polygon_loader.cpp)
target_link_libraries(polygon_loader ${catkin_LIBRARIES} third_part base)

install(TARGETS
    polygon_plugins_A polygon_plugins_B polygon_loader third_part base
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY
    include
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES 
    polygon_plugins.xml 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

