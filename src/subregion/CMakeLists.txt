cmake_minimum_required(VERSION 3.0.2)
project(subregion)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(PROJECT_DEPS
  roscpp
  std_msgs
  common_msgs
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  visualization_msgs
  common_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES subregion
#  CATKIN_DEPENDS actionlib geometry_msgs nav_msgs roscpp std_msgs visualization_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/subregion_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
