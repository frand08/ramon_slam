cmake_minimum_required(VERSION 2.8.3)
project(ramon_slam2d)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
)

# No se por que se caga con esto el make
# generate_messages(DEPENDENCIES std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ramon_slam2d
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_library(map_utils src/map_utils.cpp)

add_executable(slam2d src/main.cpp src/slam2d.cpp)
target_link_libraries(slam2d ${catkin_LIBRARIES} map_utils)