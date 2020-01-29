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

add_library(brasenham src/brasenham_line_algorithm.cpp)
add_library(contour src/contour_extraction.cpp)
add_library(mle src/mle.cpp)

add_executable(lidar_transform src/lidar_transform.cpp)
target_link_libraries(lidar_transform ${catkin_LIBRARIES} brasenham contour mle)

add_executable(ramon_slam2d src/main.cpp src/ramon_slam2d.cpp)
target_link_libraries(ramon_slam2d ${catkin_LIBRARIES})