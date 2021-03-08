cmake_minimum_required(VERSION 2.8.3)
project(ramon_slam3d)

## Compile as C++14, PCL needs c++14 or above
# -march=native is good in general for release builds of any software and not only math intensive. 
# -mfpmath=sse enables use of XMM registers in floating point instructions
add_compile_options(-std=c++14 -O3 -mfpmath=sse)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  pcl_ros
  eigen_conversions
)

find_package(OpenCV REQUIRED)

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# No se por que se caga con esto el make
# generate_messages(DEPENDENCIES std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ramon_slam3d
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

# add_library(map3d_utils src/map3d_utils.cpp)

add_executable(disparity src/disparity.cpp)
target_link_libraries(disparity ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(pcl_test src/pcl_test.cpp)
target_link_libraries(pcl_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(slam3d src/main.cpp src/slam3d.cpp)
target_link_libraries(slam3d ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(holz_rgbd src/holz_rgbd.cpp)
target_link_libraries(holz_rgbd ${catkin_LIBRARIES} ${PCL_LIBRARIES})
