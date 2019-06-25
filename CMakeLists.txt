cmake_minimum_required(VERSION 2.8.3)
project(ndt_map)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  ndt_gpu
  ndt_cpu
  utils
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
)

add_library(ndt_map src/ndt_map.cpp)
target_link_libraries(ndt_map
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${GTSAM_LIBRARIES}
  gtsam
)

add_executable(map_node src/main.cpp)
target_link_libraries(map_node ndt_map)
