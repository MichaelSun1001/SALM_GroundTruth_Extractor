cmake_minimum_required(VERSION 3.0.2)
project(groundtruth_tum_extractor)

add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rosbag
  roscpp
  sensor_msgs
)

list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

catkin_package(
  CATKIN_DEPENDS geometry_msgs rosbag roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

add_executable(navsat_fix_to_tum
  src/navsat_fix_to_tum.cpp
)

target_link_libraries(navsat_fix_to_tum
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

add_executable(pose_stamped_to_tum
  src/pose_stamped_to_tum.cpp
)

target_link_libraries(pose_stamped_to_tum
  ${catkin_LIBRARIES}
)
