cmake_minimum_required(VERSION 3.0.2)
project(lane_detection)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  std_msgs
  sensor_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lane_detection
#  CATKIN_DEPENDS cv_bridge roscpp std_msgs
#  DEPENDS system_lib
)

find_package (OpenCV 4 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_subdirectory(src)
