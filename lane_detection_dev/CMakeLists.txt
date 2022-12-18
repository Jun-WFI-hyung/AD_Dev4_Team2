cmake_minimum_required(VERSION 3.0.2)
project(lane_detection)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  roscpp
  std_msgs
)

find_package (OpenCV 4 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES lane_detection
  CATKIN_DEPENDS cv_bridge geometry_msgs roscpp std_msgs
  DEPENDS opencv
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(lane_detection 
  src/main.cpp 
#  include/${PROJECT_NAME}/func.cpp
)

target_link_libraries(lane_detection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
