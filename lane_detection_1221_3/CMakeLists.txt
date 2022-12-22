cmake_minimum_required(VERSION 3.0.2)
project(lane_detection_1221_3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  xycar_msgs
)

# find_package (OpenCV 4 REQUIRED)
find_package(OpenCV 4.5.5 REQUIRED PATHS ~/OpenCV4/install/lib/cmake/opencv4)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lane_detection_1221_3
  CATKIN_DEPENDS cv_bridge roscpp std_msgs xycar_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(lane_detection_1221_3 
  src/main.cpp
#  include/${PROJECT_NAME}/func.cpp
)

target_link_libraries(lane_detection_1221_3 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
