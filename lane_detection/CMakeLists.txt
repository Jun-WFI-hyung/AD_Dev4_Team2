cmake_minimum_required(VERSION 3.0.2)
project(simul)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  roscpp
  std_msgs
)

find_package (OpenCV 4 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES simul
  CATKIN_DEPENDS cv_bridge geometry_msgs roscpp std_msgs
  DEPENDS opencv
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(simul_img 
  src/main.cpp 
  include/${PROJECT_NAME}/func_Simulator.cpp
  include/${PROJECT_NAME}/func_SimulCV.cpp
  include/${PROJECT_NAME}/func_LD.cpp
)

target_link_libraries(simul_img ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})