
add_executable(${PROJECT_NAME} run.cpp)
target_link_libraries(${PROJECT_NAME} core_lib ros_lib draw_lib  data_lib yaml-cpp)

add_subdirectory(core)
add_subdirectory(data)
add_subdirectory(ros)
add_subdirectory(viewer)