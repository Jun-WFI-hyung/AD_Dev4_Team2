add_library(core_lib
            system.cpp
            system.h
            tracking_module.cpp
            tracking_module.h
)

target_link_libraries(core_lib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )
