add_subdirectory(core)
add_subdirectory(ros)

add_executable(livo_slam main.cpp)
target_link_libraries(livo_slam PRIVATE livo_slam_ros_lib livo_slam_warnings)
