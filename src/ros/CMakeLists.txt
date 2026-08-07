add_library(livo_slam_ros_lib STATIC livo_slam_node.cpp)

target_include_directories(livo_slam_ros_lib PUBLIC ${PROJECT_SOURCE_DIR}/src)

target_link_libraries(
  livo_slam_ros_lib
  PUBLIC rclcpp::rclcpp ${sensor_msgs_TARGETS} Eigen3::Eigen Sophus::Sophus
  PRIVATE livo_slam_warnings)
