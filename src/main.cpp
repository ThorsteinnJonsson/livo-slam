#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "ros/livo_slam_node.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<livo_slam::LivoSlamNode>());
  rclcpp::shutdown();
  return 0;
}
