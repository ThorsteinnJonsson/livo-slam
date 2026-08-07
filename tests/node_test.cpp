#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "ros/livo_slam_node.h"

TEST(LivoSlamNode, ConstructsWithExpectedName) {
  rclcpp::init(0, nullptr);
  const auto node = std::make_shared<livo_slam::LivoSlamNode>();
  EXPECT_STREQ(node->get_name(), "livo_slam");
  rclcpp::shutdown();
}
