#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace livo_slam {

class LivoSlamNode : public rclcpp::Node {
 public:
  LivoSlamNode();

 private:
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void LidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

 private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
};

}  // namespace livo_slam
