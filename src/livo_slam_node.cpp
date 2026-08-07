#include "livo_slam_node.h"

#include <functional>
#include <rclcpp/qos.hpp>

namespace livo_slam {

LivoSlamNode::LivoSlamNode() : Node("livo_slam") {
  RCLCPP_INFO(get_logger(), "livo_slam node started");

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/imu", rclcpp::SensorDataQoS{},
      std::bind(&LivoSlamNode::ImuCallback, this, std::placeholders::_1));

  lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/os1_cloud_node1/points", rclcpp::SensorDataQoS{},
      std::bind(&LivoSlamNode::LidarCallback, this, std::placeholders::_1));

  camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/left/image_raw", rclcpp::SensorDataQoS{},
      std::bind(&LivoSlamNode::CameraCallback, this, std::placeholders::_1));
}

void LivoSlamNode::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  RCLCPP_INFO(get_logger(), "Got IMU: %d", msg->header.stamp.sec);
}
void LivoSlamNode::LidarCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  RCLCPP_INFO(get_logger(), "Got pointcloud: %zu", msg->data.size());
}

void LivoSlamNode::CameraCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  RCLCPP_INFO(get_logger(), "Got image: %zu", msg->data.size());
}

}  // namespace livo_slam
