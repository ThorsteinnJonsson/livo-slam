#include "ros/livo_slam_node.h"

#include <rclcpp/qos.hpp>
#include <string>
#include <thread>

#include "ros/conversions.h"

namespace livo_slam {

LivoSlamNode::LivoSlamNode() : Node("livo_slam") {
  const auto imu_topic =
      declare_parameter<std::string>("imu_topic", "/imu/imu");
  const auto lidar_topic =
      declare_parameter<std::string>("lidar_topic", "/os1_cloud_node1/points");
  const auto camera_topic =
      declare_parameter<std::string>("camera_topic", "/left/image_raw");

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS().keep_last(2000),
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
        ImuCallback(msg);
      });

  lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
        LidarCallback(msg);
      });

  camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
      camera_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        CameraCallback(msg);
      });

  std::thread([this] {
    while (auto g = buffer_.PopMeasureGroup()) {
      RCLCPP_INFO(get_logger(),
                  "group pts=%zu imu=%zu start=%.4f end=%.4f span=%.4f",
                  g->cloud.points.size(), g->imu.size(), g->cloud.start_time,
                  g->cloud.end_time, g->cloud.end_time - g->cloud.start_time);
    }
  }).detach();

  RCLCPP_INFO(get_logger(), "subscribed to %s, %s, %s", imu_topic.c_str(),
              lidar_topic.c_str(), camera_topic.c_str());
}

void LivoSlamNode::ImuCallback(
    const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  buffer_.PushImu(ToImuSample(*msg));
}

void LivoSlamNode::LidarCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  buffer_.PushPoints(ToPoints(*msg));
}

void LivoSlamNode::CameraCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  buffer_.PushImage(ToImageFrame(*msg));
}

}  // namespace livo_slam
