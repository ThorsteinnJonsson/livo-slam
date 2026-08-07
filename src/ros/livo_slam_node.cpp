#include "ros/livo_slam_node.h"

#include <cstdint>
#include <rclcpp/qos.hpp>
#include <string>

#include "ros/conversions.h"

namespace livo_slam {

LivoSlamNode::LivoSlamNode() : Node("livo_slam"), pipeline_(ReadConfig()) {
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

  worker_ = std::jthread([this] { Run(); });

  RCLCPP_INFO(get_logger(), "subscribed to %s, %s, %s", imu_topic.c_str(),
              lidar_topic.c_str(), camera_topic.c_str());
}

PipelineConfig LivoSlamNode::ReadConfig() {
  const InitParams defaults;

  PipelineConfig config;
  config.init.window_size = static_cast<std::size_t>(declare_parameter<int64_t>(
      "imu_init.window_size", static_cast<int64_t>(defaults.window_size)));
  config.init.gravity_magnitude = declare_parameter<double>(
      "imu_init.gravity_magnitude", defaults.gravity_magnitude);
  config.init.max_accel_var = declare_parameter<double>(
      "imu_init.max_accel_var", defaults.max_accel_var);
  config.init.max_gyro_var =
      declare_parameter<double>("imu_init.max_gyro_var", defaults.max_gyro_var);
  return config;
}

LivoSlamNode::~LivoSlamNode() { buffer_.Shutdown(); }

void LivoSlamNode::Run() {
  while (const auto group = buffer_.PopMeasureGroup()) {
    pipeline_.Process(*group);
  }
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
