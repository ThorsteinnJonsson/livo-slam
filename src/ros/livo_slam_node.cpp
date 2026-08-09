#include "ros/livo_slam_node.h"

#include <cstdint>
#include <rclcpp/qos.hpp>
#include <string>
#include <vector>

#include "ros/conversions.h"

namespace livo_slam {
LivoSlamNode::LivoSlamNode()
    : Node("livo_slam"),
      config_(ReadConfig()),
      pipeline_(config_),
      visualizer_(config_.viz) {
  const auto imu_topic =
      declare_parameter<std::string>("imu_topic", "/imu/imu");
  const auto lidar_topic =
      declare_parameter<std::string>("lidar_topic", "/os1_cloud_node1/points");
  const auto camera_topic =
      declare_parameter<std::string>("camera_topic", "/left/image_raw");

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::QoS(rclcpp::KeepLast(2000)).reliable(),
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
        ImuCallback(msg);
      });

  lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic, rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
        LidarCallback(msg);
      });

  camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
      camera_topic, rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        CameraCallback(msg);
      });

  worker_ = std::jthread([this] { Run(); });

  RCLCPP_INFO(get_logger(), "subscribed to %s, %s, %s", imu_topic.c_str(),
              lidar_topic.c_str(), camera_topic.c_str());
}

PipelineConfig LivoSlamNode::ReadConfig() {
  const ImuInitParams defaults;

  PipelineConfig config;
  config.init.window_size = static_cast<std::size_t>(declare_parameter<int64_t>(
      "imu_init.window_size", static_cast<int64_t>(defaults.window_size)));
  config.init.gravity_magnitude = declare_parameter<double>(
      "imu_init.gravity_magnitude", defaults.gravity_magnitude);
  config.init.max_accel_var = declare_parameter<double>(
      "imu_init.max_accel_var", defaults.max_accel_var);
  config.init.max_gyro_var =
      declare_parameter<double>("imu_init.max_gyro_var", defaults.max_gyro_var);

  const InitialStateParams initial_defaults;
  config.initial.rotation_std = declare_parameter<double>(
      "initial_state.rotation_std", initial_defaults.rotation_std);
  config.initial.position_std = declare_parameter<double>(
      "initial_state.position_std", initial_defaults.position_std);
  config.initial.velocity_std = declare_parameter<double>(
      "initial_state.velocity_std", initial_defaults.velocity_std);
  config.initial.gyro_bias_std = declare_parameter<double>(
      "initial_state.gyro_bias_std", initial_defaults.gyro_bias_std);
  config.initial.accel_bias_std = declare_parameter<double>(
      "initial_state.accel_bias_std", initial_defaults.accel_bias_std);
  config.initial.gravity_std = declare_parameter<double>(
      "initial_state.gravity_std", initial_defaults.gravity_std);

  const VizParams viz_defaults;
  config.viz.enabled =
      declare_parameter<bool>("viz.enabled", viz_defaults.enabled);
  config.viz.grpc_url =
      declare_parameter<std::string>("viz.grpc_url", viz_defaults.grpc_url);

  // clang-format off
  const std::vector<double> identity = {1.0, 0.0, 0.0, 0.0, 
                                        0.0, 1.0, 0.0, 0.0,
                                        0.0, 0.0, 1.0, 0.0,
                                        0.0, 0.0, 0.0, 1.0};
  // clang-format on

  const auto imu_from_lidar =
      declare_parameter<std::vector<double>>("lidar.T_body_lidar", identity);
  if (imu_from_lidar.size() != 16) {
    RCLCPP_ERROR(get_logger(),
                 "lidar.T_body_lidar has %zu entries, expected 16; using "
                 "identity",
                 imu_from_lidar.size());
  }
  config.lidar.imu_from_lidar = ToSe3(imu_from_lidar);

  const LidarParams lidar_defaults;
  config.lidar.min_range =
      declare_parameter<double>("lidar.min_range", lidar_defaults.min_range);
  config.lidar.max_range =
      declare_parameter<double>("lidar.max_range", lidar_defaults.max_range);

  const ImuNoiseParams noise_defaults;
  config.imu_noise.gyro_noise_std = declare_parameter<double>(
      "imu_noise.gyro_noise_std", noise_defaults.gyro_noise_std);
  config.imu_noise.accel_noise_std = declare_parameter<double>(
      "imu_noise.accel_noise_std", noise_defaults.accel_noise_std);
  config.imu_noise.gyro_random_walk_std = declare_parameter<double>(
      "imu_noise.gyro_random_walk_std", noise_defaults.gyro_random_walk_std);
  config.imu_noise.accel_random_walk_std = declare_parameter<double>(
      "imu_noise.accel_random_walk_std", noise_defaults.accel_random_walk_std);
  return config;
}

LivoSlamNode::~LivoSlamNode() { buffer_.Shutdown(); }

void LivoSlamNode::Run() {
  while (const auto group = buffer_.PopMeasureGroup()) {
    const ProcessResult result = pipeline_.Process(*group);
    visualizer_.SetTime(group_index_++, group->image.timestamp);
    visualizer_.LogState(result.estimate.state);
    visualizer_.LogCloud(result.cloud_world);
    visualizer_.LogDeskewedCloud(result.cloud_deskewed_world);
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
