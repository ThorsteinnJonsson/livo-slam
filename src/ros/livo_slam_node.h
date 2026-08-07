#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>

#include "core/config/pipeline_config.h"
#include "core/pipeline/livo_pipeline.h"
#include "core/pipeline/measurement_buffer.h"

namespace livo_slam {

class LivoSlamNode : public rclcpp::Node {
 public:
  LivoSlamNode();
  ~LivoSlamNode() override;

  LivoSlamNode(const LivoSlamNode&) = delete;
  LivoSlamNode& operator=(const LivoSlamNode&) = delete;
  LivoSlamNode(LivoSlamNode&&) = delete;
  LivoSlamNode& operator=(LivoSlamNode&&) = delete;

 private:
  void ImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
  void LidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);
  void CameraCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

  void Run();

  PipelineConfig ReadConfig();

 private:
  MeasurementBuffer buffer_;
  LivoPipeline pipeline_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;

  std::jthread worker_;
};

}  // namespace livo_slam
