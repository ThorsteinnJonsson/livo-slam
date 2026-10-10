#pragma once

#include <cstdint>
#include <fstream>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <thread>

#include "core/config/pipeline_config.h"
#include "core/pipeline/livo_pipeline.h"
#include "core/pipeline/measurement_buffer.h"
#include "viz/color_cloud.h"
#include "viz/visualizer.h"

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
  void CompressedCameraCallback(
      const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg);

  void Run();
  void ReplayBag();
  void ProcessGroup(const MeasureGroup& group);
  void WriteTrajectory(const Estimate& estimate);

  PipelineConfig ReadConfig();

 private:
  PipelineConfig config_;
  MeasurementBuffer buffer_;
  LivoPipeline pipeline_;
  Visualizer visualizer_;
  std::optional<ColorCloud> color_cloud_;
  std::int64_t group_index_ = 0;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
      compressed_camera_sub_;

  std::string imu_topic_;
  std::string lidar_topic_;
  std::string camera_topic_;
  std::string bag_path_;
  bool camera_compressed_ = false;
  double imu_accel_scale_ = 1.0;

  std::ofstream trajectory_file_;

  std::jthread worker_;
};

}  // namespace livo_slam
