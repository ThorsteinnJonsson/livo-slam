#include "ros/livo_slam_node.h"

#include <cstdint>
#include <format>
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

  if (declare_parameter<bool>("ros.output_trajectory", false)) {
    const auto path = declare_parameter<std::string>(
        "ros.trajectory_path", "/workspace/evaluation/trajectory.tum");
    trajectory_file_.open(path, std::ios::out | std::ios::trunc);
    if (trajectory_file_) {
      RCLCPP_INFO(get_logger(), "writing trajectory to %s", path.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "could not open %s for writing", path.c_str());
    }
  }

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
  config.initial.inv_exposure_std = declare_parameter<double>(
      "initial_state.inv_exposure_std", initial_defaults.inv_exposure_std);

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

  const auto T_imu_lidar =
      declare_parameter<std::vector<double>>("lidar.T_body_lidar", identity);
  if (T_imu_lidar.size() != 16) {
    RCLCPP_ERROR(get_logger(),
                 "lidar.T_body_lidar has %zu entries, expected 16; using "
                 "identity",
                 T_imu_lidar.size());
  }
  config.lidar.T_imu_lidar = ToSe3(T_imu_lidar);

  const LidarParams lidar_defaults;
  config.lidar.min_range =
      declare_parameter<double>("lidar.min_range", lidar_defaults.min_range);
  config.lidar.max_range =
      declare_parameter<double>("lidar.max_range", lidar_defaults.max_range);
  config.lidar.voxel_leaf_size = declare_parameter<double>(
      "lidar.voxel_leaf_size", lidar_defaults.voxel_leaf_size);
  config.lidar.time_shift =
      declare_parameter<double>("lidar.time_shift", lidar_defaults.time_shift);

  const VoxelMapParams map_defaults;
  config.map.voxel_size =
      declare_parameter<double>("map.voxel_size", map_defaults.voxel_size);

  std::vector<int64_t> layer_init_defaults;
  for (std::size_t points : map_defaults.layer_init_points) {
    layer_init_defaults.push_back(static_cast<int64_t>(points));
  }
  const auto layer_init_points = declare_parameter<std::vector<int64_t>>(
      "map.layer_init_points", layer_init_defaults);
  if (layer_init_points.size() != VoxelMapParams::kMaxLayerDepth) {
    RCLCPP_ERROR(get_logger(),
                 "map.layer_init_points has %zu entries, expected %zu; using "
                 "defaults",
                 layer_init_points.size(), VoxelMapParams::kMaxLayerDepth);
  } else {
    for (std::size_t i = 0; i < VoxelMapParams::kMaxLayerDepth; ++i) {
      config.map.layer_init_points[i] =
          static_cast<std::size_t>(layer_init_points[i]);
    }
  }

  config.map.max_points_per_voxel =
      static_cast<std::size_t>(declare_parameter<int64_t>(
          "map.max_points_per_voxel",
          static_cast<int64_t>(map_defaults.max_points_per_voxel)));
  config.map.max_plane_variance = declare_parameter<double>(
      "map.max_plane_variance", map_defaults.max_plane_variance);
  config.map.max_match_distance = declare_parameter<double>(
      "map.max_match_distance", map_defaults.max_match_distance);

  const LidarUpdateParams update_defaults;
  config.lidar_update.max_iterations =
      static_cast<std::size_t>(declare_parameter<int64_t>(
          "lidar_update.max_iterations",
          static_cast<int64_t>(update_defaults.max_iterations)));
  config.lidar_update.point_noise_std = declare_parameter<double>(
      "lidar_update.point_noise_std", update_defaults.point_noise_std);
  config.lidar_update.rotation_epsilon = declare_parameter<double>(
      "lidar_update.rotation_epsilon", update_defaults.rotation_epsilon);
  config.lidar_update.position_epsilon = declare_parameter<double>(
      "lidar_update.position_epsilon", update_defaults.position_epsilon);

  const CameraUpdateParams camera_update_defaults;
  config.camera_update.max_iterations =
      static_cast<std::size_t>(declare_parameter<int64_t>(
          "camera_update.max_iterations",
          static_cast<int64_t>(camera_update_defaults.max_iterations)));
  config.camera_update.patch_size =
      static_cast<std::size_t>(declare_parameter<int64_t>(
          "camera_update.patch_size",
          static_cast<int64_t>(camera_update_defaults.patch_size)));
  config.camera_update.intensity_noise_std =
      declare_parameter<double>("camera_update.intensity_noise_std",
                                camera_update_defaults.intensity_noise_std);
  config.camera_update.outlier_threshold =
      declare_parameter<double>("camera_update.outlier_threshold",
                                camera_update_defaults.outlier_threshold);
  config.camera_update.rotation_epsilon =
      declare_parameter<double>("camera_update.rotation_epsilon",
                                camera_update_defaults.rotation_epsilon);
  config.camera_update.position_epsilon =
      declare_parameter<double>("camera_update.position_epsilon",
                                camera_update_defaults.position_epsilon);

  const CameraParams camera_defaults;
  const auto T_imu_cam =
      declare_parameter<std::vector<double>>("camera.T_body_cam", identity);
  if (T_imu_cam.size() != 16) {
    RCLCPP_ERROR(get_logger(),
                 "camera.T_body_cam has %zu entries, expected 16; using "
                 "identity",
                 T_imu_cam.size());
  }
  config.camera.T_imu_cam = ToSe3(T_imu_cam);

  config.camera.width = static_cast<std::uint32_t>(declare_parameter<int64_t>(
      "camera.width", static_cast<int64_t>(camera_defaults.width)));
  config.camera.height = static_cast<std::uint32_t>(declare_parameter<int64_t>(
      "camera.height", static_cast<int64_t>(camera_defaults.height)));

  config.camera.fx = declare_parameter<double>("camera.fx", camera_defaults.fx);
  config.camera.fy = declare_parameter<double>("camera.fy", camera_defaults.fy);
  config.camera.cx = declare_parameter<double>("camera.cx", camera_defaults.cx);
  config.camera.cy = declare_parameter<double>("camera.cy", camera_defaults.cy);

  config.camera.k1 = declare_parameter<double>("camera.k1", camera_defaults.k1);
  config.camera.k2 = declare_parameter<double>("camera.k2", camera_defaults.k2);
  config.camera.p1 = declare_parameter<double>("camera.p1", camera_defaults.p1);
  config.camera.p2 = declare_parameter<double>("camera.p2", camera_defaults.p2);

  config.camera.time_shift = declare_parameter<double>(
      "camera.time_shift", camera_defaults.time_shift);

  const ProcessNoiseParams noise_defaults;
  config.process_noise.gyro_noise_std = declare_parameter<double>(
      "process_noise.gyro_noise_std", noise_defaults.gyro_noise_std);
  config.process_noise.accel_noise_std = declare_parameter<double>(
      "process_noise.accel_noise_std", noise_defaults.accel_noise_std);
  config.process_noise.gyro_random_walk_std =
      declare_parameter<double>("process_noise.gyro_random_walk_std",
                                noise_defaults.gyro_random_walk_std);
  config.process_noise.accel_random_walk_std =
      declare_parameter<double>("process_noise.accel_random_walk_std",
                                noise_defaults.accel_random_walk_std);
  return config;
}

LivoSlamNode::~LivoSlamNode() { buffer_.Shutdown(); }

void LivoSlamNode::Run() {
  while (const auto group = buffer_.PopMeasureGroup()) {
    const ProcessResult result = pipeline_.Process(*group);
    visualizer_.SetTime(group_index_++, group->image.timestamp);
    visualizer_.LogState(result.estimate.state);
    visualizer_.LogCloud(result.cloud_world);
    visualizer_.LogMap(result.map_planes);
    WriteTrajectory(result.estimate);
  }
}

void LivoSlamNode::WriteTrajectory(const Estimate& estimate) {
  // Groups before IMU initialization carry a default estimate at time zero.
  if (!trajectory_file_ || estimate.timestamp <= 0.0) {
    return;
  }

  const Eigen::Vector3d& p = estimate.state.position;
  const Eigen::Quaterniond q = estimate.state.rotation.unit_quaternion();
  trajectory_file_
      << std::format(
             "{:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f}\n",
             estimate.timestamp, p.x(), p.y(), p.z(), q.x(), q.y(), q.z(),
             q.w())
      << std::flush;
}

void LivoSlamNode::ImuCallback(
    const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  buffer_.PushImu(ToImuSample(*msg));
}

void LivoSlamNode::LidarCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  buffer_.PushPoints(ToPoints(*msg, config_.lidar.time_shift));
}

void LivoSlamNode::CameraCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  buffer_.PushImage(ToImageFrame(*msg, config_.camera.time_shift));
}

}  // namespace livo_slam
