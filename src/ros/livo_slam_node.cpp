#include "ros/livo_slam_node.h"

#include <cstdint>
#include <format>
#include <print>
#include <rclcpp/qos.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "ros/conversions.h"

namespace livo_slam {
LivoSlamNode::LivoSlamNode()
    : Node("livo_slam"),
      config_(ReadConfig()),
      pipeline_(config_),
      visualizer_(config_.viz, config_.camera) {
  imu_topic_ = declare_parameter<std::string>("ros.imu_topic", "/imu/imu");
  lidar_topic_ = declare_parameter<std::string>("ros.lidar_topic",
                                                "/os1_cloud_node1/points");
  camera_topic_ =
      declare_parameter<std::string>("ros.camera_topic", "/left/image_raw");
  camera_compressed_ = declare_parameter<bool>("ros.camera_compressed", false);
  bag_path_ = declare_parameter<std::string>("ros.bag_path", "");
  imu_accel_scale_ = declare_parameter<double>("imu.accel_scale", 1.0);

  const auto& imu_topic = imu_topic_;
  const auto& lidar_topic = lidar_topic_;
  const auto& camera_topic = camera_topic_;

  const bool live = bag_path_.empty();

  if (live) {
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

    const auto camera_qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
    if (camera_compressed_) {
      compressed_camera_sub_ =
          create_subscription<sensor_msgs::msg::CompressedImage>(
              camera_topic, camera_qos,
              [this](const sensor_msgs::msg::CompressedImage::ConstSharedPtr&
                         msg) { CompressedCameraCallback(msg); });
    } else {
      camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
          camera_topic, camera_qos,
          [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
            CameraCallback(msg);
          });
    }
  }

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

  if (config_.viz.enabled && config_.viz.color_cloud) {
    color_cloud_.emplace(config_.viz);
  }

  if (live) {
    worker_ = std::jthread([this] { Run(); });
    RCLCPP_INFO(get_logger(), "subscribed to %s, %s, %s", imu_topic.c_str(),
                lidar_topic.c_str(), camera_topic.c_str());
  } else {
    worker_ = std::jthread([this] {
      ReplayBag();
      rclcpp::shutdown();
    });
    RCLCPP_INFO(get_logger(), "replaying bag %s", bag_path_.c_str());
  }
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
  config.viz.color_cloud =
      declare_parameter<bool>("viz.color_cloud", viz_defaults.color_cloud);
  config.viz.color_cloud_voxel_size = declare_parameter<double>(
      "viz.color_cloud_voxel_size", viz_defaults.color_cloud_voxel_size);
  config.viz.color_cloud_max_range = declare_parameter<double>(
      "viz.color_cloud_max_range", viz_defaults.color_cloud_max_range);
  config.viz.color_cloud_max_contrast = declare_parameter<double>(
      "viz.color_cloud_max_contrast", viz_defaults.color_cloud_max_contrast);
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

  const VisualMapParams visual_map_defaults;
  config.visual_map.grid_size =
      static_cast<std::size_t>(declare_parameter<int64_t>(
          "visual_map.grid_size",
          static_cast<int64_t>(visual_map_defaults.grid_size)));
  config.visual_map.min_corner_score = declare_parameter<double>(
      "visual_map.min_corner_score", visual_map_defaults.min_corner_score);
  config.visual_map.min_seconds_between_patches = declare_parameter<double>(
      "visual_map.min_seconds_between_patches",
      visual_map_defaults.min_seconds_between_patches);
  config.visual_map.min_patch_pixel_distance =
      declare_parameter<double>("visual_map.min_patch_pixel_distance",
                                visual_map_defaults.min_patch_pixel_distance);

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
  config.camera_update.max_view_angle_deg =
      declare_parameter<double>("camera_update.max_view_angle_deg",
                                camera_update_defaults.max_view_angle_deg);
  config.camera_update.max_depth_discontinuity =
      declare_parameter<double>("camera_update.max_depth_discontinuity",
                                camera_update_defaults.max_depth_discontinuity);
  config.camera_update.selection_grid_size =
      static_cast<std::size_t>(declare_parameter<int64_t>(
          "camera_update.selection_grid_size",
          static_cast<int64_t>(camera_update_defaults.selection_grid_size)));
  config.camera_update.estimate_exposure =
      declare_parameter<bool>("camera_update.estimate_exposure",
                              camera_update_defaults.estimate_exposure);
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

  const auto model = declare_parameter<std::string>("camera.distortion_model");
  if (model == "radtan") {
    RadTanParams radtan;
    radtan.k1 = declare_parameter<double>("camera.radtan.k1");
    radtan.k2 = declare_parameter<double>("camera.radtan.k2");
    radtan.p1 = declare_parameter<double>("camera.radtan.p1");
    radtan.p2 = declare_parameter<double>("camera.radtan.p2");
    config.camera.distortion = radtan;
  } else if (model == "equidistant") {
    EquidistantParams equidistant;
    equidistant.k1 = declare_parameter<double>("camera.equidistant.k1");
    equidistant.k2 = declare_parameter<double>("camera.equidistant.k2");
    equidistant.k3 = declare_parameter<double>("camera.equidistant.k3");
    equidistant.k4 = declare_parameter<double>("camera.equidistant.k4");
    config.camera.distortion = equidistant;
  } else {
    throw std::runtime_error("camera.distortion_model is '" + model +
                             "', expected 'radtan' or 'equidistant'");
  }

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
  config.process_noise.inv_exposure_random_walk_std =
      declare_parameter<double>("process_noise.inv_exposure_random_walk_std",
                                noise_defaults.inv_exposure_random_walk_std);
  return config;
}

LivoSlamNode::~LivoSlamNode() { buffer_.Shutdown(); }

void LivoSlamNode::ProcessGroup(const MeasureGroup& group) {
  const ProcessResult result = pipeline_.Process(group);
  visualizer_.SetTime(group_index_++, group.image.timestamp);
  visualizer_.LogState(result.estimate.state);
  visualizer_.LogCloud(result.cloud_world);
  visualizer_.LogMap(result.map_planes);
  visualizer_.LogCamera(result.image, result.visual_observations);
  visualizer_.LogVisualPoints(result.visual_observations);

  if (color_cloud_) {
    const Sophus::SE3d T_world_imu(result.estimate.state.rotation,
                                   result.estimate.state.position);
    color_cloud_->Add(result.cloud_imu.points, T_world_imu, config_.camera,
                      result.color_image, result.estimate.state.inv_exposure);
    if (group_index_ % 10 == 0) {
      visualizer_.LogColorCloud(color_cloud_->TakeNew());
    }
  }

  WriteTrajectory(result.estimate);
}

void LivoSlamNode::Run() {
  while (const auto group = buffer_.PopMeasureGroup()) {
    ProcessGroup(*group);
  }
}

// Reads the bag on this thread and processes every group it can form before
// reading on, so nothing is ever dropped for being behind.
void LivoSlamNode::ReplayBag() {
  rosbag2_cpp::Reader reader;
  try {
    reader.open(bag_path_);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "could not open bag %s: %s", bag_path_.c_str(),
                 error.what());
    return;
  }

  // Livox bags carry CustomMsg, which we decode ourselves.
  bool livox_lidar = false;
  for (const auto& topic : reader.get_all_topics_and_types()) {
    if (topic.name == lidar_topic_) {
      livox_lidar = topic.type.find("CustomMsg") != std::string::npos;
      RCLCPP_INFO(get_logger(), "lidar topic %s is %s", topic.name.c_str(),
                  topic.type.c_str());
    }
  }

  const rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;
  const rclcpp::Serialization<sensor_msgs::msg::PointCloud2>
      lidar_serialization;
  const rclcpp::Serialization<sensor_msgs::msg::Image> image_serialization;
  const rclcpp::Serialization<sensor_msgs::msg::CompressedImage>
      compressed_serialization;

  while (reader.has_next() && rclcpp::ok()) {
    const auto bag_message = reader.read_next();
    const rclcpp::SerializedMessage serialized(*bag_message->serialized_data);

    if (bag_message->topic_name == imu_topic_) {
      sensor_msgs::msg::Imu message;
      imu_serialization.deserialize_message(&serialized, &message);
      buffer_.PushImu(ToImuSample(message, imu_accel_scale_));
    } else if (bag_message->topic_name == lidar_topic_) {
      if (livox_lidar) {
        buffer_.PushPoints(
            ToPointsFromLivox(serialized, config_.lidar.time_shift));
      } else {
        sensor_msgs::msg::PointCloud2 message;
        lidar_serialization.deserialize_message(&serialized, &message);
        buffer_.PushPoints(ToPoints(message, config_.lidar.time_shift));
      }
    } else if (bag_message->topic_name == camera_topic_) {
      if (camera_compressed_) {
        sensor_msgs::msg::CompressedImage message;
        compressed_serialization.deserialize_message(&serialized, &message);
        buffer_.PushImage(ToImageFrame(message, config_.camera.time_shift,
                                       color_cloud_.has_value()));
      } else {
        sensor_msgs::msg::Image message;
        image_serialization.deserialize_message(&serialized, &message);
        buffer_.PushImage(ToImageFrame(message, config_.camera.time_shift,
                                       color_cloud_.has_value()));
      }
    } else {
      continue;
    }

    while (const auto group = buffer_.TryPopMeasureGroup()) {
      ProcessGroup(*group);
    }
  }

  while (const auto group = buffer_.TryPopMeasureGroup()) {
    ProcessGroup(*group);
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
  buffer_.PushImu(ToImuSample(*msg, imu_accel_scale_));
}

void LivoSlamNode::LidarCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  buffer_.PushPoints(ToPoints(*msg, config_.lidar.time_shift));
}

void LivoSlamNode::CameraCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  buffer_.PushImage(
      ToImageFrame(*msg, config_.camera.time_shift, color_cloud_.has_value()));
}

void LivoSlamNode::CompressedCameraCallback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg) {
  buffer_.PushImage(
      ToImageFrame(*msg, config_.camera.time_shift, color_cloud_.has_value()));
}

}  // namespace livo_slam
