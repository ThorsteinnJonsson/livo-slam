#pragma once

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "core/config/camera_params.h"
#include "core/types/cloud.h"
#include "core/types/color_point.h"

namespace livo_slam {

// Lidar points that landed in the camera image, kept one per voxel. A voxel
// collects a few observations and takes the median, so a roof edge that
// occasionally samples the sky behind it still comes out the roof colour.
// Visualization only.
class ColorCloud {
 public:
  explicit ColorCloud(double voxel_size) : voxel_size_(voxel_size) {}

  void Add(const std::vector<Point>& points_imu,
           const Sophus::SE3d& T_world_imu, const CameraParams& camera_params,
           const cv::Mat& color_image);

  // Voxels the camera has moved past, so each point is only ever sent once.
  // Pass true at the end of a run to take the ones still collecting.
  std::vector<ColorPoint> TakeNew(bool flush_all = false);

 private:
  static constexpr std::size_t kMaxSamples = 5;
  // A sample sitting on a strong edge is probably on the wrong side of it.
  static constexpr int kContrastRadius = 2;
  static constexpr double kMaxLocalContrast = 40.0;
  // How long a voxel keeps collecting after it was last seen.
  static constexpr std::int64_t kFlushAge = 20;

  struct VoxelIndex {
    std::int64_t x = 0;
    std::int64_t y = 0;
    std::int64_t z = 0;
    bool operator==(const VoxelIndex& other) const = default;
  };

  struct VoxelHash {
    std::size_t operator()(const VoxelIndex& index) const {
      return static_cast<std::size_t>(index.x * 73856093 ^ index.y * 19349663 ^
                                      index.z * 83492791);
    }
  };

  struct Pending {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    std::array<cv::Vec3b, kMaxSamples> samples{};
    std::size_t count = 0;
    std::int64_t last_seen = 0;
  };

  // Gray world white balance over the pixels we actually sample. The sky is
  // much less green than the surfaces and has no lidar returns, so averaging
  // the whole image would under correct.
  Eigen::Vector3d WhiteBalanceGains() const;
  ColorPoint Resolve(const Pending& pending,
                     const Eigen::Vector3d& gains) const;

  double voxel_size_;
  std::int64_t frame_ = 0;
  Eigen::Vector3d channel_sum_ = Eigen::Vector3d::Zero();
  std::int64_t sample_count_ = 0;
  std::unordered_map<VoxelIndex, Pending, VoxelHash> pending_;
  std::unordered_set<VoxelIndex, VoxelHash> done_;
};

}  // namespace livo_slam
