#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <opencv2/core.hpp>
#include <sophus/se3.hpp>
#include <unordered_set>
#include <vector>

#include "core/config/camera_params.h"
#include "core/config/viz_params.h"
#include "core/types/cloud.h"
#include "core/types/color_point.h"

namespace livo_slam {

// Lidar points that landed in the camera image, kept one per voxel and
// coloured the first time they are seen. Visualization only.
class ColorCloud {
 public:
  explicit ColorCloud(const VizParams& params)
      : voxel_size_(params.color_cloud_voxel_size),
        max_range_(params.color_cloud_max_range),
        max_contrast_(params.color_cloud_max_contrast) {}

  void Add(const std::vector<Point>& points_imu,
           const Sophus::SE3d& T_world_imu, const CameraParams& camera_params,
           const cv::Mat& color_image, double inv_exposure);

  // Points added since the last call, so each one is only ever sent once.
  std::vector<ColorPoint> TakeNew();

 private:
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

  // Gray world white balance over the pixels we actually sample. The sky is
  // much less green than the surfaces and has no lidar returns, so averaging
  // the whole image would under correct.
  Eigen::Vector3d WhiteBalanceGains() const;

  double voxel_size_;
  double max_range_;
  double max_contrast_;
  Eigen::Vector3d channel_sum_ = Eigen::Vector3d::Zero();
  double exposure_sum_ = 0.0;
  std::int64_t frame_count_ = 0;
  std::int64_t sample_count_ = 0;
  std::unordered_set<VoxelIndex, VoxelHash> occupied_;
  std::vector<ColorPoint> new_points_;
};

}  // namespace livo_slam
