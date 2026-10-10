#include "viz/color_cloud.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

#include "core/camera/projection.h"

namespace livo_slam {
namespace {

double Luminance(const cv::Vec3b& bgr) {
  return 0.114 * bgr[0] + 0.587 * bgr[1] + 0.299 * bgr[2];
}

// Blend the four pixels around the projection. Sampling the nearest one
// quantizes every colour to a whole pixel, which is what smears fine detail.
Eigen::Vector3d SamplePixel(const cv::Mat& image,
                            const Eigen::Vector2d& pixel) {
  const int column = static_cast<int>(std::floor(pixel.x()));
  const int row = static_cast<int>(std::floor(pixel.y()));
  const double du = pixel.x() - column;
  const double dv = pixel.y() - row;

  const auto& top_left = image.at<cv::Vec3b>(row, column);
  const auto& top_right = image.at<cv::Vec3b>(row, column + 1);
  const auto& bottom_left = image.at<cv::Vec3b>(row + 1, column);
  const auto& bottom_right = image.at<cv::Vec3b>(row + 1, column + 1);

  Eigen::Vector3d blended = Eigen::Vector3d::Zero();
  for (int channel = 0; channel < 3; ++channel) {
    blended[channel] = (1.0 - du) * (1.0 - dv) * top_left[channel] +
                       du * (1.0 - dv) * top_right[channel] +
                       (1.0 - du) * dv * bottom_left[channel] +
                       du * dv * bottom_right[channel];
  }
  return blended;
}

double LocalContrast(const cv::Mat& image, const Eigen::Vector2d& pixel,
                     int radius) {
  const int center_column = static_cast<int>(pixel.x());
  const int center_row = static_cast<int>(pixel.y());
  double lowest = 255.0;
  double highest = 0.0;

  for (int dv = -radius; dv <= radius; ++dv) {
    const int row = center_row + dv;
    if (row < 0 || row >= image.rows) continue;
    for (int du = -radius; du <= radius; ++du) {
      const int column = center_column + du;
      if (column < 0 || column >= image.cols) continue;

      const double luminance = Luminance(image.at<cv::Vec3b>(row, column));
      lowest = std::min(lowest, luminance);
      highest = std::max(highest, luminance);
    }
  }
  return highest - lowest;
}

}  // namespace

void ColorCloud::Add(const std::vector<Point>& points_imu,
                     const Sophus::SE3d& T_world_imu,
                     const CameraParams& camera_params,
                     const cv::Mat& color_image, double inv_exposure) {
  if (color_image.empty()) return;

  // The estimated exposure drifts by a factor of ten over a run, so the same
  // wall comes out a different brightness depending on when it was seen.
  // Scaling by it puts every frame back on one scale.
  exposure_sum_ += inv_exposure;
  ++frame_count_;
  const double reference = exposure_sum_ / static_cast<double>(frame_count_);
  const double exposure_scale =
      reference > 0.0 && inv_exposure > 0.0 ? inv_exposure / reference : 1.0;

  const Sophus::SE3d T_cam_imu = camera_params.T_imu_cam.inverse();

  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> sampled;
  for (const Point& point : points_imu) {
    const Eigen::Vector3d point_cam = T_cam_imu * point.position;
    if (point_cam.z() <= 0.0) continue;
    // Far points are smaller than a pixel, so they add bulk but no detail.
    if (max_range_ > 0.0 && point_cam.norm() > max_range_) continue;

    const std::optional<Eigen::Vector2d> pixel =
        Project(camera_params, point_cam);
    if (!pixel || !InFrame(color_image, *pixel, 2)) continue;
    if (max_contrast_ > 0.0 &&
        LocalContrast(color_image, *pixel, 2) > max_contrast_) {
      continue;
    }

    const Eigen::Vector3d position = T_world_imu * point.position;
    const VoxelIndex index{
        static_cast<std::int64_t>(std::floor(position.x() / voxel_size_)),
        static_cast<std::int64_t>(std::floor(position.y() / voxel_size_)),
        static_cast<std::int64_t>(std::floor(position.z() / voxel_size_))};
    if (!occupied_.insert(index).second) continue;

    sampled.emplace_back(position,
                         SamplePixel(color_image, *pixel) * exposure_scale);
  }

  for (const auto& [position, bgr] : sampled) {
    channel_sum_ += bgr;
  }
  sample_count_ += static_cast<std::int64_t>(sampled.size());
  const Eigen::Vector3d gains = WhiteBalanceGains();

  const auto balance = [&gains](double value, int channel) {
    return static_cast<std::uint8_t>(
        std::clamp(value * gains[channel], 0.0, 255.0));
  };
  for (const auto& [position, bgr] : sampled) {
    new_points_.push_back(
        {position, balance(bgr[2], 2), balance(bgr[1], 1), balance(bgr[0], 0)});
  }
}

std::vector<ColorPoint> ColorCloud::TakeNew() {
  return std::exchange(new_points_, {});
}

Eigen::Vector3d ColorCloud::WhiteBalanceGains() const {
  if (sample_count_ == 0) return Eigen::Vector3d::Ones();
  const Eigen::Vector3d average =
      channel_sum_ / static_cast<double>(sample_count_);
  if (average.minCoeff() <= 0.0) return Eigen::Vector3d::Ones();
  return Eigen::Vector3d::Constant(average.mean()).cwiseQuotient(average);
}

}  // namespace livo_slam
