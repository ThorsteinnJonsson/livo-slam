#include "viz/color_cloud.h"

#include <algorithm>
#include <cmath>
#include <optional>

#include "core/camera/projection.h"

namespace livo_slam {
namespace {

double Luminance(const cv::Vec3b& bgr) {
  return 0.114 * bgr[0] + 0.587 * bgr[1] + 0.299 * bgr[2];
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
                     const cv::Mat& color_image) {
  if (color_image.empty()) return;

  ++frame_;
  const Sophus::SE3d T_cam_imu = camera_params.T_imu_cam.inverse();

  for (const Point& point : points_imu) {
    const Eigen::Vector3d point_cam = T_cam_imu * point.position;
    if (point_cam.z() <= 0.0) continue;

    const std::optional<Eigen::Vector2d> pixel =
        Project(camera_params, point_cam);
    if (!pixel || !InFrame(color_image, *pixel, kContrastRadius)) continue;
    if (LocalContrast(color_image, *pixel, kContrastRadius) >
        kMaxLocalContrast) {
      continue;
    }

    const Eigen::Vector3d position = T_world_imu * point.position;
    const VoxelIndex index{
        static_cast<std::int64_t>(std::floor(position.x() / voxel_size_)),
        static_cast<std::int64_t>(std::floor(position.y() / voxel_size_)),
        static_cast<std::int64_t>(std::floor(position.z() / voxel_size_))};
    if (done_.contains(index)) continue;

    const cv::Vec3b bgr = color_image.at<cv::Vec3b>(
        static_cast<int>(pixel->y()), static_cast<int>(pixel->x()));

    Pending& pending = pending_[index];
    pending.last_seen = frame_;
    if (pending.count == 0) pending.position = position;
    if (pending.count < kMaxSamples) {
      pending.samples[pending.count++] = bgr;
      channel_sum_ += Eigen::Vector3d(bgr[0], bgr[1], bgr[2]);
      ++sample_count_;
    }
  }
}

std::vector<ColorPoint> ColorCloud::TakeNew(bool flush_all) {
  const Eigen::Vector3d gains = WhiteBalanceGains();
  std::vector<ColorPoint> ready;

  for (auto it = pending_.begin(); it != pending_.end();) {
    const bool settled = it->second.count >= kMaxSamples ||
                         frame_ - it->second.last_seen >= kFlushAge;
    if (flush_all || settled) {
      ready.push_back(Resolve(it->second, gains));
      done_.insert(it->first);
      it = pending_.erase(it);
    } else {
      ++it;
    }
  }
  return ready;
}

ColorPoint ColorCloud::Resolve(const Pending& pending,
                               const Eigen::Vector3d& gains) const {
  // The middle sample by brightness, so one stray background pixel cannot
  // drag the colour. Taking a real sample keeps it a colour we actually saw.
  std::array<cv::Vec3b, kMaxSamples> sorted = pending.samples;
  const auto end = sorted.begin() + static_cast<std::ptrdiff_t>(pending.count);
  std::sort(sorted.begin(), end, [](const cv::Vec3b& a, const cv::Vec3b& b) {
    return Luminance(a) < Luminance(b);
  });
  const cv::Vec3b& bgr = sorted[pending.count / 2];

  const auto balance = [&gains](std::uint8_t value, int channel) {
    return static_cast<std::uint8_t>(
        std::clamp(value * gains[channel], 0.0, 255.0));
  };
  return {pending.position, balance(bgr[2], 2), balance(bgr[1], 1),
          balance(bgr[0], 0)};
}

Eigen::Vector3d ColorCloud::WhiteBalanceGains() const {
  if (sample_count_ == 0) return Eigen::Vector3d::Ones();
  const Eigen::Vector3d average =
      channel_sum_ / static_cast<double>(sample_count_);
  if (average.minCoeff() <= 0.0) return Eigen::Vector3d::Ones();
  return Eigen::Vector3d::Constant(average.mean()).cwiseQuotient(average);
}

}  // namespace livo_slam
