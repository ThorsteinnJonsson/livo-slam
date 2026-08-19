#include "projection.h"

#include <optional>

namespace livo_slam {

std::optional<Eigen::Vector2d> Project(const CameraParams& params,
                                       const Eigen::Vector3d& point_cam) {
  if (point_cam.z() <= 0.0) return std::nullopt;

  const double u = params.fx * point_cam.x() / point_cam.z() + params.cx;
  const double v = params.fy * point_cam.y() / point_cam.z() + params.cy;
  return Eigen::Vector2d(u, v);
}

bool InFrame(const CameraParams& params, const Eigen::Vector2d& pixel,
             int margin) {
  const double low = static_cast<double>(margin);
  const double high_x = static_cast<double>(params.width) - low;
  const double high_y = static_cast<double>(params.height) - low;
  // clang-format off
  return low <= pixel.x() && pixel.x() < high_x && 
         low <= pixel.y() && pixel.y() < high_y;
  // clang-format on
}

double SampleBilinear(const cv::Mat& image, const Eigen::Vector2d& pixel) {
  // TODO: Implement
  return -1.0;
}

Eigen::Matrix<double, 2, 3> ProjectionJacobian(
    const CameraParams& params, const Eigen::Vector3d& point_cam) {
  // TODO: Implement
  return {};
}

}  // namespace livo_slam