#include "projection.h"

#include <optional>

namespace livo_slam {

std::optional<Eigen::Vector2d> Project(const CameraParams& params,
                                       const Eigen::Vector3d& point_cam) {
  // TODO: Implement
  return std::nullopt;
}

bool InFrame(const CameraParams& params, const Eigen::Vector2d& pixel,
             int margin) {
  // TODO: Implement
  return false;
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