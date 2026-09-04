#include "projection.h"

#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <variant>

namespace livo_slam {

namespace {

Eigen::Vector2d Distort(const RadTanParams& params,
                        const Eigen::Vector2d& normalized) {
  // Distortion model is (following OpenCV's coefficient order, r² = x² + y²):
  //   radial = 1 + k1*r² + k2*r⁴
  //   x_d = x*radial + 2*p1*x*y + p2*(r² + 2x²)
  //   y_d = y*radial + p1*(r² + 2y²) + 2*p2*x*y

  const double x = normalized.x();
  const double y = normalized.y();
  const double r2 = x * x + y * y;
  const double radial = 1.0 + params.k1 * r2 + params.k2 * r2 * r2;

  const double x_d =
      x * radial + 2.0 * params.p1 * x * y + params.p2 * (r2 + 2.0 * x * x);
  const double y_d =
      y * radial + params.p1 * (r2 + 2.0 * y * y) + 2.0 * params.p2 * x * y;

  return {x_d, y_d};
}

Eigen::Matrix2d DistortionJacobian(const RadTanParams& params,
                                   const Eigen::Vector2d& normalized) {
  // Derivative of Distort, with a = k1 + 2*k2*r²:
  //   d(x_d)/dx = radial + 2x²a + 2*p1*y + 6*p2*x
  //   d(x_d)/dy = 2xy*a + 2*p1*x + 2*p2*y
  //   d(y_d)/dx = 2xy*a + 2*p1*x + 2*p2*y
  //   d(y_d)/dy = radial + 2y²a + 6*p1*y + 2*p2*x

  const double x = normalized.x();
  const double y = normalized.y();
  const double r2 = x * x + y * y;
  const double radial = 1.0 + params.k1 * r2 + params.k2 * r2 * r2;
  const double a = params.k1 + 2.0 * params.k2 * r2;

  // Symmetric, so the cross term is shared.
  const double cross =
      2.0 * x * y * a + 2.0 * params.p1 * x + 2.0 * params.p2 * y;

  Eigen::Matrix2d jacobian;
  jacobian(0, 0) =
      radial + 2.0 * x * x * a + 2.0 * params.p1 * y + 6.0 * params.p2 * x;
  jacobian(0, 1) = cross;
  jacobian(1, 0) = cross;
  jacobian(1, 1) =
      radial + 2.0 * y * y * a + 6.0 * params.p1 * y + 2.0 * params.p2 * x;
  return jacobian;
}

Eigen::Vector2d Distort(const EquidistantParams& /*params*/,
                        const Eigen::Vector2d& /*normalized*/) {
  throw std::runtime_error("equidistant distortion is not implemented");
}

Eigen::Matrix2d DistortionJacobian(const EquidistantParams& /*params*/,
                                   const Eigen::Vector2d& /*normalized*/) {
  throw std::runtime_error("equidistant distortion is not implemented");
}

}  // namespace

Eigen::Matrix2d DistortionJacobian(const CameraParams& params,
                                   const Eigen::Vector2d& normalized) {
  return std::visit(
      [&normalized](const auto& distortion) {
        return DistortionJacobian(distortion, normalized);
      },
      params.distortion);
}

Eigen::Vector2d Distort(const CameraParams& params,
                        const Eigen::Vector2d& normalized) {
  return std::visit(
      [&normalized](const auto& distortion) {
        return Distort(distortion, normalized);
      },
      params.distortion);
}

std::optional<Eigen::Vector2d> Project(const CameraParams& params,
                                       const Eigen::Vector3d& point_cam) {
  if (point_cam.z() <= 0.0) return std::nullopt;

  const Eigen::Vector2d distorted = Distort(
      params, {point_cam.x() / point_cam.z(), point_cam.y() / point_cam.z()});

  return Eigen::Vector2d(params.fx * distorted.x() + params.cx,
                         params.fy * distorted.y() + params.cy);
}

bool InFrame(const cv::Mat& image, const Eigen::Vector2d& pixel,
             std::size_t margin) {
  const double low = static_cast<double>(margin);
  const double high_x = static_cast<double>(image.cols) - low;
  const double high_y = static_cast<double>(image.rows) - low;
  // clang-format off
  return low <= pixel.x() && pixel.x() < high_x && 
         low <= pixel.y() && pixel.y() < high_y;
  // clang-format on
}

double SampleBilinear(const cv::Mat& image, const Eigen::Vector2d& pixel) {
  const int u0 = static_cast<int>(pixel.x());
  const int v0 = static_cast<int>(pixel.y());
  const double du = pixel.x() - static_cast<double>(u0);
  const double dv = pixel.y() - static_cast<double>(v0);

  // clang-format off
  const double sample =
    (1.0 - du) * (1.0 - dv) * image.at<std::uint8_t>(v0,     u0) +
           du  * (1.0 - dv) * image.at<std::uint8_t>(v0,     u0 + 1) +
    (1.0 - du) *        dv  * image.at<std::uint8_t>(v0 + 1, u0) +
           du  *        dv  * image.at<std::uint8_t>(v0 + 1, u0 + 1);
  // clang-format on
  return sample;
}

Eigen::Matrix<double, 2, 3> ProjectionJacobian(
    const CameraParams& params, const Eigen::Vector3d& point_cam) {
  // For pinhole with distortion, the projection is:
  // u = fx * x_distorted + cx
  // v = fy * y_distorted + cy
  // where (x_distorted, y_distorted) = Distort(x, y), see DistortionJacobian.
  //
  // By the chain rule the jacobian is diag(fx, fy) * D * N, where D is
  // d(x_distorted, y_distorted)/d(x, y) and N is d(x, y)/d(point_cam):
  // N = 1/z * [ 1, 0, -x ]
  //           [ 0, 1, -y ]
  //
  // With zero distortion D is the identity and this reduces to the pinhole
  // form:
  // [ fx/z,    0, -fx*x/z ]
  // [    0, fy/z, -fy*y/z ]

  const double inverse_z = 1.0 / point_cam.z();
  const Eigen::Vector2d normalized(point_cam.x() * inverse_z,
                                   point_cam.y() * inverse_z);

  // clang-format off
  Eigen::Matrix<double, 2, 3> normalization;
  normalization << 1.0, 0.0, -normalized.x(),
                   0.0, 1.0, -normalized.y();
  normalization *= inverse_z;
  // clang-format on

  const Eigen::Vector2d focal(params.fx, params.fy);
  return focal.asDiagonal() * DistortionJacobian(params, normalized) *
         normalization;
}

}  // namespace livo_slam
