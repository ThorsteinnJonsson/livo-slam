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
  // Derivative of RadTan Distort, with a = k1 + 2*k2*r²:
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

Eigen::Vector2d Distort(const EquidistantParams& params,
                        const Eigen::Vector2d& normalized) {
  // Distortion model is (Kannala-Brandt, r = sqrt(x² + y²)):
  //   theta = atan(r)
  //   theta_d = theta*(1 + k1*theta² + k2*theta⁴ + k3*theta⁶ + k4*theta⁸)
  //   x_d = (theta_d/r) * x
  //   y_d = (theta_d/r) * y
  // The scale theta_d/r is 0/0 on the optical axis; its limit there is 1, so
  // r below a small epsilon returns the point unchanged.

  const double x = normalized.x();
  const double y = normalized.y();
  const double r = std::sqrt(x * x + y * y);
  if (r < 1e-6) {
    // Return unchanged to prevent division by zero.
    return normalized;
  }

  const double theta = std::atan(r);
  const double theta2 = theta * theta;
  const double theta4 = theta2 * theta2;
  const double theta6 = theta4 * theta2;
  const double theta8 = theta4 * theta4;
  const double theta_distorted =
      theta * (1.0 + params.k1 * theta2 + params.k2 * theta4 +
               params.k3 * theta6 + params.k4 * theta8);

  const double x_d = theta_distorted / r * x;
  const double y_d = theta_distorted / r * y;

  return {x_d, y_d};
}

Eigen::Matrix2d DistortionJacobian(const EquidistantParams& params,
                                   const Eigen::Vector2d& normalized) {
  // Derivative of equidistant Distort.
  // Only the distance from center changes, so
  // (x_d, y_d) = s · (x, y)  with s = θd / r.
  // Here s depends on x and y through r, hence
  // ∂s/∂x = (ds/dr)·(∂r/∂x) = (ds/dr)·(x/r)
  // ∂s/∂y = (ds/dr)·(∂r/∂y) = (ds/dr)·(y/r)
  // which gives
  // ∂x_d/∂x = s + (ds/dr)·(x²/r)      ∂x_d/∂y =     (ds/dr)·(xy/r)
  // ∂y_d/∂x =     (ds/dr)·(xy/r)      ∂y_d/∂y = s + (ds/dr)·(y²/r)
  // which equals
  //   s*I + (ds/dr / r) * [ x²  xy ]
  //                       [ xy  y² ]
  // Then
  // θd = θ(1 + k1*θ² + k2*θ⁴ + k3*θ⁶ + k4*θ⁸)   (Kannala-Brandt)
  // dθd/dθ  = 1 + 3k1*θ² + 5k2*θ⁴ + 7k3*θ⁶ + 9k4*θ⁸
  // dθd/dr  = (dθd/dθ) / (1 + r²)
  // ds/dr   = ((dθd/dr)·r − θd) / r²

  const double x = normalized.x();
  const double y = normalized.y();
  const double r = std::sqrt(x * x + y * y);
  if (r < 1e-6) {
    // Both the scale and its derivative are 0/0 here; the limit is no change.
    return Eigen::Matrix2d::Identity();
  }

  const double theta = std::atan(r);
  const double theta2 = theta * theta;
  const double theta4 = theta2 * theta2;
  const double theta6 = theta4 * theta2;
  const double theta8 = theta4 * theta4;

  const double theta_distorted =
      theta * (1.0 + params.k1 * theta2 + params.k2 * theta4 +
               params.k3 * theta6 + params.k4 * theta8);
  const double d_distorted_d_theta =
      1.0 + 3.0 * params.k1 * theta2 + 5.0 * params.k2 * theta4 +
      7.0 * params.k3 * theta6 + 9.0 * params.k4 * theta8;
  const double d_distorted_d_r = d_distorted_d_theta / (1.0 + r * r);

  const double scale = theta_distorted / r;
  const double d_scale_d_r = (d_distorted_d_r * r - theta_distorted) / (r * r);
  const double outer_scale = d_scale_d_r / r;

  // Symmetric, so the cross term is shared.
  const double cross = outer_scale * x * y;

  Eigen::Matrix2d jacobian;
  jacobian(0, 0) = scale + outer_scale * x * x;
  jacobian(0, 1) = cross;
  jacobian(1, 0) = cross;
  jacobian(1, 1) = scale + outer_scale * y * y;
  return jacobian;
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
