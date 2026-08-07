#pragma once

#include <Eigen/Core>
#include <cmath>
#include <sophus/so3.hpp>

namespace livo_slam {

inline Eigen::Matrix<double, 3, 2> S2TangentBasis(const Eigen::Vector3d& g) {
  const Eigen::Vector3d u = g.normalized();
  const Eigen::Vector3d reference = std::abs(u.z()) < 0.9
                                        ? Eigen::Vector3d::UnitZ()
                                        : Eigen::Vector3d::UnitX();
  const Eigen::Vector3d b1 = (reference - reference.dot(u) * u).normalized();
  const Eigen::Vector3d b2 = u.cross(b1);

  Eigen::Matrix<double, 3, 2> basis;
  basis.col(0) = b1;
  basis.col(1) = b2;
  return basis;
}

inline Eigen::Vector3d S2BoxPlus(const Eigen::Vector3d& g,
                                 const Eigen::Vector2d& delta) {
  return Sophus::SO3d::exp(S2TangentBasis(g) * delta) * g;
}

inline Eigen::Vector2d S2BoxMinus(const Eigen::Vector3d& a,
                                  const Eigen::Vector3d& b) {
  const Eigen::Vector3d ua = a.normalized();
  const Eigen::Vector3d ub = b.normalized();
  const Eigen::Vector3d cross = ub.cross(ua);
  const double sine = cross.norm();
  const double cosine = ub.dot(ua);

  Eigen::Vector3d phi = Eigen::Vector3d::Zero();
  if (sine > 1e-12) {
    phi = std::atan2(sine, cosine) * cross / sine;
  }
  return S2TangentBasis(b).transpose() * phi;
}

}  // namespace livo_slam
