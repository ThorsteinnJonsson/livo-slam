#pragma once

#include <Eigen/Core>
#include <sophus/so3.hpp>

namespace livo_slam {

struct State {
  Sophus::SO3d rotation;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.81);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline constexpr int kErrorDim = 18;

inline constexpr int kIdxRotation = 0;
inline constexpr int kIdxPosition = 3;
inline constexpr int kIdxVelocity = 6;
inline constexpr int kIdxGyroBias = 9;
inline constexpr int kIdxAccelBias = 12;
inline constexpr int kIdxGravity = 15;

using ErrorState = Eigen::Matrix<double, kErrorDim, 1>;
using ErrorMatrix = Eigen::Matrix<double, kErrorDim, kErrorDim>;

inline State BoxPlus(const State& state, const ErrorState& delta) {
  State out;
  out.rotation =
      state.rotation * Sophus::SO3d::exp(delta.segment<3>(kIdxRotation));
  out.position = state.position + delta.segment<3>(kIdxPosition);
  out.velocity = state.velocity + delta.segment<3>(kIdxVelocity);
  out.gyro_bias = state.gyro_bias + delta.segment<3>(kIdxGyroBias);
  out.accel_bias = state.accel_bias + delta.segment<3>(kIdxAccelBias);
  out.gravity = state.gravity + delta.segment<3>(kIdxGravity);
  return out;
}

inline ErrorState BoxMinus(const State& a, const State& b) {
  ErrorState delta;
  delta.segment<3>(kIdxRotation) = (b.rotation.inverse() * a.rotation).log();
  delta.segment<3>(kIdxPosition) = a.position - b.position;
  delta.segment<3>(kIdxVelocity) = a.velocity - b.velocity;
  delta.segment<3>(kIdxGyroBias) = a.gyro_bias - b.gyro_bias;
  delta.segment<3>(kIdxAccelBias) = a.accel_bias - b.accel_bias;
  delta.segment<3>(kIdxGravity) = a.gravity - b.gravity;
  return delta;
}

}  // namespace livo_slam
