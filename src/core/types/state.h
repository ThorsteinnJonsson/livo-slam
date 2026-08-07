#pragma once

#include <Eigen/Core>
#include <sophus/so3.hpp>

#include "core/math/s2.h"

namespace livo_slam {

struct State {
  Sophus::SO3d rotation;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity = Eigen::Vector3d(0.0, 0.0, -9.81);

  Sophus::SO3d rotation_imu_lidar;
  Eigen::Vector3d position_imu_lidar = Eigen::Vector3d::Zero();

  Sophus::SO3d rotation_imu_camera;
  Eigen::Vector3d position_imu_camera = Eigen::Vector3d::Zero();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline constexpr int kErrorDim = 29;

inline constexpr int kIdxRotation = 0;
inline constexpr int kIdxPosition = 3;
inline constexpr int kIdxVelocity = 6;
inline constexpr int kIdxGyroBias = 9;
inline constexpr int kIdxAccelBias = 12;
inline constexpr int kIdxGravity = 15;
inline constexpr int kIdxLidarRotation = 17;
inline constexpr int kIdxLidarPosition = 20;
inline constexpr int kIdxCameraRotation = 23;
inline constexpr int kIdxCameraPosition = 26;

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
  out.gravity = S2BoxPlus(state.gravity, delta.segment<2>(kIdxGravity));
  out.rotation_imu_lidar =
      state.rotation_imu_lidar *
      Sophus::SO3d::exp(delta.segment<3>(kIdxLidarRotation));
  out.position_imu_lidar =
      state.position_imu_lidar + delta.segment<3>(kIdxLidarPosition);
  out.rotation_imu_camera =
      state.rotation_imu_camera *
      Sophus::SO3d::exp(delta.segment<3>(kIdxCameraRotation));
  out.position_imu_camera =
      state.position_imu_camera + delta.segment<3>(kIdxCameraPosition);
  return out;
}

inline ErrorState BoxMinus(const State& a, const State& b) {
  ErrorState delta;
  delta.segment<3>(kIdxRotation) = (b.rotation.inverse() * a.rotation).log();
  delta.segment<3>(kIdxPosition) = a.position - b.position;
  delta.segment<3>(kIdxVelocity) = a.velocity - b.velocity;
  delta.segment<3>(kIdxGyroBias) = a.gyro_bias - b.gyro_bias;
  delta.segment<3>(kIdxAccelBias) = a.accel_bias - b.accel_bias;
  delta.segment<2>(kIdxGravity) = S2BoxMinus(a.gravity, b.gravity);
  delta.segment<3>(kIdxLidarRotation) =
      (b.rotation_imu_lidar.inverse() * a.rotation_imu_lidar).log();
  delta.segment<3>(kIdxLidarPosition) =
      a.position_imu_lidar - b.position_imu_lidar;
  delta.segment<3>(kIdxCameraRotation) =
      (b.rotation_imu_camera.inverse() * a.rotation_imu_camera).log();
  delta.segment<3>(kIdxCameraPosition) =
      a.position_imu_camera - b.position_imu_camera;
  return delta;
}

}  // namespace livo_slam
