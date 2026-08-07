#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <sophus/so3.hpp>
#include <span>
#include <vector>

#include "core/types/imu_sample.h"

namespace livo_slam {

struct InitParams {
  std::size_t window_size = 200;
  double gravity_magnitude = 9.81;
  double max_accel_var = 0.05;
  double max_gyro_var = 0.01;
};

struct InitResult {
  bool ok = false;

  Sophus::SO3d rotation;
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
  double timestamp = 0.0;

  std::size_t window_size = 0;
  double accel_mean_norm = 0.0;
  double max_accel_var = 0.0;
  double max_gyro_var = 0.0;
};

class ImuInitializer {
 public:
  explicit ImuInitializer(InitParams params);

  InitResult Add(std::span<const ImuSample> samples);

 private:
  InitParams params_;
  std::vector<ImuSample> window_;
};

}  // namespace livo_slam
