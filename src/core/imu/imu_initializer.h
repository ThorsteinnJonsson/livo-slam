#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <sophus/so3.hpp>
#include <span>
#include <vector>

#include "core/config/init_params.h"
#include "core/types/imu_sample.h"

namespace livo_slam {

enum class InitStatus { kCollecting, kNotStationary, kInitialized };

struct InitResult {
  InitStatus status = InitStatus::kCollecting;

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
  explicit ImuInitializer(ImuInitParams params);

  void Add(std::span<const ImuSample> samples);

  InitStatus status() const { return status_; }
  const std::optional<InitResult>& init_result() const { return init_result_; }

 private:
  ImuInitParams params_;

  InitStatus status_ = InitStatus::kCollecting;
  std::optional<InitResult>
      init_result_;  // Only filled when status == InitStatus::kInitialized

  std::vector<ImuSample> window_;
};

}  // namespace livo_slam
