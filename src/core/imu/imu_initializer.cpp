#include "core/imu/imu_initializer.h"

#include <Eigen/Geometry>

namespace livo_slam {
namespace {

struct MeanVar {
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  Eigen::Vector3d var = Eigen::Vector3d::Zero();
};

template <typename Accessor>
MeanVar ComputeMeanVar(std::span<const ImuSample> samples, Accessor get) {
  const auto count = static_cast<double>(samples.size());

  MeanVar result;
  for (const ImuSample& sample : samples) {
    result.mean += get(sample);
  }
  result.mean /= count;

  for (const ImuSample& sample : samples) {
    const Eigen::Vector3d deviation = get(sample) - result.mean;
    result.var += deviation.cwiseProduct(deviation);
  }
  result.var /= count;

  return result;
}

InitResult InitializeStatic(std::span<const ImuSample> window,
                            const InitParams& params) {
  InitResult result;
  result.window_size = window.size();
  if (window.empty()) {
    return result;
  }

  const MeanVar accel = ComputeMeanVar(
      window, [](const ImuSample& s) { return s.linear_acceleration; });
  const MeanVar gyro = ComputeMeanVar(
      window, [](const ImuSample& s) { return s.angular_velocity; });

  result.accel_mean_norm = accel.mean.norm();
  result.max_accel_var = accel.var.maxCoeff();
  result.max_gyro_var = gyro.var.maxCoeff();

  if (result.max_accel_var > params.max_accel_var ||
      result.max_gyro_var > params.max_gyro_var) {
    return result;
  }

  result.rotation = Sophus::SO3d(Eigen::Quaterniond::FromTwoVectors(
      accel.mean.normalized(), Eigen::Vector3d::UnitZ()));
  result.gravity = Eigen::Vector3d(0.0, 0.0, -params.gravity_magnitude);
  result.gyro_bias = gyro.mean;
  result.timestamp = window.back().timestamp;
  result.ok = true;
  return result;
}

}  // namespace

ImuInitializer::ImuInitializer(InitParams params) : params_(params) {
  window_.reserve(params_.window_size);
}

InitResult ImuInitializer::Add(std::span<const ImuSample> samples) {
  InitResult result;
  result.window_size = window_.size();

  for (const ImuSample& sample : samples) {
    window_.push_back(sample);
    if (window_.size() < params_.window_size) {
      result.window_size = window_.size();
      continue;
    }

    result = InitializeStatic(window_, params_);
    if (result.ok) {
      return result;
    }
    window_.erase(window_.begin());
  }

  return result;
}

}  // namespace livo_slam
