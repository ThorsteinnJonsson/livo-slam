#include "core/estimator/initial_estimate.h"

namespace livo_slam {

Estimate MakeInitialEstimate(const InitResult& result,
                             const InitialStateParams& params) {
  Estimate estimate;

  estimate.state.rotation = result.rotation;
  estimate.state.gravity = result.gravity;
  estimate.state.gyro_bias = result.gyro_bias;
  estimate.timestamp = result.timestamp;

  ErrorState diagonal;
  diagonal.segment<3>(kIdxRotation)
      .setConstant(std::pow(params.rotation_std, 2));
  diagonal.segment<3>(kIdxPosition)
      .setConstant(std::pow(params.position_std, 2));
  diagonal.segment<3>(kIdxVelocity)
      .setConstant(std::pow(params.velocity_std, 2));
  diagonal.segment<3>(kIdxGyroBias)
      .setConstant(std::pow(params.gyro_bias_std, 2));
  diagonal.segment<3>(kIdxAccelBias)
      .setConstant(std::pow(params.accel_bias_std, 2));
  diagonal.segment<3>(kIdxGravity).setConstant(std::pow(params.gravity_std, 2));
  diagonal(kIdxInvExposure) = std::pow(params.inv_exposure_std, 2);
  estimate.covariance = diagonal.asDiagonal();

  return estimate;
}

}  // namespace livo_slam
