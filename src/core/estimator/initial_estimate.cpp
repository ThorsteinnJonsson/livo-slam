#include "core/estimator/initial_estimate.h"

namespace livo_slam {
namespace {

double Square(double value) { return value * value; }

}  // namespace

Estimate MakeInitialEstimate(const InitResult& result,
                             const InitialStateParams& params) {
  Estimate estimate;

  estimate.state.rotation = result.rotation;
  estimate.state.gravity = result.gravity;
  estimate.state.gyro_bias = result.gyro_bias;
  estimate.timestamp = result.timestamp;

  ErrorState diagonal;
  diagonal.segment<3>(kIdxRotation).setConstant(Square(params.rotation_std));
  diagonal.segment<3>(kIdxPosition).setConstant(Square(params.position_std));
  diagonal.segment<3>(kIdxVelocity).setConstant(Square(params.velocity_std));
  diagonal.segment<3>(kIdxGyroBias).setConstant(Square(params.gyro_bias_std));
  diagonal.segment<3>(kIdxAccelBias).setConstant(Square(params.accel_bias_std));
  diagonal.segment<3>(kIdxGravity).setConstant(Square(params.gravity_std));
  estimate.covariance = diagonal.asDiagonal();

  return estimate;
}

}  // namespace livo_slam
