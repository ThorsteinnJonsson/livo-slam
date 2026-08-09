#include "core/estimator/initial_estimate.h"

#include <gtest/gtest.h>

namespace livo_slam {
namespace {

InitResult MakeResult() {
  InitResult result;
  result.status = InitStatus::kInitialized;
  result.rotation = Sophus::SO3d::exp(Eigen::Vector3d(0.0, 0.05, 0.0));
  result.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  result.gyro_bias = Eigen::Vector3d(0.001, -0.002, 0.003);
  result.timestamp = 1234.5;
  return result;
}

TEST(MakeInitialEstimate, CopiesObservedQuantities) {
  const InitResult result = MakeResult();
  const Estimate estimate = MakeInitialEstimate(result, InitialStateParams{});

  EXPECT_LT((estimate.state.rotation.inverse() * result.rotation).log().norm(),
            1e-12);
  EXPECT_EQ(estimate.state.gravity, result.gravity);
  EXPECT_EQ(estimate.state.gyro_bias, result.gyro_bias);
  EXPECT_DOUBLE_EQ(estimate.timestamp, result.timestamp);
}

TEST(MakeInitialEstimate, LeavesUnobservableQuantitiesAtZero) {
  const Estimate estimate =
      MakeInitialEstimate(MakeResult(), InitialStateParams{});

  EXPECT_EQ(estimate.state.position, Eigen::Vector3d::Zero());
  EXPECT_EQ(estimate.state.velocity, Eigen::Vector3d::Zero());
  EXPECT_EQ(estimate.state.accel_bias, Eigen::Vector3d::Zero());
}

TEST(MakeInitialEstimate, CovarianceDiagonalIsSquaredStdDevs) {
  InitialStateParams params;
  params.rotation_std = 0.02;
  params.position_std = 0.003;
  params.velocity_std = 0.04;
  params.gyro_bias_std = 0.005;
  params.accel_bias_std = 0.06;
  params.gravity_std = 0.007;

  const Estimate estimate = MakeInitialEstimate(MakeResult(), params);
  const ErrorMatrix& covariance = estimate.covariance;

  EXPECT_DOUBLE_EQ(covariance(kIdxRotation, kIdxRotation), 0.02 * 0.02);
  EXPECT_DOUBLE_EQ(covariance(kIdxPosition, kIdxPosition), 0.003 * 0.003);
  EXPECT_DOUBLE_EQ(covariance(kIdxVelocity, kIdxVelocity), 0.04 * 0.04);
  EXPECT_DOUBLE_EQ(covariance(kIdxGyroBias, kIdxGyroBias), 0.005 * 0.005);
  EXPECT_DOUBLE_EQ(covariance(kIdxAccelBias, kIdxAccelBias), 0.06 * 0.06);
  EXPECT_DOUBLE_EQ(covariance(kIdxGravity, kIdxGravity), 0.007 * 0.007);
}

TEST(MakeInitialEstimate, CovarianceIsDiagonal) {
  const Estimate estimate =
      MakeInitialEstimate(MakeResult(), InitialStateParams{});
  const ErrorMatrix diagonal_only = estimate.covariance.diagonal().asDiagonal();
  EXPECT_LT((estimate.covariance - diagonal_only).norm(), 1e-15);
}

}  // namespace
}  // namespace livo_slam
