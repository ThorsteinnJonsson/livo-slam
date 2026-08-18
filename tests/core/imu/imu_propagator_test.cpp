#include "core/imu/imu_propagator.h"

#include <gtest/gtest.h>

#include <vector>

namespace livo_slam {
namespace {

Estimate MakeEstimate() {
  Estimate estimate;
  estimate.timestamp = 0.0;
  estimate.state.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  estimate.covariance = ErrorMatrix::Identity() * 1e-6;
  return estimate;
}

std::vector<ImuSample> MakeSamples(std::size_t count, double dt,
                                   const Eigen::Vector3d& accel,
                                   const Eigen::Vector3d& gyro) {
  std::vector<ImuSample> samples;
  samples.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    ImuSample sample;
    sample.timestamp = dt * static_cast<double>(i);
    sample.linear_acceleration = accel;
    sample.angular_velocity = gyro;
    samples.push_back(sample);
  }
  return samples;
}

TEST(Propagate, StationaryPlatformDoesNotDrift) {
  const auto samples = MakeSamples(41, 0.0025, Eigen::Vector3d(0.0, 0.0, 9.81),
                                   Eigen::Vector3d::Zero());
  const auto result =
      Propagate(MakeEstimate(), samples, 0.1, ProcessNoiseParams{});

  EXPECT_LT(result.estimate.state.position.norm(), 1e-12);
  EXPECT_LT(result.estimate.state.velocity.norm(), 1e-12);
}

TEST(Propagate, ConstantAccelerationIntegratesAnalytically) {
  const auto samples = MakeSamples(41, 0.0025, Eigen::Vector3d(1.0, 0.0, 9.81),
                                   Eigen::Vector3d::Zero());
  const auto result =
      Propagate(MakeEstimate(), samples, 0.1, ProcessNoiseParams{});

  EXPECT_NEAR(result.estimate.state.velocity.x(), 0.1, 1e-12);
  EXPECT_NEAR(result.estimate.state.position.x(), 0.005, 1e-12);
}

TEST(Propagate, EstimateLandsOnEndTime) {
  const auto samples = MakeSamples(10, 0.0025, Eigen::Vector3d(0.0, 0.0, 9.81),
                                   Eigen::Vector3d::Zero());
  const auto result =
      Propagate(MakeEstimate(), samples, 0.5, ProcessNoiseParams{});
  EXPECT_DOUBLE_EQ(result.estimate.timestamp, 0.5);
}

TEST(Propagate, EmptySamplesOnlyAdvancesTime) {
  const Estimate estimate = MakeEstimate();
  const auto result = Propagate(estimate, {}, 0.25, ProcessNoiseParams{});

  EXPECT_DOUBLE_EQ(result.estimate.timestamp, 0.25);
  EXPECT_LT(BoxMinus(result.estimate.state, estimate.state).norm(), 1e-15);
  EXPECT_TRUE(result.trajectory.empty());
}

TEST(Propagate, TrajectoryStartsAtEstimateTimeAndCoversSamples) {
  const auto samples = MakeSamples(5, 0.01, Eigen::Vector3d(0.0, 0.0, 9.81),
                                   Eigen::Vector3d::Zero());
  const auto result =
      Propagate(MakeEstimate(), samples, 0.1, ProcessNoiseParams{});

  ASSERT_FALSE(result.trajectory.empty());
  EXPECT_DOUBLE_EQ(result.trajectory.front().timestamp, 0.0);
  EXPECT_DOUBLE_EQ(result.trajectory.back().timestamp,
                   samples.back().timestamp);
}

TEST(Propagate, RotationIntegratesConstantAngularRate) {
  const double rate = 0.5;
  const auto samples = MakeSamples(101, 0.001, Eigen::Vector3d(0.0, 0.0, 9.81),
                                   Eigen::Vector3d(0.0, 0.0, rate));
  const auto result =
      Propagate(MakeEstimate(), samples, 0.1, ProcessNoiseParams{});

  EXPECT_NEAR(result.estimate.state.rotation.log().z(), rate * 0.1, 1e-9);
}

TEST(Propagate, CovarianceGrows) {
  const Estimate estimate = MakeEstimate();
  const auto samples = MakeSamples(41, 0.0025, Eigen::Vector3d(0.0, 0.0, 9.81),
                                   Eigen::Vector3d::Zero());
  const auto result = Propagate(estimate, samples, 0.1, ProcessNoiseParams{});

  EXPECT_GT(result.estimate.covariance(kIdxRotation, kIdxRotation),
            estimate.covariance(kIdxRotation, kIdxRotation));
  EXPECT_GT(result.estimate.covariance(kIdxVelocity, kIdxVelocity),
            estimate.covariance(kIdxVelocity, kIdxVelocity));
}

TEST(Propagate, GyroBiasIsRemovedFromMeasurements) {
  Estimate estimate = MakeEstimate();
  estimate.state.gyro_bias = Eigen::Vector3d(0.0, 0.0, 0.5);

  const auto samples = MakeSamples(101, 0.001, Eigen::Vector3d(0.0, 0.0, 9.81),
                                   Eigen::Vector3d(0.0, 0.0, 0.5));
  const auto result = Propagate(estimate, samples, 0.1, ProcessNoiseParams{});

  EXPECT_LT(result.estimate.state.rotation.log().norm(), 1e-12);
}

}  // namespace
}  // namespace livo_slam
