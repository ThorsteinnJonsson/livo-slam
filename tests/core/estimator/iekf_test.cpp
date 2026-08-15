#include "core/estimator/iekf.h"

#include <gtest/gtest.h>

namespace livo_slam {
namespace {

Estimate MakePrior() {
  Estimate prior;
  prior.state.rotation = Sophus::SO3d::exp({0.1, -0.2, 0.3});
  prior.state.position = {1.0, 2.0, 3.0};
  prior.timestamp = 42.0;

  ErrorState diagonal;
  for (int i = 0; i < kErrorDim; ++i) {
    diagonal(i) = 0.01 * (i + 1);
  }
  prior.covariance = diagonal.asDiagonal();
  return prior;
}

IekfSystem MakeSystem() {
  IekfSystem system;
  Eigen::Matrix<double, 6, 1> jacobian;
  jacobian << 0.3, -0.7, 0.2, 0.0, 0.0, 1.0;
  system.hth.block<6, 6>(kIdxRotation, kIdxRotation) =
      100.0 * jacobian * jacobian.transpose();
  system.htz.segment<6>(kIdxRotation) = 100.0 * jacobian * 0.05;
  return system;
}

TEST(SolveIekfStep, ZeroOffsetGivesTheInformationFormUpdate) {
  const Estimate prior = MakePrior();
  const IekfSystem system = MakeSystem();

  const IekfStepResult result = SolveIekfStep(prior, prior.state, system);

  const ErrorMatrix expected =
      (system.hth + prior.covariance.inverse()).inverse();
  EXPECT_TRUE(result.increment.isApprox(expected * system.htz));
}

TEST(SolveIekfStep, ReturnsTheInformationFormCovariance) {
  const Estimate prior = MakePrior();
  const IekfSystem system = MakeSystem();

  const IekfStepResult result = SolveIekfStep(prior, prior.state, system);

  const ErrorMatrix expected =
      (system.hth + prior.covariance.inverse()).inverse();
  EXPECT_TRUE(result.covariance.isApprox(expected));
}

TEST(SolveIekfStep, ShrinksTheConstrainedDirectionsOnly) {
  const Estimate prior = MakePrior();

  const IekfStepResult result =
      SolveIekfStep(prior, prior.state, MakeSystem());

  // MakeSystem uses one Jacobian with a (0, 0, 1) position part, so position z
  // is constrained while x and y are not. Nothing may gain variance.
  EXPECT_LT(result.covariance(kIdxPosition + 2, kIdxPosition + 2),
            prior.covariance(kIdxPosition + 2, kIdxPosition + 2));
  EXPECT_DOUBLE_EQ(result.covariance(kIdxPosition, kIdxPosition),
                   prior.covariance(kIdxPosition, kIdxPosition));

  for (int i = 0; i < kErrorDim; ++i) {
    EXPECT_LE(result.covariance(i, i), prior.covariance(i, i) + 1e-12)
        << "component " << i;
  }
}

TEST(SolveIekfStep, EmptySystemPullsTheIterateBackToThePrior) {
  const Estimate prior = MakePrior();

  State current = prior.state;
  current.position += Eigen::Vector3d(0.3, -0.1, 0.2);
  current.rotation = current.rotation * Sophus::SO3d::exp({0.02, 0.01, -0.03});

  const IekfStepResult result = SolveIekfStep(prior, current, IekfSystem{});

  const State recovered = BoxPlus(current, result.increment);
  EXPECT_LT(BoxMinus(recovered, prior.state).cwiseAbs().maxCoeff(), 1e-12);
}

TEST(SolveIekfStep, EmptySystemLeavesTheCovarianceUnchanged) {
  const Estimate prior = MakePrior();

  const IekfStepResult result =
      SolveIekfStep(prior, prior.state, IekfSystem{});

  EXPECT_TRUE(result.covariance.isApprox(prior.covariance));
}

TEST(IekfConverged, TrueWhenBothPartsAreSmall) {
  ErrorState increment = ErrorState::Zero();
  increment.segment<3>(kIdxRotation).setConstant(1e-5);
  increment.segment<3>(kIdxPosition).setConstant(1e-5);

  EXPECT_TRUE(IekfConverged(increment, 1e-3, 1e-3));
}

TEST(IekfConverged, FalseWhenTheRotationIsTooLarge) {
  ErrorState increment = ErrorState::Zero();
  increment(kIdxRotation) = 1.0;

  EXPECT_FALSE(IekfConverged(increment, 1e-3, 1e-3));
}

TEST(IekfConverged, FalseWhenThePositionIsTooLarge) {
  ErrorState increment = ErrorState::Zero();
  increment(kIdxPosition) = 1.0;

  EXPECT_FALSE(IekfConverged(increment, 1e-3, 1e-3));
}

TEST(IekfConverged, IgnoresTheOtherErrorStateBlocks) {
  ErrorState increment = ErrorState::Zero();
  increment(kIdxVelocity) = 1.0;
  increment(kIdxGyroBias) = 1.0;
  increment(kIdxGravity) = 1.0;

  EXPECT_TRUE(IekfConverged(increment, 1e-3, 1e-3));
}

TEST(IekfConverged, UsesSeparateThresholds) {
  ErrorState increment = ErrorState::Zero();
  increment(kIdxPosition) = 0.05;

  EXPECT_FALSE(IekfConverged(increment, 1.0, 1e-3));
  EXPECT_TRUE(IekfConverged(increment, 1.0, 1.0));
}

}  // namespace
}  // namespace livo_slam
