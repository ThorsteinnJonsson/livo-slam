#include "core/types/state.h"

#include <gtest/gtest.h>

namespace livo_slam {
namespace {

State MakeState() {
  State state;
  state.rotation = Sophus::SO3d::exp(Eigen::Vector3d(0.1, -0.2, 0.3));
  state.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  state.velocity = Eigen::Vector3d(-0.5, 0.25, 0.75);
  state.gyro_bias = Eigen::Vector3d(0.01, 0.02, 0.03);
  state.accel_bias = Eigen::Vector3d(-0.04, 0.05, 0.06);
  state.gravity = Eigen::Vector3d(0.1, -0.2, -9.79);
  return state;
}

TEST(State, BoxPlusWithZeroDeltaIsIdentity) {
  const State state = MakeState();
  const State out = BoxPlus(state, ErrorState::Zero());
  EXPECT_LT(BoxMinus(out, state).norm(), 1e-12);
}

TEST(State, BoxMinusInvertsBoxPlus) {
  const State state = MakeState();

  ErrorState delta;
  for (int i = 0; i < kErrorDim; ++i) {
    delta(i) = 0.01 * static_cast<double>(i + 1);
  }

  const ErrorState recovered = BoxMinus(BoxPlus(state, delta), state);
  EXPECT_LT((recovered - delta).norm(), 1e-12);
}

TEST(State, ErrorDimensionsMatchBlockLayout) {
  EXPECT_EQ(kIdxRotation, 0);
  EXPECT_EQ(kIdxGravity + 3, kErrorDim);
}

}  // namespace
}  // namespace livo_slam
