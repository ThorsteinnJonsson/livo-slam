#pragma once

#include "core/types/state.h"

namespace livo_slam {

struct Estimate {
  State state;
  ErrorMatrix covariance = ErrorMatrix::Zero();
  double timestamp = 0.0;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace livo_slam
